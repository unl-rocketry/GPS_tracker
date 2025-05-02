#![feature(string_from_utf8_lossy_owned)]
#![no_std]
#![no_main]

use mcp9808::reg_conf::{Configuration, CriticalLock};
use alloc::string::{String, ToString};
use alloc::vec::Vec;
use core::ptr::addr_of_mut;
use embassy_executor::Spawner;
use embassy_sync::{
    channel::{Channel, Sender},
    blocking_mutex::raw::CriticalSectionRawMutex,
};
use embassy_time::Timer;
use embedded_io::Write;
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{AnyPin, Pin};
use esp_hal::i2c;
use esp_hal::i2c::master::{AnyI2c, I2c};
use esp_hal::system::{CpuControl, Stack};
use esp_hal::timer::timg::TimerGroup;
use esp_hal::uart::{AnyUart, Config, DataBits, Parity, StopBits, Uart};
use esp_hal_embassy::Executor;
use gps_tracker::{EnvironmentalInfo, GpsInfo, TelemetryPacket};
use log::info;
use mcp9808::address::SlaveAddress;
use mcp9808::MCP9808;
use mcp9808::reg_conf::{ShutdownMode, WindowLock};
use mcp9808::reg_res::ResolutionVal;
use mcp9808::reg_temp_generic::ReadableTempRegister;
use nmea::{Nmea, SentenceType};
use static_cell::StaticCell;

extern crate alloc;

static mut APP_CORE_STACK: Stack<8192> = Stack::new();
static GPS_CHANNEL: Channel<CriticalSectionRawMutex, GpsInfo, 1> = Channel::new();
static TEMP_CHANNEL: Channel<CriticalSectionRawMutex, EnvironmentalInfo, 1, > = Channel::new();

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    // generator version: 0.3.1

    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 72 * 1024);

    let timer0 = TimerGroup::new(peripherals.TIMG1);
    esp_hal_embassy::init(timer0.timer0);

    info!("Embassy initialized!");

    let mut cpu_control = CpuControl::new(peripherals.CPU_CTRL);

    let gps_receiver = GPS_CHANNEL.receiver();
    let temp_reciever = TEMP_CHANNEL.receiver();

    let sda = peripherals.GPIO15.degrade();
    let scl = peripherals.GPIO13.degrade();

    let _guard = cpu_control
        .start_app_core(unsafe { &mut *addr_of_mut!(APP_CORE_STACK) }, move || {
            static EXECUTOR: StaticCell<Executor> = StaticCell::new();
            let executor = EXECUTOR.init(Executor::new());
            executor.run(|spawner| {
                spawner.spawn(gps_reader(peripherals.GPIO14.degrade(), AnyUart::from(peripherals.UART1), GPS_CHANNEL.sender())).ok();
                spawner.spawn(temp_reader(sda, scl, AnyI2c::from(peripherals.I2C0), TEMP_CHANNEL.sender())).ok();
            });
        })
        .unwrap();

    // Set up the RFD serial port. This must utilize the proper port on the esp
    let mut rfd_send = Uart::new(
        peripherals.UART2, 
        Config::default().with_baudrate(57600).with_stop_bits(StopBits::_1).with_data_bits(DataBits::_8).with_parity(Parity::None))
        .unwrap()
        .with_tx(peripherals.GPIO26);

    let mut gps_data = None;
    let mut env_data = None;

    loop {
        if let Ok(g) = gps_receiver.try_receive() { gps_data = Some(g) }
        if let Ok(g) = temp_reciever.try_receive() { env_data = Some(g) }

        // Construct a packet from the data
        let packet = TelemetryPacket {
            gps: gps_data,
            power_info: None,
            environmental_info: env_data,
        };

        // Calculate the CRC of the packet based on its data.
        let packet_crc = packet.crc();

        // Write the data out
        rfd_send
            .write_all(packet_crc.to_string().as_bytes())
            .unwrap();
        rfd_send.write_all(b" ").unwrap();
        let json_vec = serde_json::to_vec(&packet).unwrap();
        rfd_send.write_all(&json_vec).unwrap();
        rfd_send.write_all(b"\n").unwrap();

        info!(
            "Sent {} bytes, checksum 0x{:0X}",
            json_vec.len(),
            packet_crc
        );

        rfd_send.flush().unwrap();
        Timer::after_millis(250).await;
    }
}

#[embassy_executor::task]
async fn gps_reader(pin: AnyPin, uart: AnyUart, channel_sender: Sender<'static, CriticalSectionRawMutex, GpsInfo, 1>) {// Set up and configure the NMEA parser.

    // Set up the GPS serial port. This must utilize the proper port on the esp
    let mut gps_port = Uart::new(uart, Config::default().with_baudrate(9600))
        .unwrap()
        .with_rx(pin);

    // Set up and configure the NMEA parser.
    let mut nmea_parser = Nmea::create_for_navigation(&[SentenceType::GGA]).unwrap();

    let mut buffer = Vec::new();
    let mut anotherbuffer = [0u8; 1];

    loop {
        let byte_count = gps_port.read(&mut anotherbuffer).unwrap_or_default();

        if byte_count == 0 {
            continue;
        }
        
        if anotherbuffer[0] != b'\n' {
            buffer.push(anotherbuffer[0]);
            continue
        }

        let new_string = String::from_utf8_lossy_owned(buffer.clone());
        buffer.clear();

        for line in new_string
            .lines()
            .filter(|l| !l.is_empty())
            .filter(|l| l.starts_with("$"))
        {
            let _ = nmea_parser.parse_for_fix(line);
        }

        if nmea_parser.latitude.is_none()
            || nmea_parser.longitude.is_none()
            || nmea_parser.altitude.is_none()
        {
            continue;
        }

        let gps_data = GpsInfo {
            latitude: nmea_parser.latitude.unwrap(),
            longitude: nmea_parser.longitude.unwrap(),
            altitude: nmea_parser.altitude.unwrap(),
        };

        let _ = channel_sender.try_send(gps_data);
    }
}

#[embassy_executor::task]
async fn temp_reader(sda: AnyPin, scl: AnyPin, i2c: AnyI2c, channel_sender: Sender<'static, CriticalSectionRawMutex, EnvironmentalInfo, 1>) {

    let i2c_bus = I2c::new(
        i2c,
        i2c::master::Config::default().with_timeout(i2c::master::BusTimeout::Maximum),
    )
        .unwrap()
        .with_sda(sda)
        .with_scl(scl)
        .into_async();
    // let i2c_bus = RefCell::new(i2c_bus);

    let mut mcp = MCP9808::new(i2c_bus);
    mcp.set_address(SlaveAddress::Default);
    let mut conf =mcp.read_configuration().unwrap();
    conf.set_shutdown_mode(ShutdownMode::Continuous);
    conf.set_window_lock(WindowLock::Unlocked);
    conf.set_critical_lock(CriticalLock::Unlocked);
    mcp.write_register(conf).unwrap();

    loop {
        let temp_reg= mcp.read_temperature().unwrap();
        let temp: f64 = temp_reg.get_celsius(ResolutionVal::Deg_0_0625C) as f64;

        let env_info = EnvironmentalInfo {
            pressure: 0.0,
            temperature: temp,
        };

        let _ = channel_sender.try_send(env_info);
    }
}