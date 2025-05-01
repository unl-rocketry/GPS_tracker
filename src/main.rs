#![no_std]
#![no_main]

use alloc::string::{String, ToString};
use embassy_executor::Spawner;
use embassy_sync::{
    channel::{Channel, Receiver, Sender},
    blocking_mutex::{raw::CriticalSectionRawMutex, NoopMutex},
};
use embassy_time::{Duration, Timer};
use embedded_io::Write;
use esp_backtrace as _;
use esp_hal::Blocking;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{AnyPin, Pin};
use esp_hal::timer::timg::TimerGroup;
use esp_hal::uart::{AnyUart, Config, Uart};
use gps_tracker::{GpsInfo, TelemetryPacket};
use log::{debug, info};
use nmea::{Nmea, SentenceType};

extern crate alloc;

static GPS_CHANNEL: Channel<CriticalSectionRawMutex, GpsInfo, 1> = Channel::new();

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    // generator version: 0.3.1

    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 72 * 1024);

    let timer0 = TimerGroup::new(peripherals.TIMG1);
    esp_hal_embassy::init(timer0.timer0);

    info!("Embassy initialized!");

    let gps_receiver = GPS_CHANNEL.receiver();
    spawner.spawn(gps_reader(peripherals.GPIO1.degrade(), AnyUart::from(peripherals.UART1), GPS_CHANNEL.sender())).unwrap(); //TODO make sure this is using the right GPIO pin

    // Set up the RFD serial port. This must utilize the proper port on the esp
    let mut rfd_send = Uart::new(peripherals.UART2, Config::default().with_baudrate(57600))
        .unwrap()
        .with_tx(peripherals.GPIO2); //TODO make sure this is the right GPIO pin

    let mut gps_data = None;

    loop {
        match gps_receiver.try_receive() {
            Ok(g) => gps_data = Some(g),
            Err(_) => {}
        };

        // Construct a packet from the data
        let packet = TelemetryPacket {
            gps: gps_data,
            power_info: None,
            environmental_info: None,
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

        debug!(
            "Sent {} bytes, checksum 0x{:0X}",
            json_vec.len(),
            packet_crc
        );

        rfd_send.flush().unwrap();
        Timer::after(Duration::from_millis(250)).await;
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

    let mut buffer = [0u8; 4096];

    loop {
        let byte_count = gps_port.read(&mut buffer).unwrap();

        if byte_count == 0 {
            continue;
        }

        let new_string = String::from_utf8_lossy(&buffer[..byte_count]);

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

        channel_sender.send(gps_data).await;
    }
}