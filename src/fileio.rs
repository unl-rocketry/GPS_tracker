use embedded_sdmmc::{BlockDevice, Directory, Error, Mode, TimeSource};
fn write_file<D: BlockDevice, T: TimeSource, const DIRS: usize, const FILES: usize, const VOLUMES: usize>(
    root_dir: &mut Directory<D, T, DIRS, FILES, VOLUMES>,
) -> Result<(), Error<D::Error>>
{
    let my_other_file = root_dir.open_file_in_dir("MY_DATA.CSV", Mode::ReadWriteCreateOrAppend)?;
    my_other_file.write(b"Timestamp,Signal,Value\n")?;
    my_other_file.write(b"2025-01-01T00:00:00Z,TEMP,25.0\n")?;
    my_other_file.write(b"2025-01-01T00:00:01Z,TEMP,25.1\n")?;
    my_other_file.write(b"2025-01-01T00:00:02Z,TEMP,25.2\n")?;
    // Don't forget to flush the file so that the directory entry is updated
    my_other_file.flush()?;
    Ok(())
}


fn example<S, D, T>(spi: S, delay: D, ts: T) -> Result<(), Error<SdCardError>>
where
    S: embedded_hal::spi::SpiDevice,
    D: embedded_hal::delay::DelayNs,
    T: TimeSource,
{
    let sdcard = SdCard::new(spi, delay);
    println!("Card size is {} bytes", sdcard.num_bytes()?);
    let volume_mgr = VolumeManager::new(sdcard, ts);
    let volume0 = volume_mgr.open_volume(VolumeIdx(0))?;
    println!("Volume 0: {:?}", volume0);
    let root_dir = volume0.open_root_dir()?;
    let mut my_file = root_dir.open_file_in_dir("MY_FILE.TXT", Mode::ReadOnly)?;
    while !my_file.is_eof() {
        let mut buffer = [0u8; 32];
        let num_read = my_file.read(&mut buffer)?;
        for b in &buffer[0..num_read] {
            print!("{}", *b as char);
        }
    }
    Ok(())
}