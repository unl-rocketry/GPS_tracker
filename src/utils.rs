/// Calculate the CRC for some arbitrary data.
#[must_use]
pub fn crc8(arr: &[u8]) -> u8 {
    let mut crc = 0x00;
    for element in arr {
        crc ^= element;
        for _ in 0..8 {
            if crc & 0x80 > 0 {
                crc = (crc << 1) ^ 0xd5;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}
