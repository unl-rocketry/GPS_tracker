#![no_std]

mod utils;

use serde::{Deserialize, Serialize};
use utils::crc8;

/// A packet sent from the rocket to the ground station.
///
/// Contains information about position and internal payload conditions.
/// Most fields are optional, as it is possible for any part of the payload
/// to be not functioning while still grabbing some data from it.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct TelemetryPacket {
    /// Full GPS telemetry information
    pub gps: Option<GpsInfo>,

    /// Environmental information
    pub environmental_info: Option<EnvironmentalInfo>,

    /// Battery related information
    pub power_info: Option<PowerInfo>,
}

impl TelemetryPacket {
    /// Calculate CRC from json serialized packet data.
    pub fn crc(&self) -> u8 {
        let self_json = serde_json::to_vec(self).unwrap();
        crc8(&self_json)
    }

    /// Validate the packet against its CRC.
    pub fn validate(&self, crc: u8) -> bool {
        let self_json = serde_json::to_string(self).unwrap();
        let new_crc = crc8(self_json.as_bytes());

        // If they aren't equal, the data is invalid!
        new_crc == crc
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct PowerInfo {
    /// The voltage of the main battery
    pub voltage: u16,
    /// Current being drawn by all components from the main battery
    pub current: u16,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct EnvironmentalInfo {
    /// Pressure of the inside of the payload
    pub pressure: f64,
    /// Temperature of the inside of the payload
    pub temperature: f64,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct GpsInfo {
    pub latitude: f64,
    pub longitude: f64,
    pub altitude: f32,
}
