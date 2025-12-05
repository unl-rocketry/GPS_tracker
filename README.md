# GPS_Tracker
Custom Prototype GPS tracker built to test the Aerospace club Rocketry team's Rotator which was then used to receive live video from the rocket at comp with the full payload.

This GPS tracker uses a esp32 connected to an RPF 900 to send down data which is gathered by a Adafruit GPS and a MCP9808 temprature sensor.

The packet format specified in [/src/lib.rs](https://github.com/unl-rocketry/GPS_tracker/blob/main/src/lib.rs) is also shared with [arowss](https://github.com/unl-rocketry/arowss) and [archer-gui](https://github.com/unl-rocketry/archer-gui) from the 2024-2025 year.
