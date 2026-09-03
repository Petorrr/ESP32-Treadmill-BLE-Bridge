This repository is a BLE bridge for treadmills that expose speed, distance, elapsed time etc via Bluetooth using the FTMS messaging protocol.
The bridge connects to the first treadmill it finds and subscribes to the FTMS message, in order to receive the necessary information and then sets up a Garmin Running Speed&Cadence (RSC) sensor.
Typically the RSC information broadcasted will be exposed as an available Footpod sensor on a Garmin watch, allowing you to connect your watch for Speed, Distance and Time coming from the treadmill.
The cadence calculation switches automatically between a vibration sensor logic on GPIO3 with an internal pull-up, or if the sensor is not installed, to a simulated cadence routine based on an empirical evidence based calculation from the current speed.

The configuration is tested and running on an ESP32-C3 Supermini using PlatformIO, but can easily be reconfigured to any other ESP32-C3 or ESP32 in general.
The Blue LED is used as status indicator:
  - Quick flashing (0,2 Hertz): waiting for Treadmill and Watch
  - Long on (800 ms) - short off (200 ms) flashing (1 sec period): Watch connected, waiting for Treadmill
  - Short on (200 ms) - long off (800 ms) flashing (1 sec period): Treadmill connected, waiting for Watch
  - ON: both the Treadmill and Watch are connected. Ready to use during Treadmill runs on your Garmin watch.

