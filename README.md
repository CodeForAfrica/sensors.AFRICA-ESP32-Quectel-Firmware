# About 

This firmware is developed for an environmental monitoring project measuring particulate matter, temperature and humidity using [Plantower](https://www.plantower.com/en/) and DHT sensors.

# Description

The firmware is built and tested for a board running on **ESP32-S3 N16R8** microcontroller and a **Quectel EC200U-CN GSM** module.

<img width="916" alt="PCB with ES32-S3 and Quectel modem" title="PCB with ES32-S3 and Quectel modem" src="https://github.com/user-attachments/assets/2cd0dc95-dc52-4207-b7c3-d389645c5e6b" />
<br> <figcaption>PCB with ES32-S3 and Quectel modem</figcaption>   


## Features
 - SD card logging.
 - RTC update via network carrier local time.
 - Automatic network band selection.
 - JSON and CSV payload generation.
 - Power saving mechanisms through sleep modes and extended sampling cycles.


# Development

## Firmware upload
PlatformIO IDE was used to develop this firmware. For users familiar with Visual Studio Code, install the PlatformIO extension. 
Open the project using PlatformIO and wait for PlatformIO to configure the environment, and install the necessary libraries. Select the port the device is connected to and then upload the firmware. Both the JTAG and UART inferfaces can be used to program the device.

## API integration
Open serial monitor to capture the device id which bears the format `<prefix>-<chip id>` e.g. `ESP32-19271G2328BE`. Navigate to [sensors.AFRICA API docs](https://api.sensors.africa/docs) to learn how to use the API and onboard the device via the admin dashboard (*Admin access will be required for this*).

## Debugging
Debugging is available via the USB-UART interface. To enable debugging, uncomment the debug options in platformio.ini file. On VS code, hit the debug icon and select the `PIO Debug` option.

## Troubleshooting

- If the device is not detected by your computer, install the necessary drivers for the JTAG or UART interface. E.g. CP2102 drivers
- Network registration issues:
  - Ensure you have an ACTIVE SIM card with NO PIN, and with loaded data bundles or airtime. 
  - Ensure there is cell service in your area for your preferred network provider. 
  - Restart the device ensuring the GSM module is rebooted.
  - Add an extra antenna via the ufl connector and expose it.
- Firmware upload issues: 
  - Ensure that the upload port is not being used by another service e.g serial monitor
  - If the JTAG port fails to upload, select the UART port instead.
  

