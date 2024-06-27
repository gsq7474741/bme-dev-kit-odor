# Odor Data Collector using BME688 Development Kit

This is a project to collect odor data using the BME688 Development Kit. The BME688 is a high-accuracy, low-power gas, pressure, and temperature sensor that can be easily integrated into any microcontroller or system. The data collected from the sensor will be used to train a machine learning model to predict the odor of a given air sample.

## Getting Started

To get started, follow the steps below:

1. Purchase the BME688 Development Kit from Adafruit.
2. Install PlatformIO on your computer.
3. Clone this repository to your computer.
4. Open the project in PlatformIO.


## Running the Code

To run the code, follow the steps below:

1. Plug in the SD card and CR1220 coin cell battery to the BME688 board.
2. Connect the BME688 board to the Adafruit Feather ESP32 board. 
3. Plug the USB cable into your Adafruit board.
4. Compile and upload the code to your microcontroller using PlatformIO.
5. Open the Serial Monitor to view the logs.
6. After collecting enough data, take the SD card out of the BME688 board.
7. Open the file in the SD card using BEM AI Studio on your computer.

## Data Collection

The data collected from the BME688 includes the following:

- Temperature (degrees Celsius)
- Humidity (%)
- Pressure (hPa)
- Gas Resistance (Ohms)

## Machine Learning Model

WIP