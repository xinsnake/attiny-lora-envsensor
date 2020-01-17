# ATtiny-LoRa-EnvSensor

I am currently developing a ATTiny1614 + RFM95W + BME280 environment sensor
which will send data to The Things Network via an existing gateway. The new
ATTiny boards support hardware SPI which is what I am using here to communicate
between the uC and different modules.

## Hardware Toolchain

The hardware I selected:

- Self-designed ATTinyX14 Development Board (Schematics to be added)
- RFM95W with antenna
- BME280 3.3v with SPI interface (6-pin board)

Tools I am using for development:

- Atmel Studio 7
- Atmel-ICE / MPLAB® PICkit 4
- Kingst Logic Analyzer

## Modification

- For BME280 take out all the resistors on the surface to reduce power consumption
  by ~450uA. We are using SPI so no pull-up resistors are needed here.

## The Things Network

- I haven't done anything on OTAA, and currently keys are hard-coded.
- Copy `key.example.h` file and name it `key.h`, then replace the values there.

## Power consumption

I don't have a precise tool to measure low current, but based on my multimeter:

- Wake up time lasts for about 0.2s, not sure the total power consumption, but peak
  current is around 60mA.
- Sleep current is around 7uA.

## Schematics

![Schematic.png](/imgs/Schematic.png)
