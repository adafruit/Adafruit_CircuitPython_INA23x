# SPDX-FileCopyrightText: Copyright (c) 2025 Liz Clark for Adafruit Industries
#
# SPDX-License-Identifier: MIT

import time
import board
import adafruit_ina23x

# Create I2C bus
i2c = board.I2C()

# Create INA237/238 instance
ina23x = adafruit_ina23x.INA23X(i2c)

# Configure the sensor (optional - these are just examples)
# ina23x.set_calibration(shunt_res=0.1, max_current=3.2)  # Default values
# ina23x.mode = adafruit_ina23x.Mode.CONTINUOUS  # Already default
# ina23x.averaging_count = adafruit_ina23x.AveragingCount.COUNT_4

conv_times = [50, 84, 150, 280, 540, 1052, 2074, 4120]
avg_counts = [1, 4, 16, 64, 128, 256, 512, 1024]

print(f"Bus conversion time: {conv_times[ina23x.bus_voltage_conv_time]} microseconds")
print(f"Shunt conversion time: {conv_times[ina23x.shunt_voltage_conv_time]} microseconds")
print(f"Samples averaged: {avg_counts[ina23x.averaging_count]}")

while True:
    print("\nCurrent Measurements:")
    print(f"Current: {ina23x.current * 1000:.2f} mA")
    print(f"Bus Voltage: {ina23x.bus_voltage:.2f} V")
    print(f"Shunt Voltage: {ina23x.shunt_voltage * 1000:.2f} mV")
    print(f"Power: {ina23x.power * 1000:.2f} mW")
    print(f"Temperature: {ina23x.die_temperature:.2f} °C")

    # Check if conversion is ready (useful in triggered mode)
    # if ina23x.conversion_ready:
    #     print("Conversion ready!")

    time.sleep(1)
