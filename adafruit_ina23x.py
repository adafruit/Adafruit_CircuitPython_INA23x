# SPDX-FileCopyrightText: Copyright (c) 2025 Liz Clark for Adafruit Industries
#
# SPDX-License-Identifier: MIT
"""
`adafruit_ina23x`
================================================================================

CircuitPython driver for the INA237 and INA238 DC Current Voltage Power Monitors


* Author(s): Liz Clark

Implementation Notes
--------------------

**Hardware:**

* `Adafruit INA237 Breakout <https://www.adafruit.com/product/6340>`_
* `Adafruit INA238 Breakout <https://www.adafruit.com/product/6349>`_

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://circuitpython.org/downloads

* Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
* Adafruit's Register library: https://github.com/adafruit/Adafruit_CircuitPython_Register
"""

import time

from adafruit_bus_device.i2c_device import I2CDevice
from adafruit_register.i2c_bit import ROBit, RWBit
from adafruit_register.i2c_bits import ROBits, RWBits
from adafruit_register.i2c_struct import ROUnaryStruct, UnaryStruct
from micropython import const

try:
    import typing  # pylint: disable=unused-import

    from busio import I2C
except ImportError:
    pass

__version__ = "0.0.0+auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_INA23x.git"

# Register addresses
_CONFIG = const(0x00)
_ADCCFG = const(0x01)
_SHUNTCAL = const(0x02)
_VSHUNT = const(0x04)
_VBUS = const(0x05)
_DIETEMP = const(0x06)
_CURRENT = const(0x07)
_POWER = const(0x08)
_DIAGALRT = const(0x0B)
_SOVL = const(0x0C)
_SUVL = const(0x0D)
_BOVL = const(0x0E)
_BUVL = const(0x0F)
_TEMPLIMIT = const(0x10)
_PWRLIMIT = const(0x11)
_MFG_UID = const(0x3E)
_DVC_UID = const(0x3F)

# Constants
_INA23X_DEFAULT_ADDR = const(0x40)
_INA237_DEVICE_ID = const(0x238)
_INA238_DEVICE_ID = const(0x238)  # Same as INA237
_TEXAS_INSTRUMENTS_ID = const(0x5449)


class Mode:
    """Operating mode constants for INA23X"""

    SHUTDOWN = const(0x00)
    TRIG_BUS = const(0x01)
    TRIG_SHUNT = const(0x02)
    TRIG_BUS_SHUNT = const(0x03)
    TRIG_TEMP = const(0x04)
    TRIG_TEMP_BUS = const(0x05)
    TRIG_TEMP_SHUNT = const(0x06)
    TRIG_TEMP_BUS_SHUNT = const(0x07)
    CONT_BUS = const(0x09)
    CONT_SHUNT = const(0x0A)
    CONT_BUS_SHUNT = const(0x0B)
    CONT_TEMP = const(0x0C)
    CONT_TEMP_BUS = const(0x0D)
    CONT_TEMP_SHUNT = const(0x0E)
    CONT_TEMP_BUS_SHUNT = const(0x0F)

    # Convenience aliases
    TRIGGERED = TRIG_TEMP_BUS_SHUNT
    CONTINUOUS = CONT_TEMP_BUS_SHUNT

    # Valid modes set for validation
    _VALID_MODES = {
        SHUTDOWN,
        TRIG_BUS,
        TRIG_SHUNT,
        TRIG_BUS_SHUNT,
        TRIG_TEMP,
        TRIG_TEMP_BUS,
        TRIG_TEMP_SHUNT,
        TRIG_TEMP_BUS_SHUNT,
        CONT_BUS,
        CONT_SHUNT,
        CONT_BUS_SHUNT,
        CONT_TEMP,
        CONT_TEMP_BUS,
        CONT_TEMP_SHUNT,
        CONT_TEMP_BUS_SHUNT,
    }


class ConversionTime:
    """Conversion time constants for INA23X"""

    TIME_50_US = const(0)
    TIME_84_US = const(1)
    TIME_150_US = const(2)
    TIME_280_US = const(3)
    TIME_540_US = const(4)
    TIME_1052_US = const(5)
    TIME_2074_US = const(6)
    TIME_4120_US = const(7)

    _VALID_TIMES = {
        TIME_50_US,
        TIME_84_US,
        TIME_150_US,
        TIME_280_US,
        TIME_540_US,
        TIME_1052_US,
        TIME_2074_US,
        TIME_4120_US,
    }


class AveragingCount:
    """Averaging count constants for INA23X"""

    COUNT_1 = const(0)
    COUNT_4 = const(1)
    COUNT_16 = const(2)
    COUNT_64 = const(3)
    COUNT_128 = const(4)
    COUNT_256 = const(5)
    COUNT_512 = const(6)
    COUNT_1024 = const(7)

    _VALID_COUNTS = {
        COUNT_1,
        COUNT_4,
        COUNT_16,
        COUNT_64,
        COUNT_128,
        COUNT_256,
        COUNT_512,
        COUNT_1024,
    }


class AlertType:
    """Alert type constants for INA23X"""

    NONE = const(0x0)
    CONVERSION_READY = const(0x1)
    OVERTEMPERATURE = const(0x2)
    OVERPOWER = const(0x4)
    UNDERVOLTAGE = const(0x8)
    OVERVOLTAGE = const(0x10)
    UNDERSHUNT = const(0x20)
    OVERSHUNT = const(0x40)

    _VALID_TYPES = {
        NONE,
        CONVERSION_READY,
        OVERTEMPERATURE,
        OVERPOWER,
        UNDERVOLTAGE,
        OVERVOLTAGE,
        UNDERSHUNT,
        OVERSHUNT,
    }


class INA23X:  # noqa: PLR0904
    """Driver for the INA237/INA238 current and power sensor.

    :param ~busio.I2C i2c_bus: The I2C bus the INA23X is connected to.
    :param int address: The I2C device address. Defaults to :const:`0x40`
    :param bool skip_reset: Skip resetting the device on init. Defaults to False.
    """

    # Configuration register bits
    _reset = RWBit(_CONFIG, 15, register_width=2, lsb_first=False)
    _adc_range = RWBit(_CONFIG, 4, register_width=2, lsb_first=False)

    # ADC Configuration register bits
    _mode = RWBits(4, _ADCCFG, 12, register_width=2, lsb_first=False)
    _vbus_conv_time = RWBits(3, _ADCCFG, 9, register_width=2, lsb_first=False)
    _vshunt_conv_time = RWBits(3, _ADCCFG, 6, register_width=2, lsb_first=False)
    _temp_conv_time = RWBits(3, _ADCCFG, 3, register_width=2, lsb_first=False)
    _avg_count = RWBits(3, _ADCCFG, 0, register_width=2, lsb_first=False)

    # Diagnostic/Alert register bits
    _alert_latch = RWBit(_DIAGALRT, 15, register_width=2, lsb_first=False)
    _alert_conv = RWBit(_DIAGALRT, 14, register_width=2, lsb_first=False)
    _alert_polarity = RWBit(_DIAGALRT, 12, register_width=2, lsb_first=False)
    _alert_type = RWBits(7, _DIAGALRT, 5, register_width=2, lsb_first=False)
    _conversion_ready = ROBit(_DIAGALRT, 1, register_width=2, lsb_first=False)
    _alert_flags = ROBits(12, _DIAGALRT, 0, register_width=2, lsb_first=False)

    # Measurement registers
    _raw_dietemp = ROUnaryStruct(_DIETEMP, ">h")
    _raw_vbus = ROUnaryStruct(_VBUS, ">H")
    _raw_vshunt = ROUnaryStruct(_VSHUNT, ">h")
    _raw_current = ROUnaryStruct(_CURRENT, ">h")
    _raw_power = ROUnaryStruct(_POWER, ">H")

    # Calibration register
    _shunt_cal = UnaryStruct(_SHUNTCAL, ">H")

    # ID registers
    _manufacturer_id = ROUnaryStruct(_MFG_UID, ">H")
    _device_id = ROUnaryStruct(_DVC_UID, ">H")

    def __init__(
        self, i2c_bus: I2C, address: int = _INA23X_DEFAULT_ADDR, skip_reset: bool = False
    ) -> None:
        self.i2c_device = I2CDevice(i2c_bus, address)

        # Verify manufacturer ID
        if self._manufacturer_id != _TEXAS_INSTRUMENTS_ID:
            raise ValueError("Failed to find INA237/INA238 - incorrect manufacturer ID")
        # Verify device ID (both INA237 and INA238 use the same ID)
        if self.device_id not in {_INA237_DEVICE_ID, _INA238_DEVICE_ID}:
            raise ValueError("Failed to find INA237/INA238 - incorrect device ID")

        self._shunt_res = 0.1  # Default shunt resistance
        self._current_lsb = 0.0
        if not skip_reset:
            self.reset()
            time.sleep(0.002)  # 2ms delay for first measurement
        self.set_calibration(0.015, 10.0)
        self.averaging_count = AveragingCount.COUNT_16
        self.bus_voltage_conv_time = ConversionTime.TIME_150_US
        self.shunt_voltage_conv_time = ConversionTime.TIME_280_US

    def reset(self) -> None:
        """Reset the sensor to default configuration."""
        self._reset = True
        self._alert_conv = True
        self.mode = Mode.CONTINUOUS

    @property
    def device_id(self) -> int:
        """Device ID"""
        return (self._device_id >> 4) & 0xFFF

    @property
    def shunt_resistance(self) -> float:
        """The shunt resistance in ohms."""
        return self._shunt_res

    def set_calibration(self, shunt_res: float = 0.015, max_current: float = 10.0) -> None:
        """Set the calibration based on shunt resistance and maximum expected current.

        :param float shunt_res: Shunt resistance in ohms
        :param float max_current: Maximum expected current in amperes
        """
        self._shunt_res = shunt_res
        # INA237/238 uses 2^15 as divisor
        self._current_lsb = max_current / (1 << 15)
        self._update_shunt_cal()

    def _update_shunt_cal(self) -> None:
        """Update the shunt calibration register."""
        # Scale factor based on ADC range
        scale = 4 if self._adc_range else 1

        # INA237/238 formula: SHUNT_CAL = 819.2 × 10^6 × CURRENT_LSB × RSHUNT × scale
        shunt_cal = int(819.2e6 * self._current_lsb * self._shunt_res * scale)
        self._shunt_cal = min(shunt_cal, 0xFFFF)

    @property
    def adc_range(self) -> int:
        """ADC range setting. 0 = ±163.84mV, 1 = ±40.96mV"""
        return self._adc_range

    @adc_range.setter
    def adc_range(self, value: int) -> None:
        if value not in {0, 1}:
            raise ValueError("ADC range must be 0 or 1")
        self._adc_range = value
        self._update_shunt_cal()

    @property
    def mode(self) -> int:
        """Operating mode of the sensor."""
        return self._mode

    @mode.setter
    def mode(self, value: int) -> None:
        if value not in Mode._VALID_MODES:
            raise ValueError(f"Invalid mode 0x{value:02X}. Must be one of the Mode.* constants")
        self._mode = value

    @property
    def averaging_count(self) -> int:
        """Number of samples to average."""
        return self._avg_count

    @averaging_count.setter
    def averaging_count(self, value: int) -> None:
        if value not in AveragingCount._VALID_COUNTS:
            raise ValueError(
                f"Invalid averaging count {value}. Must be one of the AveragingCount.* constants"
            )
        self._avg_count = value

    @property
    def bus_voltage_conv_time(self) -> int:
        """Bus voltage conversion time setting."""
        return self._vbus_conv_time

    @bus_voltage_conv_time.setter
    def bus_voltage_conv_time(self, value: int) -> None:
        if value not in ConversionTime._VALID_TIMES:
            raise ValueError(
                f"Invalid conversion time {value}. Must be one of the ConversionTime.* constants"
            )
        self._vbus_conv_time = value

    @property
    def shunt_voltage_conv_time(self) -> int:
        """Shunt voltage conversion time setting."""
        return self._vshunt_conv_time

    @shunt_voltage_conv_time.setter
    def shunt_voltage_conv_time(self, value: int) -> None:
        if value not in ConversionTime._VALID_TIMES:
            raise ValueError(
                f"Invalid conversion time {value}. Must be one of the ConversionTime.* constants"
            )
        self._vshunt_conv_time = value

    @property
    def temp_conv_time(self) -> int:
        """Temperature conversion time setting."""
        return self._temp_conv_time

    @temp_conv_time.setter
    def temp_conv_time(self, value: int) -> None:
        if value not in ConversionTime._VALID_TIMES:
            raise ValueError(
                f"Invalid conversion time {value}. Must be one of the ConversionTime.* constants"
            )
        self._temp_conv_time = value

    @property
    def die_temperature(self) -> float:
        """Die temperature in degrees Celsius."""
        # INA237/238 uses 12 bits (15:4) with 125 m°C/LSB
        return (self._raw_dietemp >> 4) * 0.125

    @property
    def bus_voltage(self) -> float:
        """Bus voltage in volts."""
        # INA237/238 uses 3.125 mV/LSB
        return self._raw_vbus * 0.003125

    @property
    def shunt_voltage(self) -> float:
        """Shunt voltage in volts."""
        # Scale depends on ADC range
        scale = 1.25e-6 if self._adc_range else 5.0e-6  # µV/LSB
        return self._raw_vshunt * scale

    @property
    def current(self) -> float:
        """Current in amperes."""
        return self._raw_current * self._current_lsb

    @property
    def power(self) -> float:
        """Power in watts."""
        # INA237/238 power LSB = 20 × current_lsb
        return self._raw_power * 20.0 * self._current_lsb

    @property
    def conversion_ready(self) -> bool:
        """Check if conversion is complete."""
        return bool(self._conversion_ready)

    @property
    def alert_type(self) -> int:
        """Alert type configuration."""
        return self._alert_type

    @alert_type.setter
    def alert_type(self, value: int) -> None:
        # Alert type can be a combination of flags, so we check if all bits are valid
        valid_mask = (
            AlertType.CONVERSION_READY
            | AlertType.OVERTEMPERATURE
            | AlertType.OVERPOWER
            | AlertType.UNDERVOLTAGE
            | AlertType.OVERVOLTAGE
            | AlertType.UNDERSHUNT
            | AlertType.OVERSHUNT
        )
        if value & ~valid_mask:
            raise ValueError(
                f"Invalid alert type 0x{value:02X}. Must be a combination of AlertType.* constants"
            )
        self._alert_type = value

    @property
    def alert_polarity(self) -> int:
        """Alert pin polarity. 0 = active high, 1 = active low."""
        return self._alert_polarity

    @alert_polarity.setter
    def alert_polarity(self, value: int) -> None:
        if value not in {0, 1}:
            raise ValueError("Alert polarity must be 0 or 1")
        self._alert_polarity = value

    @property
    def alert_latch(self) -> int:
        """Alert latch enable. 0 = transparent, 1 = latched."""
        return self._alert_latch

    @alert_latch.setter
    def alert_latch(self, value: int) -> None:
        if value not in {0, 1}:
            raise ValueError("Alert latch must be 0 or 1")
        self._alert_latch = value

    @property
    def alert_flags(self) -> int:
        """Current alert flags."""
        return self._alert_flags
