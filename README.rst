Introduction
============


.. image:: https://readthedocs.org/projects/adafruit-circuitpython-ina23x/badge/?version=latest
    :target: https://docs.circuitpython.org/projects/ina23x/en/latest/
    :alt: Documentation Status


.. image:: https://raw.githubusercontent.com/adafruit/Adafruit_CircuitPython_Bundle/main/badges/adafruit_discord.svg
    :target: https://adafru.it/discord
    :alt: Discord


.. image:: https://github.com/adafruit/Adafruit_CircuitPython_INA23x/workflows/Build%20CI/badge.svg
    :target: https://github.com/adafruit/Adafruit_CircuitPython_INA23x/actions
    :alt: Build Status


.. image:: https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/astral-sh/ruff/main/assets/badge/v2.json
    :target: https://github.com/astral-sh/ruff
    :alt: Code Style: Ruff

CircuitPython driver for the INA237 and INA238 DC Current Voltage Power Monitor


Dependencies
=============
This driver depends on:

* `Adafruit CircuitPython <https://github.com/adafruit/circuitpython>`_
* `Bus Device <https://github.com/adafruit/Adafruit_CircuitPython_BusDevice>`_
* `Register <https://github.com/adafruit/Adafruit_CircuitPython_Register>`_
* `Adafruit CircuitPython INA228 <https://github.com/adafruit/Adafruit_CircuitPython_INA228>`_

Please ensure all dependencies are available on the CircuitPython filesystem.
This is easily achieved by downloading
`the Adafruit library and driver bundle <https://circuitpython.org/libraries>`_
or individual libraries can be installed using
`circup <https://github.com/adafruit/circup>`_.

`Purchase INA237 from the Adafruit shop <http://www.adafruit.com/products/6340>`_

`Purchase INA238 from the Adafruit shop <http://www.adafruit.com/products/6349>`_

Installing from PyPI
=====================

On supported GNU/Linux systems like the Raspberry Pi, you can install the driver locally `from
PyPI <https://pypi.org/project/adafruit-circuitpython-ina23x/>`_.
To install for current user:

.. code-block:: shell

    pip3 install adafruit-circuitpython-ina23x

To install system-wide (this may be required in some cases):

.. code-block:: shell

    sudo pip3 install adafruit-circuitpython-ina23x

To install in a virtual environment in your current project:

.. code-block:: shell

    mkdir project-name && cd project-name
    python3 -m venv .venv
    source .env/bin/activate
    pip3 install adafruit-circuitpython-ina23x

Installing to a Connected CircuitPython Device with Circup
==========================================================

Make sure that you have ``circup`` installed in your Python environment.
Install it with the following command if necessary:

.. code-block:: shell

    pip3 install circup

With ``circup`` installed and your CircuitPython device connected use the
following command to install:

.. code-block:: shell

    circup install adafruit_ina23x

Or the following command to update an existing version:

.. code-block:: shell

    circup update

Usage Example
=============

.. code-block:: python

    import time
    import board
    import adafruit_ina23x

    i2c = board.I2C()
    ina23x = adafruit_ina23x.INA23X(i2c)

    while True:
        print(f"Current: {ina23x.current * 1000:.2f} mA")
        print(f"Bus Voltage: {ina23x.bus_voltage:.2f} V")
        print(f"Shunt Voltage: {ina23x.shunt_voltage * 1000:.2f} mV")
        print(f"Power: {ina23x.power * 1000:.2f} mW")
        print(f"Temperature: {ina23x.die_temperature:.2f} °C")
        print()

        time.sleep(2)

Documentation
=============
API documentation for this library can be found on `Read the Docs <https://docs.circuitpython.org/projects/ina23x/en/latest/>`_.

For information on building library documentation, please check out
`this guide <https://learn.adafruit.com/creating-and-sharing-a-circuitpython-library/sharing-our-docs-on-readthedocs#sphinx-5-1>`_.

Contributing
============

Contributions are welcome! Please read our `Code of Conduct
<https://github.com/adafruit/Adafruit_CircuitPython_INA23x/blob/HEAD/CODE_OF_CONDUCT.md>`_
before contributing to help this project stay welcoming.
