.. _arduino_zero:

Arduino/Genuino Zero
####################

Overview
********

The Arduino Zero is a maker-friendly development board with
Atmel’s Embedded Debugger (EDBG), which provides a full
debug interface without the need for additional hardware.

.. image:: img/arduino_zero.png
     :width: 500px
     :align: center
     :alt: Arduino Zero

Hardware
********

- ATSAMD21G18A ARM Cortex-M0+ processor at 48 MHz
- 32.768 kHz crystal oscillator
- 256 KiB flash memory and 32 KiB of RAM
- 3 user LEDs
- One reset button
- On-board USB based EDBG unit with serial console
- Native USB port

Supported Features
==================

The arduino_zero board configuration supports the following hardware
features:

+-----------+------------+-------------------------------------+
| Interface | Controller | Driver/Component                    |
+===========+============+=====================================+
| NVIC      | on-chip    | nested vector interrupt controller  |
+-----------+------------+-------------------------------------+
| SYSTICK   | on-chip    | systick                             |
+-----------+------------+-------------------------------------+
| WDT       | on-chip    | Watchdog                            |
+-----------+------------+-------------------------------------+
| GPIO      | on-chip    | I/O ports                           |
+-----------+------------+-------------------------------------+
| USART     | on-chip    | Serial ports                        |
+-----------+------------+-------------------------------------+

Other hardware features are not currently supported by Zephyr.

The default configuration can be found in the Kconfig
:file:`boards/arm/arduino_zero/arduino_zero_defconfig`.

Connections and IOs
===================

The `Arduino store`_ has detailed information about board
connections. Download the `Arduino Zero Schematic`_ for more detail.

System Clock
============

The SAMD21 MCU is configured to use the 32.768 kHz external oscillator
with the on-chip PLL generating the 48 MHz system clock.  The internal
APB and GCLK unit are set up in the same way as the upstream Arduino
libraries.

Serial Port
===========

The SAMD21 MCU has 5 SERCOM based USARTs. One of the USARTs
(SERCOM5) is connected to the onboard Atmel Embedded Debugger (EDBG).
SERCOM0 is available on the D0/D1 pins.

Programming and Debugging
*************************

The Arduino Zero comes with a Atmel Embedded Debugger (EDBG).  This
provices a debug interface to the SAMD21 chip and is supported by
OpenOCD.

Flashing
========

#. Build the Zephyr kernel and the :ref:`hello_world` sample application:

   .. code-block:: console

      $ cd $ZEPHYR_BASE/samples/hello_world/
      $ make BOARD=arduino_zero

#. Connect the Arduino Zero to your host computer using the USB debug
   port.

#. Run your favorite terminal program to listen for output. Under Linux the
   terminal should be :code:`/dev/ttyACM0`. For example:

   .. code-block:: console

      $ minicom -D /dev/ttyACM0 -o

   The -o option tells minicom not to send the modem initialization
   string. Connection should be configured as follows:

   - Speed: 115200
   - Data: 8 bits
   - Parity: None
   - Stop bits: 1

#. To flash an image:

   .. code-block:: console

      $ make BOARD=arduino_zero flash

   You should see "Hello World! arm" in your terminal.

References
**********

.. target-notes::

.. _Arduino Store:
    https://store.arduino.cc/genuino-zero

.. _Arduino Zero Schematic:
    https://www.arduino.cc/en/uploads/Main/Zero_V1.0.pdf
