.. _development_note:

================
Developer's Note
================

This page explains the design considerations and implementation details of the
NTURT Rear Box.

The C Programming Language
==========================

Built-in Functions
------------------

- :c:func:`__builtin_ffs`
- :c:func:`__builtin_clz`

`GCC built-in functions<https://gcc.gnu.org/onlinedocs/gcc/Other-Builtins.html>_`

Zephyr 
======

Dynamic Memory Allocation
-------------------------

Zephyr provides both ``libc`` :c:func:`malloc` and its own :c:func:`k_melloc`
functions for dynamic memory allocation. It replaces the common ``libc``
implementations with its own one as described in `Zephyr's documentation
<https://docs.zephyrproject.org/3.6.0/develop/languages/c/index.html#dynamic-memory-management>`_.

However, :c:func:`malloc` does not share the same heap with :c:func:`k_melloc`
[#]_, namely enabling one does not necessarily enable the other. :c:func:`malloc`
can be enabled by setting the Kconfig option
`CONFIG_COMMON_LIBC_MALLOC_ARENA_SIZE
<https://docs.zephyrproject.org/3.6.0/kconfig.html#CONFIG_COMMON_LIBC_MALLOC_ARENA_SIZE>`_
to a non-zero value. By the same token, :c:func:`k_melloc` can be enabled by
setting the Kconfig option `CONFIG_HEAP_MEM_POOL_SIZE
<https://docs.zephyrproject.org/3.6.0/kconfig.html#CONFIG_HEAP_MEM_POOL_SIZE>`_
to a non-zero value.

Reference
~~~~~~~~~

.. [#] `Zephyr malloc.c source code
  <https://github.com/zephyrproject-rtos/zephyr/blob/v3.6-branch/lib/libc/common/source/stdlib/malloc.c>`_

Zephyr Drivers
==============

Device Tree
-----------

Using device tree to describe hardware configurations across different platforms
is a great design in my opinion, especially for microcontrollers where there are
lots of vendors with their unique features for similar peripherals. However,
just like Zephyr's disadvantages, the documentation and examples are few,
especially for vendor specific configurations. Here are some tips for using
device tree in Zephyr:

Device tree includes
~~~~~~~~~~~~~~~~~~~~

Decive tree supports including other device tree files, but without a language
support service, it is hard to find where the included files are. The following
are some common places where these files may be:

- Board support package (BSPs) files in ``zephyr/boards``
- Device tree header files (.dtsi) in ``zephyr/dts``
- Vendor specific device tree files (especially for ping control) in
  ``modules/hal/<vendor>/dts``
- Device tree bindings for C macros in ``zephyr/include/zephyr/dt-bindings``

Device tree macros
~~~~~~~~~~~~~~~~~~

As device tree in Zephyr is processed into C macros for the compiler to further
process, it supports including C hearder files as well as using macros defined
in it.

For example, you may see ``clock-frequency = <DT_FREQ_K(48)>;``, in clock
configs where ``DT_FREQ_K`` is a macro defined as ``DT_FREQ_M(x) ((x) * 1000)``
in ``zephyr/dts/common/freq.h``.

STM32 device tree
~~~~~~~~~~~~~~~~~~

For STM32, it is a good idea to use STM32CubeMX as a reference when configuring
the device tree as most of the time is just copying the configurations from
STM32CubeMX to the device tree.

However, for peripherals that support domain clocks, clock source macros
``STM32_SRC_*`` and clock selection macros ``*_SEL(X)`` are used to determine
the clock source for the domain.

For example, to configure fdcan1 for stm32g474re to use PLLQ as the clock
source, the following code snippet is used:

.. code-block:: dts

  &fdcan1 {
    clocks = <&rcc STM32_CLOCK_BUS_APB1 0x02000000>,
        <&rcc STM32_SRC_PLL_Q FDCAN_SEL(1)>;
  };

``STM32_SRC_*`` is easy to determine, but ``*_SEL(X)`` is not. To determine it,
you have to refer to the clock configuration register (CCIPR) of the reset and
clock control (RCC) section in the reference manual, where the value of ``X`` is
listed in the table.

.. note::

  The default configurations in ``stm32*.dts`` may only define bus clock
  source, but you still have to copy it to your own device tree and add the
  domain clock of your choice.

Direct Memory Access (DMA)
--------------------------

Direct memory access (DMA) sees very limited support in Zephyr, especially in
documentation and samples. Currently only UART and SPI drivers has wide support
for DMA, throuth `UART async API
<https://docs.zephyrproject.org/3.6.0/reference/peripherals/uart.html#uart-async-api>`_
and SPI vendor specific Kconfig options `CONFIG_SPI_*_DMA
<https://docs.zephyrproject.org/3.6.0/kconfig.html#!CONFIG_SPI_.*DMA>`_ ("wider"
support of eight vendors), and limited two vendors support for I\ :sup:`2`\ C
through `CONFIG_I2C_*_DMA
<https://docs.zephyrproject.org/3.6.0/kconfig.html#!CONFIG_I2C_.*DMA>`_.

Since unlike UART has native API support for DMA, SPI and I\ :sup:`2`\ C drivers
may have some creative ways to utilize DMA. For example, for STM32 SPI, DMA is
used in sychronous API :c:func:`spi_transceive` (but not in async ones) to
context switch out current thread and let DMA handle the data transfer [#]_.

To use DMA in STM32, please refer to DMA sections in the reference manual to
check which DMA channels are available for each peripherals. For later newer
STM32 series that has DMA mux, please refer to the DMAMUX sections such as
table 91 for STM32G4 series.

The use of DMA is advised, but care must be taken to ensure the espected
behaviors.

Reference
~~~~~~~~~

.. [#] `Zephyr STM32 SPI driver source code
  <https://github.com/zephyrproject-rtos/zephyr/blob/v3.6-branch/drivers/spi/spi_ll_stm32.c#L1080>`_
  that uses DMA in synchronous API

General Purpose Input/Output (GPIO)
-----------------------------------

Zephyr provides basic GPIO driver using the `GPIO API
<https://docs.zephyrproject.org/3.6.0/hardware/peripherals/gpio.html>`_ that can
perform basic operations such as digital read, write, and interrupt. However,
for more advanced features such as LED effects and button debouncing, you have
to rely on more advanced drivers and subsystems. Below are two drivers and
subsystems that just do that:

Light Emitting Diode (LED)
~~~~~~~~~~~~~~~~~~~~~~~~~~

Zephyr provides special `LED API
<https://docs.zephyrproject.org/3.6.0/hardware/peripherals/led.html>`_ that
controls various kinds of LEDs such as RGB LEDs and LED strips. Through
``gpio-leds`` device binding, you can control LEDs connected to GPIOs using the
LED API.

.. note::

  Since there may be multiple LEDs defined under one ``gpio-leds`` device, the
  LED API requires ``LED number`` to specify which LED to control. And the ``LED
  number`` is the order of the LED defined in the device tree.

Input
~~~~~

Zephyr provides special input subsystem designed for various kinds of inputs
such as key triggers, movement, etc through `Input API
<https://docs.zephyrproject.org/3.6.0/services/input/index.html>`_. It can also
be used for debouncing buttons through ``gpio-keys`` device binding. However,
currently it only supports asynchronous callbacks and sychronous queues on all
available input devices.

.. note::

  Every children of ``gpio-keys`` devices must have a unique ``zephyr,code``
  property to identify the key. Available options start from `INPUT_KEY_RESERVED
  <https://docs.zephyrproject.org/3.6.0/services/input/index.html#c.INPUT_KEY_RESERVED>`_.

Universal Asynchronous Receiver-Transmitter (UART)
--------------------------------------------------

STM32 UART provides hardware flow control for both RS232 and RS485 transceivers
(using ``CTS``, ``RTS`` pins for RS232 and ``DE`` pins for RS485). Since the
activation / deactivation time of the transceiver takes time, STM32 UART driver
provides a feature to delay the transmission of the first bit after toggling the
pins. For RS458 transceiver ``MAX487E`` that we used, it takes up to 3000ns to
finish the transaction [#]_. So for a baud rate of 115200, it will take 0.35 bit
time. With over sampling of 16 times per bit, it's 5.5 or minimum 6 sample time,
which cooresponds to ``de-assert-time`` and ``de-deassert-time`` in the device
tree.

Reference
~~~~~~~~~

.. [#] MAX487E Datasheet, Switching Characteristics, Driver Disable Time from
  Low

Battery Backed RAM (BBRAM)
--------------------------

Zephyr provides a battery backed RAM (BBRAM) driver that allows you to store
data across system resets through `BBRAM API
<https://docs.zephyrproject.org/3.6.0/hardware/peripherals/bbram.html>`_.
Depending on the hardware, the data may be persisted even if the main power is
lost, being kept by the dedicated battery, hence the name.

However, not all STM32 serise device tree include ``st,stm32-bbram`` device that
corrsepond to BBRAM. To use it, add it to ``st,stm32-rtc`` device in the device
tree overlay like so:

.. code-block:: dts

  &rtc {
    bbram: backup_regs {
        compatible = "st,stm32-bbram";
        st,backup-regs = <32>;
        status = "okay";
    };
  };

Where ``st,backup-regs`` is the number of backup register of the STM32 and
the exact values should refer to the reference manuals.

CAN Bus
-------

The driver for controller area network (CAN) driver provides a nice feature of
figuring out the sync jump width and other parameters for the bus automatically,
you only need to provide the baud rate and the sampling point.

Weirdly, maximum baud rate for CAN bus is set to 800kbps in Zephyr [#]_.

Reference
~~~~~~~~~

.. [#] `Zephyr CAN driver source code
  <https://github.com/zephyrproject-rtos/zephyr/blob/v3.6-branch/include/zephyr/drivers/can/can_mcan.h#L1318>`_
  that limits the maximum baud rate to 800kbps

Zephyr Services
===============

Logging
-------
Zephyr provides a comprehensive logging system that provides various logging
backends (i.e., where these log messages will be stored) such as console or file
system.

File system backend enabled by Kconfig option `CONFIG_LOG_BACKEND_FS
<https://docs.zephyrproject.org/3.6.0/kconfig.html#CONFIG_LOG_BACKEND_FS>`_ is
particularly handy since it will only log message only after the file system is
mounted.

LittleFS
--------

LittleFS support both non-volatile memory (NVM) such as internal flash or
external SPI flash and block device such as SD cards or USB drives. However,
since there is little to no example for the latter, some quirks are worth noting
here, and a SD card block device is used here as an example.

Though LittleFS provides `device tree bindings
<https://docs.zephyrproject.org/3.6.0/build/dts/api/bindings/fs/zephyr%2Cfstab%2Clittlefs.html#dtbinding-zephyr-fstab-littlefs>`_
for configuring the file system, it is mainly designed for NVM. For block
devices, LittleFS will determine the block size when mounting the device, and
set other parameters such as the read and program size the same as the block
size and lookhead size four times the block size [#]_. Since it does not know
how big the block size will be, it simply uses :c:func:`melloc` to allocate the
read, program, and lookhead buffers for the block device [#]_, so be sure to
enable :c:func:`melloc` and set the heap size to six times the size of the block
size.

Additionally, LittleFS uses :c:func:`k_heap_alloc` for allocating file caches
[#]_ using a memory pool controlled by Kconfig option
`CONFIG_FS_LITTLEFS_CACHE_SIZE
<https://docs.zephyrproject.org/latest/kconfig.html#CONFIG_FS_LITTLEFS_CACHE_SIZE>`_,
so also make sure to set it to values greater than block size.

Since the automount feature is not available for block devices, they must be
mounted manually. The following code snippet shows how to do so:

.. code-block:: c

  static struct fs_littlefs lfsfs;
  static struct fs_mount_t mp = {
      .type = FS_LITTLEFS,
      .fs_data = &lfsfs,
      .flags = FS_MOUNT_FLAG_USE_DISK_ACCESS,
      .storage_dev = CONFIG_SDMMC_VOLUME_NAME,
      .mnt_point = "/" CONFIG_SDMMC_VOLUME_NAME ":",
  };

  fs_mount(&mp);

Reference
~~~~~~~~~

.. [#] `LittleFS littlefs_init_cfg() source code
  <https://github.com/zephyrproject-rtos/zephyr/blob/v3.6.0/subsys/fs/littlefs_fs.c#L822>`_
  that initializes read, program, and lookhead buffer sizes
.. [#] `LittleFS lfs_init() source code
  <https://github.com/zephyrproject-rtos/littlefs/blob/zephyr/lfs.c#L4114>`_
  that allocate read, program, and lookhead buffer
.. [#] `LittleFS littlefs_open() source code
  <https://github.com/zephyrproject-rtos/zephyr/blob/v3.6.0/subsys/fs/littlefs_fs.c#L302>`_
  that allocate file cache

CAN Open Node
-------------

The `CANopenNode track in Zephy
<https://github.com/zephyrproject-rtos/canopennode>`_ is currently outdated to
the latest version of `CANopenNode
<https://github.com/CANopenNode/CANopenNode>`_, and the `CANopenEditor
<https://github.com/CANopenNode/CANopenEditor>`_ does not support this legacy
version of CANopenNode (it has a legacy exporter, but the generated code lacks
some type definitions that it can't compile).

But CAN open is not that really open as a lot of the specifications are not
free. For now I need CiA 302.

However, since the current version of CANopenNode in Zephyr is still operational
and provides the necessary features, it's too good to not use it. Here are some
notes for using CANopenNode in Zephyr:

CAN Reception
~~~~~~~~~~~~~

callbacks
^^^^^^^^^

CAN bus driver in Zephyr uses hardware filters to filter out messages, and only
messages that pass the filter will be received by the application using callback
functions from an interrupt context [#]_. Since callbacks are called from ISR,
caution must be taken when setting callbacks related to CAN bus reception in
CANopenNode. For example, :c:func:`CO_EM_initCallbackRx` and
:c:func:`CO_NMT_initCallback` both execute in the ISR context, please
investigate the source code to see what context the callback is executed.

Filters
^^^^^^^

Filters are added using :c:func:`CO_CANrxBufferInit` defined in ``CO_driver.c``.
The following is a list of filters added by CANopenNode:

- :c:func:`CO_EM_init` in ``CO_Emergency.c``
- :c:func:`CO_HBcons_monitoredNodeConfig` in ``CO_HBconsumer.c``, one for each
  monitored node
- :c:func:`CO_LSSmaster_init` in ``CO_LSSmaster.c`` *number not investigated*
- :c:func:`CO_LSSslave_init` in ``CO_LSSslave.c`` *number not investigated*
- :c:func:`CO_NMT_init` in ``CO_NMT_Heartbeat.c``
- :c:func:`CO_RPDO_init` in ``CO_PDO.c``, called once for each RPDO by
  :c:func:`CO_CANopenInitPDO` in ``CANopen.c``
- :c:func:`CO_SDO_init` in ``CO_SDO.c``
- :c:func:`CO_SDOclient_setup` in ``CO_SDOmaster.c``, one for each SDO client
- :c:func:`CO_SYNC_init` in ``CO_SYNC.c``
- :c:func:`CO_TIME_init` in ``CO_TIME.c``

.. note::

  Since STM32 only supports up to 28 standard ID filters, caution must be taken
  when configuring CANopenNode.

Service Data Object (SDO)
~~~~~~~~~~~~~~~~~~~~~~~~~

Each object dictionary (OD) entry can add additional functionalities by
registering a callback function using :c:func:`CO_OD_configure`. And,
CANopenNode already registered some common OD entries to provide functionalities
according to the CiA 301 standard. The following is a list of registered ODs:

- 0x1003: Pre-defined error field
- 0x1005: COB-ID SYNC message
- 0x1006: Communication cycle period
- 0x1010: Store parameters
- 0x1011: Restore default parameters
- 0x1014: COB-ID EMCY
- 0x1016: Consumer heartbeat time
- 0x1019: Synchronous counter overflow value
- 0x1200: SDO server parameter
- 0x1400 to 0x15FF: RPDO communication parameter
- 0x1600 to 0x17FF: RPDO mapping parameter
- 0x1800 to 0x19FF TPDO communication parameter
- 0x1A00 to 0x1BFF TPDO mapping parameter

Error Handling
~~~~~~~~~~~~~~

Error status bits
^^^^^^^^^^^^^^^^^

CANopenNode uses an optional OD entry ``Error status bits`` of type
``OCTET_STRING`` and length more than 12 to store error status. You are
responsible for setting it in OD and register it to CANopenNode using
:c:func:`CO_EM_init`. The first 6 bytes (and hence the minimum length of 12 of
the octet string) is used internally by CANopenNode to store error status, and
the rest 26 bytes can be used for manaufacturer specific errors. The definitions
of the error status bits can be found in `CO_EM_errorStatusBits_t
<https://canopennode.github.io/CANopenSocket/group__CO__Emergency.html#ga587034df9d350c8e121c253f1d4eeacc>`_.

.. note::

  The length of ``Error status bits`` must grow coorespondingly to the number of
  manufacturer specific errors.

Error register
^^^^^^^^^^^^^^

CANopenNode also helps mamnge generic, communication and manufacturer-specific
bits of ``Error register`` at OD 0x1001 [#]_. It sets communication bits when
internal communication error occurs, and manufacturer-specific bits when any of
the manaufacturer specific errors in ``Error status bits`` are set. However, it
only set generic bit when ``CO_EM_errorStatusBits_t`` between 0x28 to 0x2F are
set, which does **NOT** adhere to the standard stating that: "The generic error
shall be signaled at any error situation [#]_."

EMCY write
^^^^^^^^^^

In CANopen standard the EMCY write payload has the following format [#]_:

.. code-block:: none

    0        1          2         3                              7
  +------------+----------------+----------------------------------+
  | error code | error register | manufacturer-specific error code |
  +------------+----------------+----------------------------------+

CANopenNode uses the first byte of manaufacturer-specific error code to transmit
its ``Error status bits``, so the payload becomes:

.. code-block:: none

    0        1         2                  3           4                              7
  +------------+----------------+------------------------------------------------------+
  | error code | error register | error status bits | manufacturer-specific error code |
  +------------+----------------+------------------------------------------------------+

CANopenNode also recognizes the first byte of manaufacturer-specific error code
as ``Error status bits`` when receiving EMCY messages from other nodes. The
callback for receiving EMCY registered using :c:func:`CO_EM_initCallbackRx` has
the prototype:

.. code-block:: c

  void pFunctSignalRx(const uint16_t ident,
                      const uint16_t errorCode,
                      const uint8_t errorRegister,
                      const uint8_t errorBit,
                      const uint32_t infoCode);

Where ``errorBit`` is for ``Error status bits`` (and ``infoCode`` for the rest
of the manufacturer-specific error code).

Pre-defined error fields
^^^^^^^^^^^^^^^^^^^^^^^^

CANopenNode also helps to maintain ``Pre-defined error fields`` at OD 0x1003 for
recording errors that happened [#]_. Once an error is reported using
:c:func:`CO_errorReport`, it will be recorded to ``Pre-defined error fields`` in
the following format:

.. code-block:: none

  32     24               16           0
  +------+----------------+------------+
  | 0x00 | error register | error code |
  +------+----------------+------------+
  MSB                                LSB

where error code is one of the standard error codes defined in CiA 301.

EMCY reception
^^^^^^^^^^^^^^

CANopenNode will receive all EMCY messages from the bus [#]_ and call the
callback registered using :c:func:`CO_EM_initCallbackRx`. It does not provide
support for ``Emergency consumer object`` at OD 0x1014.

Reference
~~~~~~~~~

.. [#] `Zephyr CAN bus driver documentation
  <https://docs.zephyrproject.org/3.6.0/hardware/peripherals/can/controller.html#receiving>`_
  on receiving messages
.. [#] `CANopenNode CO_EM_process() source code (1)
  <https://github.com/zephyrproject-rtos/canopennode/blob/zephyr/stack/CO_Emergency.c#L251>`_ 
  that manages error register
.. [#] CiA 301, section 7.5.2.2 Error register
.. [#] CiA 301, section 7.2.7.3.1 Protocol EMCY write
.. [#] `CANopenNode CO_EM_process() source code (2)
  <https://github.com/zephyrproject-rtos/canopennode/blob/zephyr/stack/CO_Emergency.c#L310>`_
  that mantains pre-defined error fields
.. [#] `CANopenNode CO_EM_init() source code
  <https://github.com/zephyrproject-rtos/canopennode/blob/zephyr/stack/CO_Emergency.c#L179>`_
  that receives all EMCY messages
