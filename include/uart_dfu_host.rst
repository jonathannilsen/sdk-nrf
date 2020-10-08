.. _lib_uart_dfu_host:

UART DFU host
#############

The UART DFU host lets the device *receive* device firmware upgrades over UART.
It exists as a layer between :ref:`lib_uart_blob_rx` and :ref:`lib_dfu_target`.
See (dfutargetuart) for how to *send* device firmware upgrades over UART.

The UART DFU host is initialized with :cpp:func:`uart_dfu_host_init`.
DFU over UART is disabled by default and must be disabled with :cpp:func:`uart_dfu_host_enable`.
The UART DFU host can be disabled again with :cpp:func:`uart_dfu_host_disable`. 


UART DFU image generation
*************************

When :option:`CONFIG_UART_DFU_HOST` and :option:`CONFIG_MCUBOOT` are enabled, the build system will automatically generate an update binary that can be used to perform an MCUboot style upgrade over UART.
This update binary is simply a regular MCUboot update image that is prepended with a special byte sequence.
The update binary is named ``app_update_uart.bin`` and is located in the ``zephyr`` subfolder of the build directory.


API documentation
*****************

| Header file: :file:`include/uart_dfu_host.h`
| Source files: :file:`lib/uart_dfu_host`

.. doxygengroup:: uart_dfu_host 
   :project: nrf
   :members:
