.. _lib_uart_blob:

UART Binary Large OBject (BLOB) transfer
########################################

The UART BLOB transfer library implements a protocol for transferring Binary
Large OBjects (BLOBs), i.e. arbitrary binary data over UART.
The library is split into two submodules, the sender ``uart_blob_tx``
and receiver ``uart_blob_rx`` which can be enabled and configured
independently.


UART BLOB protocol
******************

The library uses a protocol that was designed to be easily compatible with the
API of :ref:`lib_dfu_target`. While this protocol is largely hidden behind the
UART BLOB sender/receiver API, it may be useful to understand it to help
understand the APIs.

The protocol uses a packet format consisting of a header with opcode and a payload that varies depending on the opcode.

+-------+---------------------------+----------------------+
|       | Header                    | Payload              |
+-------+===========================+======================+
| Name  | (unused)| status | opcode | Opcode-specific data |
+-------+---------+--------+--------+----------------------+
| Length| 4b      | 1b     | 3b     | 1B-252B              |
+-------+---------+--------+--------+----------------------+

There are four operations defined, each of which is associated with one or more
opcodes:



.. list-table:: Protocol operations, opcodes and packet sizes
   :header-rows: 1

   * - Operation
     - Opcode(s)
     - Packet size (incl. header)
   * - Initialize BLOB transfer
     - ``UART_BLOB_OPCODE_INIT`` (0x00)
     - 5B
   * - Write BLOB fragment
     - ``UART_BLOB_OPCODE_WRITEH`` (0x01),
       ``UART_BLOB_OPCODE_WRITEC`` (0x02)
     - 5B (for WRITEH) or 2B-253B (for WRITEC)
   * - Request BLOB offset
     - ``UART_BLOB_OPCODE_OFFSET`` (0x03)
     - 5B
   * - Finish BLOB transfer
     - ``UART_BLOB_OPCODE_DONE`` (0x04)
     - 5B

The intended protocol operation is shown below:




UART BLOB sender
****************

The UART BLOB sender implements the sender role in the UART BLOB protocol.
It provides APIs for initializing a transfer, writing fragments, querying for the current
offset.

The sender is initialized with :cpp:func:`uart_blob_tx_init`.
This function takes a :cpp:struct:`uart_blob_tx_cb` with callbacks that are used to dispatch asynchronous events.
The callbacks are used for three types of events:

* :cpp:member:`uart_blob_tx_cb::status_cb` is used for status replies to :cpp:func:`uart_blob_tx_send_init`, :cpp:func:`uart_blob_tx_send_write` and :cpp:func:`uart_blob_tx_send_done` and error responses to :cpp:func:`uart_blob_tx_send_offset`.
* :cpp:member:`uart_blob_tx_cb::offset_cb` is used for (successful) offset replies to :cpp:func:`uart_blob_tx_send_offset`.
* :cpp:member:`uart_blob_tx_cb::evt_cb` is used for both 



UART BLOB receiver
******************

API documentation
*****************

UART BLOB sender
================

| Header file: :file:`include/uart_blob_tx.h`
| Source files: :file:`lib/uart_blob`

.. doxygengroup:: uart_blob_tx
   :project: nrf
   :members:

UART BLOB receiver
==================

| Header file: :file:`include/uart_blob_rx.h`
| Source files: :file:`lib/uart_blob`

.. doxygengroup:: uart_blob_rx
   :project: nrf
   :members:

