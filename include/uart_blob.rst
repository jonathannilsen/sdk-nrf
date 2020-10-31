.. _lib_uart_blob:

.. |base| replace:: UART BLOB

UART Binary Large OBject (BLOB) transfer
########################################

The |base| transfer library implements a protocol for transferring Binary Large OBjects (BLOBs), i.e. arbitrary binary data over UART.
The library is split into two submodules, the sender ``uart_blob_tx`` and receiver ``uart_blob_rx`` which can be enabled and configured independently.


|base| protocol
******************

The library uses a protocol that was designed to be easily compatible with the API of :ref:`lib_dfu_target`.
While this protocol is largely hidden behind the |base| sender/receiver API, it may be useful to understand it to help understand the APIs.

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


|base| sender
****************

The |base| sender implements the sender role in the |base| protocol.
It provides APIs for initializing a transfer, writing fragments, querying for the current offset.

The sender is initialized with :cpp:func:`uart_blob_tx_init`.
This function takes a :cpp:struct:`uart_blob_tx_cb` with callbacks that are used to dispatch asynchronous events.

Before transfers can be made, the sender must be enabled with :cpp:func:`uart_blob_tx_enable`.
An event with type :cpp:enumerator:`UART_BLOB_EVT_STARTED<uart_blob::UART_BLOB_EVT_STARTED>` is dispatched when the sender is ready to send.
The sender can be disabled with :cpp:func:`uart_blob_tx_disable`.
This will abort any ongoing transfers.
An event with type :cpp:enumerator:`UART_BLOB_EVT_STOPPED<uart_blob::UART_BLOB_EVT_STOPPED>` is dispatched when the sender is disabled.

The callback functions passed during initialization are used for three types of events:

* :cpp:member:`uart_blob_tx_cb::status_cb` is used for status replies to :cpp:func:`uart_blob_tx_send_init`, :cpp:func:`uart_blob_tx_send_write` and :cpp:func:`uart_blob_tx_send_done` and error responses to :cpp:func:`uart_blob_tx_send_offset`.
* :cpp:member:`uart_blob_tx_cb::offset_cb` is used for (successful) offset replies to :cpp:func:`uart_blob_tx_send_offset`.
* :cpp:member:`uart_blob_tx_cb::evt_cb` is called for non-message events. These events are described below.




|base| receiver
******************

The |base| receiver implements the receiver role in the |base| protocol.
Transfers are initiated by the |base| sender.
The receiver provides an API for starting to listen for incoming transfers, stopping to listen for transfers and aborting ongoing transfers.

The receiver is initialized with :cpp:func`uart_blob_rx_init`.
This function takes a buffer for storing received BLOB fragments and the length of the buffer.
It also takes a :cpp:struct:`uart_blob_rx_cb` with callbacks that are used to dispatch asynchronous events.

To start listening for incoming transfers, call :cpp:func:`uart_blob_rx_enable`.
To stop listening for transfers, or abort the ongoing transfer, call :cpp:func:`uart_blob_rx_disable`.

The callback functions passed during initialization are used for the following events:
* :cpp:member:`uart_blob_rx_cb::init_cb` is called when the sender starts a new object transfer.
* :cpp:member:`uart_blob_rx_cb::write_cb` is called when a full fragment has been received.
* :cpp:member:`uart_blob_rx_cb::offset_cb` is called when the sender requests the current offset of the transfer.
  The callback should write the current offset to the given offset pointer.
* :cpp:member:`uart_blob_rx_cb::done_cb` is called when the sender finalizes the current object transfer.
* :cpp:member:`uart_blob_rx_cb::evt_cb` is called for non-message events. These events are described below.

The |base| receiver uses 


API documentation
*****************

|base| common structures
========================

| Header file: :file:`include/uart_blob.h`

.. doxygengroup:: uart_blob
   :project: nrf
   :members:


|base| sender
================

| Header file: :file:`include/uart_blob_tx.h`
| Source files: :file:`lib/uart_blob`

.. doxygengroup:: uart_blob_tx
   :project: nrf
   :members:


|base| receiver
==================

| Header file: :file:`include/uart_blob_rx.h`
| Source files: :file:`lib/uart_blob`

.. doxygengroup:: uart_blob_rx
   :project: nrf
   :members:

