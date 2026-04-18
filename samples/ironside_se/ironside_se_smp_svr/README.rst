.. _ironside_se_smp_svr_sample:

IronSide SE SMP Server
######################

.. contents::
   :local:
   :depth: 2

This sample demonstrates how to update the IronSide SE firmware over UART
using a custom MCUmgr command group on top of the Simple Management Protocol
(SMP) server.

It reuses the standard SMP chunked-upload wire format (``off``, ``data``,
``len`` fields) so that existing SMP client tooling can drive the transfer
with minimal adaptation.  The sample provides three commands that together
enable a complete over-SMP update cycle using ``west ncs-ironside-se-update``.

Requirements
************

* nRF54H20 DK (nrf54h20dk/nrf54h20/cpuapp)
* An IronSide SE release ZIP

Overview
********

The sample registers a custom MCUmgr command group
(``MGMT_GROUP_ID_PERUSER + 2``) with the following commands:

.. list-table::
   :header-rows: 1

   * - ID
     - Name
     - Op
     - Purpose
   * - 0
     - upload
     - write
     - Chunked blob transfer + ``ironside_se_update()``
   * - 1
     - version_get
     - read
     - Read ISE + Recovery firmware versions from boot report
   * - 2
     - status_get
     - read
     - Read last update status from boot report

Upload protocol (ID 0)
======================

Each SMP write request carries CBOR fields identical to those used by the
full-modem-firmware-update (FMFU) library:

* ``off`` — byte offset into the update blob
* ``data`` — chunk payload
* ``len`` — total blob length (sent with the first chunk)

The handler writes incoming chunks to a dedicated staging partition in MRAM
(``ironside_se_update_partition``), which must reside in the IronSide SE
valid update address range.  When the last byte is received, the handler
calls ``ironside_se_update()`` and responds with success.

version_get (ID 1)
==================

An SMP read request with no payload.  Returns the firmware versions for both
the IronSide SE (``uslot``) and IronSide SE Recovery (``rslot``) slots as
read from the boot report:

.. code-block:: json

   {
     "uslot": { "version_int": 385876000, "extraversion": "23.4.0-live+27" },
     "rslot": { "version_int": 385876000, "extraversion": "23.4.0-live+27" }
   }

``version_int`` is the packed 32-bit version (``MAJOR.MINOR.PATCH.SEQNUM``).
``extraversion`` is the human-readable version string from the boot report.

status_get (ID 2)
=================

An SMP read request with no payload.  Returns the last update status code
from the boot report:

.. code-block:: json

   { "status": 4026531848 }

The status code maps to the ``UpdateStatus`` enum values used by the
``west ncs-ironside-se-update`` command (e.g. ``0xF0000008`` =
``UROT_ACTIVATED``).

Building and running
********************

.. code-block:: console

   west build -b nrf54h20dk/nrf54h20/cpuapp samples/ironside_se/ironside_se_smp_svr
   west flash

After flashing, observe on the console:

.. code-block:: console

   [00:00:00.xxx] <inf> ironside_se_smp_svr: IronSide SE SMP server ready (build: ...)

Performing an update
====================

The ``west ncs-ironside-se-update`` command supports updating over SMP when
the ``--smp-port`` argument is provided:

.. code-block:: console

   west ncs-ironside-se-update --zip nrf54h20_soc_binaries_v20.0.1.zip --smp-port /dev/ttyACM0

This performs the full update procedure:

1. Query current firmware versions
2. For each firmware slot (Recovery first, then IronSide SE):

   a. Upload the update blob
   b. Reset the device
   c. Wait for reboot and verify version

3. Report final result

Updating a single slot
======================

Use ``--firmware-slot`` to restrict the update to one slot:

.. code-block:: console

   west ncs-ironside-se-update --zip firmware.zip --smp-port /dev/ttyACM0 --firmware-slot uslot

Run ``west ncs-ironside-se-update --help`` for all options.

Dependencies
************

This sample uses the following libraries:

* :ref:`IronSide SE update MCUmgr group <lib_ironside_se_update_mgmt>` (``CONFIG_MGMT_IRONSIDE_SE_UPDATE``)
* Zephyr MCUmgr SMP transport (UART)
* IronSide SE call API (``CONFIG_IRONSIDE_SE_CALL``)
