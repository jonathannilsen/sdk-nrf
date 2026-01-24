.. _ug_nrf54h20_ironside:

IronSide Secure Element
#######################

The IronSide Secure Element (|ISE|) is a firmware for the :ref:`Secure Domain <ug_nrf54h20_secure_domain>` of the nRF54H20 SoC that provides security features based on the :ref:`PSA Certified Security Framework <ug_psa_certified_api_overview>`.

|ISE| provides the following features:

* :ref:`Global memory configuration <ug_nrf54h20_ironside_se_uicr>`
* :ref:`Peripheral configuration <ug_nrf54h20_ironside_se_periphconf_devicetree>`
* Boot commands

  * :ref:`ERASEALL <ug_nrf54h20_ironside_se_eraseall_command>`
  * :ref:`DEBUGWAIT <ug_nrf54h20_ironside_se_debugwait_command>`
* An alternative boot path with a :ref:`secondary firmware <ug_nrf54h20_ironside_se_secondary_firmware>`
* :ref:`CPUCONF service <ug_nrf54h20_ironside_se_cpuconf_service>`
* :ref:`Update service <ug_nrf54h20_ironside_se_update_service>`
* PSA Crypto service (:ref:`ug_crypto_architecture_implementation_standards_ironside`)
* PSA Internal Trusted Storage service (:ref:`ug_nrf54h20_ironside_se_secure_storage`)

.. toctree::
   :maxdepth: 2
   :caption: Subpages:

   ug_nrf54h20_ironside_update
   ug_nrf54h20_ironside_global_resources
   ug_nrf54h20_ironside_protect
   ug_nrf54h20_ironside_secure_storage
   ug_nrf54h20_ironside_boot
   ug_nrf54h20_ironside_services

.. todo: move somewhere fitting

Using NVR to store user data
----------------------------

The nRF54H20 SoC has several areas in the MRAM NVR pages that are reserved for storing user data.
The areas have different properties that should be considered when deciding which area to use for storing the data.

UICR.CUSTOMER (MRAM10 NVR page 0)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The UICR contains a reserved field named CUSTOMER which can be used to store arbitrary user data.
The data stored in this area inherits the properties of the UICR:

* The area is erased along with the rest of the UICR as part of the ERASEALL procedure
* The area is configured as read only and integrity checked as boot after UICR.LOCK is enabled

As a result, enabling UICR LOCK makes the CUSTOMER area behave similarly to OTP memory.

.. TODO: need better naming

MRAM10 NVR page 1
^^^^^^^^^^^^^^^^^

The second NVR page (TODO better naming) can also be used to store user data.
|ISE| defines no special semantics for NVR page 1, and this area therefore behaves as generic non-volatile storage:

* The area is *not* erased as part of the ERASEALL procedure
* The area is not integrity checked at boot.

.. note::
   It is not currently possible to configure the second NVR page as read-only.
