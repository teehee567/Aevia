/* Flash layout — Seeed XIAO nRF52840, factory Adafruit bootloader + S140 SoftDevice.

     0x00000000 - 0x00001000   MBR                 (4  KB)
     0x00001000 - 0x00027000   SoftDevice S140 v7  (152 KB)
     0x00027000 - 0x000F4000   User application    (820 KB)  <-- our code lives here
     0x000F4000 - 0x00100000   Adafruit bootloader (48 KB)

   If your board has the older v6.1.1 SoftDevice (older bootloaders), change
   FLASH ORIGIN to 0x00026000 and LENGTH to 824K. Symptom of a wrong address:
   board boots straight back into the bootloader after flashing. */

MEMORY
{
  FLASH : ORIGIN = 0x00027000, LENGTH = 820K

  /* Bottom 128 KB of RAM is reserved for the SoftDevice if it ever runs.
     We don't enable it here, but leaving the gap means we can add BLE
     later without re-laying-out RAM. */
  RAM   : ORIGIN = 0x20020000, LENGTH = 128K
}
