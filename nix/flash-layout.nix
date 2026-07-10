# Single source for the ESP32 flash layout, in flash order. Consumed by
# nix/firmware.nix (flash-manifest.json parts) and nix/apps.nix (esptool
# write-flash arguments + the human-readable offsets line).
#
# 0x1000 (bootloader) and 0x8000 (partition table) are fixed by the IDF 4.4
# ESP32 image layout; 0xe000 (otadata) and 0x10000 (app0) MUST match
# partitions.csv — a mismatch flashes to the wrong offset with no error.
[
  { offset = "0x1000"; file = "bootloader.bin"; }
  { offset = "0x8000"; file = "partition-table.bin"; }
  { offset = "0xe000"; file = "ota_data_initial.bin"; }
  { offset = "0x10000"; file = "firmware.bin"; }
]
