/* STM32F302R8: 64 KB flash, 16 KB SRAM. The last 2 KB flash page holds the
   persisted parameter blob (mmc-drive nvparam), so the linker never sees it. */
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 62K
  RAM   : ORIGIN = 0x20000000, LENGTH = 16K
}
