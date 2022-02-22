# stlink_gba_multiboot

Designed for https://bennvenn.myshopify.com/products/multiboot-adaptor-for-gba

Flash `gba_multiboot.hex` using the instructions from https://bennvenn.myshopify.com/pages/diy-projects-1

Upload to the GBA using `upload.py /dev/yourTTYDevice /path/to/your_program_mb.gba`

NOTE: An unmodified GBA link cable WILL NOT WORK! A GBC link cable is required. If you have a soldering iron, you can modify a GBA link cable to work by desoldering the orange wire on the purple-tip side of the cable box and soldering it to the red wire on the opposite (optionally: install a switch to allow toggling between GND and the TX/RX twist)

For building from source, use STM32CubeIDE (or bring your own buildscripts)

