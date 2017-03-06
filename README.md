# hkfree zx-uniper

Universal programmable peripheral for ZX Spectrum.

"Bring new experience to your retrocomputing."

This work is supported by [hkfree.org](http://www.hkfree.org) community network.

## Supported emulated hardware

Currently supported hardware and the way it's emulated is rather proof-of-concept than ready-to-use solution. Thus, user must be currently able to compile zx-uniper on his/her own in order to use it.

* external ROM
  * alternative ROM or IF2 Cartridge ROM
  * currently in form of C array in a header file (use bin2h) compiled into to the firmware
* Divide interface with 32kB RAM
  * read/write support, only 1 sector transfer currently supported, no error handling yet
  * emulates IDE drive via USB Host Mass Storage (flash drive)

## Architecture

* zx-uniper is built around the STM32F767VIT6 ARM microcontroller (original proof-of-concept has been designed for LPC4337 dual Core-M4+Core-M0 ARM controller, see `lpc4337` branch)
* see [Wiki](https://github.com/pavkriz/zx-uniper/wiki) for details regarding the development
