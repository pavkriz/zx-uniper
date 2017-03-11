zcc +zx -subtype=if2 -clib=new -startup=33 -O3 --list -create-app -o uniper_splash uniper_splash.c
bin2h uniper_splash_rom < uniper_splash.rom > ..\inc\roms\uniper_splash_rom.h 