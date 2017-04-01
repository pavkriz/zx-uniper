; Place this routine to RAM (not ROM),
; because we want to page our ("uniper splash") custom ROM (emulated by zx-uniper)
; out and page the original (physical) ZX ROM in.
SECTION smc_user 

PUBLIC _copy_rom2rom

EXTERN ZX_SPLASH_ROMREAD_START_BORDER
EXTERN ZX_SPLASH_ROMREAD_END_BORDER    

INCLUDE "uniper_splash_common.inc"

_copy_rom2rom:

copy_rom2rom:
    ; Disable intetrupts during the operation.
    ; Interrupt would, well, actually interrupt our operation ;-)
    ; And we don't need intterupts here.
    di

    ; Set border to predefined value ZX_SPLASH_ROMREAD_START_BORDER.
    ; ZX-uniper sniffs writes to port 254.
    ; When ZX_SPLASH_ROMREAD_START_BORDER value is sniffed, zx-uniper will
    ; page the physical ZX's ROM in (leaves ROMCS in high-Z, letting ULA
    ; to control this line). 
    ld a, ZX_SPLASH_ROMREAD_START_BORDER
    out (254), a

    ; Wait in order to give zx-uniper some time (eg. to reconfigure bus operation handlers).
    ld b, 255
wait1:  
    nop
    nop
    nop
    nop  
    djnz wait1
    
    ; Now we want to read the whole physical ROM, while zx-uniper sniffs
    ; the reads and stores values from data-bus to it's RAM.
    ; We abuse the CPI instruction here (to read values from ROM), because
    ; it has minimum side-effects (in contrast to LDI or OUTI).
    
    ; Start reading at address 0 (beginning of the ROM)    
    ld hl, 0   
    ; Read 16 kB (the size of the ROM)
    ; TODO ZX Spectrum 128k is not supported yet, we copy only one ROM (page) here 
    ld bc, 0x4000
    
    ; Let's go reading!
    ; We unroll the loop a little in order to make it faster.
    ; (Make sure 0x4000 modulo number of operations in the unroller loop = 0)
read_rom_loop:    
    cpi
    cpi
    cpi
    cpi
    cpi
    cpi
    cpi
    cpi
    cpi
    cpi
    cpi
    cpi
    cpi
    cpi
    cpi
    cpi
    jp pe, read_rom_loop
     
    ; Here, we are done reading the ROM
    
    ; Set border to predefined value ZX_SPLASH_ROMREAD_END_BORDER in order
    ; to notify zx-uniper we are done (but it should actually know on its 
    ; own due to sniffing our operations).
    ld a, ZX_SPLASH_ROMREAD_END_BORDER
    out (254), a
    
    ; Wait in order to give zx-uniper some time (eg. to reconfigure bus operation handlers).
    ld b, 255
wait2:  
    nop
    nop
    nop
    nop  
    djnz wait2
    
    ; Enable interrupts again
    ei

    ; Restore black border
    ld a, 0
    out (254), a

inf_loop:    
    jp inf_loop 
    