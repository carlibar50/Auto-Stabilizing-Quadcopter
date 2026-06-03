MEMORY {
    /* Pico 2 W: 4 MB de flash externa, 520 KB de SRAM. */
    FLASH : ORIGIN = 0x10000000, LENGTH = 4096K
    RAM   : ORIGIN = 0x20000000, LENGTH = 512K
}

/* El RP2350 valida la imagen leyendo un "Image Definition" justo después
   de la tabla de vectores. Aquí lo colocamos en .start_block. */
SECTIONS {
    .start_block : ALIGN(4)
    {
        __start_block_addr = .;
        KEEP(*(.start_block));
        . = ALIGN(4);
    } > FLASH

} INSERT AFTER .vector_table;

_stext = ORIGIN(FLASH) + 0x200;

/* Secciones necesarias para la feature `binary-info` de embassy-rp:
   picotool puede leer aqui metadata del programa. */
SECTIONS {
    .bi_entries : ALIGN(4)
    {
        __bi_entries_start = .;
        KEEP(*(.bi_entries));
        . = ALIGN(4);
        __bi_entries_end = .;
    } > FLASH

} INSERT AFTER .text;