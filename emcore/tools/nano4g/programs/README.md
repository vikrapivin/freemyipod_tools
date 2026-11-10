# Preserved original FMSS programs

`ERASE.s` and `WRITE.s` are unmodified copies of the annotated restore-OS
programs in `code/root-restore/fmss_asm/`. Each source header records its image
coordinates. The round-6 family assembles them using the bundled `fmssasm`;
no Apple image is patched, and no binary instruction array is stored here.
The regression test compares both assembled streams against the independent
Python assembler. Register aliases used by these sources retain their original
names and offsets.

The host supplies the documented CPU parameter blocks for one operation at a
time on blocks 3000 and 3001. The original program result and pointer advances
are retained. Separate raw/decoded reads check erase and program payloads.
