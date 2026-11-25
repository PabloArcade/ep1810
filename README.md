# ep1810dump v0.94 -- source

Altera Classic EPLD (EP1810 / EP1800I, EP910 / EP900I, EP610 / EP600I) fuse map decompiler by Darksoft.
Reads one fuse file (.jed or .pof) and writes a design that MAX+PLUS II 10.2 compiles back into the same part,
every pin and logic cell in its original place.
Built for arcade PCB preservation. Use at your own risk.

## Commands

```
ep1810dump decode <file> [-d DEVICE] [-o FILE]      text report: device, package, pin use, equations
ep1810dump emit   <file> [-d DEVICE] [-n NAME] -o DIR
                                                    DIR/NAME.tdf, NAME.acf, NAME.v, REPORT.txt
ep1810dump dump   <file> [-d DEVICE] [-o FILE]      fuses, logic cell by logic cell
ep1810dump diff   <file A> <file B> [-o FILE]       fuse differences between two files
ep1810dump convert <file> -o OUT.{jed|pof} [-d DEVICE]
                                                    the same fuses as a .jed or a .pof
ep1810dump devices                                  device names accepted
ep1810dump selftest [--fail]                        built-in checks
ep1810dump --version

Exit codes: 0 done, 3 refused by name (the reason is printed), 2 usage, 1 error.
```

Examples:

```
ep1810dump decode chip.jed                          report on standard output
ep1810dump decode chip.jed -d EP1810LC-45           a read-back with no device name in it
ep1810dump emit chip.pof -n u12 -o out              out/u12.tdf, u12.acf, u12.v, REPORT.txt
ep1810dump diff old.jed new.jed
ep1810dump convert chip.jed -o chip.pof -d EP1810LC-45
```

- **decode** prints device, package, pin use and the equations of every logic cell.
- **emit** writes `NAME.tdf` (AHDL) and `NAME.acf` (device, pin and logic cell locations) for MAX+PLUS II,
  `NAME.v` (behavioural Verilog of the programmed part, registers start at 0) and `REPORT.txt`.
  DIR must exist; NAME defaults to `design`.
- **dump** prints the fuses of every logic cell, term by term.
- **diff** prints the fuses that differ between two files of the same family.
- **convert** re-encodes a fuse file to the other format; .pof output needs a device.
- **devices** lists the 158 device names accepted, with family and package.

The device comes from the file: the `ND` note of a JEDEC file, the device packet of a POF. `-d` names it
when the file does not, or selects the other package of the same family. A JEDEC file read back from a
programmer usually has no device name: give `-d`.

The banner `ep1810 decoder tool by Darksoft v0.94. Use at your own risk.` goes to standard error on every run.
In the output `pN` is package pin N and `lcN` is logic cell N. A register whose clock and data depend on
the same signal is listed as `RACING`.

## Supported parts and packages

    EP1810 / EP1800I   J-lead 68, PGA 68
    EP910  / EP900I    DIP 40, J-lead 44
    EP610  / EP600I    DIP 24, J-lead 28

## The parts

Every logic cell has ten product terms over the array lines: terms 0-7 are OR-ed, term 8 clears the
register, term 9 is the output enable or, when the cell selects it, the register clock. Five fuses per
cell select inversion, clock source, register type (D or T), what the cell's own array lines carry (the
cell or its pin), and registered or combinational output. The EP1810 has four quadrants of twelve logic
cells; four cells per quadrant are global: their pins reach every quadrant.

## Known limits

- A logic cell whose fuse pattern MAX+PLUS II does not write is refused by name and nothing is written.
  Example: `GLOBAL_CELL_BRANCH_UNSEEN`, a global logic cell of the EP1810 in use with its pin-select fuse set.
- Tested on files written by MAX+PLUS II. Not tested on files read from boards.

## Rebuild

    ./build.sh

Needs a C99 compiler. Also builds `ep1810dump.exe` when `x86_64-w64-mingw32-gcc` is installed.
C library only. Every device table is compiled in (`src/tables.h`, checksums in `TABLES.sha256`).

## License

GPL-3.0-or-later. Copyright (C) DARKSOFT. See `LICENSE`.
