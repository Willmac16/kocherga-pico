
# Auto Loader

This is just a mockup I made to test out and get seperately linked but combined deployment binarys that launch each other working.
Its literally just pico-examples' blink.c, except every blink alternates between a copy to ram and an XIP flash binary.
This relies on the lkb-c2r.S and lkb-off.S assembly files to perform the same behavior of exit_from_boot2 with different target binary offsets in flash.

Also of note, the elf_adder.sh file in the root of this repo is a script that will add code and data sections of one elf file to another, making a "Franken-Elf" that has two different binaries in one. While this will upload (with some warnings) over SWD, GDB does need to be told about the "Donor" elf file, so it can find the symbols for the code and data sections. This is done with the `add-symmbol-file` command.
