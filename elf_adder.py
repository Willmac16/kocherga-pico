#
# This script adds sections marked as code or data of one elf to another at their original LMA
# This approach to elf modification was based off of https://stackoverflow.com/questions/31125217/how-to-make-duplicate-sections-in-elf-file
#
# $1: Destination/Base Elf
#
# $2: Source/Donor Elf


import sys, os
import makeelf.elf as elf

if len(sys.argv) < 3:
  print("Usage: python elf_adder.py <destination_elf> <source_elf>")
  sys.exit(1)

destination_elf = sys.argv[1]
source_elf = sys.argv[2]

donor = elf.ELF.from_file(source_elf)[0]
dest = elf.ELF.from_file(destination_elf)[0]


_, donor_sects_body = donor.get_section_by_name(".shstrtab")
donor_section_names = donor_sects_body.split(b'\x00')[1:]

for section_name in donor_section_names[:-1]:
  head, body = donor.get_section_by_name(section_name)

  target_sect_name = section_name + b'_two'

  dest.append_section(target_sect_name, body, head.sh_addr)


fd = os.open(destination_elf, os.O_WRONLY | os.O_CREAT | os.O_TRUNC)
os.write(fd, bytes(dest))
os.close(fd)
