#
# This script adds sections marked as code or data of one elf to another at their original LMA
# This approach to elf modification was based off of https://stackoverflow.com/questions/31125217/how-to-make-duplicate-sections-in-elf-file
#
# $1: Destination/Base Elf
# 
# $2: Source/Donor Elf
#

echo "Adding $2 to $1"

rm -f out.elf
arm-none-eabi-objcopy $1 out.elf

rm -f temp.txt

# Line Nums (+1) of Code and Data Sections
# |READONLY|DEBUGGING|OCTETS
arm-none-eabi-objdump -h $2 | grep -n -E "CODE|DATA" | awk '{print $1}' | sed -e 's/://' > temp.txt

while read p; do
  line_num=$(echo "$p-1" | bc)
  flags_line_num=$(echo "$p" | bc)

  line=$(arm-none-eabi-objdump -h $2 | sed -n $line_num'p')
  flags_line=$(arm-none-eabi-objdump -h $2 | sed -n $flags_line_num'p' | tr -d ' ')

  name=$(echo "$line" | awk '{print $2}')
  VMA=$(echo "$line" | awk '{print $4}' | tr "[:lower:]" "[:upper:]")
  LMA=$(echo "$line" | awk '{print $5}' | tr "[:lower:]" "[:upper:]")

  decimal_VMA=$(echo "obase=10; ibase=16; $VMA" | bc)
  decimal_LMA=$(echo "obase=10; ibase=16; $LMA" | bc)

  dup_name=$name"_two"
  rm temp.bin
  arm-none-eabi-objcopy -O binary --only-section=$name $2 temp.bin
  arm-none-eabi-objcopy --add-section $dup_name=temp.bin out.elf

  arm-none-eabi-objcopy --change-section-address $dup_name=$decimal_LMA out.elf
#   arm-none-eabi-objcopy --change-section-vma $dup_name=$decimal_VMA out.elf
#   arm-none-eabi-objcopy --change-section-lma $dup_dame=$decimal_LMA out.elf
  

  arm-none-eabi-objcopy --set-section-flags $dup_name=$flags_line out.elf

  echo "$dup_name ^@ 0x$VMA 0d$decimal_VMA @ 0x$LMA 0d$decimal_LMA"
  echo "$flags_line"
  echo ""
done < temp.txt

arm-none-eabi-objdump -h out.elf
mv out.elf $1
