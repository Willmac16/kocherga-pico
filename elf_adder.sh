#
# This script adds sections marked as code or data of one elf to another at their original LMA
# This approach to elf modification was based off of https://stackoverflow.com/questions/31125217/how-to-make-duplicate-sections-in-elf-file
#
# $1: Destination/Base Elf
# 
# $2: Source/Donor Elf
#

rm -f out.elf
arm-none-eabi-objcopy $1 out.elf

rm -f temp.txt

# Line Nums (+1) of Code and Data Sections
arm-none-eabi-objdump -h $2 | grep -n -E "CODE|DATA" | awk '{print $1}' | sed -e 's/://' > temp.txt

while read p; do
  line_num=$(echo "$p-1" | bc)

  line=$(arm-none-eabi-objdump -h $2 | sed -n $line_num'p')

  name=$(echo "$line" | awk '{print $2}')
  LMA=$(echo "$line" | awk '{print $5}')
  decimal_addr=$(echo "obase=10; ibase=16; ${LMA:u}" | bc)

  dup_name=$name"_two"
  rm temp.bin
  arm-none-eabi-objcopy -O binary --only-section=$name auto-loader-offset.elf temp.bin
  arm-none-eabi-objcopy --add-section $dup_name=temp.bin out.elf

  arm-none-eabi-objcopy --change-section-address $dup_name=$decimal_addr out.elf 
  
  arm-none-eabi-objcopy --set-section-flags $dup_name=contents,alloc,load,readonly,code out.elf

  echo "$name @ 0x$LMA 0d$decimal_addr"
  echo ""
done < temp.txt

arm-none-eabi-objdump -h out.elf
mv out.elf $1
