cd build
#check MyOS.bin file is x86 multiboot file or not
grub-file --is-x86-multiboot MyOS.bin

#building the iso file
mkdir -p isodir/boot/grub
cp MyOS.bin isodir/boot/MyOS.bin
cp ../grub.cfg isodir/boot/grub/grub.cfg
grub-mkrescue -o MyOS.iso isodir
cd ../