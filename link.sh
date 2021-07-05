rm -rf build
mkdir build
cd build
#linking the kernel with kernel.o and boot.o files
ld -m elf_i386 -T ../elf_ia32_efi.lds ../freertos/build/pc_support.o ../freertos/build/startup.o ../freertos/build/freestanding_functions.o ../freertos/build/heap_4.o  ../freertos/build/list.o ../freertos/build/port.o ../freertos/build/timers.o ../freertos/build/HPET.o ../freertos/build/queue.o ../freertos/build/tasks.o ../freertos/build/printf-stdarg.o ../freertos/build/portASM.o ../freertos/build/main.o -o MyOS.bin -nostdlib
cd ..
