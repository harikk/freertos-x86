reset
cd freertos
rm -rf build
gcc -m32 -c main.c tasks.c queue.c heap_4.c list.c timers.c port/port.c port/portASM.S inc/pc/startup.S inc/pc/printf-stdarg.c inc/pc/HPET.c inc/pc/freestanding_functions.c inc/pc/pc_support.c -I ./ -I ./inc/pc -I ./inc -I ./port -I -std=gnu99 -ffreestanding -O2 -Wall -Wextra
mkdir build
mv *.o build
cd ..