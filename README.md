Build Kernel 3.4.67 MT6572


cd 3467
make O=~/3467/out/OBJ ARCH=arm CROSS_COMPILE=~/3467/toolchains/arm-eabi-4.8/bin/arm-eabi- muse72_s4_kk_debug_defconfig
make O=~/3467/out/OBJ ARCH=arm CROSS_COMPILE=~/3467/toolchains/arm-eabi-4.8/bin/arm-eabi- -j4
