export ARCH=arm
export CROSS_COMPILE=/home/array/Android/zt280-ICS-amlogic/prebuilt/linux-x86/toolchain/arm-2010q1/bin/arm-none-linux-gnueabi-

#cp arch/arm/configs/meson_defconfig .config
#make menuconfig
#make vmlinux

TOP=${PWD}
export MKIMAGE=${TOP}/arch/arm/boot/mkimage

