#!/bin/sh

sed -i 's#.*CONFIG_INITRAMFS_SOURCE=.*#CONFIG_INITRAMFS_SOURCE="zt180.cpio"#g' .config
sed -i 's#.*CONFIG_CONFIG_SDCARD_SLEEP=.*#CONFIG_CONFIG_SDCARD_SLEEP=3#g' .config

make ARCH=arm CROSS_COMPILE="ccache arm-none-linux-gnueabi-" uImage -j 4
cp arch/arm/boot/uImage uImage
cp arch/arm/boot/uImage /tftpboot/uImage

