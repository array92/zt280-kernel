#!/bin/sh

rm uImage
rm ZT280.kernel
sed -i 's#.*CONFIG_INITRAMFS_SOURCE=.*#CONFIG_INITRAMFS_SOURCE="../out/target/product/b01ref/root/"#g' .config
sed -i 's#.*CONFIG_CONFIG_SDCARD_SLEEP=.*#CONFIG_CONFIG_SDCARD_SLEEP=0#g' .config
sed -i 's/CONFIG_IMAGE_RECOVERY=y/# CONFIG_IMAGE_RECOVERY is not set/g' .config

make ARCH=arm CROSS_COMPILE="ccache arm-none-linux-gnueabi-" uImage -j 4
mv arch/arm/boot/uImage uImage
#cp arch/arm/boot/uImage /tftpboot/uImage
./mkimage -A arm -T firmware -C none -a 0xffffffff -e 0x0 -n  "LK:ZT280" -d uImage ZT280.kernel

