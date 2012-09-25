#!/bin/sh

rm uImage_recovery
rm ZT280.recovery

sed -i 's#.*CONFIG_INITRAMFS_SOURCE=.*#CONFIG_INITRAMFS_SOURCE="../out/target/product/b01ref/recovery_ics/root/"#g' .config
sed -i 's#.*CONFIG_CONFIG_SDCARD_SLEEP=.*#CONFIG_CONFIG_SDCARD_SLEEP=3#g' .config
sed -i 's/# CONFIG_IMAGE_RECOVERY is not set/CONFIG_IMAGE_RECOVERY=y/g' .config

##cp -f ../out/target/product/b01ref/system/bin/recovery ../out/target/product/b01ref/recovery/root/sbin/
##cp -f ../device/amlogic/b01ref/busybox ../out/target/product/b01ref/recovery/root/sbin/
make ARCH=arm CROSS_COMPILE="ccache arm-none-linux-gnueabi-" uImage -j 4

mv arch/arm/boot/uImage uImage_recovery
#cp arch/arm/boot/uImage /tftpboot/uImage_recovery

./mkimage -A arm -T firmware -C none -a 0xffffffff -e 0x0 -n  "LK:ZT280" -d uImage_recovery ZT280.recovery


