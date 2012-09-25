#!/bin/sh


if [ ! $MK_LOGO_RECOVERY ];then
export MK_LOGO_RECOVERY=${MK_LOGO}
fi

#cp logo/kernel/logo_linux_${MK_LOGO_RECOVERY}_clut224.ppm drivers/video/logo/#logo_linux_clut224.ppm
#rm drivers/video/logo/logo_linux_clut224.c

#cp -f logo/android/initlogo.rle.bak.${MK_LOGO_RECOVERY} ../out/target/product/b01ref/root/initlogo.rle.bak

if [ -e uImage_recovery ];then
rm uImage_recovery;
fi

if [ -e ZT280.recovery ];then
rm ZT280.recovery;
fi

sed -i 's#.*CONFIG_INITRAMFS_SOURCE=.*#CONFIG_INITRAMFS_SOURCE="device/target/f04ref/recovery/root/"#g' .config
sed -i 's#.*CONFIG_CONFIG_SDCARD_SLEEP=.*#CONFIG_CONFIG_SDCARD_SLEEP=3#g' .config
sed -i 's/# CONFIG_IMAGE_RECOVERY is not set/CONFIG_IMAGE_RECOVERY=y/g' .config

#cp -f ../out/target/product/b01ref/system/bin/recovery ../out/target/product/b01ref/recovery/root/sbin/
#cp -f ../device/amlogic/b01ref/busybox ../out/target/product/b01ref/recovery/root/sbin/

make CROSS_COMPILE=$CROSS_COMPILE uImage -j6

mv arch/arm/boot/uImage arch/arm/boot/uImage_recovery


mkimage -A arm -T firmware -C none -a 0xffffffff -e 0x0 -n "LK:ZT280_${MK_VERSION}_${MK_SUBVERSION}" -d arch/arm/boot/uImage_recovery ZT280.recovery

mv ZT280.recovery $MK_ROOT/$MK_VERSION/$MK_SUBVERSION/

