#!/bin/sh

#cp logo/kernel/logo_linux_${MK_LOGO}_clut224.ppm drivers/video/logo/logo_linux_clut224.ppm
#rm drivers/video/logo/logo_linux_clut224.c

#cp -f logo/android/initlogo.rle.bak.${MK_LOGO} ../out/target/product/b01ref/root/initlogo.rle.bak

if [ -e uImage_recovery ];then
rm uImage_recovery;
fi

if [ -e ZT280.recovery ];then
rm ZT280.recovery;
fi

sed -i 's#.*CONFIG_INITRAMFS_SOURCE=.*#CONFIG_INITRAMFS_SOURCE="device/target/f04ref/root/"#g' .config
sed -i 's#.*CONFIG_CONFIG_SDCARD_SLEEP=.*#CONFIG_CONFIG_SDCARD_SLEEP=0#g' .config
sed -i 's/CONFIG_IMAGE_RECOVERY=y/# CONFIG_IMAGE_RECOVERY is not set/g' .config


make CROSS_COMPILE=$CROSS_COMPILE uImage -j6

mkimage -A arm -T firmware -C none -a 0xffffffff -e 0x0 -n "LK:ZT280_${MK_VERSION}_${MK_SUBVERSION}" -d arch/arm/boot/uImage ZT280.kernel


mv ZT280.kernel $MK_ROOT/$MK_VERSION/$MK_SUBVERSION/

