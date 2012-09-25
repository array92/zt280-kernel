#!/bin/sh

##ROOT_DIR=/home/work/android/ml/amlogic_new/

ROOT_KERNEL=`pwd`
ROOT_DIR=${ROOT_KERNEL}/../
echo ${ROOT_DIR}

cp ${ROOT_DIR}/out/target/product/b04ref/system/bin/test_tool ${ROOT_DIR}/test_tools/root_update/
chmod 777 -R ${ROOT_DIR}/test_tools/root_update/
cd ${ROOT_DIR}/test_tools/root_update/
find . | cpio -o -H newc > ../zt180_test.cpio
cp ${ROOT_DIR}/test_tools/zt180_test.cpio ${ROOT_KERNEL}

cd ${ROOT_KERNEL}

sed -i 's#.*CONFIG_INITRAMFS_SOURCE=.*#CONFIG_INITRAMFS_SOURCE="zt180_test.cpio"#g' .config
sed -i 's#.*CONFIG_CONFIG_SDCARD_SLEEP=.*#CONFIG_CONFIG_SDCARD_SLEEP=10#g' .config

make ARCH=arm CROSS_COMPILE="ccache arm-none-linux-gnueabi-" uImage -j 4
cp arch/arm/boot/uImage uImage_testtool
cp arch/arm/boot/uImage /tftpboot/uImage_testtool
./mkimage -A arm -T firmware -C none -a 0xffffffff -e 0x0 -n  "LK:ZT280" -d uImage_testtool ZT280.hw

