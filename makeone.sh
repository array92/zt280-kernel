#!/bin/sh


if [ ! -d $MK_ROOT/$MK_VERSION/$MK_SUBVERSION ];then
	mkdir -p $MK_ROOT/$MK_VERSION/$MK_SUBVERSION/
fi

#rm .config
#make meson_BOARD_${MK_VERSION}_${MK_SUBVERSION}_defconfig
#cp .config arch/arm/configs/meson_BOARD_${MK_VERSION}_${MK_SUBVERSION}_defconfig

#./makeone_recovery.sh
./makeone_release.sh

## mv uImage ${MK_ROOT}/${MK_VERSION}/${MK_SUBVERSION}/
## mv uImage_recovery ${MK_ROOT}/${MK_VERSION}/${MK_SUBVERSION}/
## mv ZT280.kernel ${MK_ROOT}/${MK_VERSION}/${MK_SUBVERSION}/
## mv ZT280.recovery ${MK_ROOT}/${MK_VERSION}/${MK_SUBVERSION}/

#cd ${MK_ROOT}/${MK_VERSION}
#tar jcvf ${MK_VERSION}_${MK_SUBVERSION}.kernel.tar.bz2 ${MK_SUBVERSION}
#mv ${MK_VERSION}_${MK_SUBVERSION}.kernel.tar.bz2 ../

