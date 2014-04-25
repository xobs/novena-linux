#!/bin/sh
make -j4 LOADADDR=0x10008000 LD=gold uImage modules \
	&& make dtbs \
	&& sudo cp arch/arm/boot/uImage /boot/bootloader/uimage \
	&& sudo cp arch/arm/boot/dts/imx6q-novena.dtb /boot/bootloader/uimage.dtb \
	&& sudo make modules_install
