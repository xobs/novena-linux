#!/bin/sh
make -j4 LD=gold zImage dtbs \
	&& echo "Constructing bootloader..." \
	&& cat arch/arm/boot/zImage > bootloader \
	&& cat arch/arm/boot/dts/imx6dl-novena-aqs.dtb >> bootloader
