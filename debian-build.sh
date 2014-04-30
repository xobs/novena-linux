#!/bin/sh
make -j4 \
	LOADADDR=0x10008000 \
	LD=gold \
	KBUILD_DEBARCH=armhf \
	KBUILD_IMAGE=uImage \
	KBUILD_DTB=imx6q-novena.dtb \
	KDEB_PKGVERSION=1.1.novena \
	EMAIL="xobs@kosagi.com" \
	NAME="Sean Cross" \
	dtbs || exit 1
make -j4 \
	LOADADDR=0x10008000 \
	LD=gold \
	KBUILD_DEBARCH=armhf \
	KBUILD_IMAGE=uImage \
	KBUILD_DTB=imx6q-novena.dtb \
	KDEB_PKGVERSION=1.1.novena \
	EMAIL="xobs@kosagi.com" \
	NAME="Sean Cross" \
	deb-pkg
