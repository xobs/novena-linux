#!/bin/sh
filename="$1"
if [ "x${filename}" = "x" ]
then
	echo "Usage: $0 [filename].bmp"
	exit 1
fi

if [ ! -e "${filename}" ]
then
	echo "File '${filename}' not found"
	exit 1
fi

convert "${filename}" tmp.ppm
ppmquant 224 tmp.ppm > tmp-quant.ppm || exit 1
pnmnoraw tmp-quant.ppm > drivers/video/logo/logo_linux_clut224.ppm || exit 1
rm -f tmp.ppm tmp-quant.ppm
echo "Converted"
