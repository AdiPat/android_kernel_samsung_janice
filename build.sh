#!/bin/bash 
#
# Kernel Build script for GT-I9070
#
# Written by Aditya Patange aka Adi_Pat adithemagnificent@gmail.com 
#
# TO BUILD THE KERNEL- 
#
# Edit TOOLCHAIN path and INITRAMFS path accordingly. 
#
# .version-number (default is 0)
#
# EXAMPLE: ./build.sh 10
# Pass as a parameter
# 

## Misc Stuff ##

red='tput setaf 1'
green='tput setaf 2'
yellow='tput setaf 3'
blue='tput setaf 4'
violet='tput setaf 5'
cyan='tput setaf 6'
white='tput setaf 7'
normal='tput sgr0'
bold='setterm -bold'

### ### ### ### 


# SET SOME PATH VARIABLES
# Modify these as per requirements
ROOT="/home/aditya/i9070"
# Toolchain path = 
TOOLCHAIN="/home/aditya/Toolchain/arm-eabi-linaro-4.6.2/bin/arm-eabi"
KERNEL_DIR="/home/aditya/i9070/JB"
RAMDISK_DIR="/home/aditya/i9070/initramfs"
MODULES_DIR="$RAMDISK_DIR/lib/modules"
OUT="/home/aditya/i9070/out"
DEFCONFIG=janice_defconfig # Default
KERNEL=kernel.bin.md5

# More Misc stuff
echo $2 > VERSION
VERSION='cat VERSION'
clear
clear
clear
clear

###################### DONE ##########################
$cyan
echo "***********************************************"
echo "|~~~~~~~~COMPILING SIRI KERNEL ~~~~~~~~~~~~~~~|"
echo "|---------------------------------------------|"
$yello
echo "***********************************************"
echo "-----------------------------------------------"
$red
echo "---------- Adi_Pat @ XDA-DEVELOPERS -----------"
$yello
echo "-----------------------------------------------"
echo "***********************************************"
$normal

echo ">> Cleaning source"

# Clean old built kernel in out folder 
if [ -a $OUT/$KERNEL ]; then
rm $OUT/$KERNEL
fi

# Import Defconfig
cd $KERNEL_DIR 
export ARCH=arm CROSS_COMPILE=$TOOLCHAIN-
make -j clean mrproper
if [ $1 = "release" ]; then
echo "Importing RELEASE DEFCONFIG "
make -j $RELEASE_DEFCONFIG ARCH=arm CROSS_COMPILE=$TOOLCHAIN-
elif [ $1 = "debug" ]; then 
echo "Importing DEBUG DEFCONFIG"
make -j $DEBUG_DEFCONFIG ARCH=arm CROSS_COMPILE=$TOOLCHAIN-
else
echo "Importing $DEFCONFIG"
make -j $DEFCONFIG ARCH=arm CROSS_COMPILE=$TOOLCHAIN-
fi

# Set Release Version 
if [ -n VERSION ]; then
echo "Release version is 0"
echo "0" > .version
else 
echo "Release version is $VERSION" 
echo $VERSION > .version
rm VERSION
fi

# Build Modules
$bold
echo ">> COMPILING!"
echo ">> Building Modules" 
$white
make -j84 ARCH=arm CROSS_COMPILE=$TOOLCHAIN-
$normal
echo "Copying modules"
find -name '*.ko' -exec cp -av {} $MODULES_DIR/ \;

# Strip unneeded symbols
cd $MODULES_DIR
echo ">> Strip modules for size"

for m in $(find . | grep .ko | grep './')
do echo $m
$TOOLCHAIN-strip --strip-unneeded $m
done

# Build zImage
$white
echo ">> Building zImage"
cd $KERNEL_DIR 
make -j84 zImage ARCH=arm CROSS_COMPILE=$TOOLCHAIN-
$normal
if [ -a $KERNEL_DIR/arch/arm/boot/zImage ];
then
echo "Preparing Kernel......"
cd $ROOT
mkdir $OUT
cp $KERNEL_DIR/arch/arm/boot/zImage $OUT/kernel.bin
# Make md5
cd $OUT
md5sum –t kernel.bin >> kernel.bin
mv kernel.bin kernel.bin.md5
$bold
$yellow
echo "kernel.bin.md5 at $OUT/kernel.bin.md5"
$cyan
echo "DONE, PRESS ENTER TO FINISH"
$normal
read ANS
else
$red
echo "No compiled zImage at $KERNEL_DIR/arch/arm/boot/zImage"
echo "Compilation failed - Fix errors and recompile "
echo "Press enter to end script"
$normal
read ANS
fi
 
