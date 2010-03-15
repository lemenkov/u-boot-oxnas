#!/bin/bash
make clean && make oxnas_config && make
../wd-mybook-bootloader/tools/packager u-boot.bin u-boot.img
mv u-boot.img ~/fuse/sshfs/sulaco/home/petro/

