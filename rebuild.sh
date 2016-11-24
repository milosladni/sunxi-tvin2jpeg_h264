#!/bin/sh
echo "PATH:"
echo $PATH

sb2 make clean
sb2 make DEBUG="true" && scp sunxi-tvin2jpeg_h264 root@192.168.1.124:/root/sunxi-tvin2jpeg_h264 && echo "Build Finished: Copy sunxi-tvin2jpeg_h264 Appl.."


