tvin2jpeg Licenses
======================

Proof of Concept TV-IN and hardware accelerated H264-JPEG encoder for sunxi.
Example app for TV-IN, captures N frames from /dev/video1 (tvin cvbs), send output to hdmi or parallel LCD, 
compress it to jpeg images /tmp/testImageXXX.jpg and/or h264 saving the raw h264 stream to /tmp/tvin/h264.
This is a basic example for using the tvin, H264-JPEG hardware encoder for sunxi SoCs.
This example includes:
- Using analogue input TV decoder (CVBS).
- Compress tvin to jpeg using sw libjpegturbo.
- Hw scaling. Scale input PAL 720x576 to VGA or QVGA.
- Simultaneous HW encoding jpeg and h264 with some kind of time slicing.
It is just a proof of concept and not recommended for production use!

Edit:
Based on PoC sunxi-tvin by Enrico Butera: https://github.com/ebutera/sunxi-tvin
Based on PoC jpeg encoder by Manuel Braga: https://gitorious.org/recedro/jepoc
But gitorious has been closed down and therefore the link "https://gitorious.org/recedro/jepoc" is no longer working.
Based on PoC h264 encoder by Jens Kuske: https://github.com/jemk/cedrus

Modified by Milos Ladicorbic milos.ladicorbic@gmail.com
I merged this three PoC software and tested it on Allwinner A20 (awsomA20 - https://aw-som.com/product_info.php?products_id=32).

Licence:
Because appl also uses libjpegturbo read libjpegturbo licence:
LICENSE.md or https://github.com/libjpeg-turbo/libjpeg-turbo/blob/master/LICENSE.md
This software is based in part on the work of the Independent JPEG Group.

Limitations:
- no B frames
- constant QP
- only raw nv12 or nv16 input and bytestream output
- ... many more

Usage:
     "Options:\n"
     "-h | --help          Print this message\n"
     "-c | --count         Number of frames to grab\n"
     "-C | --compress      Number of frames to compress\n"
     "-d | --device name   Video device name [%s]\n"
     "-o | --outputMarks   Print outputs marks\n"
     "-q | --quality       Jpeg enc quality\n"
     "-r | --raw           Number of raw frames to save\n"
     "-p | --preview       Preview frames to display\n"
     "-f | --reafFile      Get frame from file\n"
     "     --yuvToRgb      Convert yuv4:2:2 -> rgb -> jpeg\n"
     "     --jpegEnc       Select software or hardware jpeg encoder (Default: 1)\n"
     "                      | 0 -> disable  |\n"
     "                      | 1 -> SW       |\n"
     "                      | 2 -> HW       |\n"
     "     --scale         Scale source image to (Default: none):\n"
     "                     NOTE: it must be uset with HW encoder.\n"
     "                      | 0 -> none                  |\n"
     "                      | 1 -> ARBITRARY-SCALER-VGA  |\n"
     "                      | 2 -> ARBITRARY-SCALER_QVGA |\n"
     "     --h264Enc       Enable h264Encoder (Default: Disabled)\n"

Examples:
- Take 100 frames from tvd and show it on display:
$ ./sunxi-tvin2jpeg --p -c 100

- Take 100 frames, show it on display and compress first 10 frames to jpeg by SW encoder and libjpegturbo:
$ ./sunxi-tvin2jpeg --p -c 100 -C 10 -q 85

- Take 10 times frame from file, convert it from yuv_4:2:2_nv12_UVUV to RGB and compress it to jpeg:
$ ./sunxi-tvin2jpeg --p -c 10 -C 10 -q 85 --readFile /tmp/frame_000_yuy2_720x576.raw --yuvToRgb

- Take 200 frames from tvin, scale frame with hw veisp from PAL 720x576 to QVGA 320x240, 
compress it to jpeg with hw encoder, compress it to h264.
$ time ./sunxi-tvin2jpeg --p -c 200 -C 200 -q 85 --jpegEnc 2 --scale 2 --h264enc --o
You will have to mux the raw stream to a container to add fps information.
$ ffmpeg -r 25 -i /tmp/tvin.h264 -vcodec copy /tmp/tvin.mp4
Play it with vlc, ffplay etc...
