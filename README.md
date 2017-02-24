tvin2jpeg Licenses
======================

Proof of Concept TV-IN and hardware accelerated H264-JPEG encoder and JPEG decoder for sunxi.
Example app for TV-IN, captures N frames from /dev/video1 (tvin cvbs), send output to hdmi or parallel LCD. 
Compress it to jpeg images /tmp/testImageXXX.jpg and/or h264 saving the raw h264 stream to /tmp/tvin.h264 
and mux it to mkv container /tmp/tvin.mkv. After each jpeg-encoded frame, jpeg decoder decode same 
/tmp/testImageXXX.jpeg image and show it on dispplay (right up position).
This is a basic example for using the tvin, H264-JPEG hardware encoder and JPEG decoder for sunxi SoCs.
This example includes:
- Using analogue input TV decoder (CVBS).
- Compress tvin to jpeg using sw libjpegturbo.
- Hw scaling. Scale input PAL 720x576 to VGA or QVGA.
- Simultaneous HW encoding jpeg, h264 and decoding jpeg with some kind of time slicing.
- Simultaneous jpegEnc, jpegDec, h264Enc, h264Dec.
It is just a proof of concept and not recommended for production use!

Edit:
Based on PoC sunxi-tvin by Enrico Butera: https://github.com/ebutera/sunxi-tvin
Based on PoC jpeg encoder by Manuel Braga: https://gitorious.org/recedro/jepoc
But gitorious has been closed down and therefore the link "https://gitorious.org/recedro/jepoc" is no longer working.
Based on PoC h264 encoder and jpeg decoder by Jens Kuske: https://github.com/jemk/cedrus

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
     "\nUsage: %s [options]\n"
     "Version %s\n"
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
     "     --jpegEnc       Enable jpeg encoder.\n"
     "                     Select software or hardware jpeg encoder:\n"
     "                         | 1 -> SW       |\n"
     "                         | 2 -> HW       |\n"
     "     --scale         Scale source image to (Default: none):\n"
     "                         NOTE: It can be only used with HW encoder.\n"
     "                         | 0 -> none                  |\n"
     "                         | 1 -> ARBITRARY-SCALER-VGA  |\n"
     "                         | 2 -> ARBITRARY-SCALER_QVGA |\n"
     "     --h264Enc       Enable H264 Encoder (Default: Disabled)\n"
     "     --jpegDec       Enable jpeg Decoder (Default: Disabled)\n"
     "                         Note: It can be only used with jpeg encoder.\n"
     "     --h264Dec       Enable H264 Decoder (Default: Disabled)\n"


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
Play it with vlc, ffplay etc...

- Take 500 frames, show frame on display by sunxi display engine in smaller resolution 542x359 
position x30y121 (use display scaler), encode frame to jpeg, decode same frame from jpeg to raw, 
show decodec frame on display (right up position 200x150 x580y121), encode frame from camera to h264 
/tmp/tvin.h264 and mux it to mkv container /tmp/tvin.mkv.
Do all this operation simultaneous 25 fps and ~20% CPU.
$ ./sunxi-tvin2jpeg_h264 -c 500 -C 500 -q 85 --jpegEnc 2 --scale 1 --p --h264enc --jpegDec

- All four operation simultaneous:
(The source for h264 decoder is raw h264 bitstream)
$ time ./sunxi-tvin2jpeg_h264 -c 300 -q 85 --jpegEnc 2 --scale 2 --jpegDec --h264Dec /tmp/tvin_2.h264
