#!/bin/tcsh

# /CLUSTERHOMES/LMS/sequences/YUV/hevc

#set input_file = "/CLUSTERHOMES/LMS/sequences/YUV/hevc/ClassB/Kimono1_1920x1080_24.yuv"
#set input_file = "/CLUSTERHOMES/LMS/sequences/YUV/hevc/ClassC/BQMall_832x480_60.yuv"
set input_file = "/CLUSTERHOMES/LMS/sequences/hevc/ClassC/PartyScene_832x480_50.yuv"
#set input_file = "/CLUSTERHOMES/LMS/sequences/hevc/ClassC/RaceHorses_832x480_30.yuv"
#set input_file = "/CLUSTERHOMES/LMS/sequences/hevc/ClassC/BasketballDrill_832x480_50.yuv"
#set input_file = "/CLUSTERHOMES/LMS/sequences/hevc/ClassD/RaceHorses_416x240_30.yuv"
#set input_file = "/CLUSTERHOMES/LMS/sequences/hevc/ClassD/BlowingBubbles_416x240_50.yuv"
#set input_file = "/CLUSTERHOMES/LMS/sequences/hevc/ClassD/BasketballPass_416x240_50.yuv"
#set input_file = "/CLUSTERHOMES/LMS/sequences/hevc/ClassD/BQSquare_416x240_60.yuv"
#set input_file = "/CLUSTERHOMES/heindel/sequences/medical/YUV420_512x512/CT_ABDOMEN_23000RBZ7R_04_dicom420_8bit_8frames.yuv"
#set input_file = "/CLUSTERHOMES/heindel/sequences/ClassE/KristenAndSara_1280x720_60.yuv"


#set input_file = "/CLUSTERHOMES/heindel/sequences/PCS2013/ElFuente_1920x1080_2997fps_300f.yuv"



#set enc_conf_file = "cfg/encoder_intra_main.cfg"
set enc_conf_file = "cfg/encoder_lowdelay_P_main.cfg"
set bitstream = "str.hevc"
set qp = "32"
set width = "832"
set height = "480"
set framerate = "50"
set frames = "5"

#set GOPSize = "8"
#set IntraPeriod = "16"


set exe_string = "bin/TAppEncoderStatic -c "$enc_conf_file" -i "$input_file" -b "$bitstream" -f "$frames" -fs 0 -fr "$framerate" -q "$qp" -hgt "$height" -wdt "$width
#set exe_string = "$exe_string --IntraPeriod="$IntraPeriod" --GOPSize="$GOPSize

# für verlustlos
#set exe_string = "$exe_string --TransquantBypassEnableFlag --CUTransquantBypassFlagValue"


echo "Executing: "$exe_string
$exe_string

#create output folder for images...
mkdir -p jpg
# ...and clean it if necessary
rm jpg/*.jpg

bin/TAppDecoderStatic -b $bitstream # -o rec_dec.yuv
#bin/TAppDecoderStatic_visu -b $bitstream # -o rec_dec.yuv

