# HARP in HM Decoder
This is a small tool based on HM-14.0 with a modified decoder in order to visualize the HEVC block structure.

The visualization code from HARP (https://lms.lnt.de/HARP/) has been added to the decoder in order to create images in the HARP fashion. It is contained in the folder 'visualize'.

## Eclipse
Project files for eclipse are contained:
- .project
- .cproject

## How to build
Either import the project to Eclipse. Build settings should work out of the box.

Or navigate to build/linux and call `make` there (or `make -j4 release` for a release build using 4 threads)

Test run: Execute testskript.sh and check the jpg folder for the created images afterwards.
Note that the sequence 'PartyScene' from ClassC consists of three identical pictures at the beginning when analyzing the results. ;-)