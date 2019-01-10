This is the FFMPEG codec library, for reading and writing video.

This is an incomplete version. It is built by downloading FFMPEG
and configuring it with this command:

./configure --disable-v4l --disable-dv1394    --disable-audio-oss --disable-network --disable-ffserver --cc=qcc

and then performing a "make".  
Many of the features will not work, but enough works to be able to create logging video.

Just doing a "make" in this directory will build the whole thing.
"make install" will install it in our project directories.


J. Nagle
Team Overbot
November, 2003