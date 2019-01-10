John Pierre's roadfollower, ported to QNX by J. Nagle

26 JAN 2004 -- J. Nagle

Basic road-following functions work OK.

Note that reading the picture from the server CAN interfere with reading
road follower output, despite the comments to the contrary.  Each
takes up a frame.

Logging to video generates poor-quality video, due to ffmpeg compression
problems.