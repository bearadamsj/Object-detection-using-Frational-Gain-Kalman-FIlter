function  background_frame  = BackgroundExt( input_video )
sample_step = 2;
vid = VideoReader(input_video);
%vid = VideoReader('sample.mov');
%frame = vid.read(inf);
Height = vid.height;
Width = vid.width;
nframes = vid.NumberOfFrames; %Number of frames
background = zeros(Height,Width,3); %Initial Background Image
%% First Stage: averaging over all of the background samples
for i=1:sample_step:nframes-sample_step
    background = background + double(read(vid,i));
end
background = sample_step*background/(nframes);
%imshow(uint8(background));
background_frame = background;
end
