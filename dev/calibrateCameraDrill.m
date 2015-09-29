vid =  imaq.VideoDevice('winvideo', 1,'MJPG_1920x1080');
videoPlayer = vision.VideoPlayer('Position', [0 0 1024  800]);
% Calculate difference image and display it.
frameCount=0;
while  frameCount < 10000
    % Get the next frame.
    frame = step(vid);
    frameCount = frameCount + 1;
    in_frame = insertShape(frame, 'circle', [1920/2 1080/2 1], 'LineWidth', 1);
    step(videoPlayer,in_frame);
end
