clear
delete(instrfind({'Port'},{'COM5'}));

addpath('dependencies/Sutter MP285');

% sutter_port = 'COM5';
% sutter = sutterMP285(sutter_port);

vid =  videoinput('winvideo', 1,'MJPG_1920x1080');

% Calculate difference image and display it.
ims = {};
distances = [.25,.5,1,2,3,5,7,8,8.5,9,10];
distances = distances*1000; %to microns

moveTo(sutter,[0 0 0]);
setOrigin(sutter);
%1080x1920
fixed = rgb2gray(getsnapshot(vid));

for dim = 1:2
    for i=1:length(distances)
       dis = distances(1,i);
        if(dim==1)
            pos = [dis 0 0];
        else
            pos=  [0 dis 0];
        end
        moveTo(sutter,pos); 
        im = getsnapshot(vid);
        
        ims{i,1}= im;
        ims{i,2} = distances(1,i);
    end
    [optimizer,metric]=imregconfig('monomodal');
    tform_calib = [];
    for i=1:length(distances)
        moving = rgb2gray(ims{i,1});
        tform = imregtform(moving,fixed,'translation',optimizer,metric);
        tform_calib = [tform_calib;tform.T(3,1:2)];
    end
    
    figure;
    if dim ==1
        calibx = tform_calib;
    else
        caliby=tform_calib;
    end
end
x_calibration = [0,distances(1,:);0,calibx(:,1)'];
y_calibration = [0,distances(1,:);0,abs(caliby(:,2)')]

p1_x = polyfit(x_calibration(2,:),x_calibration(1,:),1);
p1_y = polyfit(y_calibration(2,:),y_calibration(1,:),1);

lin_x = linspace(0,150); 
pol_x = polyval(p1_x,lin_x);
pol_y = polyval(p1_y,lin_x);

hold on;

plot(x,pol_x,'r--');
plot(x,pol_y,'g--');