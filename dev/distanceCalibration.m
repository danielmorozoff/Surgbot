load('distance_calibration_images.mat')
fixed= rgb2gray(v1);
moving = rgb2gray(vyx);
[optimizer, metric] = imregconfig('monomodal');
tform = imregtform(moving, fixed, 'translation', optimizer, metric);

