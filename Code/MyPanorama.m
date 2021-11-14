function [pano] = MyPanorama()    %% SET UP
clear;
N_Best = 300;
MATCH_THRESHOLD = .5;
RANSAC_THRESHOLD = 8;
N_Max = 10000;
INLIER_MULTIPLIER = 0.9;
global PLOT_FIG;
PLOT_FIG = false;

%For Detect Corners
%detectCorners('Images/Input/Set1/3.jpg',1);
%detectCorners('../Images/Input/Set1/2.jpg',2);
%detectCorners('../Images/Input/Set1/3.jpg',3);


%     image_set = '../Images/test_images/test_images/*.jpg';
image_set = '../Images/train_images/Set1/*.jpg';
path = dir(image_set);
%     img_path = '../Images/test_images/TestSet1/';
img_path = '../Images/train_images/Set1/';
total_images = length(path);
pano = imread(fullfile(path(1).folder, path(1).name));


%% Implementation
for i = 2:total_images
    %% ANMS
    
    I1 = pano;
    I2 = imread(fullfile(path(i).folder, path(i).name));
    
    [x_best1,y_best1] = ANMS(rgb2gray(I1), N_Best);
    [x_best2,y_best2] = ANMS(rgb2gray(I2), N_Best);
    
    %% Feature Descriptor
    H = fspecial('gaussian', 40);
    
    D1 = featureDescriptors(x_best1,y_best1, H, I1);
    D2 = featureDescriptors(x_best2,y_best2, H, I2);
    
    %% Feature Matching
    [matchedPoints1, matchedPoints2] = featureMatching(x_best1,...
        y_best1,x_best2,y_best2,D1, D2,MATCH_THRESHOLD);
    if PLOT_FIG
        figure
        hold on
        showMatchedFeatures(I1, I2, matchedPoints1, ...
            matchedPoints2, 'montage');
        title("Feature Matching w/o Ransac")
        hold off
    end
    
    %% RANSAC to estimate Robust Homography
    percent_inlier = length(matchedPoints1 * INLIER_MULTIPLIER);
    [inliers1, inliers2, H] = ransac(matchedPoints1, ...
        matchedPoints2, N_Max, RANSAC_THRESHOLD,percent_inlier);
    
    if PLOT_FIG
        figure
        hold on
        showMatchedFeatures(I1, I2, inliers1, inliers2, 'montage');
        title("Feature Matching w Ransac")
        hold off
    end
    
    %% Blending Images
    % SOURCE: https://www.mathworks.com/help/vision/ug/
    %           feature-based-panoramic-image-stitching.html
    imageSize(i,:) = size(I2);
    
    % Estimate the transformation between I(n) and I(n-1).
    %         tforms(i) = estimateGeometricTransform(inliers2, inliers1,...
    %             'projective', 'Confidence', 99.9, 'MaxNumTrials', 2000);
    
    tforms(i) =  invert(projective2d(H.'));
    
    % Compute T(n) * T(n-1) * ... * T(1)
    tforms(i).T = tforms(i).T * tforms(i-1).T;
    
    pano = I2;
end
%% Blending Images
% SOURCE: https://www.mathworks.com/help/vision/ug/
%           feature-based-panoramic-image-stitching.html

% Compute the output limits  for each transform
for i = 1:numel(tforms)
    [xlim(i,:), ylim(i,:)] = outputLimits(tforms(i),...
        [1 imageSize(i,2)], [1 imageSize(i,1)]);
end

% compute the average X limits for each transforms
%     and find the image that is in the center.
avgXLim = mean(xlim, 2);

[~, idx] = sort(avgXLim);

centerIdx = floor((numel(tforms)+1)/2);

centerImageIdx = idx(centerIdx);

% apply the center image's inverse transform to all the other
Tinv = invert(tforms(centerImageIdx));

for i = 1:numel(tforms)
    tforms(i).T = tforms(i).T * Tinv.T;
end

% Initialize Panorama
for i = 1:numel(tforms)
    [xlim(i,:), ylim(i,:)] = outputLimits(tforms(i), ...
        [1 imageSize(i,2)], [1 imageSize(i,1)]);
end

maxImageSize = max(imageSize);

% Find the minimum and maximum output limits
xMin = min([1; xlim(:)]);
xMax = max([maxImageSize(2); xlim(:)]);

yMin = min([1; ylim(:)]);
yMax = max([maxImageSize(1); ylim(:)]);

% Width and height of panorama.
width  = round(xMax - xMin);
height = round(yMax - yMin);

% Initialize the panorama.
panorama = zeros(height, width, 3);

% Create the Panorama
% Use imwarp to map images into the panorama
% Create a 2-D spatial reference object defining
% the size of the panorama.
xLimits = [xMin xMax];
yLimits = [yMin yMax];
panoramaView = imref2d([height width], xLimits, yLimits);

im_warps    = [];
%     im_warps_B  = [];
%     im_warps_RB = [];


binary_im_warps    = [];
%     binary_im_warps_B  = [];
%     binary_im_warps_RB = [];

% Get the H transformation for each image's pixel
for i = 1:total_images
    I = imread(img_path+string(i)+'.jpg');
    warpedImage = imwarp(I, tforms(i), 'OutputView', panoramaView);
    %         [B,RB]= imwarp(I, tforms(i), 'OutputView', panoramaView);
    if PLOT_FIG
        figure
        hold on
        imshow(warpedImage);
        title ("Wrapped Image : " + i)
        hold off
    end
    mask = imwarp(true(size(I,1),size(I,2)), tforms(i), ...
        'OutputView', panoramaView);
    
%         [Bin_B,Bin_RB]= imwarp(true(size(I,1),size(I,2)), tforms(i), ...
%             'OutputView', panoramaView);
    
    if PLOT_FIG
        figure
        hold on
        imshow(mask);
        title ("Binarry Wrapped Image : " + i)
        hold off
    end
    
    im_warps{end + 1} = warpedImage;
    %         im_warps_B{end + 1} = B;
    %         im_warps_RB{end + 1} = RB;
    
    binary_im_warps{end + 1} = mask;
    %         binary_im_warps_B{end + 1} = Bin_B;
    %         binary_im_warps_RB{end + 1} = Bin_RB;
end

for i = 1 : height
    for j = 1 : width
        sum_pix = double(zeros(1,3));
        count = 0;
        for warp = 1 : length(im_warps)
            if binary_im_warps{warp}(i,j,:)
                sum_pix(:,1) = sum_pix(:,1) + ...
                    double(im_warps{warp}(i,j,1)); % R pixel of all 3,2,1
                sum_pix(:,2) = sum_pix(:,2) + ...
                    double(im_warps{warp}(i,j,2)); % G pixel of all 3,2,1
                sum_pix(:,3) = sum_pix(:,3) + ...
                    double(im_warps{warp}(i,j,3)); % B pixel of all 3,2,1
                count = count + 1;
            end
        end
        
        if count ~= 0
            avg_pix = sum_pix ./ count;
        end % else remains 0 0 0 or black
        panorama(i,j,:) = avg_pix;
    end
end

figure
hold on
imshow(uint8(panorama))
title("Panorama")
hold off


pano = panorama;
end

%% Feature Description
% https://www.mathworks.com/help/images/ref/cornermetric.html
function detectCorners(imagePath,num)
    hold on;
    I = rgb2gray(imread(imagePath));
    
    filter = [0.25 0.5 0.25];
    C = cornermetric(I,'FilterCoefficients',filter);
    corner_peaks = imregionalmax(C);
    corner_idx = find(corner_peaks == true);
    [r,g,b] = deal(I);
    r(corner_idx) = 255;
    g(corner_idx) = 0;
    b(corner_idx) = 0;
    RGB = cat(3,r,g,b);
    C_adjusted = imadjust(C);
    figure(num);
    montage({RGB});
    %montage({RGB})
    %Laying out a comparison how the corners are being detected. We are
    %basically merging the original image with the adjusted corner matrix
    %points, which results to the third image in the output
    hold off;
end

%% ANMS
function [x_best,y_best] = ANMS(I, NBest)
disp("IN ANMS")
global PLOT_FIG;

filter = [0.25 0.5 0.25];
cornerScoreImg = cornermetric(I,'FilterCoefficients',filter);

features = imregionalmax(cornerScoreImg);

% Getting size of image and inverting
[y_size, x_size] = size(I);

x = [];
y = [];

% Finding N_strong
for i = 1:y_size
    for j = 1:x_size
        if(features(i,j) == 1)
            x = [x; j];
            y = [y; i];
        end
    end
end

N_strong = size(x, 1);
r = Inf(N_strong, 1);

% Finding distance
for i = 1:N_strong
    for j = 1:N_strong
        if cornerScoreImg(y(j), x(j)) > cornerScoreImg(y(i), x(i))
            ED = (x(j) - x(i))^2 + (y(j) - y(i))^2;
            if (ED < r(i))
                r(i) = ED;
            end
        end
    end
end

% Sort radius in descending order
[~, idx] = sort(r, 'descend');

% FROM N STRONG GET THE N BEST
x_best = x(idx(1:NBest));
y_best = y(idx(1:NBest));

if PLOT_FIG
    figure
    imshow(I)
    hold on
    plot(x_best,y_best,'.',"Color",'r');
    title("ANMS")
    hold off
end
end

%% FEAUTE DESCRIPTOR
function descpritor = featureDescriptors(x_best,y_best, H, I)
disp("IN FEATURE DESCRIPTOR")

descpritor = [];
[N_best, ~] = size(x_best);
% pad for corner cases
I = padarray(I, [20 20], 0, 'both');
%     [h,w] = size(I);
for i = 1:N_best
    
    x = x_best(i) + 20;
    y = y_best(i) + 20;
    %         if x < 20 || y < 20 || x + 20 > h || y + 20 > w
    %             continue
    %         else
    
    % 40 x 40 centered on x,y
    patch = I(y-20:y+20, x-20:x+20);
    patch_filter = imfilter(patch, H, 'replicate');
    
    img_descpritor = imresize(patch_filter, [8 8]);
    img_descpritor = double(reshape(img_descpritor, [64,1]));
    
    % standardize
    img_descpritor = img_descpritor - mean(img_descpritor);
    img_descpritor = img_descpritor / std(img_descpritor);
    
    descpritor = [descpritor img_descpritor];
end
end

%% FEATURE MATCHING
function [match1,match2] = featureMatching(x_best1,y_best1,x_best2,...
    y_best2,d1, d2,threshold)
match1 = [];
match2 = [];
[N_best_1, ~] = size(x_best1);
[N_best_2, ~] = size(x_best2);
disp(N_best_1);
disp(N_best_2);


for i = 1:N_best_1
    for j = 1:N_best_2
        sumSqr(j) = sum((d1(:,i) - d2(:,j)).^2);
    end
    [RadiusValue, RadiusIdx] = sort(sumSqr);
    if (RadiusValue(1)/RadiusValue(2) < threshold)
        match1 = [match1 ; [x_best1(i),y_best1(i)]];
        
        match2 = [match2 ; [x_best2(RadiusIdx(1)), ...
            y_best2(RadiusIdx(1))]];
    end
end
end

%% RANSAC
function [inliers1, inliers2, H] = ransac(match1, match2, N_max, ...
    threshold, percent_inlier)
disp("IN RANSAC");
population = length(match1);

inliers_set = [];
inliers_set_max = [];

H = [];
i = 0;
while (i < N_max) && (length(inliers_set_max) < percent_inlier)
    r_idx = randi([1, population], 4, 1);
    
    x = [match1(r_idx(1),1); match1(r_idx(2),1);  ...
        match1(r_idx(3),1); match1(r_idx(4),1)];
    y = [match1(r_idx(1),2); match1(r_idx(2),2);  ...
        match1(r_idx(3),2); match1(r_idx(4),2)];
    
    X = [match2(r_idx(1),1); match2(r_idx(2),1);  ...
        match2(r_idx(3),1); match2(r_idx(4),1)];
    Y = [match2(r_idx(1),2); match2(r_idx(2),2);  ...
        match2(r_idx(3),2); match2(r_idx(4),2)];
    
    H_est = est_homography(X, Y, x, y);
    
    for j = 1:population
        [X, Y] = apply_homography(H_est, match1(j,1), match1(j,2));
        % p1' - Hpi
        difference = [match2(j,1), match2(j,2)] - [X, Y];
        difference = difference.';
        ssd = sum(difference.^2);
        if ssd < threshold
            inliers_set = [inliers_set j];
        end
    end
    
    if length(inliers_set) > length(inliers_set_max)
        inliers_set_max = inliers_set;
        H = H_est;
    end
    
    inliers_set = [];
    i = i + 1;
end

inliers1 = match1(inliers_set_max, :);
inliers2 = match2(inliers_set_max, :);
sprintf('Ransac finished on %d\n',i);
end

%% Vision Blender (not used)
% https://www.mathworks.com/help/vision/ug/feature-based-panoramic-image-stitching.html
% blender = vision.AlphaBlender('Operation', 'Binary mask', ...
%     'MaskSource', 'Input port');  
% 
% % Create a 2-D spatial reference object defining the size of the panorama.
% xLimits = [xMin xMax];
% yLimits = [yMin yMax];
% panoramaView = imref2d([height width], xLimits, yLimits);
% 
% % Create the panorama.
% for i = 1:numImages
%     
%     I = readimage(buildingScene, i);   
%    
%     % Transform I into the panorama.
%     warpedImage = imwarp(I, tforms(i), 'OutputView', panoramaView);
%                   
%     % Generate a binary mask.    
%     mask = imwarp(true(size(I,1),size(I,2)), tforms(i), 'OutputView', panoramaView);
%     
%     % Overlay the warpedImage onto the panorama.
%     panorama = step(blender, panorama, warpedImage, mask);
% end
% 
% figure
% imshow(panorama)