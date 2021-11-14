# CMSC426_Project2: Panorama Stitching

### Feel Free to add any links or resources to this README. I might have missed some important stuff.

## Deadline: 
    - 11:59:59 PM, October 20, 2020 (Eastern Standard Time)
    - 08:59:59 PM, October 20, 2020 (Pacific Standard Time)
    - 07:59:59 AM, October 21, 2020 (Gulf Standard Time)
    - 09:29:59 AM, October 21, 2020 (Indian Standard Time)

### Links
- Project Description
    - [Link](https://cmsc426.github.io/2020/proj/p2/)
- Lecture material 
    - [Basics](https://cmsc426.github.io/pano-prereq/) 
    - [Panorama Stitching](https://cmsc426.github.io/pano/) 
- Lecture Video
    - [Link](https://umd.zoom.us/rec/play/-SjV3iJ_48eQWBbUFjrtkB8Hr4FUifbF4rKQkSq9FU-bOg1-ypkxgr8w9E9P5wESKE9JYkSReCu5UVnX.Vpi7fF1tC5FZY0ah)

### TODO List for the Project

- [ ] Cylinder Projection -- IDK what this means (Avi)
- [X] Detect Corners (needs to be verified by others) -- this is not in rubric (Avi)
- [X] ANMS (in progress)
- [X] Feature Descriptors 
- [ ] Feature Matching
- [ ] RANSAC and Homography Estimation
- [ ] Image Warping (and Blending)
- [ ] Write Report

### Important notes about the project
- We are given 3 sets of training images
- Test images will be released before 24 hrs of deadline.
- Functions allowed are:
    - `imfilter`
    - `conv2`
    - `imrotate`
    - `im2double`
    - `rgb2gray`
    - `fspecial`
    - `imtransform`
    - `imwarp` and `imref2d`
    - `meshgrid`
    - `sub2ind`
    - `ind2sub `
    - Any plotting and matrix operation/manipulations

- When run, `MyPanorama.m` must load a set of images from `*Images/input/*`, and return the resulting panorama.

