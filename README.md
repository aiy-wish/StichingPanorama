# Stitching Panorama

### Corners Detection and ANMS

In order to find the potential corners from the image, we first converted the
image to Black & White Image. We then used cornermetric() function with
filter values (which we got from manually rerunning the program and
observing) and then using imregionalMax() function to find N_strong
points. After detecting the corners, we use ANMS to filter out N_best filter
points from the N_strong.

### Feature Descriptors and Feature Matching
Now that we have found the N best feature points (i.e. N Best after using the
ANMS algorithm), we now need to define every feature point by a feature
vector.

We took a patch of 40x40 points centered for every feature point. Due to
some feature points being present towards the edges we choose to pad
our image with 0 in all direction. We proceeded to apply a Gaussian blur to
the patch using fspecial. We then took a subsample of size 8x8 which we
reshaped to obtain the feature vector of the size, 64 x 1. Lastly, we
standardized the feature vector, where mean is 0 and variance of 1 in order
to remove bias.

For feature matching, we used two images, I1 and I2 along with their
corresponding feature vectors, and the ratio between the
distances(constant). We compared each feature point from image 1 to
that of image 2 by using euclidean distances (computing the sum of square
difference between all points) and chose the two closest vectors after
sorting based on size. If the ration of the two vectors was less than our
threshold of 0.5 we would consider it as a feature match, from I1 to I2.

### RANSAC to estimate Robust Homography

We used RANSAC to better our results from feature matching to compute
the Homography Transformation matrix which we used later on in
constructing the panorama. The Ransac Algorithm, we followed was
1. Select four feature pairs (at random) from image 1 and from image 2.
2. Estimate the homography, H est using these 8 points.
3. Apply H est to all the 4 points on from Image 1
4. Compute the SSD between the resulting points and the 4 random points
from image 2
5. If the SSD value is less than our threshold of 5 consider them as inliers
6. Repeat till we reach our max iterations or if the number of inliers is less
than the maximum percentage of inliers (0.9 * N_best).
7. Recompute the H est on all inliers u


### Run
When run, `MyPanorama.m` must load a set of images from `*Images/input/*`, and return the resulting panorama.

