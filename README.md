# Computational-Photography-and-Computer-Vision-Final-Project

Kevin Shi 

CS435 - Computational Photography

Spring 2025

Final Project - Panoramic Stitching

---
My project implements an image stitching pipeline, starting from manually stitching images to automatically detecting keypoints, estimating a homography using RANSAC, and finally blending images into a panorama.

Features of my program:
*My program requires two images, where one is slightly offset from the other (via rotation and/or translation) and have significant overlap of content. I assume that the two images are the same size. In my code, img1 is the base image.*

1. First, I hard code four point correspondences and display the images side-by-side (as one image) with the point correspondences color coded as dots in the image.
2. Second, using the hard coded four point correspondences, I compute the transformation matrix, project, and blend the two images to stitch them together. 
3. Third, at this point, I want to automate the stitching process, so I extract keypoints for each image by looking for local maxima in the Difference of Gaussian (DoG) image. I then prune non-interesting keypoints by only keeping ones that are local maxima (in gradient magnitude), removing ones near the border of the image and that are near the bottom of the image, and removing keypoints that are in areas of low contrast. 
4. Fourth, I extract descriptors/representations of my images using 9x9 RGB patches, and then match the descriptors from one image to ones in the other with Euclidean distance. This results in a few matching keypoints.  
5. Fifth, to handle bad keypoint matches, I estimate a transformation matrix using RANSAC and get four good keypoint matches. I impose a constraint to have some distance between the keypoint matches. Lastly, I compute the transformation matrix, project, and blend the two images to stitch them together. 

The zip file contains three folders:
1. main_run/
	Images: img1.png, img2.png — pictures from near the Philadelphia Art Museum
	Script: main.m — Main script used for the project

2. test_run1/
	Images: lancaster1.png, lancaster2.png — 7-Eleven on Lancaster Ave
	Script: main_lancaster.m — Identical to main.m except for panoramic stitching using hard-coded correspondences (parts 1 and 2 of instructions)

3. test_run2/
	Images: rec_center1.jpeg, rec_center2.jpeg — Drexel Recreation Center
	Script: main_rec_center.m — Identical to main.m except for panoramic stitching using hard-coded correspondences (parts 1 and 2 of instructions)

---
Instructions to run my scripts:
1. In MATLAB, set the current folder to the appropriate subfolder so that the script can load the images in the same folder (main_run, test_run1, or test_run2).

2. Run one of the following scripts from the command line:
	main               —>    For the main run (Art Museum images)
	main_lancaster     —>    For the Lancaster Ave test run
	main_rec_center    —>    For the Rec Center test run

3. NOTE: Running Additional Tests
Depending on the additional testing images, it is likely that my code will potentially throw an error for the manual keypoint stitching (parts 1 and 2 of instructions) since my manual stitching is specific to my original image.
As a result, to run additional tests, I suggest you either comment out or delete lines 18-132 in main.m to only have the code for the automatic stitching pipeline. You should also change filename1 and filename2 at the top of main.m to the path of your images. In my code, filename1 is the base image. 

