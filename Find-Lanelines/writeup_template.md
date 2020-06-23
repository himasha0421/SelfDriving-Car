# **Finding Lane Lines on the Road** 
---
**Finding Lane Lines on the Road**

The goals / steps of this project are the following:
* Make a pipeline that finds lane lines on the road

[//]: # (Image References)

[image1]: ./examples/grayscale.jpg "Grayscale"

---

### 1. My working  pipeline.

My pipeline consisted of 6 steps.
1. First images are converted into the gray scale image type
2. Use Canny edge detector for detect edges of the gray scale image
3. Define a feild of view for the lane lines and define a ROI for the image
4. crop the edge output using above ROI
5. Hough transform is used to refine the lane lines
6. extrapolate the lane lines for continous line segment

#### Jupyter notebook contain all the functions for above steps 

If you'd like to include images to show how the pipeline works, here is how to include an image: 

![alt text][image1]


### 2. Identify potential shortcomings with  current pipeline


* One potential shortcoming would be what would happen when the lane lines curavtuer having more deviation and hough transform is not capable of resulting the lane lines
* Another shortcoming could be this algorithm is not robust on illumination changes
* Major draw back on paramter tunning , because these paramters are not robust on real world scenarios


### 3. Suggest possible improvements to  pipeline

* A possible improvement would be to producing a parameter tunning pipline sperately
* Another potential improvement could be to use camera calibaration , undistort images , use birdeye view for curvatuer detection , use different colour space
