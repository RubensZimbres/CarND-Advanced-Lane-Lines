## Advanced Lane Finding
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

The goals / steps of this project are the following:

* Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.
* Apply a distortion correction to raw images.
* Use color transforms, gradients, etc., to create a thresholded binary image.
* Apply a perspective transform to rectify binary image ("birds-eye view").
* Detect lane pixels and fit to find the lane boundary.
* Determine the curvature of the lane and vehicle position with respect to center.
* Warp the detected lane boundaries back onto the original image.
* Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.


[//]: # (Image References)

[image1]: ./output_images/Camera_test.png.png "Undistorted and Transformed image"
[image2]: ./output_images/src.png "Source Image"
[image3]: ./output_images/undistorted.png "Undistorted Image"
[image4]: ./output_images/combined_result.png "Combined Thresholding"
[image5]: ./output_images/Transformation.png "Transformation: Source and Destination Points"
[image6]: ./output_images/example_result.png "Example of the result"
[image]: ./output_images/ ""
[image]: ./output_images/ ""
[image]: ./output_images/ ""


# Writeup

All the code related to this project located in [this jupyter notebook](https://github.com/yosoufe/CarND-Advanced-Lane-Lines/blob/master/Advanced%20Lane%20Finding.ipynb)

## Camera Calibration:
The code related to camera calibration located in Camera Calibration section of [the notebook](https://github.com/yosoufe/CarND-Advanced-Lane-Lines/blob/master/Advanced%20Lane%20Finding.ipynb). It consists of the following steps:

0. importing calibration images (chessboard images)
0. initiation of object and image points list
0. converting to grayscale
0. finding corners as imagepoints and appending object points (constant). Corners of some the images could not be found because the corners were outside of the image or very close to the margine of the image. I used those images to test the model at the last step.
0. saving the resulting corner detection in an output folder over the original images
0. calibrate the opencv camera model using the object and image points
0. In order to test the model, we use the undistort function on a test image with the model gained above. This is in the section of "Test the Calibration" in [the notebook](https://github.com/yosoufe/CarND-Advanced-Lane-Lines/blob/master/Advanced%20Lane%20Finding.ipynb). Forexample the below image is showing the undistoring and transformation result using the model.

![alt text][image1]

## Pipeline:
The pipeline is starting from a source image shown below
![alt text][image2]

### 1. Undistorting Image:
First of all, using the camera model, I undistorted the image. Below is showing the undistorted one
![alt text][image3]

### 2. Perspective Transformation
Now I created some helper functions for the main lane finder function. First of all a function to calculate the transformation matrices from the camera view to the bird-eye view. For this purpose 4 points from one of the test images which were showing a flat strait roud has been chosen. The four corresponding point in destination image. This level is done manually by hard coding. I do  not used any automatic method to find these points. The following image demonstrates the four points in the source and transformed images. This part of the code located in Transformation section of [the notebook](https://github.com/yosoufe/CarND-Advanced-Lane-Lines/blob/master/Advanced%20Lane%20Finding.ipynb).

![alt text][image5]

and the points are:

| Source        | Destination   | 
|:-------------:|:-------------:| 
| 213, 705      | 341, 705      | 
| 1094, 705     | 1222, 705     |
| 686, 450      | 1222, 0       |
| 594, 450      | 341, 0        |

### 3. Creating Thresholded Binary Image
Next, there are some helper functions located in "Thresholding" section for thresholding over S or L channel, Sobel x or y, magnitude of sobel and direction of the sobel and comnimation of the filters. In that section in [the notebook](https://github.com/yosoufe/CarND-Advanced-Lane-Lines/blob/master/Advanced%20Lane%20Finding.ipynb) there are some images as practices with these filters. In order to have smaller report here I avoid the repeatition.

From Now on two approach is possible which each of them has its own pros and cons:

1. First do the colorspace transformation, thresholding and then image transformation to the bird-eye view. The detail is like this. First transformations using:
```
s_channel_bin, sobel_x, sobel_mag, sobel_dir, combined_image = grad_thresholding(
        img_undistorted,
        s_channel_thresh =(70,255),
        sobel_x_thresh=(30,255),
        sobel_mag_thresh=(15, 300),
        dir_thresh=(0,100),
        ksize=7)
```
Then warp transformation to get the eye-bird view on combined binary image. The combined binary image is obtained by combinig the thresholded binary image on magnitude of sobel and S_Channel:
```
warped1 = cv2.warpPerspective(combined_image, M, dst_size, flags=cv2.INTER_LINEAR)
```
* plus: It is usually more sensitive to the lanes far away.
* negative: It is noisier on area close to the car.

2. First do the image transformation to the bird-eye view and then colorspace transformation and thresholding. The detail is like this. First getting bird-eye view using on RGB undistorted image:
```
warped2 = cv2.warpPerspective(img_undistorted, M, dst_size, flags=cv2.INTER_LINEAR)
```
and then only using the sobel in the x direction on the transformed RGB image:
```
s_channel_bin2, sobel_x2, sobel_mag2, sobel_dir2, combined_image2 = grad_thresholding(
    warped2,
    s_channel_thresh =(30,255),
    sobel_x_thresh=(30,255),
    sobel_mag_thresh=(20, 300),
    dir_thresh=(0,100),
    ksize=15)
```

* Positive: At close area to the car has cleaner detection
* Negative: At far area it cannot detect anything

The final output that I used is a combination of these two. That means to get the warped binary image to find the lane pixles, a small portion of it from the top I used the first method and for the rest of the image I used the second method. This partitioning is happing using this code:
```
binary_warped = np.zeros_like(sobel_x2)
heigth_brd = int(binary_warped.shape[0]/3)
binary_warped[:heigth_brd,:] = warped1[:heigth_brd,:]
binary_warped[heigth_brd:,:] = sobel_x2[heigth_brd:,:]
```
You can see the result of this approach on the following imge. It stills can be better although. The three image on the bottom are showing the output using the first method, the second method and the combination method respectively from the left. As you can see the combined method can see the lane far away and it is cleaner in close ranges.

![alt text][image4]

### 4. Lane-line Pixel Detection
At this stage we have some filterd warped binary image. It is time to find the lane pixles.
There are some helper function in section "Lane-line Pixel Detection" in [the notebook](https://github.com/yosoufe/CarND-Advanced-Lane-Lines/blob/master/Advanced%20Lane%20Finding.ipynb).

This starts with creating a histogram of the lower part of the binary image. That means summing up the pixels in direct of y-axis. Then search fot the pixel index with the highest values at left and right half of the histogram. This is the starting point of searching for the lane-line pixels.

Then some windows with specific width and heigth are constructed to search for the pixels. The starting point is bottom of the image at the peak of the histogram. Nonezero pixels in these windows are considered as lane-line pixels and indecies of the pixels are stored in `left_lane_inds` and `left_lane_inds` variables. If the number of these pixels are more that specific value (here 50) the next window would be at middle of these pixels in x direction. In the next layer above, the new windows are used to find the lane-lines' pixels. And this approach is continues to the top of the image (y = 0). The windows are shown as well on the image of the previous section with green rectangles on bottom right. This functionality is implemented in `find_lane_from_histogram`.

There is another method. If a lane-line is detected, for the next frame we can use the same line and a margine to look for the lines' pixels. That is imeplemented in `find_lane_from_pre_frame`. In this case sometime when the road is not clear and noisy, one of the lanes overlap the other one it continuse to the  end of the video. In order to avoid that I wrote a fuction called `check_two_lines` which check if the distance of two lines at top of the image is smaller that 400 pixels in warped image, that means the lane detection is not good enough and it should use again the histogram method.

```
def check_two_lines(lane_fit):
    left = lane_fit[0]
    right = lane_fit[1]
    y = 0
    return np.polyval(right,y)-np.polyval(left,y)>400
```

### 5. Calculation of road curvature and car position deviation from the center of the road
This calculation is done in the helper function called `find_road_curv_car_pos`. The theory behind finding the radius of the curvature can be found [here](http://mathworld.wolfram.com/RadiusofCurvature.html) as well. I had twp 2nd order polynomial functions and based on those I calculated to curvatures and the average of them is reported on the image.

To calculate the distance from the center, I calculate the distance of two lanes at the bottom of the warped image. The I know from [here](https://en.wikipedia.org/wiki/Lane) that the standart lane width in High way is 12 ft or 3.7 inches. So the gain that turns the the pixel coordinate to real lane width is 3.7 devided by the distance of two lanes in pixels (meter/pixels).

```
def find_road_curv_car_pos(lane_fit, y ,img_width):
    left = lane_fit[0]
    right = lane_fit[1]
    left_r = (1 + (2*left[0]+left[1])**2)**(3/2)/abs(2*left[0])
    right_r = (1 + (2*right[0]+right[1])**2)**(3/2)/abs(2*right[0])
    gain_pix2dis =  3.7/(np.polyval(right,y)-np.polyval(left,y))
    dev_from_center = (np.polyval(right,y)+np.polyval(left,y))/2 - img_width/2
    dev_from_center *= gain_pix2dis 
    return (left_r+right_r)/2,dev_from_center
```
### 6. Example result

![alt text][image1]

## Resulting Videos
I have two videos to demostrate. On the first one, I never used the previous frame to detect the lane-lines of the current lane. On the second I used that information

First link: 

Second link: https://youtu.be/tPk_HAh3VOQ

## Discussion:
Fisrt of all hard coded transformation can not give a proper results in terms of the radius of the curvature in real worl meter. As you can see in the video when the car get a small pitch angle, the transformed lines are not anymore parallel. I propose that the transformation should be optimised online in order that the resulting lane-lines become paraller. Then the curvature calculation can be correct.

Secondly the perpesctive transformed image does not have enough resolution in far area. Better camera with higher resolution may make the it better but the computing time of the higher resolution image is more. Maybe it would be better to resize the image to lower resolution after the perspective transformation

Finally finding a good threshholds on different filter to ger a robust result on all situation is very difficult and I do not have any idea how to make it better. Maybe a Machine Learning approach would be better.
