## Project: Search and Sample Return

### Writeup / README

### Notebook Analysis
#### 1. Run the functions provided in the notebook on test images (first with the test data provided, next on data you have recorded). Add/modify functions to allow for color selection of obstacles and rock samples.

The function to find and add obstacle is in Color Thresholding section. I use color_thresh function to get the threshold and mask, and using np.absolute to generate the obstacle map.

The function to find rocks is added in Calibration Data section. I compare the pix color to yellow to figure out if there is a rock or not.

#### 1. Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles and rock samples into a worldmap.  Run `process_image()` on your test data using the `moviepy` functions provided to create video output of your result. 

In process_image(), since I do not change the resolution, I use the same source and destination coordinations which I learn from courses to transform the perspective. Then I apply the color threshold to the 


### Autonomous Navigation and Mapping

#### 1. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these functions were modified as they were.


#### 2. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your writeup.  

Here I'll talk about the approach I took, what techniques I used, what worked and why, where the pipeline might fail and how I might improve it if I were going to pursue this project further.  


![alt text][image3]
