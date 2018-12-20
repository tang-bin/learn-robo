# Project: Perception Pick & Place
---

## Exercise 1, 2 and 3 pipeline implemented
### 1. Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented.

Filtering and RANSAC plane fitting code is in the file `segmentation.py`.

The only problem is that after my RANSAC plane fitting, I can remove the desk top, but the desk's front side always shows. I adjusted the parameters but it doesn't work. Anyway, it looks like does not impact the final result after the recongnition.

### 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.

### 3. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.

These 2 implements are both in file `object_recognition.py`.

I did some small changes in `capture_features.py` to make the prediction a little more accurate. And the results look good.

![COnfusion Matrixes][img0]

## Pick and Place Setup

### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.

All code is in file `project_template.py`. I moved all major code from `object_recognition.py` to this file and added some more. Such as getting the dropbox and pick up positions, convert arm name from object group which read from parameters. The results look good and here the screenshots for each world.

![World 1][img1]

![World 2][img2]

![World 3][img3]

### Summary

When I tried the world 3 in the Pick and Place Setup part, I encountered couple of times that the recongnition method divided the data into 9 objects. It only happend 2 or 3 times and hard to duplicate, I cannot debug and figure out why it happened. And I never met the same problem in world 1 and 2.

I am feeling so frustrated that I don't have enough time to implement the robot motion part. I tried a little bit but cannot make it work, and today is the due date for this project, I have to summit what I have done and hurry to the next one. It's holiday season, I am worry a lot about the deadline. If I can complete the next project in time, I will back to all the missing challenges in the extended 4 weeks.

[img0]: ./imgs/img0.png
[img1]: ./imgs/img1.png
[img2]: ./imgs/img2.png
[img3]: ./imgs/img3.png
