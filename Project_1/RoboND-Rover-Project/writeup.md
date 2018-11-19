## Project: Search and Sample Return

### Writeup / README

### Notebook Analysis
#### 1. Run the functions provided in the notebook on test images (first with the test data provided, next on data you have recorded). Add/modify functions to allow for color selection of obstacles and rock samples.

The function to find and add obstacle is in **Color Thresholding** section. I use the `color_thresh` function to get the threshold and mask and using `np.absolute` to generate the obstacle map.

The function to find rocks is in the **Calibration Data** section. I compare the pix color to yellow to figure out if there is a rock or not.

#### 2. Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles, and rock samples into a world map.  Run `process_image()` on your test data using the `moviepy` functions provided to create video output of your result. 

In `process_image()`, since I do not change the resolution, I use the same source and destination coordinations which I learn from courses to transform the perspective. Then I apply the color threshold to the processed image. I check the rock by its color threshold (dark yellow) using the function I add in the previous step. If it finds any rock, add it to the world map. 

### Autonomous Navigation and Mapping

#### 1. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and an explanation are provided in the write-up of how and why these functions were modified as they were.

**perception_step()**

In `perception_step()` function, I did the similar thing as in `process_image()`. I added the code to check if Rover's `roll` or `pitch` is too far away from zero to that will impact the accuracy of the mapping. I set the threshold to `0.3`, which means if `roll` or `pitch` is larger than `0.3deg` and less than `359.7deg`, the world map will not be updated. In this case, the fidelity will be around **80-85%**. I think it's fair enough for now.

I also tried to add some rotation function to rotate the image to balance the error which introduced by the rolling and pitching, but I found it does not work. With Mentor Carlos' help, I found that my approach is incorrect, there needs a much more complicated algorithm, so I cannot implement my thought this way, but I still leave my code in the file so that I can try to figure out the correct way in future.

**decision_step()**

I added/changed following parts in `decision_step()`.

1. **Against the wall**

    I considered making the Rover go against the right wall. So I change the way to compute the steer (I call it `rotation` in code). I set up three thresholds:
    ```
    rightThreshold = -30deg
    countThreshold = 100
    timeThreshold = 4
    ```
Then I compare the _"valid points"_ separated by the `rightThreshold`. If there are more than 100 (=`countThreshold`) points in the right part and there are four times more points in left, consider steering the Rover to the right. Otherwise, to the next step.

2. **Facing intersection**

    If Rover is not met to steer right, I separate all points to 3 parts: less than -17deg (right), larger than 17deg (left) and all others (mid), and count the points in each piece. If the right count and left count is much larger than the mid count (2 times), consider there is an obstacle in the middle of the road, or the Rover is facing an intersection, then I will steer the Rover to the right.
3. **Go back**

    Each time the current position `(x, y)` is cached as `prevPos`, and I added another int variable called `stuck`. Compare Rover's current position and the `prevPos`, if it's too short, for example, less than 0.01, consider the Rover is stuck and not moving. The variable stuck will plus 1. Once the stuck is larger than a threshold ( I use 100, which means Rover was not moving for about 5 seconds), change Rover's mode to `fallback` and try to move Rover back.

    I added another variable called `fallbackCount`, similar but reversed as `stuck`, set it to 60 at the beginning of being stuck, and minus one each time during the `fallback` mode. And in this mode, Rover will move back quickly.

    I also introduce another expression to compute the steer when going back:  
    ```
    -20 * (80 - Rover.fallbackCount) / 80
    ```
    There is no exact algorithm to prove this expression works, I composed it based on my animation programming experience, I tried to make the backward smoothly, and it looks work well enough.

    When `fallbackCount` reached 0, change Rover's mode to "forward" and move again.

4. **Go straight**

    If the forward `rotation` is less than `10deg` and larger than `10deg`, and it's not in the **"against right"** mode as I described in section 1, change the `steer` to 0, make Rover go straight. I believe this can cause the Rover to move a little faster and smoother in the long straight road.

#### 2. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your write-up.  

1. **Against the wall**

    It works but not good enough as I expected. The Rover can go against the right wall but always keep a much longer distance to the wall than I expected. That's the problem that I could not make the Rover close enough to the rock, although I already implemented the pickup code, it never works. But with this function, the Rover can always map the world in a reasonably high percentage **90 - 95%**.

2. **Facing intersection**

    In this part, the Rover can always pick the correct way when meeting an intersection. But for the obstacles, it does not work well. The Rover has still been stuck or crashes the small obstacles. For the big ones, the Rover will take them as the same situation as the intersection, so it works.

3. **Go back**

    This function works pretty well. It can always help the Rover to get out from the stuck position, no matter it's small obstacles, big obstacles, or the corner of the dead end.

4. **Go straight**

    It works, but anyhow, because of the function of **"against the wall,"** it does not work as I expected. It still swaggers sometime.

**Next**

I am not familiar with the Python programming language and the matrix and vector computing. I spend more than one week to go over them so that I can begin force on the project programming. I still have some thought, but due to the lack of the base and time, I cannot implement them for now. Hopefully, I could finish them in the future.

The biggest problem of my code as I know, for now, is that sometimes the Rover will circle all the time the big empty area, such as the center of the map, especially if circling always avoid the obstacles, it can never stop. I think one way to solve this problem is to add the `weight` to each mapped point, the lower the `weight` of one place, the less chance to go that way. I tried, but unfortunately, I cannot implement it successfully.

Another problem is that I cannot indicate the small obstacles. The Rover always crash and stick on the small obstacles, with the help of the go back function, it can eventually get out but waste lots of time.

