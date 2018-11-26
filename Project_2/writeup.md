# Project: Kinematics Pick & Place

---

[joint]: ./img/img1.png
[DH]: ./img/img2.png
[rs]: ./img/img3.png
[theta2]: ./img/img4.png
[plan]: ./img/img5.png

## Kinematic Analysis
### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

From kr210.urdf.xarco we can find the joint definitions:

Joint | x | y | z | type | rpy | rotation
--- | --- | --- | --- | --- | --- | ---
base | 0 | 0 | 0 | fixed | 0 0 0 | None
1 | 0 | 0 | 0.33 | revolute | 0 0 1 | Z
2 | 0.35 | 0 | 0.42 | revolute | 0 1 0 | Y
3 | 0 | 0 | 1.25 | revolute | 0 1 0 | Y
4 | 0.96 | 0 | -0.054 | revolute | 1 0 0 | X
5 | 0.54 | 0 | 0 | revolute | 0 1 0 | Y
6 | 0.193 | 0 | 0 | revolute | 1 0 0 | X
gripper | 0.11 | 0 | 0 | fixed | 0 0 0 | None

Table 1: Joint definitions

Since all joints (besides base joint) type are revolute, so we can consider they rotate along the given axes. So the joint figure will like:

![Figure 1: Joints][joint]

I use $J_i$ to indicate the joint, also line out the rotation axes. To transfer it to DH parameter frames, we can make all the rotation axes to Z axes, and translate the framed original. Then we can get this:

![Figure 2: DH parameter frames][DH]

I also marked the joints in Figure 2 to compare with the DH frames.

Using the frame in Figure 2 and the values in Figure 1, we can get the DH paramter table:


Links | $\alpha_{i-1}$ | $a_{i-1}$ | $d_i$ | $\theta_i$
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | $q_i$
1->2 | -90 | 0.35 | 0 | -90 + $q_2$
2->3 | 0 | 1.25 | 0 | $q_3$
3->4 | -90 | -0.054 | 1.5 | $q_4$
4->5 | 90 | 0 | 0 | $q_5$
5->6 | -90 | 0 | 0 | $q_6$
6->EE | 0 | 0 | 0.303 | 0

Table 2: DH Parameter Table

### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Define a transformation matrix like this:

```python
def getMatrix(alpha, a, d, q):
    return Matrix([
        [cos(q),                -sin(q),                0,              a               ]
        , [sin(q) * cos(alpha),   cos(q) * cos(alpha),    -sin(alpha),    -sin(alpha) * d ]
        , [sin(q) * sin(alpha),   cos(q) * sin(alpha),    cos(alpha),     cos(alpha) * d  ]
        , [0,                     0,                      0,              1               ]])

```

Then we can get matrix for each joint:

```python
T0_1 = getMatrix(alpha0, a0, d1, q1)
T1_2 = getMatrix(alpha1, a1, d2, q2)
T2_3 = getMatrix(alpha2, a2, d3, q3)
T3_4 = getMatrix(alpha3, a3, d4, q4)
T4_5 = getMatrix(alpha4, a4, d5, q5)
T5_6 = getMatrix(alpha5, a5, d6, q6)
T6_G = getMatrix(alpha6, a6, d7, q7)
```

And the matrix between base_link (0) and gripper_link (G):

```python
T0_G = simplify(T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G)
```

Then correct the difference:
```python
R_z = Matrix([
    [cos(pi),   -sin(pi),   0,  0]
    , [sin(pi), cos(pi),    0,  0]
    , [0,       0,          1,  0]
    , [0,       0,          0,  1]])

R_y = Matrix([
    [cos(-pi/2),    0,  sin(-pi/2),     0]
    , [0,           1,  0,              0]
    , [-sin(-pi/2), 0,  cos(-pi/2),     0]
    , [0,           0,  0,              1]])

T = simplify(T0_G * R_z * R_y)
```
_To be honest I do not quite understand why we need to correct this. Looks like that from URDF definition to DH table, there is a 180deg rotation along Z axes and 90deg rotation along Y axes, that's why we need to add this correction. But why can't we change the definition in the URDF definition to make it match the DH table?_

### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

Since we have $WC = P - ^0_GR * d_G$, we can get WC position easily.

$\theta_1 = \arctan (WC_y / WC_x)$

To calculate $\theta_2$ we have this geometric diagram:

![Figure 3, Theta 2][theta2]

$\angle a = \arccos({S_b^2 + S_c^2 - S_a^2 \over 2 S_b S_c})$
$\angle e = arctan({WC_z - 0.75 \over \sqrt{WC_x^2 + WC_y^2} - 0.35})$
$\theta_2 = {\pi\over2} - \angle a - \angle e$
$\theta_3 = {\pi\over2} - \angle b - 0.036$

To calculate $\theta_{3,4,5}$ we need the transform matrix from $joint_0$ to $joint_3$ named $R_{0,3}$, suppose $r_{ij} = R_{0-3}[i, j]$, then:

$\theta_4 = \arctan({r_{22} \over -r_{02}})$
$\theta_5 = \arctan({\sqrt{r_{02}^2 + r_{22}^2} \over r_{12}})$
$\theta_6 = \arctan({-r_{11} \over r_{10}})$

## Project Implementation

### Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

In the `IK_server.py`, I just moved the code tested in `IK_debug.py` to it. At first, it didn't work well, so I tried to copy some of the code from demo video. Some of the issues were caused by my mistake, but some are so wired. For example, in my code I calculated $S_a$ like `Sa = sqrt(1.5 ** 2 + 0.054 ** 2)`, I could not get the current $\theta_3$. If I use `Sa = 1.501` which I copied from the demo video, it works pretty well. I don't understand why but it seems to work this way.

Here the result I tested my code, I got 10/10 successful pick and place.

![Figure 4, result][rs]

### Problems

The major problem is that the speed is so slow. It's not about the animation but the plan. Check this:

![Figure 5, bad plan][plan]

To reach the lower left cell, this plan did not go to the target position directly. Instead, it moved the arm with a long and inefficient circle route. Since in the `IK_server.py` we only calculate the positions, I am not sure how can I change my code to improve it.

Other problems were due to my lack of English and related acknowledge. I have been learning and working by using Chinese for many years, some of the basic math definitions, even they are very simple, I cannot get the meaning at the beginning. For example, I spend several days to realize that "normal" is another way to say "orthonormal," which is a familiar concept I learned in middle school (maybe I am wrong again).

I also met a few errors in class. The figure to calculate a2 and d2 are inversed in the 2nd guide video. In the IK algorithm description, the $X$ distance from $J_2$ to $WC$ should minus 0.35 which is the offset from $J_0$ to the $J_2$. Also, there is no introduction for "joint_state_publisher" and I did not know how to start it in RViz.

