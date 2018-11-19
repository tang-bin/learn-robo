# Project: Kinematics Pick & Place

---

[img1]: ./img/img1.png
[img2]: ./img/img2.png

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

Since all joints (besides base joint) type are revolute, so we can simply consider they are rotate along the given axes. So the joint figure will like:

![Figure 1: Joints][img1]

I use $J_i$ to indicate the joint, also line out the rotation axes. To transfer it to DH parameter frames, we can make all the rotation axes to Z axes, and translate the frame original. Then we can get this:

![Figure 2: DH paramter frames][img2]

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

Define transformation matrix like this:

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

Then correct the defferenece:
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
_To be honest I am not quit understant why we need to correct this. Looks like that from URDF definition to DH table, there is a 180deg rotation along Z axes and 90deg rotaion along Y axes, that's why we need to add this correction. But why can't we just change the definition in the URDF definition to make it match the DH table?_

### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's where you can draw out and show your math for the derivation of your theta angles. 


## Project Implementation

### Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:


