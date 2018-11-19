#!/usr/bin/env python

import numpy as np
from numpy import array
from sympy import symbols, cos, sin, pi, simplify, sqrt, atan2
from sympy.matrices import Matrix

### Create symbols for joint variables
q1, q2, q3, q4, q5, q6, q7 = symbols("q1:8") # theta
d1, d2, d3, d4, d5, d6, d7 = symbols("d1:8")
a0, a1, a2, a3, a4, a5, a6 = symbols("a0:7")
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols("alpha0:7")

### KUKA KR210

# DH Parameters
s = {
        alpha0: 0, a0: 0, d1: 0.75, q1: q1, 
        alpha1: -pi/2, a1: 0.35, d2: 0, q2: -pi/2 + q, 
        alpha2: 0, a2: 1.23, d3: 0, q3: q3, 
        alpha3: -pi/2, a3: 0.054, d4: 1.50, q4: q4,
        alpha4: pi/2, a4: 0, d5:0, q5: q5, 
        alpha5: -pi/2, a5: 0, d6: 0, q6: q6, 
        alpha6: 0, a6: 0, d7: 0.303, q7: 0}


### Homogenenous Transforms
# base_link to link1

def getMatrix(alpha, a, d, q):
    return Matrix([
        [cos(q), -sin(q), 0, a]
        , [sin(q) * cos(alpha), cos(q) * cos(alpha), -sin(alpha), -sin(alpha) * d]
        , [sin(q) * sin(alpha), cos(q) * sin(alpha), cos(alpha), cos(alpha) * d]
        , [0, 0, 0, 1]])

T0_1 = getMatrix(alpha0, a0, d1, q1).subs(s)
T1_2 = getMatrix(alpha1, a1, d2, q2).subs(s)
T2_3 = getMatrix(alpha2, a2, d3, q3).subs(s)
T3_4 = getMatrix(alpha3, a3, d4, q4).subs(s)
T4_5 = getMatrix(alpha4, a4, d5, q5).subs(s)
T5_6 = getMatrix(alpha5, a5, d6, q6).subs(s)
T6_G = getMatrix(alpha6, a6, d7, q7).subs(s)

T0_2 = simplify(T0_1 * T1_2)
T0_3 = simplify(T0_2 * T2_3)
T0_4 = simplify(T0_3 * T3_4)
T0_5 = simplify(T0_4 * T4_5)
T0_6 = simplify(T0_5 * T5_6)
T0_G = simplify(T0_6 * T6_G)

# Correction Needed to Account of Orientation Difference Between Definition of Gripper_Link in URDF vs DH COnvention
R_z = Matrix([
    [cos(pi), -sin(pi), 0, 0]
    , [sin(pi), cos(pi), 0, 0]
    , [0, 0, 1, 0]
    , [0, 0, 0, 1]])

R_y = Matrix([
    [cos(-pi/2), 0, sin(-pi/2), 0]
    , [0, 1, 0, 0]
    , [-sin(-pi/2), 0, cos(-pi/2), 0]
    , [0, 0, 0, 1]])

R_corr = simplify(R_z * R_y)


#### Numerically evaluate transforms (compare this with output of tf_echo)
print("T0_1 = ", T0_1.evalf(subs={q1:0, q2:0, q3:0, q4:0, q5: 0, q6:0}))}
print("T0_2 = ", T0_1.evalf(subs={q1:0, q2:0, q3:0, q4:0, q5: 0, q6:0}))}
print("T0_3 = ", T0_1.evalf(subs={q1:0, q2:0, q3:0, q4:0, q5: 0, q6:0}))}
print("T0_4 = ", T0_1.evalf(subs={q1:0, q2:0, q3:0, q4:0, q5: 0, q6:0}))}
print("T0_5 = ", T0_1.evalf(subs={q1:0, q2:0, q3:0, q4:0, q5: 0, q6:0}))}
print("T0_6 = ", T0_1.evalf(subs={q1:0, q2:0, q3:0, q4:0, q5: 0, q6:0}))}
print("T0_G = ", T0_1.evalf(subs={q1:0, q2:0, q3:0, q4:0, q5: 0, q6:0}))}


# Total Homogeneous Transform between base_link and gripper_link with orientaition correction applied
T_total = simplify(T0_G * R_corr)
