## Project: Kinematics Pick & Place
[//]: # (Image References)

[fk-demo]: ./misc_images/fk-demo.png
[inverse-position]: ./misc_images/inverse-position.png
[robot-diagram]: ./misc_images/KR210-diagram.jpg
[project-launch]: ./misc_images/project-launch.png
[r3_6]: ./misc_images/R3_6-arbitrary.png
[t0_g]: ./misc_images/T0_G.png
[wc-eq]: ./misc_images/wc-equation.png
[ik-debug]: ./misc_images/ik-debug.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

![Forward Kinematics Demo][fk-demo]

I obtained the DH parameters table by sketching the diagram of KR210 and do an analysis on it:

![KR210 Diagram Analysis][robot-diagram]

Then deriving the DH parameters table:
Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | d1 | q1
1->2 | - pi/2 | a1 | 0 | q2 - pi/2
2->3 | 0 | a2 | 0 | q3
3->4 | - pi/2 | a3 | d4 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | - pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0 | 0

Then by using the informations from the `kr210.urdr.xacro`, I derived the value of the constant parameter to be the followings:
- a1 = 0.35: `joint2` x translation
- a2 = 1.25: `joint3` z translation
- a3 = -0.054: `joint4` z translation
- d1 = 0.75: `joint1` z translation + `joint2` z translation
- d4 = 1.50: `joint4` x translation + `joint5` x translation
- d6 = 0.303: `joint6` x translation + `gripper_joint` x translation

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Individual tranformation matrices and the a homogenenous transform from the base link to the gripper can be obtained by the following code:
```python
# Assuming sympy and mpmath have been imported
# Create symbols
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') # theta_i
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
	   
# Create Modified DH parameters
s = {alpha0:     0, a0:      0, d1:  0.75,
     alpha1: -pi/2, a1:   0.35, d2:     0, q2: q2-pi/2,
     alpha2:     0, a2:   1.25, d3:     0,
     alpha3: -pi/2, a3: -0.054, d4:  1.50,
     alpha4:  pi/2, a4:      0, d5:     0,
     alpha5: -pi/2, a5:      0, d6:     0,
     alpha6:     0, a6:      0, d7: 0.303, q7: 0}

# Defining function to return a transformation matrix
def transform_matrix(alpha, a, d, q):
    T = Matrix([[           cos(q),           -sin(q),           0,             a],
                [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                [                0,                 0,           0,             1]])

    return T

# Creating individual transformation matrices
T0_1 = transform_matrix(alpha0, a0, d1, q1).subs(s)
T1_2 = transform_matrix(alpha1, a1, d2, q2).subs(s)
T2_3 = transform_matrix(alpha2, a2, d3, q3).subs(s)
T3_4 = transform_matrix(alpha3, a3, d4, q4).subs(s)
T4_5 = transform_matrix(alpha4, a4, d5, q5).subs(s)
T5_6 = transform_matrix(alpha5, a5, d6, q6).subs(s)
T6_G = transform_matrix(alpha6, a6, d7, q7).subs(s)

# Homogeneous Transform
T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G

```


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

##### Inverse Position

First, I need to obtain the position of the wrist center. Given only gripper position and its pose, I could obtain the equation of the wrist center as the following:
![Arbitrary Homogeneous Transform][t0_g]
![Wrist Center Equation][wc-eq]

The value of **n** can be obtained by calculating the matrix homogeneous matrix by the following code:
```python
R_G = Rz.subs({y: yaw}) * R_y.subs({p: pitch}) * R_x.subs({r: roll}) * R_corr # R_G stands for the rotation position of the gripper
```
where R_x,y,z are rotation matrices ans R_corr is a rotation matrix to correct the difference in pose between gripper's reference frame in DH and in URDF.

After obtaining the wrist position, I can obtain `theta1`, `theta2`, & `theta3` trigonometrically as follow:

![Inverse Position Trigonometry][inverse-position]

Assuming wrist center is in the form of a vector [WCx, WCy, WCz]:
- theta1 = atan2(WCy, WCx)
- theta2 = pi/2 - angle_a - (angle in radian formed by lines in x and y axis from `joint2` to `WC`)
  - angle_a is obtained with cosine law
- theta3 = pi/2 - angle_b - (angle in radian formed by line from `joint3` to WC and the x axis)
  - angle_b is obtained with cosine law

##### Inverse Orientation

Obtaining `theta4`, `theta5`, & `theta6` is conducted by solving the equation derived from comparing the arbitrary R3_6 matrix, which is a rotation matrix from link 4 to the gripper, with its resultant, which is obtained by multiplying the `R_G` matrix with the inverse of `R0_3` from the left. The following is the arbitrary matrix R3_6 obtained from the code `R3_6 = T0_3[0:3,0:3].inv("LU") * T0_G[0:3,0:3]`:
![R3_6 Rotational Matrix][r3_6]

The equation for `theta4-6` would then be the followings:
- theta4 = atan2(R3_6[2,2], -R3_6[0,2])
- theta5 = atan2(sqrt(R3_6[1,0]*R3_6[1,0] + R3_6[1,1]*R3_6[1,1]), R3_6[1,2])
- theta6 = atan2(-R3_6[1,1], R3_6[1,0])

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

My code for doing inverse kinematics is inside the service request handler of `IK_server.py`. In general, I placed code for variables and calculations that don't depend on gripper's pose and position to be outside the loop. Also, I follow a suggestion from the lesson to make composition matrix after subtituting the symbols, where posible, to optimize the code execution.

The first part of the code is defining the symbols and individual transformation matrices (only the essential ones), while also declaring the correctional matrix for the difference in gripper's reference frame. Then, the inverse kinematics to obtain `theta`s for each pose is inside the loop.

For each loop, firstly I obtain the position of the wrist center (which is `joint5`) by the method discussed at section about kinematics analysis above. Then I conducted inverse position to determine `theta1-3` by trigonometry analysis as also discussed at the above section. Finally, I conducted inverse orientation to obtain `theta4-6` by solving the equations derived from comparing R3_6 in arbitrary value with the resultant one.

![Launching the project][project-launch]

After running the pick and place simulation, I found that my code does the job, but it takes a long time to complete. In the future, I would like to shorten the time on which the code is executed.

Further, the following is the result of the IK debug on my code and it's executing test case number 1:
![Result on IK Debug][ik-debug]
