## Project: Kinematics Pick & Place
### Taariq Hassan (thassan743)

---

[//]: # (Image References)

[image1]: ./misc_images/Kuka_DH.jpg
[image2]: ./misc_images/IK_123.jpg
[image3]: ./misc_images/IK_23.jpg

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

The image below shows the drawings, annotations and notes I made when determining the DH parameters for the Kuka KR210. My method pretty much followed the project videos for KR210 Forward Kinematics [1](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/398f60b1-b112-41ec-b8bc-59e95a2cec70), [2](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/7adb245d-28b0-4d52-a863-0c4c98e8da86) and [3](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/6b2a1c78-1efd-4c5c-8010-7952b57af9bb).

![alt text][image1]

From the above, and using the parameters from the URDF file, the following DH parameter table was derived.

i | alpha<sub>i-1</sub> | a<sub>i-1</sub> | d<sub>i</sub> | theta<sub>i</sub>
:---: | --- | --- | --- | ---
1 | 0 | 0 | 0.75 | q<sub>1</sub>
2 | -pi/2 | 0.35 | 0 | q<sub>2</sub>
3 | 0 | 1.25 | 0 | q<sub>3</sub>
4 | -pi/2 | -0.054 | 1.5 | q<sub>4</sub>
5 | pi/2 | 0 | 0 | q<sub>5</sub>
6 | -pi/2 | 0 | 0 | q<sub>6</sub>
7 (gripper) | 0 | 0 | 0.303 | 0

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

The code to determine the individual transformation matrices about each joint can be seen below. The name of each matrix is derived from the 2 links the joint connects. For example `T1_2` represents the transform matrix about joint 2 which connects links 1 and 2.

```
T0_1 = Matrix([[             cos(q1),            -sin(q1),            0,              a0 ],
			   [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1 ],
			   [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1 ],
			   [                   0,                   0,            0,               1 ]])
T0_1 = T0_1.subs(s)

T1_2 = Matrix([[             cos(q2),            -sin(q2),            0,              a1 ],
			   [ sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2 ],
			   [ sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2 ],
			   [                   0,                   0,            0,               1 ]])
T1_2 = T1_2.subs(s)

T2_3 = Matrix([[             cos(q3),            -sin(q3),            0,              a2 ],
			   [ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3 ],
			   [ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3 ],
			   [                   0,                   0,            0,               1 ]])
T2_3 = T2_3.subs(s)

T3_4 = Matrix([[             cos(q4),            -sin(q4),            0,              a3 ],
			   [ sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4 ],
			   [ sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4 ],
			   [                   0,                   0,            0,               1 ]])
T3_4 = T3_4.subs(s)

T4_5 = Matrix([[             cos(q5),            -sin(q5),            0,              a4 ],
			   [ sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5 ],
			   [ sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5 ],
			   [                   0,                   0,            0,               1 ]])
T4_5 = T4_5.subs(s)

T5_6 = Matrix([[             cos(q6),            -sin(q6),            0,              a5 ],
			   [ sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6 ],
			   [ sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6 ],
			   [                   0,                   0,            0,               1 ]])
T5_6 = T5_6.subs(s)

T6_G = Matrix([[             cos(q7),            -sin(q7),            0,              a6 ],
			   [ sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7 ],
			   [ sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7 ],
			   [                   0,                   0,            0,               1 ]])
T6_G = T6_G.subs(s)
```

The above matrices can be simplified by substituting the DH parameters determined earlier.

The transform from base_link to gripper_link can be determined by a simple multiplication of the above matrices as follows: (Note: additional rotations are needed to align the gripper_link frame to the world frame)
```
T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G
```

However, the homogeneous transform matrix from base_link to gripper_link can also be determined using only the position and orientation of the gripper_link.

```
T0_G = Matrix([[R0_G, Pxyz],
               [   0,    1]])
```

Here, `R0_G` represents the orientation of the gripper_link given by a rotation matrix, and `Pxyz` represents the position vector.

The rotation part of the transform (`R0_G`) is determined using the gripper_link's `roll`, `pitch`, `yaw` parameters as follows:

```
R_x = Matrix([[ 1,              0,          0],
			  [ 0,        cos(roll), -sin(roll)],
			  [ 0,        sin(roll),  cos(roll)]])

R_y = Matrix([[ cos(pitch),        0,  sin(pitch)],
			  [          0,        1,           0],
			  [-sin(pitch),        0,  cos(pitch)]])

R_z = Matrix([[ cos(yaw), -sin(yaw), 0],
			  [ sin(yaw),  cos(yaw), 0],
			  [ 0,              0,     1]])

R0_G = R_x * R_y * R_z
```

We therefore now have two equations for `T0_G`, one in terms of each joint angle and the robot parameters, and the other in terms of only the position and orientation of the end effector. This proves to be extremely useful when performing inverse kinematics as will be seen later.

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and Inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

The fact that the Kuka KR210 has a spherical wrist formed by joints 4, 5 and 6, the Inverse Kinematics problem can be split into Inverse Position and Inverse Orientation Kinematics. The Inverse Position Kinematics problem is the process of determining the first three joint angles, `theta1`, `theta2` and `theta3` to place joint 5 (the wrist centre) at the correct position. The Inverse Orientation problem is then to orient the wrist correctly (by calculating `theta4`, `theta5` and `theta6`) such that the end effector is pointing the right way.

We will start with the Inverse Position Kinematics.

The first step is to notice that the position of the end effector is not the same as the position of the wrist centre. The wrist centre is at joint5. Therefore, the position of the wrist centre can be calculated by subtracting the distance from the gripper_link to joint 5. However, this subtraction needs to take into account the orientation of the wrist. Luckily we calculated the homogeneous transform matrix from base_link to gripper_link previously. Therefore, the position of the wrist centre is calculated as follows:
```
WC = Matrix([px, py, pz]) - d7 * R0_G * Matrix([1,0,0])
```

Here, `px`, `py`, `pz` are the end effector positions, `d7` is the length from joint5 to the gripper_link and `R0_G` is the rotation part of the transform.

Now that we have the position of the wrist centre, we can proceed to calculate the first three joint angles, `theta1`, `theta2` and `theta3`. The image below shows a simplified drawing of the current problem.

![alt text][image2]

From the above image we can see that `theta1` can be calculated quite easily as follows:
```
theta1 = atan2(WC[1],WC[0])
```

If only `theta2` and `theta3` were that simple!

To calculate `theta2` and `theta3`, [this video](https://youtu.be/llUBbpWVPQE?t=3m39s) and the awesome Slack community helped tremendously. The image below summarizes the method I followed.

![alt text][image3]

With the above image as reference, we calculate `theta3` by constructing a triangle from joint2 to joint3 to joint5 (wrist centre). Then, using the cosine rule, we can calculate `alpha`, then `beta` and finally `theta3`. The need for `alpha` and `beta` is due to the offset of 0.054m between joints 3 and 4. Thus, `theta3` was calculated as follows (Note: the code below does not directly reflect what is in my `IK_server.py` file. I changed the variable names here to match the above image for clarity.):
```
#Calculate new WC position relative to joint2 origin O2 in the plane of the arm.
WC_x = sqrt(WC[0]**2 + WC[1]**2) - a1
WC_z = WC[2] - d1

#Calculate the lengths of the sides of the triangle formed by joints j2,j3,j5
#This is used to calculate theta3 using the cosine rule.
r = sqrt(WC_x**2 + WC_z**2)
b = sqrt(a3**2 + d4**2)
c = a2

D = (r**2 - b**2 - c**2) / (2 * b * c)

#Calculate angles
alpha = atan2(sqrt(1-D**2), D)
beta = atan2(abs(a3), d4)
theta3 = alpha - beta - pi/2
```
Here, angles `alpha, beta, theta3` and lengths `r, b, c` are as shown in the image. All other parameters are from the DH table.

Then, `theta2` can be calculated (again, refer to image and above code for notation):
```
alpha1 = atan2(WC_x, WC_z)
beta1 = atan2(c * sin(alpha), a2 + c * cos(alpha))
theta2 = alpha1 - beta1
```

It is worth mentioning that there are two possible solutions for `theta3` and therefore `theta2` known as the 'elbow up' and 'elbow down' configurations. This is also discussed in the video linked to above. For the purposes of this project, only the 'elbow up' configuration was used.

So now that `theta1`, `theta2`, and `theta3` have been calculated, the Inverse Position Kinematics part of the problem is complete. The Inverse Orientation Kinematics problem is then to determine the remaining three joint angles. To do this, we make use of the two equations for `T0_G` determined previously. Since we now know the first three angles, we can substitute these into the first equation for `T0_G`. We are therefore left with only 3 unknowns, the last three joint angles. Lets go ahead and derive the formula used to determine the remaining joint angles (Note: I only used the rotation part of the transform matrices since we are only concerned about orientation. Also note: I will use `R0_G_q` to denote the matrix in terms of the angles and `R0_G_eep` to denote the matrix in terms of the end effector pose. Lastly, this is not the exact code I implemented but rather a derivation of the equation used to provide clarity.):
```
R0_G_q = R0_G_eep

#Split LHS into 2 matrices
R0_3_q * R3_G_q = R0_G_eep

#'Divide' both sides by R0_3_q
R0_3_q.inverse() * R0_3_q * R3_G_q = R0_3_q.inverse() * R0_G_eep
R3_G_q = R0_3_q.inverse() * R0_G_eep
R3_G_q = R0_3_q.transpose() * R0_G_eep
```

We can see from the above that the LHS has three unknows, `theta1`, `theta2`, and `theta3` and the RHS is known. The three remaining joint angles were then calculated using a simple call to the `euler_from_matrix` function in the `tf.transformations` library (again, thanks to the Slack community for this). Note that further axis rotation corrections were applied to R3_6 before calling this function to transform back into the gripper frame.
```
theta4, theta5, theta6 = tf.transformations.euler_from_matrix(np.array(R3_6).astype(np.float64), "ryzy")
```

The final step was to impose limits on each joint angle, taking into account the physical robot. The range of motion for each joint was obtained from the URDF file. The calculated joint angles were then clipped to be within this range.

We now have values for all joint angles and have thus completed the Inverse Kinematics problem.

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

Most of my code has been discussed in the previous sections of this report. However, I will briefly go through the two sections, namely the Forward Kinematics and Inverse Kinematics, and discuss anything that was not mentioned. I will then also discuss the shortfalls and possible improvements of my code.

##### Forward Kinematics
The Forward Kinematics steps start with defining the DH parameter and joint angle symbols, as well as the DH parameter table, in lines 32-48. The individual transformation matrices are then defined in terms of these symbolic parameters and DH parameters.

In lines 93-97, the matrix multiplications are performed to calculate the total homogeneous transform, `T0_G` as discussed earlier. The transform from base_link to link3, `T0_3`, is also calculated here but is only used in the Inverse Kinematics step as was discussed. When calculating `T0_G`, the simplify step was removed. It was found that simplifying the matrix took extremely long and removing this step made the FK run about 20x faster with no noticeable loss of accuracy.

Lastly, in lines 99-111, the difference in orientation of the gripper_link frame is taken into account. This consists of performing two rotations to the gripper frame by multiplying the total homogeneous transform matrix by two additional transform matrices.

Finally, it is worth mentioning that the above Forward Kinematics steps were moved outside of the `for` loop in the `handle_calculate_IK` function. This was done so that the loop could execute faster. Since the Forward Kinematics matrices are declared symbolically, there is no need to re-calculate each loop. The matrices can just be evaluated with different parameters within the loop.

##### Inverse Kinematics
All the Inverse Kinematics steps are executed within the `for` loop, which starts on line 113. The pose of the end effector is first decomposed into individual position and orientation parameters. In lines 132-143, the rotation matrix of the end effector, `R0_G` is constructed. The position of the wrist centre can then be calculated in line 146.

Following this was the calculation of `theta1`, `theta2` and `theta3` on lines 148-179, using the method described in detail previously. One addition to the code was to check the value of `D` used in the calculation of `theta3`. In some cases, `D` would exceed 1 which resulted in the value of `theta3` being a complex number. In this case, `D` was limited to 1.

`theta4`, `theta5` and `theta6` were then calculated on lines 181-201 as discussed earlier using the `euler_from_matrix` function. Lastly, the limits to the joint angles were imposed on lines 203-210.

##### Shortfalls and Improvements
There are still a number of issues that I have found with the code.

Firstly, the Kuka arm in the simulator still runs a lot slower than I would like. This is likely due to a number of external factors such as the virtual machine and the simulator. I allocated 3 cores, 6GB memory and 1GB graphics memory to the simulator and it still ran pretty slowly. The speed of the Kinematics calculations could still be increased in a number of ways such as moving away from `sympy` entirely and just using `numpy` for the matrix calculations. The effect this has on accuracy would need to be investigated however. With regards to the VM and simulator, I haven't spent any time investigating whether the performance could be optimized. So short of upgrading the host machine or doing a native install of Ubuntu, there is not much I can do on that front.

Due to the slow speed of the simulator, I did not record 10 pick/place cycles.

I have not investigated what causes the value of D, used in the `theta3` calculation, to exceed 1. Limiting the value to 1 does not seem to have severe effects on the performance/accuracy of the arm. Therefore, whether the value of D should be allowed to exceed 1, and how to handle it, needs to be determined. Also, the code should be improved to allow for the 'elbow down' configuration of the arm as well since this implementation only allows for 'elbow up'.

The limits imposed on the joint angles were based solely on the constraints defined in the URDF file for the Kuka arm. This does not take into account any constraints the environment or the workspace imposes on the robot. Therefore, this could be explored as a future improvement.

Last but not least, a recording of a single pick/place cycle can be viewed [here](https://youtu.be/nsaDQmVEe9M). Only a single cycles was recorded as the VM and simulator were just running way too slowly (**video sped up 10x**), and crashing regularly as well. Time taken for IK calculations can be seen in the terminal screen in the video.
