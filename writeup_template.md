## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/kuka_diagram.jpg
[image2]: ./misc_images/invKinematics.png
[image3]: ./misc_images/invKinematics.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

![alt text][image1]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Ti_i+1|_|_|_
---|---|---|---
            cos(qi+1)|           -sin(qi+1)|            0|              ai+1
 sin(qi+1)*cos(alphai)| cos(q)*cos(alphai)| -sin(alphai)| -sin(alphai)*di+1
 sin(qi+1)*sin(alphai)| cos(q)*sin(alphai)|  cos(alphai)|  cos(alphai)*di+1
                  0|                  0|            0|              1


Homogeneous transform from Base to EE

Rx | _ | _ | _
--- | --- | --- | ---
1 | 1 | 0 | 0
0 | cos(roll) | -sin(roll) | 0 
0 | sin(roll) | cos(roll) | 0
0 | 0 | 0 | 1

Ry | _ | _ | _
--- | --- | --- | ---
cos(pitch) | 1 | sin(pitch) | 0
0 | 1 | 0 | 0 
-sin(pitch) | 0 | cos(pitch) | 0
0 | 0 | 0 | 1

Rz | _ | _ | _
--- | --- | --- | ---
cos(yaw) | -sin(yaw) | 0 | 0
sin(yaw) | cos(yaw) | 0 | 0 
0 | 0 | 1 | 0
0 | 0 | 0 | 1

Pos | _ | _ | _
--- | --- | --- | ---
1 | 0 | 0 | px
0 | 1 | 0 | py
0 | 0 | 1 | pz
0 | 0 | 0 | 1

Where End effector pose:
  position = (px,py,pz)
  orientation = (roll, pitch, yaw)

T0_G = Rz* Ry * Rx * Pos

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's another image! 

![alt text][image2]
![alt text][image3]


### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


