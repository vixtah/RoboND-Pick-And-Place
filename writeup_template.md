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
[image2]: ./misc_images/invKinematics.JPG
[image3]: ./misc_images/invKinematics.JPG

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

I first derived the DH parameters and extracted the values from the urdf file. Using the DH parameters, I was able to derive the joint transformations and solve for the forward kinematics of the system.

I was able to derive the angles for theta 1-3 by analysing the diagram using trigonometry. Theta 4-6 was a little harder to solve. I took R0_6 and multiplied the front by inverse(R0_3) and post multiplied by the R_corr to get R3_6. Then I printed out the syymbolic multiplication of R3_6 and used that to solve for theta 4-6. 

The first thign i noticed was the wrist spinning wildly in the simulator which seemed to slow it down significantly. I used a conditional on theta5 to try to reduce the movement on theta 4 and 6. It seemed to help slightly. I realized that collisions would only happen at the beginning and the end of the path. So I hardcoded for the wrist to be bent back (-3pi/4) to somewhat minimize profile while traveling and increasing the simulation speed significantly. I also noticed that the gripper was not closing all the way before the next step was executing when using continue so I added a sleep after the gripper closing code.

For the most part it successfully moves the objects to the trashbin from the shelf. However there is some strange behavior occasionally that I don't understand. Sometimes the gripper chooses to drop the block off at the very edge of the bin. It still works ore of the time, however I expect it should always drop it off right in the middle. 

If i had more time, I would try to debug the dropoff phenomenon a bit more. I would also try to come up with a better algorithm to minimize movemnt for the wrist angles during travel.
