# ee106-final-project

## Goal: The end goal of this project was to showcase how robots can interact with humans, and play games. 

## How: 
This is an interesting project because it transcends the boundaries of the physical world and the digital world. By getting a robot to play a game in the real world, it showcases how we can use robots to create a form of entertainment (a dice game in this case). The problem solved was making the robot see with it's camera and use that information to pick up the dice. 

### How do we get the robot to know where the dice are?
We set up a configuration that is ready to pick up a dice, so the sawyer bot is ready and sets its camera up to view where the dice is. Once it’s all set up, the bot uses opencv’s contouring to figure out where the dice is. Upon doing that, we translate the picture’s coordinates and shift the robot left, or right according to where the dice is so that the dice ends up in the center of the camera view. Once the dice is at the center, we initiate the pick up sequence.


### In what real-world robotics applications could the work from your project be useful?
The way we approached the problem was very unique and that that approach can be used in other applications. Instead of translating to cartesian coordinates using relativity makes the problem a lot more simple, so we stuck to the robot’s frame, rather than the world frame since it made the problem a lot more simple. 


## Design: 
### What design criteria must your project meet? What is the desired functionality? 
- ** Computer Vision **: to read, recognize, and locate the dice
- ** Consistency: ** since we plan on making a game, we want to be able to have consistent success so we made checkpoints that happen along the problem to ensure we're on the right path. 

### What design choices did you make when you formulated your design? What trade-offs did you have to make?
** Path Finding **: We created our own path finding solution, by moving closer and closer to the dice to create a checkpoint where we can then pick up the dice. 
Tradeoffs: 
    - 
    - 
**Robot(s)**: We at fist were ambitious and wanted to use multiple robots, but we found it easier to just use a single robot to simulate a full game. Though it adds to the entertainment aspect, but could be a possible project for the future.

## Implementation
### Describe any hardware you used or built. Illustrate with pictures and diagrams. • What parts did you use to build your solution? 

We used a sawyer with the default gripper. For rolling the dice we 3D printed a container. 	![cup we 3d printed](/imgs/cup.jpeg)


### Describe any software you wrote in detail. Illustrate with diagrams, flow charts, and/or other appropriate visuals. This includes launch files, URDFs, etc. 
** Reading the dice **:
- We started with using concepts of Lab 6 to read dice numbers on still images
- OpenCV functions to find the dice and count circular blobs
- Outputs a number to add to total count
- Started to use camera to capture images to be processed

#### How it works: 
0. A picture is taken from the video stream
	![Normal black and white picture of dice](/imgs/1.jpeg)
1. We first apply a background remover using a threshold number. Then blur it to remove noise.
	![Gaussian Blurred picture of dice to reduce noise](/imgs/2.jpeg)
2. Then we use [] to identify the clusters
	![Opencv's contour to find the dice](/imgs/3.jpeg)
3. From the clusters we apply a box identifier, with a threshold of how large it can be to eliminate the frame rectangle.
	![Creating boxes around the dice](/imgs/4.jpeg)
4. After individual dice are identified a snapshot of each dice is taken. Then zoomed in to have more accurate counts of the dice
	![Dice counting](/imgs/5.jpeg)


** Picking up the dice **:

The high level overview of our project works like the following. From a fixed starting position, a container with a dice is picked up. The rolling motion is a preset set of instructions. Then the robot moves so that the rolled dice is in frame. It takes a few seconds to read the dice and locate the dice. Then it determines its location and picks up the dice

To pick up the dice, we use a controller. 
- We got the bot to get the dice into the center of the frame, and record it’s x and y position
- Look at the error of where the dice is and the center of the image, and that tells us how much to adjust in order to center the dice in the middle
- We do this via x and y positions relative to the camera, and we maintain a constant height
- Once we got that, we use that position to tell it where to grab with a constant offset from the camera position

## Results

### How did it end up? 
In the end we completed our project so that it could play a dice game with a singular dice. It is successful with rolling the dice and identifying the dice numbers

![Two Birds playing](/imgs/demo.gif)


