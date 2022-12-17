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
- **Computer Vision**: to read, recognize, and locate the dice
- **Consistency:** since we plan on making a game, we want to be able to have consistent success so we made checkpoints that happen along the problem to ensure we're on the right path. 

### What design choices did you make when you formulated your design? What trade-offs did you have to make?
**Path Finding**: We created our own path finding solution, by moving closer and closer to the dice to create a checkpoint where we can then pick up the dice. 
Tradeoffs: 
    - 
    - 
**Robot(s)**: We at fist were ambitious and wanted to use multiple robots, but we found it easier to just use a single robot to simulate a full game. Though it adds to the entertainment aspect, but could be a possible project for the future.
