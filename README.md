# soccer_computer_vision
By Han Vakil and Liv Dawes

## Introduction 
The goal of our project is to have a Neato play soccer. To do this, we are going to have a Neato identify a ball and a goal, find their position in the world relative to the Neato, drive into a position lined up behind the ball, and drive into the ball, bumping it towards the goal. 

## Methodology 
The first step was identifying the ball and goal. The ball we used was bright orange and the goal was a blue square. We used a color mask to isolate the ball and the goal independently on screen. This gave us a binary image where the ball or goal appears in white and everything else is black. Then, we used a contour finding function in opencv to identify the largest blob of white pixels on screen. We assume that the largest blob is the object we are interested in, which eliminates noise. Next, for the ball, we drew a minimum area circle around the blob and for the goal we drew a bounding box around the blob. From these bounding boxes we were able to get the pixel coordinates of the object on screen as well as the size on screen. 

Next, we need to figure out where the objects are in real life based on where they are on screen. First, the robot searches for objects by spinning in a circle. Once the robot sees an object on screen, it uses proportional control to slowly turn until the object is centered. At this point, we use odometry to get the heading of the robot which is the angle to the object. Then we compare the size of the object on screen to a known calibration curve to determine the distance to the object. This gives us polar coordinates that we can convert to cartesian coordinates with trig. 

Finally, we drew a line through the goal and ball and then identified a point a certain distance behind the ball. This point is the kicking point, the point that we will drive to to kick the ball. We turn to the point, drive to it, turn to the ball, and then drive at the ball at full speed to kick it. 


## Design Decisions
One design decision we had to make when working on our project was how we’d actually identify the ball and goal from the camera feed. We could have started finding all the bounded shapes with edge detection, and identified the ball and the goal from those shapes. We could also have used one of the functions that came with opencv (such as Hough circle transform) as our initial step on image processing and used color masking on the result of that to find our objects. 

We decided to start with color masking then find contours from there, after reading past reports and discovering the troubles they had starting with edge detection. It seemed it’d be significantly more computationally expensive and prone to error than starting with color masking.


## Challenges
One of the challenges we faced was the laser beam of doom. In other words, in the MAC, the lighting is really inconsistent. We found that even small things, like a reflective door opening and closing, was enough to shift the light and throw off our color mask. 

Another, similar challenge was changing lighting conditions in the hallway outside the CompRobo room. We chose our initial color masking values based on conditions during a sunny day, which didn’t work when the sun started to go down or when the lights in the hallway turned on. We used HSV color mapping instead of RGB to improve robustness against changing lighting conditions, which helped some but wasn’t fully able to overcome it. To fix this, we checked the mask every time the robot started behaving oddly, then manually recalibrated it if necessary.

A side effect of the poor lighting was that it reduced the number of pixels that the color mask identified, making the ball appear smaller on screen than it actually was. In turn, this caused our calibration curve to estimate that the ball was farther away than it actually is. 

A hardware problem we faced was the slow speed of the neato. Even if the neato drove towards the ball at full speed to try to hit it into the goal, it wasn’t able to hit the ball fast enough for it to reach the goal. 


## Improvements 
If we had more time, one of the first things we would do would be to improve our detection of the ball. Right now, we primarily rely on color masking which is inconsistent. Adding an additional detection method like corner detection or edge detection would help make our algorithm more robust. 

Another stretch goal we had for this project was to detect a moving ball. Being able to detect and track a moving ball would allow a more realistic play experience than just kicking a stationary ball. 

A third goal would be better finding objects in the world. We were able to find the heading to our goal and ball with reasonable accuracy, but the distance calculations weren’t very precise (as a result of an imperfect mask). We could have used a combination of the neato’s lidar and computer vision techniques to make the distance finding work better.


## Take-aways
One of the primary take-aways from this project was the importance of proper lighting in computer vision systems. With our color-masking based approach, lighting had a huge impact on what color was detected and if the object could be seen. As we worked in the evening while the sun went down, we would have to frequently adjust our masking settings. Going forward, with future computer vision projects, it is important to either frequently re-calibrate according to lighting or to build a system robust to different lighting changes. 

Given the complexity and numerous failure points of robotics systems, we learned it’s important to have quick methods to figure out where failures are occurring. The computer would frequently stop receiving any input from the neato camera, and the lighting would frequently change enough that the mask no longer worked, which had entirely different remedies. Being able to figure out which was going wrong (or if it was something else entirely) meant we could fix the problem in thirty seconds as opposed to ten minutes of troubleshooting.

The importance of testing code was also a significant lesson learned in this project, both in standalone functions and in implementations on the robot. We’d write code in our python files and test it on separate images/in its own function, but that wouldn’t always mean it would work on the actual robot. The image format could be slightly different, or there could be an edge case that we missed, so it’d usually take a fair amount of debugging on the actual implementation before some previously-written code would work. On the other hand, testing the code out separately from the robot meant it could be tested much faster, as we wouldn’t have to deal with all the issues that came with wiring up to the robot.

A final take-away is the importance of commenting out code. We did a lot of work together but we also did a fair amount of coding work asynchronously. It was frustrating to come back to new code having been written and not understanding why it was there and what it did. Writing code asynchronously can be really efficient and a good way to work together, but it relies on effectively communicating to each other what the code does. Otherwise, you end up with multiple functions that do almost the same thing, wasted time trying to reverse engineer the purpose of the code, or the same variables being named different things in different places. 

