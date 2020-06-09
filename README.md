# final_demo

To get this demo to work, there are a few things that need to be installed

1. Must install webots, you can find that in this link: https://cyberbotics.com/ use that link and follow their directions based off of what OS you are running
2. Must have python 3.7 installed along with libraries: matplotlib

**Things you are aloud to do in this demo:**

* You can move the cars to a position you would like and change their angular orientations as well.
* To activate the tow car, press up on the keyboard which will move the dummy car and signal the tow car to come help
* To activate the fire car, press left to light up an LED on the dummy car, the fire car will then come to perform its job
* They can be activated without the other having to complete its task first, with **limitations!!!**

**LIMITATIONS**

Unfortunately, some features were able to make the cut in time, and some were not. So there exists major limitations in our demo
* Performing a specific task when the dummy car is very close to a wall will give the helper cars some trouble when first inspecting the car
* While the Dummy car is close to a wall, if a car is waiting for its job, it may sometimes choose to wait outside of a wall which is actually inconvenient for it, this can further be improved by using a different way to measure a effeciency in a spot other than distance
* Collision avoidance has caused some trouble in our implementation and can be tricky to implement well. For this reason, when cars are performing there trajectory from the initial state to the position of the Dummy car, I ask that you give the cars an ample amount of time until starting the next task. This is to prevent collisions as it is not fully implemented.

**Step by Step Guide (after downloading and installing)**
1. Open file worlds>demo.wbt
2. Press play if the world has not already started running
2. Configure the demo how you would like (refer to "Things you are aloud to do...")
3. Press keyboard inputs for which jobs you want to perform (fire, tow are implemented)
4. Wait and see how they perform. I **HIGHLY RECOMMEND: Setting the fast forward mode** in webots as at 1x speed it runs way too slow for anybody's patience. This is mainly due to motor velocity limitations in webots. 
5. Reset the world by pressing the << button. 
6. Repeat 2-6 until you are done toying with it. 

**What you can Expect**
* Fire car will go to the dummy car do its thing, then leave the scene to a far away place (hardcoded for now)
* Tow car will go to the dummy car, find the latch (webots specific) then connect to it. From there it will stop as it is currently not programmed to handle movement well with an additional car appended to it. 
* When both cars have jobs assigned, the tow car will wait for the fire car to finish first then it will do its job. 
**Be weary of many possible bugs**
