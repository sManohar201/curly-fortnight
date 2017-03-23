# curly-fortnight
Who's a good jackal - an object tracking jackal.  

Objective for this project.

* ROS package for object tracking. 
* Launch file to bring up PS3 camera to publish images.
* Choose an object for the robot to track
* Subscriber node that takes in the image from the image publisher above and identifies the object.
* Develop a control law for tracking the object via the threshold mask.
   1. The controller should attempt remain the same approximate distance from the object.
   2. Controller must include a hysteresis/thresholding element ensuring that visual feedback results in smooth movement/tracking of the object.
* Upon pressing the "circle" button on the PS3 controller the should begin tracking your object. Tracking should stop when the "X" button is pressed.  

## Team Members:
* Haden Wasserbaech
* Phillip Scramlin
* Sabari Manohar (Team Lead)  

## For development execute the following commands
1. Clone the repository to a desired location  
      `git clone git@github.com:sManohar201/curly-fortnight.git`
2. `cd curly-fortnight/catkin_ws && catkin_make` 
3. `source devel/setup.bash`
4. `source ~/remote-jackal#.sh`

Now ssh into the jackal and follow the remaining commands
1. Copy the repository into jackal.
2. `cd curly-fortnight/catkin_ws`
3. `source devel/setup.bash`
4. `roslaunch obj_track control_start.launch`  

Once you run the roslaunch command you should be seeing output as shown below.
Markup : ![picture alt](https://lh3.googleusercontent.com/-bl1HROZ1L7g/WNMOGujefnI/AAAAAAAAACs/JWicm1txaCEjbuFuG-QxsaTVNEdKvX5_ACL0B/h987/proj6_fortnight00.png)
