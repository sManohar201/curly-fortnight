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
ssh into the jackal and follow these commands
1. Copy the repository into jackal.
      `git clone git@github.com:sManohar201/curly-fortnight.git`
2. `cd curly-fortnight/catkin_ws && catkin_make`
3. `source devel/setup.bash`
4. `roslaunch obj_track start_cam.launch`  

Now from the local repository follow the remaining commands
1. Clone the repository to a desired location  
      `git clone git@github.com:sManohar201/curly-fortnight.git`
2. `cd curly-fortnight/catkin_ws && catkin_make` 
3. `source devel/setup.bash`
4. `source ~/remote-jackal.sh jackal5`
5. `roslaunch obj_track control_start.launch`

Once you run the roslaunch command, the output should be as shown below.
![picture alt](https://lh3.googleusercontent.com/-bl1HROZ1L7g/WNMOGujefnI/AAAAAAAAACs/JWicm1txaCEjbuFuG-QxsaTVNEdKvX5_ACL0B/h987/proj6_fortnight00.png)

At this point, moving the object in front of the camera should result in robot tracking the object.

## Points to note:

1. The choosen object's color should be similar to the one shown in the image. Otherwise the HSV of the respective object should be found and altered in the [track.py](https://github.com/sManohar201/curly-fortnight/blob/master/catkin_ws/src/obj_track/scripts/track.py) program, on lines 19 and 20.  
`lower_color = np.array([75, 25, 50])  
high_color = np.array([100, 255, 255])`  
2. If the object is brought too close to the camera, the tracking function might not work. It is advisable to keep the object at least 100cm away for the function to work properly. 

## Unfinished Face Tracking:

Unfinished python script for face tracking located in pdscraml branch. This script uses Pre-learned classifier xml files for detecting faces in a video.
