# PickandPlace

  Pick and Place with noob arm and move it

  The design of the arm somehow does not allow cartesian planning to be precise. Alternatives approximations have been used, but to no degree are they accurate enough to be used.
  
  The robot works perfectly well with joint space commands, with properly tuned ros control. A similar issue was found here: https://answers.ros.org/question/341130/moveit-problem-fail-aborted-no-motion-plan-found-no-execution-attempted/
  
  This seems like a problem for either overly determined robot arms or poorly constructed ones.
  
  Pick and place works if joint trajectory control is used (Gazebo's friction properties pushed to the limit to achieve this.)
    
  Perception design was abandoned mid way seeing the problem of not being able to control the robot with cartesian co-ordinates.
  
  Perception design was based on calculating the centroid of the image that has the cube and estimating the pose by assuming z = 0.3 
  
  ```
  translation.x = (xc/width)*fov
  translation.y = (yc/height)*fov
  translation.z = 0.3
  
  orientation.x,y,z = 0.0
  orientation.w = 1.0
  ```
  Overall issue was caused by poor execution of the scara robot design.
  
  
