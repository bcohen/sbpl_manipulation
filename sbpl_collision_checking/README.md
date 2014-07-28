#SBPL Collision Checker

This version was created for my RSS 2014 project on [Planning Single-arm Manipulations with n-Arm Robots](http://www.cs.cmu.edu/~./maxim/files/planfornarms_rss14.pdf). In this project, we developed a motion planner that can compute a path for an arm or arms required to move an object from point A to point B. Some examples and a simple description can be seen in [the video](https://www.youtube.com/watch?v=rvYIqM0Ch1k).

##Why did we have to make a branch for the handoff planner?
###To improve self-collision against sometimes moving non-planning links
* The master branch can only represent non-planning links in the distance field. Recomputing the distance_field for one of the arms would be time consuming every time one of the other arms moves. Now, non-planning links can be represented as spheres OR voxels. Self-collision is performed between sphere groups by computing the euclidean distance between the spheres.

Config file for 2-arm PR2 is below. In this example, the body is represented as voxels and both arms are represented as spheres. 

collision_groups:

  - name: body
    type: voxels
    root_name: base_footprint
    tip_name: head_pan_link
    collision_links:
    ...
    
  - name: arm0
    type: spheres
    root_name: base_footprint
    tip_name: arm0_wrist_roll_link
    collision_links:
    ...
    
  - name: arm1
    type: spheres
    root_name: base_footprint
    tip_name: arm1_wrist_roll_link
    collision_links:
    ....
    
###To speed up collision checking in general
Now we support two groups of spheres for each link in a collision group. First the low-resolution collision model is checked for collisions and then the high-resolution model is checked only if the low-res model is in collision. Designing the low-res model is a balance between minimizing the number of spheres vs not making them too big or else they will always be in collision.

Now the config file for a link has "spheres" and "low_res_spheres":
    - name: upper_arm
      root: r_upper_arm_roll_link
      spheres: ua0 ua1 ua2 ua3
      low_res_spheres: lr_ua0 lr_ua1 lr_ua2
      
  


