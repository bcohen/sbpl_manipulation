#SBPL Collision Checker

This version was created for my RSS 2014 project on [Planning Single-arm Manipulations with n-Arm Robots](http://www.cs.cmu.edu/~./maxim/files/planfornarms_rss14.pdf). In this project, we developed a motion planner that can compute a path for an arm or arms required to move an object from point A to point B. Some examples and a simple description can be seen in [the video](https://www.youtube.com/watch?v=rvYIqM0Ch1k).

##Why did we have to make a branch for the handoff planner?
###To improve self-collision against sometimes moving non-planning links
* The master branch can only represent non-planning links in the distance field. Recomputing the distance_field for one of the arms would be time consuming every time one of the other arms moves. Now, non-planning links can be represented as spheres OR voxels. Self-collision is performed between sphere groups by computing the euclidean distance between the spheres. 

  NOTE: With this feature, we can use this collision checker to represent an entire robot as spheres. This would obviously be helpful for mobile robots. We can actually clean up the hacked together [pr2_collision_checker](https://github.com/bcohen/pr2_collision_checker) now. 

  Config file for 2-arm PR2 is below. In this example, the body is represented as voxels and both arms are represented as spheres. 

```
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
    collision_links:
    ...
    
  - name: arm1
    type: spheres
    root_name: base_footprint
    tip_name: arm1_wrist_roll_link
    collision_links:
    ....
```

* As it turns out, computing kinematics (using the KDL) for the kinematic chains of the planning and non-planning links can be very expensive. Actually, computing the kinematics is so much more expensive than doing lookups in the distance field that there's no comparison! When motion planning for a single arm, we usually make the assumption that the rest of the robot remains static during the planned trajectory. So it would make sense to only compute the kinematic chains once for the non-planning links. Thus, we cache them. This is done in a somewhat hacky way for now. 
  
  Below are two versions of the two main collision checking functions. One of them takes in a vector of frames that the planner can store, once assigned by the function during the first call. It's up to the planner to clear the frames vector when any non-planning links move.

```
bool isStateValid(const std::vector<double> &angles, bool verbose, bool visualize, double &dist);
bool isStateValid(const std::vector<double> &angles, std::vector<std::vector<std::vector<KDL::Frame> > > &frames, bool verbose, bool visualize, double &dist);

bool isStateToStateValid(const std::vector<double> &angles0, const std::vector<double> &angles1, int &path_length, int &num_checks, double &dist, std::vector<std::vector<double> > *path_out=NULL);
bool isStateToStateValid(const std::vector<double> &angles0, const std::vector<double> &angles1, std::vector<std::vector<std::vector<KDL::Frame> > > &frames, int &path_length, int &num_checks, double &dist, std::vector<std::vector<double> > *path_out=NULL);
```



###To speed up collision checking in general
Now we support two groups of spheres for each link in a collision group. First the low-resolution collision model is checked for collisions and then the high-resolution model is checked only if the low-res model is in collision. Designing the low-res model is a balance between minimizing the number of spheres vs not making them too big or else they will always be in collision.

Now the config file for a link has "spheres" and "low_res_spheres":

```
    - name: upper_arm
      root: r_upper_arm_roll_link
      spheres: ua0 ua1 ua2 ua3
      low_res_spheres: lr_ua0 lr_ua1 lr_ua2
```      
  
NOTE: The low_res model of the attached_object is hacked together for now! It's completely hardcoded for a specific object. This was a last minute hack. I need to remove it and instead add a call to the "Spherizer" with a larger radius for the filler spheres...I'm afraid that it will do a poor job and be way too conservative to be useful in reality.




