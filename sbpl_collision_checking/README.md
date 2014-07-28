#SBPL Collision Checker

It's very simple. The "planning links" are represented as spheres and those spheres are checked against a distance field. All other links on the robot and all objects in the environment are added to the distance field. The distance field is recomputed each time something changes other than the joint positions of the planning links.

The process for a new robot requires that you manually design the collision model. You choose the position and size for the spheres and you choose which links get added to the distance field. Then a minor amount of magic happens in which the minimum number of kinematic chains are generated that will be needed to compute the position of each link.

Look at the [config folder](https://github.com/bcohen/sbpl_manipulation/tree/master/sbpl_collision_checking/config) for robot descriptions of popular robots.



##Problems & Hacks

I ran out of time to implement some nice features...I got it working and moved on and I had to hack some things along the way. 

Please note that my first application of this collision checker was for a static robot doing pick and place with a single arm. A bunch of corners were cut around that scenario. 

###Self Collision

1. All non-planning links are put in the distance field. This requires recomputing the distance field each time the robot base (or second arm) moves (even if the world remains static). My first application of this collision checker was for a static robot doing pick and place with a single arm. It was OK for this to be the case.
2. Non-planning joint positions can be set with setJointPosition(string name, double val). The distance field has to be recomputed afterwards (You can send in a PlanningScene and it does it for you).
3. All objects that are being manipulated are put in the distance field as well. This is dumb because in a pick and place scenario on a tabletop, there shouldn't be a need to recompute the distance field each time an object is moved. In the perfect world, the distance field should be updated (voxels should be added or removed and the changes should be propagated). I know that the distance field class theoretically supports this now, but I don't feel confident that it works 100%. It might have been fixed by now and I'm just going on old information here.
4. There isn't any self collision between a planning link and another planning link (gripper or attached object to upper_arm).

###Attached Objects

1. Maintaining the state of attached objects is done in a wonky way. A simple map stores object names to object descriptions. This could be OK but I feel like attached objects needs to be cleaned up for the next reason too.
2. It's possible that someone could attach an object to an arbitrary link on the robot that a kinematic chain wasn't created for during the initializaiton of the CC and I did not yet put in the code to create a kinematic chain for the attached object link so that forward kinematics can be done for it. Right now, I think it looks through all of the chains that were automatically created during initialization (based on the robot description yaml file) and hopes that a kinematic chain that can be used to perform FK for the attached object link is found.
3. I don't remember if attaching objects of any arbitrary type is supported yet. The way it works with the types that are supported is it populates the object shape with spheres (of a set radius that is chosen with a param....could be very inefficient and very limiting....simple heuristics can be used to improve this drastically). The [code that is used to enclose the shape with spheres](https://github.com/sbpl/sbpl_geometry_utils/blob/master/include/sbpl_geometry_utils/SphereEncloser.h) is from the [sbpl_geometry_utils] (https://github.com/sbpl/sbpl_geometry_utils) package written by Andrew Dornbush. It does not compute an optimal number of spheres. It does allow you to choose whether to fill the object or not. The function works but corner cases have been found that have made it crash. I would say that it works 95+% of the time. Andrew is great. If you ever found that it crashed - he would really appreciate a ticket. The encloseMesh function has been tested a lot in our last project this past semester and some bugs were fixed. Unfortunatly, the function can be slow with a complex mesh.

###Collision Objects

1. Similar to attached objects, my method of storing the current list of objects works feels wonky but I don't remember exactly why. It's also a map that stores the name to the object description. Also, it stores the voxels for each object so that they only have to be computed once.
2. In the [sbpl_geometry_utils](https://github.com/sbpl/sbpl_geometry_utils) we use a set of [voxelizing functions](https://github.com/sbpl/sbpl_geometry_utils/blob/master/include/sbpl_geometry_utils/Voxelizer.h) that can voxelize an arbitrary shape including a mesh (returns a list of voxels). It also allows you to choose whether you want the shape to be filled or not. It works well but can be slow for large meshes or complex meshes (like entire rooms). The voxels are transformed by the pose of the object before they are added to the world.
3. In both cases, attached and collision objects - I don't remember if I ever tested cylinders...maybe I have? I don't remember.
