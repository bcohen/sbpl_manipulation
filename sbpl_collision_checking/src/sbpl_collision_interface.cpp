#include <sbpl_collision_checking/sbpl_collision_interface.h>
#include <distance_field/propagation_distance_field.h>

using namespace sbpl_arm_planner;

SBPLCollisionInterface::SBPLCollisionInterface(sbpl_arm_planner::OccupancyGrid *grid, sbpl_arm_planner::SBPLCollisionSpace *cspace) : cspace_(cspace), grid_(grid)
{
  if(cspace_ == NULL)
    ROS_ERROR("[cspace] Warning! CollisionSpace pointer is NULL. This will seg fault.");
  reference_frame_ = cspace_->getReferenceFrame();
  attached_object_frame_ = "r_gripper_r_finger_tip_link";
}

SBPLCollisionInterface::~SBPLCollisionInterface()
{
}

bool SBPLCollisionInterface::init()
{

  if(cspace_->getGroupName() == "left_arm")
  {
    attached_object_frame_ = "l_gripper_r_finger_tip_link";
  }
  else
  {
    ROS_INFO("[cspace] Planning for the right arm.");
    attached_object_frame_ = "r_gripper_r_finger_tip_link";
  }
  return true;
}

/*
void SBPLCollisionInterface::processPlanningScene(const planning_scene::PlanningSceneConstPtr &pscene)
{
  // update the planning scene
  //pscene_ = pscene;
  //tf_ = pscene_->getTransforms();

  // delete attached objects
  ROS_INFO("[cspace] Removing all attached objects.");
  cspace_->removeAttachedObject();

  // reset the world representation
  cspace_->resetDistanceField();
  cspace_->removeAllCollisionObjects();

  arm_navigation_msgs::PlanningScene pscene_msg;
  pscene_->getPlanningSceneMsg(pscene_msg);

  ROS_INFO("[cspace] Processing %d collision objects.", int(pscene_msg.world.collision_objects.size()));
  processCollisionObjects(pscene_msg.world.collision_objects);

  ROS_DEBUG("[cspace] Processing %d attached objects.", int(pscene_msg.attached_collision_objects.size()));
  processAttachedObjects(pscene_msg.attached_collision_objects);

  ROS_DEBUG("[cspace] Processing collision map with %d voxels.", int(pscene_msg.world.collision_map.boxes.size()));
  processCollisionMap(pscene_msg.world.collision_map);
  
  ROS_INFO("[cspace] Done processing planning scene.");
}
*/

void SBPLCollisionInterface::processCollisionMap(const arm_navigation_msgs::CollisionMap &collision_map)
{
  if(collision_map.header.frame_id.compare(reference_frame_) != 0)
  {
    ROS_WARN_ONCE("collision_map_occ is in %s not in %s", collision_map.header.frame_id.c_str(), reference_frame_.c_str());
    ROS_DEBUG("the collision map has %i cubic obstacles", int(collision_map.boxes.size()));
  }

  // update collision map
  if(collision_map.boxes.empty())
    ROS_DEBUG("[cspace] Received collision map is empty.");
  else
  {
    grid_->updateFromCollisionMap(collision_map);
    cmap_ = collision_map;
  }
}

void SBPLCollisionInterface::processAttachedObjects(const std::vector<arm_navigation_msgs::AttachedCollisionObject> &attached_objects)
{
  for(size_t i = 0; i < attached_objects.size(); ++i)
  {
    if(!cspace_->doesLinkExist(attached_objects[i].link_name))
    {
      ROS_WARN("[cspace] This attached object is not intended for this arm.");
    }
    // add object
    else if(attached_objects[i].object.operation.operation == arm_navigation_msgs::CollisionObjectOperation::ADD)
    {
      ROS_DEBUG("[cspace] Received a message to ADD an object (%s) with %d shapes.", attached_objects[i].object.id.c_str(), int(attached_objects[i].object.shapes.size()));
      attachObject(attached_objects[i].object, attached_objects[i].link_name);
    }
    // remove object
    else if(attached_objects[i].object.operation.operation == arm_navigation_msgs::CollisionObjectOperation::REMOVE)
    {
      ROS_DEBUG("[cspace] Removing object (%s) from gripper.", attached_objects[i].object.id.c_str());
      cspace_->removeAttachedObject();
    }
    else
      ROS_WARN("Received a collision object with an unknown operation");
  }
}

void SBPLCollisionInterface::processCollisionObjects(const std::vector<arm_navigation_msgs::CollisionObject> &collision_objects)
{
  for(size_t i = 0; i < collision_objects.size(); ++i)
  {
    // debug: have we seen this collision object before?
    if(object_map_.find(collision_objects[i].id) != object_map_.end())
      ROS_DEBUG("[cspace] We have seen this object ('%s')  before.", collision_objects[i].id.c_str());
    else
      ROS_DEBUG("[cspace] We have NOT seen this object ('%s') before.", collision_objects[i].id.c_str());
    object_map_[collision_objects[i].id] = collision_objects[i];

    cspace_->processCollisionObjectMsg(collision_objects[i]);
  }
  /*
  // update distance field
  if(!cmap_.boxes.empty())
    grid_->updateFromCollisionMap(cmap_);
  cspace_->putCollisionObjectsInGrid();
  */
}

void SBPLCollisionInterface::attachObject(const arm_navigation_msgs::CollisionObject &obj, std::string link_name)
{
  geometry_msgs::PoseStamped pose_in, pose_out;
  arm_navigation_msgs::CollisionObject object(obj);


  ROS_INFO("Received a collision object message with %d shapes.", int(object.shapes.size()));

  for(size_t i = 0; i < object.shapes.size(); i++)
  {
    pose_in.header = object.header;
    pose_in.header.stamp = ros::Time();
    pose_in.pose = object.poses[i];
    //sbpl_arm_planner::transformPose(pscene_, pose_in.pose, pose_out.pose, object.header.frame_id, attached_object_frame_);
    object.poses[i] = pose_out.pose;
    ROS_WARN("[cspace] [attach_object] Converted shape from %s (%0.2f %0.2f %0.2f) to %s (%0.3f %0.3f %0.3f)", pose_in.header.frame_id.c_str(), pose_in.pose.position.x, pose_in.pose.position.y, pose_in.pose.position.z, attached_object_frame_.c_str(), pose_out.pose.position.x, pose_out.pose.position.y, pose_out.pose.position.z);

    if(object.shapes[i].type == arm_navigation_msgs::Shape::SPHERE)
    {
      ROS_INFO("[cspace] Attaching a '%s' sphere with radius: %0.3fm", object.id.c_str(), object.shapes[i].dimensions[0]);
      cspace_->attachSphere(object.id, link_name, object.poses[i], object.shapes[i].dimensions[0]);
    }
    else if(object.shapes[i].type == arm_navigation_msgs::Shape::CYLINDER)
    {
      ROS_INFO("[cspace] Attaching a '%s' cylinder with radius: %0.3fm & length %0.3fm", object.id.c_str(), object.shapes[i].dimensions[0], object.shapes[i].dimensions[1]);
      cspace_->attachCylinder(link_name, object.poses[i], object.shapes[i].dimensions[0], object.shapes[i].dimensions[1]);
    }
    else if(object.shapes[i].type == arm_navigation_msgs::Shape::MESH)
    {
      ROS_INFO("[cspace] Attaching a '%s' mesh with %d triangles & %d vertices is NOT supported right now...", object.id.c_str(), int(object.shapes[i].triangles.size()/3), int(object.shapes[i].vertices.size()));
      cspace_->attachMesh(object.id, link_name, object.poses[i], object.shapes[i].vertices, object.shapes[i].triangles);
    }
    else if(object.shapes[i].type == arm_navigation_msgs::Shape::BOX)
    {
      ROS_INFO("[cspace] Attaching a '%s' cube with dimensions {%0.3fm x %0.3fm x %0.3fm}.", object.id.c_str(), object.shapes[i].dimensions[0], object.shapes[i].dimensions[1], object.shapes[i].dimensions[2]);
      cspace_->attachCube(object.id, link_name, object.poses[i], object.shapes[i].dimensions[0], object.shapes[i].dimensions[1], object.shapes[i].dimensions[2]);
    }
    else
      ROS_WARN("[cspace] Currently attaching objects of type '%d' aren't supported.", object.shapes[i].type);
  }
}

void SBPLCollisionInterface::getVisualization(std::string name, std::vector<visualization_msgs::Marker> &markers)
{
  visualization_msgs::Marker marker;
  markers.clear();

  if(name.compare("collision_object_voxels") == 0)
  {
    std::vector<std::vector<double> > points(1,std::vector<double>(3,0));
    std::vector<double> color(4,1);
    color[2] = 0;
    std::vector<geometry_msgs::Pose> vposes;
    cspace_->getCollisionObjectVoxelPoses(vposes);

    marker.header.seq = 0;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = reference_frame_;
    marker.ns = "collision_object_voxels";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(360.0);
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.r = 1;
    marker.color.g = 1;
    marker.color.b = 0;
    marker.color.a = 1;

    marker.points.resize(vposes.size());
    for(size_t i = 0; i < vposes.size(); ++i)
    {
      marker.points[i].x = vposes[i].position.x;
      marker.points[i].y = vposes[i].position.y;
      marker.points[i].z = vposes[i].position.z;
    }
    markers.push_back(marker);
  }
  else if(name.compare("distance_field") == 0)
  {
    distance_field::PropagationDistanceField *df = grid_->getDistanceFieldPtr();
    //df->getIsoSurfaceMarkers(0.01, 0.03, reference_frame_, ros::Time::now(),  Eigen::Affine3d::Identity(), marker);
    df->getIsoSurfaceMarkers(0.01, 0.03, reference_frame_, ros::Time::now(), tf::Transform(tf::createIdentityQuaternion(), tf::Vector3(0,0,0)), marker);
    markers.push_back(marker);
  }
  else
    ROS_ERROR("[cspace] Can't generate visualization markers for '%s'.", name.c_str());
}


void SBPLCollisionInterface::getCollisionModelVisualization(const std::vector<std::vector<double> >& traj, std::string ns, std::vector<visualization_msgs::Marker> &markers)
{
  int id = 0;
  std::vector<visualization_msgs::Marker> m;
  std_msgs::ColorRGBA color;
  color.r = 0.86; color.g = 1.28; color.b = 0.5; color.a = 0.8;
  markers.clear();

  for(size_t i = 0; i < traj.size(); ++i)
  {
    m.clear();
    getCollisionModelVisualization(traj[i], ns, id, color, m);
    id = m.back().id + 1;
    markers.insert(markers.end(), m.begin(), m.end());
  }
  ROS_INFO("[cspace] Created %d markers to visualize the '%s' trajectory with %d waypoints.", int(markers.size()), ns.c_str(), int(traj.size()));
}

void SBPLCollisionInterface::getCollisionModelVisualization(const std::vector<double> &angles, std::string ns, int id, std_msgs::ColorRGBA color, std::vector<visualization_msgs::Marker> &markers)
{
  std::vector<std::vector<double> > sph;
  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = reference_frame_;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration(0);

  cspace_->getCollisionSpheres(angles, sph);
  for(size_t j = 0; j < sph.size(); ++j)
  {
    marker.id++;
    marker.pose.position.x = sph[j][0];
    marker.pose.position.y = sph[j][1];
    marker.pose.position.z = sph[j][2];
    marker.scale.x = sph[j][3] * 2.0;
    marker.scale.y = sph[j][3] * 2.0;
    marker.scale.z = sph[j][3] * 2.0;
    marker.color = color;
    ROS_INFO("[%d] xyz: %0.2f %0.2f %0.2f  radius: %0.2f", int(j), sph[j][0], sph[j][1], sph[j][2], sph[j][3]);
    markers.push_back(marker);
  }
}

void SBPLCollisionInterface::getCollisionModelVisualization(const std::vector<trajectory_msgs::JointTrajectoryPoint> &traj, std::string ns, std::vector<visualization_msgs::Marker> &markers)
{
  std::vector<std::vector<double> > t(traj.size());

  for(size_t i = 0; i < traj.size(); ++i)
  {
    t[i].resize(traj[i].positions.size());
    for(size_t j = 0; j < traj[i].positions.size(); ++j)
      t[i][j] = traj[i].positions[j];
  }
  getCollisionModelVisualization(t, ns, markers);
}

void SBPLCollisionInterface::getInterpolatedPathVisualization(const std::vector<double> &start, const std::vector<double> &end, std::string ns, bool type, std::vector<visualization_msgs::Marker> &markers)
{
  std::vector<double> start_norm(start);
  std::vector<double> end_norm(end);
  std::vector<std::vector<double> > path;

  for(size_t i=0; i < start.size(); ++i)
  {
    start_norm[i] = angles::normalize_angle(start[i]);
    end_norm[i] = angles::normalize_angle(end[i]);
  }

  if(!cspace_->interpolatePath(start_norm, end_norm, path))
  {
    ROS_ERROR("[cspace] Failed to interpolate the path. It's probably infeasible due to joint limits.");
    return;
  }

  if(path.empty())
  {
    path.push_back(start_norm);
    path.push_back(end_norm);
  }

  if(!type) // collision model
    getCollisionModelVisualization(path, ns, markers);
  else      // mesh model
    ROS_ERROR("[cspace] Visualizing the mesh markers of the interpolated path is currently not supported.");
}

