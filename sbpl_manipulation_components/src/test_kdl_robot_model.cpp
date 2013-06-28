#include <ros/ros.h>
#include <leatherman/print.h>
#include <sbpl_manipulation_components/kdl_robot_model.h>

int main(int argc, char **argv)
{
  ros::init (argc, argv, "test_kdl_robot_model");
  ros::NodeHandle nh, ph("~");
  sleep(1);
  ros::spinOnce();

  sbpl_arm_planner::KDLRobotModel rm("torso_lift_link", "r_gripper_palm_link");

  std::string urdf;
  nh.param<std::string>("robot_description", urdf, " ");

  std::vector<std::string> pj;
  pj.push_back("r_shoulder_pan_joint");
  pj.push_back("r_elbow_flex_joint");
  pj.push_back("r_shoulder_lift_joint");
  pj.push_back("r_wrist_flex_joint");
  pj.push_back("r_upper_arm_roll_joint");
  pj.push_back("r_forearm_roll_joint");
  pj.push_back("r_wrist_roll_joint");

  if(!rm.init(urdf,pj))
  {
    ROS_ERROR("Failed to initialize the robot model. Exiting.");
    return 0;
  }
  //rm.setPlanningLink("r_wrist_roll_link");
  rm.setPlanningLink("r_gripper_palm_link");
  ROS_WARN("Robot Model Information");
  rm.printRobotModelInformation();

  std::vector<double> zeros(7,0), fka(7,0), ika(7,0), pose(6,0), posef(6,0);
  fka[0] = -0.5;
  fka[1] = -0.3;
  fka[2] =  0.0;
  fka[3] = -1.0;
  fka[4] = -0.5;
  fka[5] = -0.5;
  fka[6] =  0.0;
  KDL::Frame f;

  
  /****** Test 1: FK/IK matching? *****
  // FK
  if(!rm.computePlanningLinkFK(fka, pose))
  {
    ROS_ERROR("Failed to compute fK");
    return 0;
  }

  // IK 
  if(!rm.computeIK(pose, zeros, ika))
  {
    ROS_ERROR("Failed to compute fK");
    return 0;
  }

  ROS_INFO(" ");
  ROS_WARN("FK-IK Test 1 (kinematics_frame == planning_frame)");
  ROS_INFO("[fk]  input_angles: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f  xyz: % 0.3f % 0.3f % 0.3f  rpy: % 0.3f % 0.3f % 0.3f", 
      fka[0], fka[1], fka[2], fka[3], fka[4], fka[5], fka[6], pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
  ROS_INFO("[ik] output_angles: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f  xyz: % 0.3f % 0.3f % 0.3f  rpy: % 0.3f % 0.3f % 0.3f", 
      ika[0], ika[1], ika[2], ika[3], ika[4], ika[5], ika[6], pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
  */

  /****** Test 2: Translate Robot on map *****
  ROS_INFO(" ");
  ROS_WARN("FK-IK Test 2 (kinematics_frame: on robot body  planning_frame: map)");

  f.p.x(10.0); f.p.y(3.0); f.p.z(5.0);
  rm.setKinematicsToPlanningTransform(f, "map");

  // FK
  if(!rm.computePlanningLinkFK(fka, pose))
  {
    ROS_ERROR("Failed to compute fK");
    return 0;
  }

  // IK 
  if(!rm.computeIK(pose, zeros, ika))
  {
    ROS_ERROR("Failed to compute fK");
    return 0;
  }

  ROS_INFO("[robot pose] xyz: 10.0 3.0 5.0  rpy: 0.0 0.0 0.0");
  ROS_INFO("[fk]  input_angles: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f  xyz: % 0.3f % 0.3f % 0.3f  rpy: % 0.3f % 0.3f % 0.3f", 
      fka[0], fka[1], fka[2], fka[3], fka[4], fka[5], fka[6], pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
  ROS_INFO("[ik] output_angles: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f  xyz: % 0.3f % 0.3f % 0.3f  rpy: % 0.3f % 0.3f % 0.3f", 
      ika[0], ika[1], ika[2], ika[3], ika[4], ika[5], ika[6], pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
  */

  /****** Test 3: FK/IK matching? ******/
  ROS_INFO(" ");
  ROS_WARN("IK-FK Test 3 (kinematics_frame == planning_frame)");

  f.p.x(-0.05); f.p.y(0.0); f.p.z(0.801);
  f.M = KDL::Rotation::Quaternion(0,0,0,1);
  rm.setKinematicsToPlanningTransform(f, "map");
  pose[0] = 0.766268;
  pose[1] = -0.188;
  pose[2] = 0.790675;
  pose[3] = 0.5; 
  pose[4] = 0;
  pose[5] = 0;
  
  // IK 
  if(!rm.computeIK(pose, zeros, ika))
  {
    ROS_ERROR("Failed to compute fK");
    return 0;
  }

  ROS_INFO("[ik] output_angles: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f  xyz: % 0.3f % 0.3f % 0.3f  rpy: % 0.3f % 0.3f % 0.3f", 
      ika[0], ika[1], ika[2], ika[3], ika[4], ika[5], ika[6], pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);

  // FK
  if(!rm.computePlanningLinkFK(ika, posef))
  {
    ROS_ERROR("Failed to compute fK");
    return 0;
  }

    ROS_INFO("[fk]  input_angles: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f  xyz: % 0.3f % 0.3f % 0.3f  rpy: % 0.3f % 0.3f % 0.3f", 
      ika[0], ika[1], ika[2], ika[3], ika[4], ika[5], ika[6], posef[0], posef[1], posef[2], posef[3], posef[4], posef[5]);

  ROS_INFO("done");
  return 1;
}

