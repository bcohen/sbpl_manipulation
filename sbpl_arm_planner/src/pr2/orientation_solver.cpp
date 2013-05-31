/*
 * Written by Gokul Subramanian, MSE in Robotics & CIS 2011
 * Advised by Professor Maxim Likhachev
 * University of Pennsylvania
 * 4/14/2010
*/

#include <sbpl_arm_planner/pr2/sbpl_math.h>
#include <sbpl_arm_planner/pr2/orientation_solver.h>
#define PI 3.14159

#define DEBUG 0
#define END_EFF "endeffector" //9
#define FOREARM_ROLL "forearm" //6

#define NUM_JOINTS 7

namespace sbpl_arm_planner
{

/*Both these pitch limits are on the upper side (i.e. if forearm is aligned along x-axis with forearm_roll=0, then the end effector cannot pitch upward more than WRIST_PITCH_LIMIT_MAX and cannot come below a pitch of WRIST_PITCH_LIMIT_MIN on the upper side. */
#define WRIST_PITCH_LIMIT_MAX   114   //These values are in degrees (for ease of understanding).
#define WRIST_PITCH_LIMIT_MIN   0     //The corresponding radian values are approximately 2 and 0.1.

RPYSolver::RPYSolver(SBPLKinematicModel* arm, SBPLCollisionSpace* cspace)
{
  arm_ = arm;
  cspace_ = cspace;

  verbose_ = false;
  try_both_directions_ = false;
  num_calls_ = 0;
  num_invalid_predictions_ = 0;
  num_invalid_solution_ = 0;
  num_invalid_path_to_solution_ = 0;
}

void RPYSolver::orientationSolver(double* output, double phi, double theta, double psi, double yaw1, double pitch1, double roll1, double yaw2, double pitch2, double roll2, int attempt) 
{
  /**************************************************************************/
  //Is the desired orientation possible to attain with the given joint limits?
  /**************************************************************************/
  double wrist_pitch_limit_max;         //Used to express the wrist pitch limit in radians
  double wrist_pitch_limit_min;         //Used to express the wrist pitch limit in radians
  //The wrist pitch limit in radians
  wrist_pitch_limit_max=(PI/180.0)*WRIST_PITCH_LIMIT_MAX;
  wrist_pitch_limit_min=(PI/180.0)*WRIST_PITCH_LIMIT_MIN;

  //The PR2 FK is defined such that pitch is negative upward. Hence, the input pitch values are negated, because the following portion of the code was written assuming that pitch is positive upward.
  theta=-theta;
  pitch1=-pitch1;
  pitch2=-pitch2;

  double v1[] = {cos(theta)*cos(phi), cos(theta)*sin(phi), sin(theta)};
  double v2[] = {cos(pitch2)*cos(yaw2), cos(pitch2)*sin(yaw2), sin(pitch2)};
  double dp_v = dot_product(v1, v2, 3);
  double check_flag=dp_v<0;
  double check_flag2=fabs(dp_v)>fabs(cos(wrist_pitch_limit_max));
  double check_flag3=dp_v>0;
  double check_flag4=dp_v>cos(wrist_pitch_limit_min);

  if((check_flag && check_flag2) || (check_flag3 && check_flag4)) {
    //  std::cout << "Impossible." << std::endl;
    output[0]=0;
    output[1]=0;
    output[2]=0;
    output[3]=0;
    return;
  }

  /*****************************************************/
  //Firstly, all variables are declared and some defined
  /*****************************************************/

  double hand1[]={0, -0.5, 0, 0, 0.5, 0};
  double hand2[]={0, -0.5, 0, 0, 0.5, 0};
  double hand1_fo[6];
  double hand2_fo[6];
  double hand1_vect[3], hand2_vect[3];
  double grip1[]={0, 0, 0, 1, 0, 0};
  double grip2[]={0, 0, 0, 1, 0, 0};
  double grip1_fo[6];
  double grip2_fo[6];
  double grip1_vect[3], grip2_vect[3];
  double indicator1[]={-2.5, 0, 0, -2.5, 0, 1};
  double indicator2[]={-2.5, 0, 0, -2.5, 0, 1};
  double indicator1_fo[6];
  double indicator2_fo[6];
  double ind1_vect[3],ind2_vect[3];
  double comp1_vect[3], comp2_vect[3], project1_vect[3], project2_vect[3];
  double rot1[9], rot2[9], rot3[9];
  double temp[6], temp1[9], temp2[9], temp3[9], temp_var, temp_vect[3], temp2_vect[3];
  double w[3], w_hat[9];
  double rot_world[9], rot_world_trans[9];
  double identity[]={1, 0, 0,
                     0, 1, 0,
                     0, 0, 1};
  double unit_x[3]={1, 0, 0};
  double zero_vect[3]={0, 0, 0};
  double fo_arm_roll, wrist_pitch, wrist_roll, fs_angle, fe_angle;
  double is_orient_possible_flag=1;     //Flag returned indicates whether desired orientation is possible
  //with wrist limits. 1-possible, 0-impossible
  double c_alpha, c_beta, c_gamma, c_delta, c_eps;

  /******************************/
  //Accepting the input arguments
  /******************************/

  create_rotation_matrix(rot_world,phi,theta,psi);
  transpose(rot_world_trans, rot_world, 3, 3);

  //The start and the end orientations in the world frame
  rot1[0]=cos(yaw1);      rot1[3]=-sin(yaw1);     rot1[6]=0;
  rot1[1]=sin(yaw1);      rot1[4]=cos(yaw1);      rot1[7]=0;
  rot1[2]=0;              rot1[5]=0;              rot1[8]=1;

  rot2[0]=cos(pitch1);    rot2[3]=0;              rot2[6]=-sin(pitch1);
  rot2[1]=0;              rot2[4]=1;              rot2[7]=0;
  rot2[2]=sin(pitch1);    rot2[5]=0;              rot2[8]=cos(pitch1);

  multiply(rot3,rot1,3,3,rot2,3); //Yaw and pitch rotations
  multiply(temp,rot3,3,3,hand1,2);
  equate(hand1,temp,3,2);
  multiply(temp,rot3,3,3,grip1,2);
  equate(grip1,temp,3,2);

  w[0]=grip1[3]; //Unit vector along grip
  w[1]=grip1[4];
  w[2]=grip1[5];

  w_hat[0]=0;         w_hat[3]=-w[2];     w_hat[6]=w[1];
  w_hat[1]=w[2];      w_hat[4]=0;         w_hat[7]=-w[0];
  w_hat[2]=-w[1];     w_hat[5]=w[0];      w_hat[8]=0;

  scalar_multiply(temp1,w_hat,3,3,sin(roll1));
  multiply(temp2,w_hat,3,3,w_hat,3);
  scalar_multiply(temp3,temp2,3,3,1-cos(roll1));
  matrix_add(temp2,temp1,temp3,3,3);
  matrix_add(rot1,identity, temp2,3,3);

  multiply(temp1,rot1,3,3,hand1,2);
  equate(hand1,temp1,3,2);

  rot1[0]=cos(yaw2);      rot1[3]=-sin(yaw2);     rot1[6]=0;
  rot1[1]=sin(yaw2);      rot1[4]=cos(yaw2);      rot1[7]=0;
  rot1[2]=0;              rot1[5]=0;              rot1[8]=1;

  rot2[0]=cos(pitch2);    rot2[3]=0;              rot2[6]=-sin(pitch2);
  rot2[1]=0;              rot2[4]=1;              rot2[7]=0;
  rot2[2]=sin(pitch2);    rot2[5]=0;              rot2[8]=cos(pitch2);

  multiply(rot3,rot1,3,3,rot2,3); //Yaw and pitch rotations
  multiply(temp,rot3,3,3,hand2,2);
  equate(hand2,temp,3,2);
  multiply(temp,rot3,3,3,grip2,2);
  equate(grip2,temp,3,2);

  w[0]=grip2[3]; //Unit vector along grip
  w[1]=grip2[4];
  w[2]=grip2[5];

  w_hat[0]=0;         w_hat[3]=-w[2];     w_hat[6]=w[1];
  w_hat[1]=w[2];      w_hat[4]=0;         w_hat[7]=-w[0];
  w_hat[2]=-w[1];     w_hat[5]=w[0];      w_hat[8]=0;

  scalar_multiply(temp1,w_hat,3,3,sin(roll2));
  multiply(temp2,w_hat,3,3,w_hat,3);
  scalar_multiply(temp3,temp2,3,3,1-cos(roll2));
  matrix_add(temp2,temp1,temp3,3,3);
  matrix_add(rot1,identity,temp2,3,3);

  multiply(temp1,rot1,3,3,hand2,2);
  equate(hand2,temp1,3,2);

  //The start and the end orientations in the forearm frame
  multiply(hand1_fo,rot_world_trans,3,3,hand1,2);
  multiply(hand2_fo,rot_world_trans,3,3,hand2,2);
  multiply(grip1_fo,rot_world_trans,3,3,grip1,2);
  multiply(grip2_fo,rot_world_trans,3,3,grip2,2);

  grip1_vect[0]=grip1_fo[3];
  grip1_vect[1]=grip1_fo[4];
  grip1_vect[2]=grip1_fo[5];

  temp_var=dot_product(grip1_vect, unit_x, 3);
  scalar_multiply(comp1_vect, unit_x,3,1,temp_var);
  subtract(project1_vect,grip1_vect,comp1_vect,3,1);

  grip2_vect[0]=grip2_fo[3];
  grip2_vect[1]=grip2_fo[4];
  grip2_vect[2]=grip2_fo[5];

  temp_var=dot_product(grip2_vect, unit_x, 3);
  scalar_multiply(comp2_vect, unit_x,3,1,temp_var);
  subtract(project2_vect,grip2_vect,comp2_vect,3,1);

  ind1_vect[0]=indicator1[3]-indicator1[0];
  ind1_vect[1]=indicator1[4]-indicator1[1];
  ind1_vect[2]=indicator1[5]-indicator1[2];

  ind2_vect[0]=indicator2[3]-indicator2[0];
  ind2_vect[1]=indicator2[4]-indicator2[1];
  ind2_vect[2]=indicator2[5]-indicator2[2];

  if(!check_equality(grip1_vect, comp1_vect, 3)) {
    c_delta=dot_product(ind1_vect, project1_vect, 3)/vect_norm(project1_vect,3);
    fs_angle=acos(c_delta);

    cross_product(temp_vect, ind1_vect, project1_vect);

    if(vect_divide(temp_vect, unit_x, 3)<0) {
      fs_angle=-fs_angle;
    }

    rot1[0]=1;              rot1[3]=0;               rot1[6]=0;
    rot1[1]=0;              rot1[4]=cos(fs_angle);   rot1[7]=-sin(fs_angle);
    rot1[2]=0;              rot1[5]=sin(fs_angle);   rot1[8]=cos(fs_angle);

    multiply(temp, rot1, 3,3, indicator1, 2);
    equate(indicator1_fo, temp, 3, 2);
  }
  else {
    equate(indicator1_fo, indicator1, 3,2);
  }

  if(!check_equality(grip2_vect, comp2_vect, 3)) {
    c_eps=dot_product(ind2_vect, project2_vect, 3)/vect_norm(project2_vect, 3);
    fe_angle=acos(c_eps);

    cross_product(temp_vect, ind2_vect, project2_vect);

    if(vect_divide(temp_vect, unit_x, 3)<0) {
      fe_angle=-fe_angle;
    }

    rot1[0]=1;              rot1[3]=0;               rot1[6]=0;
    rot1[1]=0;              rot1[4]=cos(fe_angle);   rot1[7]=-sin(fe_angle);
    rot1[2]=0;              rot1[5]=sin(fe_angle);   rot1[8]=cos(fe_angle);

    multiply(temp, rot1, 3,3, indicator2, 2);
    equate(indicator2_fo, temp, 3, 2);
  }
  else {
    equate(indicator2_fo, indicator2, 3,2);
  }

  /*********************************/
  //Forearm roll is calculated now
  /*********************************/

  ind1_vect[0]=indicator1_fo[3]-indicator1_fo[0];
  ind1_vect[1]=indicator1_fo[4]-indicator1_fo[1];
  ind1_vect[2]=indicator1_fo[5]-indicator1_fo[2];

  ind2_vect[0]=indicator2_fo[3]-indicator2_fo[0];
  ind2_vect[1]=indicator2_fo[4]-indicator2_fo[1];
  ind2_vect[2]=indicator2_fo[5]-indicator2_fo[2];

  c_gamma=dot_product(ind1_vect, ind2_vect, 3);

  cross_product(temp_vect, ind1_vect, ind2_vect);
  if(vect_divide(temp_vect, unit_x, 3)>0) {
    fo_arm_roll=acos(c_gamma);
  }       
  else {
    fo_arm_roll=-acos(c_gamma);
  } 

  //In the second attempt try rotating the other way around to the same orientation
  if(attempt == 2) {fo_arm_roll = -(2*PI - fo_arm_roll);}

  rot1[0]=1;              rot1[3]=0;                  rot1[6]=0;
  rot1[1]=0;              rot1[4]=cos(fo_arm_roll);   rot1[7]=-sin(fo_arm_roll);
  rot1[2]=0;              rot1[5]=sin(fo_arm_roll);   rot1[8]=cos(fo_arm_roll);

  multiply(temp1,rot1,3,3,hand1_fo,2);
  equate(hand1_fo,temp1,3,2);
  multiply(temp1,rot1,3,3,grip1_fo,2);
  equate(grip1_fo,temp1,3,2);

  /*********************************/
  //Wrist pitch is calculated now
  /*********************************/

  grip1_vect[0]=grip1_fo[3];
  grip1_vect[1]=grip1_fo[4];
  grip1_vect[2]=grip1_fo[5];

  grip2_vect[0]=grip2_fo[3];
  grip2_vect[1]=grip2_fo[4];
  grip2_vect[2]=grip2_fo[5];

  cross_product(temp_vect, grip1_vect, grip2_vect);

  if(!check_equality(temp_vect, zero_vect, 3)) {
    c_alpha=dot_product(grip1_vect, grip2_vect, 3);

    cross_product(temp2_vect,ind2_vect,temp_vect);

    if(vect_divide(temp2_vect, unit_x, 3)>0) {
      wrist_pitch=-acos(c_alpha);
      temp_vect[0]=-temp_vect[0];
      temp_vect[1]=-temp_vect[1];
      temp_vect[2]=-temp_vect[2];
    }
    else {
      wrist_pitch=acos(c_alpha);

    }        

    scalar_multiply(w, temp_vect, 3,1,1/vect_norm(temp_vect, 3));

    w_hat[0]=0;         w_hat[3]=-w[2];     w_hat[6]=w[1];
    w_hat[1]=w[2];      w_hat[4]=0;         w_hat[7]=-w[0];
    w_hat[2]=-w[1];     w_hat[5]=w[0];      w_hat[8]=0;

    scalar_multiply(temp1,w_hat,3,3,sin(wrist_pitch));
    multiply(temp2,w_hat,3,3,w_hat,3);
    scalar_multiply(temp3,temp2,3,3,1-cos(wrist_pitch));
    matrix_add(temp2,temp1,temp3,3,3);
    matrix_add(rot1,identity, temp2,3,3);

    multiply(temp1,rot1,3,3,hand1_fo,2);
    equate(hand1_fo,temp1,3,2);
    multiply(temp1,rot1,3,3,grip1_fo,2);
    equate(grip1_fo,temp1,3,2);
  }
  else {
    wrist_pitch=0;
  }

  /**Somehow the wrist_roll calculations from within this code don't seem to be right.
    This problem has been temporarily solved by invoking FK from environment_robarm3d.cpp**/
  /*********************************/
  //Wrist roll is calculated now
  /*********************************/
  wrist_roll=0;
  hand1_vect[0]=hand1_fo[3]-hand1_fo[0];
  hand1_vect[1]=hand1_fo[4]-hand1_fo[1];
  hand1_vect[2]=hand1_fo[5]-hand1_fo[2];

  hand2_vect[0]=hand2_fo[3]-hand2_fo[0];
  hand2_vect[1]=hand2_fo[4]-hand2_fo[1];
  hand2_vect[2]=hand2_fo[5]-hand2_fo[2];

  grip1_vect[0]=grip1_fo[3];
  grip1_vect[1]=grip1_fo[4];
  grip1_vect[2]=grip1_fo[5];

  c_beta=dot_product(hand1_vect, hand2_vect, 3);
  cross_product(temp_vect, hand1_vect, hand2_vect);

  if(!check_equality(temp_vect, zero_vect, 3)) {
    if(vect_divide(temp_vect, grip1_vect, 3)>0) {
      wrist_roll=acos(c_beta);
    }
    else {
      wrist_roll=-acos(c_beta);
    }
  }

  //The output
  output[0]=is_orient_possible_flag;
  output[1]=fo_arm_roll;   
  output[2]=wrist_pitch;
  output[3]=wrist_roll;
}

bool RPYSolver::isOrientationFeasible(const double* rpy, std::vector<double> &start, std::vector<double> &prefinal, std::vector<double> &final, int &num_checks)
{
  bool try_both_rotations = try_both_directions_;
  unsigned char dist=0;
  unsigned int i=0;
  unsigned int my_count=0;
  double hand_rotations[4];
  std::vector<double>forearm_pose(6,0);
  std::vector<double>endeff_pose(6,0);
  std::vector<double>goal_joint_config(7,0);
  
  // added 6/9/12 to record how many collision checks are being done
  int num_path_checks = 0;
  num_checks = 0;

  //int link_num = 2; //added 6/15/11

  //get pose of forearm link
  if(!arm_->computeFK(start,FOREARM_ROLL,forearm_pose))
  {
    ROS_DEBUG("computeFK failed on forearm pose.\n");
    return false;
  }

  //get pose of end effector link
  if(!arm_->computeFK(start,END_EFF,endeff_pose))
  {
    ROS_DEBUG("computeFK failed on end_eff pose.\n");
    return false;
  }

#if DEBUG
  ROS_DEBUG_NAMED("orientation_solver", "Joint Angles: %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f", start[0], start[1], start[2], start[3], start[4], start[5], start[6]);
  ROS_DEBUG_NAMED("orientation_solver", "FK:forearm: %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f",forearm_pose[0], forearm_pose[1], forearm_pose[2], forearm_pose[3], forearm_pose[4], forearm_pose[5], forearm_pose[6]);
  ROS_DEBUG_NAMED("orientation_solver", "FK:endeff:  %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f",endeff_pose[0], endeff_pose[1], endeff_pose[2], endeff_pose[3], endeff_pose[4], endeff_pose[5], endeff_pose[6]);
  ROS_DEBUG_NAMED("orientation_solver", "Start yaw: %0.4f, Start pitch: %0.4f, Start roll: %0.4f",endeff_pose[5], endeff_pose[4], endeff_pose[3]);
  ROS_DEBUG_NAMED("orientation_solver", "End yaw required: %0.4f, End pitch required: %0.4f, End roll required: %0.4f", rpy[2], rpy[1], rpy[0]);
#endif

  // call orientation planner 
  orientationSolver(hand_rotations, forearm_pose[5],forearm_pose[4],forearm_pose[3], endeff_pose[5], endeff_pose[4], endeff_pose[3], rpy[2], rpy[1], rpy[0], 1); //1st attempt

  num_calls_++;

#if DEBUG
  ROS_DEBUG_NAMED("orientation_solver", "[isGoalPositionGokul] The rotations commanded in the first attempt are: %0.4f, %0.4f and %0.4f", hand_rotations[1], hand_rotations[2], hand_rotations[3]);
#endif

  if(hand_rotations[0]==0)
  {
#if DEBUG
    ROS_DEBUG_NAMED("orientation_solver", "Orientation solver predicts that this movement is impossible with the given pitch limits.");
#endif
    num_invalid_predictions_++;
    return false;
  }

SET_ANGLES_AGAIN: //GoTO label, temporary, do not continue use of this

  for(my_count=0; my_count<start.size(); my_count++)
  {
    goal_joint_config[my_count]=start[my_count];
    prefinal[my_count]=start[my_count];
  }

  for(my_count=0; my_count<3; my_count++)
  {
    goal_joint_config[4+my_count]+=hand_rotations[1+my_count];

    //check for collisions
    num_checks++;
    if(!cspace_->checkCollision(goal_joint_config, verbose_, false, dist))
    //if(!cspace_->checkLinkForCollision(goal_joint_config, link_num, verbose_, dist))
    {
      num_invalid_solution_++;
      return false;
    }
    std::vector<std::vector<double> > path(2, std::vector<double>(NUM_JOINTS,0));

    for(i = 0; i < path[0].size(); i++)
    {
      path[0][i] = start[i];
      path[1][i] = goal_joint_config[i];
    }

    //check for collisions along the path
    num_path_checks = 0;
    int path_length;
    if(!cspace_->checkPathForCollision(start, goal_joint_config, verbose_, path_length, num_path_checks, dist))
    {
      num_checks += num_path_checks;
      //try rotating in the opposite direction
      if(try_both_rotations)
      {
        ROS_WARN("[rpysolver] First rotation is invalid. Trying the second one now...");
        orientationSolver(hand_rotations, forearm_pose[5],forearm_pose[4],forearm_pose[3], endeff_pose[5], endeff_pose[4], endeff_pose[3], rpy[2], rpy[1], rpy[0], 2);
        try_both_rotations = 0;
        goto SET_ANGLES_AGAIN;
      }

      num_invalid_path_to_solution_++;
      return false;
    }
    num_checks += num_path_checks;
  }

  final = goal_joint_config;

#if DEBUG
  ROS_DEBUG_NAMED("orientation_solver", "\n \n After:");
  arm_->computeFK(final,END_EFF,endeff_pose);
  ROS_DEBUG_NAMED("orientation_solver", "The perfect roll value from FK is: %0.4f.", rpy[0]-endeff_pose[3]);

  ROS_DEBUG_NAMED("orientation_solver", "Joint angles:%0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f", final[0], final[1], final[2], final[3], final[4], final[5], final[6]);

  ROS_DEBUG_NAMED("orientation_solver", "Forward kinematics on final arm position.");
  arm_->computeFK(final,FOREARM_ROLL,forearm_pose);
  ROS_DEBUG_NAMED("orientation_solver", "FK:f_arm: %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f",forearm_pose[0], forearm_pose[1], forearm_pose[2], forearm_pose[3], forearm_pose[4], forearm_pose[5], forearm_pose[6]);
  arm_->computeFK(final,END_EFF,endeff_pose);
  ROS_DEBUG_NAMED("orientation_solver", "FK:endeff: %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f",endeff_pose[0], endeff_pose[1], endeff_pose[2], endeff_pose[3], endeff_pose[4], endeff_pose[5], endeff_pose[6]);

  ROS_DEBUG_NAMED("orientation_solver", "Succeeded.");
#endif

  return true;
}

bool RPYSolver::isOrientationFeasible(const double* rpy, std::vector<double> &start, std::vector<std::vector<double> > &path)
{
  double hand_rotations[4]={0};
  std::vector<double>forearm_pose(6,0);
  std::vector<double>endeff_pose(6,0);

  //get pose of forearm link
  if(!arm_->computeFK(start,FOREARM_ROLL,forearm_pose))
  {
    ROS_WARN("[rpysolver] computeFK failed on forearm pose.");
    return false;
  }

  //get pose of end effector link
  if(!arm_->computeFK(start,END_EFF,endeff_pose))
  {
    ROS_WARN("[rpysolver] computeFK failed on end_eff pose.");
    return false;
  }

  // avoid a divide by zero problem by checking if at goal rpy first
  if(fabs(rpy[0] - endeff_pose[3]) < 0.0001 && 
     fabs(rpy[1] - endeff_pose[4]) < 0.0001 &&
     fabs(rpy[2] - endeff_pose[5]) < 0.0001)
  {
    return false;
  }
  ROS_DEBUG("[rpysolver]  desired: %2.2f %2.2f %2.2f    actual: %2.2f %2.2f %2.2f   {diff: %2.7f %2.7f %2.7f}", rpy[0], rpy[1], rpy[2], endeff_pose[3], endeff_pose[4], endeff_pose[5], fabs(rpy[0]-endeff_pose[3]), fabs(rpy[1]-endeff_pose[4]), fabs(rpy[2]-endeff_pose[5]));

  orientationSolver(hand_rotations, forearm_pose[5],forearm_pose[4],forearm_pose[3], endeff_pose[5], endeff_pose[4], endeff_pose[3], rpy[2], rpy[1], rpy[0], 1);
  num_calls_++;

  // impossible due to the wrist pitch limit
  if(hand_rotations[0]==0)
  {
    num_invalid_predictions_++;
    return false;
  }

  path.clear();
  path.resize(1,std::vector<double>(start));
  path[0][4] += hand_rotations[1];
  path[0][5] += hand_rotations[2];
  path[0][6] += hand_rotations[3];
  return true;
}

void RPYSolver::printStats()
{
  ROS_INFO("Calls to OS: %d   Predicts Impossible: %d   Invalid Solutions: %d   Invalid Paths: %d", num_calls_, num_invalid_predictions_, num_invalid_solution_, num_invalid_path_to_solution_); 
}

}

