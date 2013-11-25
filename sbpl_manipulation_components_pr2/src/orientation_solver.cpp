/*
 * Written by Gokul Subramanian, MSE in Robotics & CIS 2011
 * Advised by Professor Maxim Likhachev
 * University of Pennsylvania
 * 4/14/2010
*/

#include <sbpl_manipulation_components_pr2/sbpl_math.h>
#include <sbpl_manipulation_components_pr2/orientation_solver.h>

namespace sbpl_arm_planner
{

/*Both these pitch limits are on the upper side (i.e. if forearm is aligned along x-axis with forearm_roll_link_roll=0, then the end effector cannot pitch upward more than WRIST_PITCH_LIMIT_MAX and cannot come below a pitch of WRIST_PITCH_LIMIT_MIN on the upper side. */

RPYSolver::RPYSolver(double wrist_pitch_min_limit, double wrist_pitch_max_limit)
{
  // absolute value (see note above)
  if(wrist_pitch_min_limit > wrist_pitch_max_limit)
  {
    wrist_pitch_min_limit_ = fabs(wrist_pitch_max_limit);
    wrist_pitch_max_limit_ = fabs(wrist_pitch_min_limit);
  }
  else
  {
    wrist_pitch_min_limit_ = fabs(wrist_pitch_min_limit);
    wrist_pitch_max_limit_ = fabs(wrist_pitch_max_limit);
  }
  //printf("[rpy-solver] wrist_pitch_limits: {min: %0.3f  max: %0.3f}\n", wrist_pitch_min_limit, wrist_pitch_max_limit);
}

bool RPYSolver::computeRPYOnly(const std::vector<double> &rpy, const std::vector<double> &start, const std::vector<double> &forearm_roll_link_pose, const std::vector<double> &endeff_link_pose, int solution_num, std::vector<double> &solution)
{
  double hand_rotations[4];

  orientationSolver(hand_rotations, forearm_roll_link_pose[5], forearm_roll_link_pose[4], forearm_roll_link_pose[3], endeff_link_pose[5], endeff_link_pose[4], endeff_link_pose[3], rpy[2], rpy[1], rpy[0], solution_num);

  if(hand_rotations[0] == 0)
    return false;

  solution = start;
  solution[4] += hand_rotations[1];
  solution[5] += hand_rotations[2];
  solution[6] += hand_rotations[3];
  return true;
}

void RPYSolver::orientationSolver(double* output, double phi, double theta, double psi, double yaw1, double pitch1, double roll1, double yaw2, double pitch2, double roll2, int attempt) 
{
  /**************************************************************************/
  //Is the desired orientation possible to attain with the given joint limits?
  /**************************************************************************/
  double wrist_pitch_limit_max;         //Used to express the wrist pitch limit in radians
  double wrist_pitch_limit_min;         //Used to express the wrist pitch limit in radians
  //The wrist pitch limit in radians
  wrist_pitch_limit_max=wrist_pitch_min_limit_; //(M_PI/180.0)*WRIST_PITCH_LIMIT_MAX;
  wrist_pitch_limit_min=wrist_pitch_max_limit_; //(M_PI/180.0)*WRIST_PITCH_LIMIT_MIN;

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
  double fo_rm_roll, wrist_pitch, wrist_roll, fs_angle, fe_angle;
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
    fo_rm_roll=acos(c_gamma);
  }       
  else {
    fo_rm_roll=-acos(c_gamma);
  } 

  //In the second attempt try rotating the other way around to the same orientation
  if(attempt == 2) {fo_rm_roll = -(2*M_PI - fo_rm_roll);}

  rot1[0]=1;              rot1[3]=0;                  rot1[6]=0;
  rot1[1]=0;              rot1[4]=cos(fo_rm_roll);   rot1[7]=-sin(fo_rm_roll);
  rot1[2]=0;              rot1[5]=sin(fo_rm_roll);   rot1[8]=cos(fo_rm_roll);

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
  output[1]=fo_rm_roll;   
  output[2]=wrist_pitch;
  output[3]=wrist_roll;
}

}
