#ifndef _GROUP_
#define _GROUP_

#include <ros/ros.h>
#include <vector>
#include <string>
#include <algorithm>
#include <boost/shared_ptr.hpp>
#include <boost/algorithm/string.hpp>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <leatherman/utils.h>
#include <leatherman/print.h>

namespace sbpl_arm_planner
{

struct Sphere
{
  std::string name;
  KDL::Vector v;
  double radius;
  int priority;
  int kdl_chain;
  int kdl_segment;
};

struct Voxels
{
  int kdl_chain;
  int kdl_segment;
  std::vector<KDL::Vector> v;
};

struct Link
{
  int type;   // spheres or voxels
  int i_chain_;
  Voxels voxels_;
  std::string name_;
  std::string root_name_;
  std::vector<Sphere> spheres_;
};

class Group
{
  public:

    Group(std::string name);
    
    ~Group();

    bool init(boost::shared_ptr<urdf::Model> urdf);

    void print();

    bool getParams(XmlRpc::XmlRpcValue grp, XmlRpc::XmlRpcValue spheres);

    void getSpheres(std::vector<Sphere*> &spheres);
    
    bool computeFK(const std::vector<double> &angles, int chain, int segment, KDL::Frame &frame);

    void setOrderOfJointPositions(const std::vector<std::string> &joint_names);

    void setJointPosition(const std::string &name, double position);

    bool getFrameInfo(std::string &name, int &chain, int &segment);

    std::string getReferenceFrame();

    void printSpheres();

    bool init_;
    enum {SPHERES, VOXELS} type_;

    std::string name_;
    std::string root_name_;
    std::string tip_name_;
    std::string redundancy_name_;
    KDL::Chain chain_;
    std::vector<KDL::Chain> chains_;
    std::vector<KDL::ChainFkSolverPos_recursive*> solvers_;
    std::vector<KDL::JntArray> joint_positions_;
    std::vector<std::vector<int> > frames_;
    std::vector<std::vector<std::string> > jntarray_names_;
    std::vector<std::vector<int> > angles_to_jntarray_;
    std::vector<std::string> joint_names_;
    std::vector<std::string> order_of_input_angles_;
    std::vector<Link> links_;
    std::vector<Sphere*> spheres_;

  private:

    boost::shared_ptr<urdf::Model> urdf_;

    //void getParams();

    bool initSpheres();

    bool initVoxels();
  
    bool initKinematics();

    bool getLinkVoxels(std::string name, std::vector<KDL::Vector> &voxels);
};

}

#endif
