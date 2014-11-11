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

typedef struct Sphere
{
  std::string name;
  KDL::Vector v;
  double radius;
  int priority;
  int kdl_chain;
  int kdl_segment;
  int link_id;

  void print(std::string text="")
  {
    ROS_INFO("[%s][%s] x: %0.3f y: %0.3f z: %0.3f radius: %0.3f priority: %d chain: %d segment: %d link_id: %d", text.c_str(), name.c_str(), v.x(), v.y(), v.z(), radius, priority, kdl_chain, kdl_segment, link_id);
  }
} Sphere;

struct Voxels
{
  int kdl_chain;
  int kdl_segment;
  std::vector<KDL::Vector> v;
};

struct Link
{
  int type;   // spheres or voxels
  int id_;
  int i_chain_;
  Voxels voxels_;
  std::string name_;
  std::string root_name_;
  std::vector<Sphere> spheres_;
  std::vector<Sphere> low_res_spheres_;

  void print(std::string text)
  { 
    ROS_INFO("[%s] name: %s  root_name: %s  type: %d  i_chain: %d  link_id: %d", text.c_str(), name_.c_str(), root_name_.c_str(), type, i_chain_, id_);

    if(!spheres_.empty())
    {
      ROS_INFO("[%s] Spheres:", text.c_str());
      for(size_t i = 0; i < spheres_.size(); ++i)
        spheres_[i].print(text);
    }
    if(!low_res_spheres_.empty())
    {
      ROS_INFO("[%s] Low Res Spheres:", text.c_str());
      for(size_t i = 0; i < low_res_spheres_.size(); ++i)
        low_res_spheres_[i].print(text);
    }
  }

  void setLinkID(int id)
  {
    id_ = id;
    for(size_t i = 0; i < spheres_.size(); ++i)
      spheres_[i].link_id = id_;
    for(size_t i = 0; i < low_res_spheres_.size(); ++i)
      low_res_spheres_[i].link_id = id_;
  }
};

class Group
{
  public:

    Group(std::string name);
    
    ~Group();

    bool init(boost::shared_ptr<urdf::Model> urdf);

    void print();

    bool getParams(XmlRpc::XmlRpcValue grp, XmlRpc::XmlRpcValue spheres);

    std::string getName();

    std::string getReferenceFrame();

    std::vector<Sphere*> getSpheres(bool low_res);

    void getSpheres(std::vector<Sphere*> &spheres, bool low_res=false);
    
    bool computeFK(const std::vector<double> &angles, int chain, int segment, KDL::Frame &frame);

    bool computeFK(const std::vector<double> &angles, std::vector<std::vector<KDL::Frame> > &frames);
    
    bool computeFK(const KDL::JntArray &angles, int chain, int segment, KDL::Frame &frame);

    void setOrderOfJointPositions(const std::vector<std::string> &joint_names);

    void setJointPosition(const std::string &name, double position);

    bool getFrameInfo(std::string &name, int &chain, int &segment);

    void printSpheres();

    void printDebugInfo();

    void setGroupToWorldTransform(const KDL::Frame &f);

    KDL::Frame getGroupToWorldTransform();

    bool init_;
    enum {SPHERES, VOXELS} type_;
    std::string tip_name_;
    std::vector<Link> links_;

    // for attached object
    Group() : init_(false){};
    void setName(std::string name);
    bool setSpheres(std::vector<Sphere*> &spheres, bool low_res);

  private:

    boost::shared_ptr<urdf::Model> urdf_;

    std::string name_;
    std::string root_name_;
    KDL::Frame T_root_to_world_;

    std::vector<KDL::Chain> chains_;
    std::vector<KDL::ChainFkSolverPos_recursive*> solvers_;
    std::vector<KDL::JntArray> joint_positions_;
    std::vector<std::vector<int> > frames_;
    std::vector<std::vector<std::string> > jntarray_names_;
    std::vector<std::vector<int> > angles_to_jntarray_;
    std::vector<std::string> joint_names_;
    std::vector<std::string> order_of_input_angles_;
    std::vector<Sphere*> spheres_;
    std::vector<Sphere*> low_res_spheres_;

    bool initSpheres();

    bool initVoxels();
  
    bool initKinematics();

    bool getLinkVoxels(std::string name, std::vector<KDL::Vector> &voxels);
};

inline void Group::setGroupToWorldTransform(const KDL::Frame &f)
{
  T_root_to_world_ = f;
}

inline KDL::Frame Group::getGroupToWorldTransform()
{
  return T_root_to_world_;
}

}

#endif
