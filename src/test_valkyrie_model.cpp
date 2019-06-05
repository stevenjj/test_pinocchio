#define PINOCCHIO_WITH_HPP_FCL

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/geometry.hpp"

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/parsers/srdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include <math.h>       


#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/frames.hpp"

#include <Configuration.h>

int main(int argc, char ** argv)
{
  std::string filename = THIS_COM"models/Valkyrie/r5_urdf_rbdl.urdf";
  std::string meshDir  = THIS_COM"/../val_model/";

  pinocchio::Model model;
  pinocchio::urdf::buildModel(filename, pinocchio::JointModelFreeFlyer(),model);

  std::vector < std::string > packageDirs;
  packageDirs.push_back(meshDir);
  pinocchio::GeometryModel geom;
  pinocchio::urdf::buildGeom(model, filename, pinocchio::COLLISION, geom, packageDirs);

  pinocchio::Data data(model);
  pinocchio::GeometryData geomData(geom);

  pinocchio::computeBodyRadius(model, geom, geomData);


  // Eigen::VectorXd q = pinocchio::randomConfiguration(model);

  Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);

  // floating base joints: x, y, z
  q[0] = 0.0;
  q[1] = 0.0;
  q[2] = 0.0;

  double theta = M_PI/4.0;
  std::cout << "pi = " << M_PI << std::endl;

  // floating base quaternion: qx, qy, qz, qw
  q[3] = 0.0;
  q[4] = 0.0;
  q[5] = sin(theta/2.0);
  q[6] = cos(theta/2.0);

  std::cout << "q = " << q.transpose() << std::endl;

  // Note that this function does not check if the quaternion is a valid unit quaternion.
  pinocchio::forwardKinematics(model,data,q);

  for (int k=0 ; k<model.njoints ; ++k){
    std::cout << model.names[k] << "\t: " << "translation:" << data.oMi[k].translation().transpose() << std::endl;
    std::cout << "rotation:\n" << data.oMi[k].rotation() << std::endl;
  }

  std::cout << "size of q: " << q.size() << std::endl;
  std::cout << "num of joints: " << model.njoints << std::endl;


  // List joint names:
  // for (int k=0 ; k<model.njoints ; ++k){
  //   std::cout << "id:" << k << " " << model.names[k] <<  std::endl;
  // }  

  // List frames
  // std::cout << "number of frames in the model:" << model.frames.size() << std::endl;
  // for (int k=0 ; k<model.frames.size() ; ++k){
  //   std::cout << "frame:" << k << " " << model.frames.names[k] <<  std::endl;
  // }


  // Get frame of the right Palm and compute its Jacobian
  pinocchio::FrameIndex rp_index = model.getFrameId("rightPalm");
  std::cout << "frame: " << model.frames[rp_index] << std::endl;

  pinocchio::Data::Matrix6x J_rpalm(6,model.nv); J_rpalm.fill(0);
  pinocchio::computeJointJacobians(model,data,q);
  pinocchio::updateFramePlacement(model, data, rp_index);
  pinocchio::getFrameJacobian(model,     data,        rp_index, pinocchio::WORLD, J_rpalm);

  std::cout << "J_rpalm w.r.t world:" << std::endl;
  std::cout << J_rpalm << std::endl;

  std::cout << "top 3 rows of J rpalm" << std::endl;
  pinocchio::Data::Matrix3x J_rpalm_trans = J_rpalm.topRows<3>() ;
  std::cout << J_rpalm_trans << std::endl;

  std::cout << "num of body radius = " << geomData.radius.size() << std::endl;

}

