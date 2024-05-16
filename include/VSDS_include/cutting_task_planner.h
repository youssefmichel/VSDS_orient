#ifndef LASA_TASK_PLANNER_H
#define LASA_TASK_PLANNER_H

#include <math.h>
#include <string.h>
#include <fstream>
#include <vector>
#include "dhdc.h"
#include "drdc.h"
#include "utility_fri.h"
#include "task_planner.h"
#include <TypeIRML.h>
#include "MotionGeneration.h"

#include "boost/filesystem.hpp"
using namespace boost::filesystem;


#ifndef LWR_JNT_NUM
#define LWR_JNT_NUM 7
#endif
#define NUMBER_OF_CART_DOFS	6
#define NUMBER_OF_JOINTS		7


using namespace std;
using namespace StiffnessProfiles;
using namespace GeneralFunction ;

class cutting_task_planner: public task_planner{

private:
    FastResearchInterface	*FRI_;
    VSDSQuat *MyVSDSQuat_ ;
    float **ptr_jacobian_;

    float  JointStiffnessValues_[LBR_MNJ],
            CommandedJointTorques_[LBR_MNJ],
            JointDampingValues_[LBR_MNJ],
            EstimatedExternalCartForcesTorques_[FRI_CART_VEC],
            EstimatedExternalJointTorques_[LBR_MNJ];
    float currentCartPose_[FRI_CART_FRM_DIM] ;
    float currentJointPosition_[LWR_JNT_NUM];
    Vec x_init_ ;
    Vec q_dot_prev_ ;
    Vec q_prev_ ;

    Vec x_prev_ ;
    Vec x_dot_prev_ ;
    Matrix3d D_transl_ ;
    Matrix3d K_transl_ ;
    Matrix3d D_orient_ ;
    int CycleCounter_ ;
    Vec x_d_ ;
    string DS_ModelName_ ;
    realtype dt_;
    void init_datalogging()  override;
    std::vector< std::vector <float> > x_rob_Vector_;
    std::vector< std::vector <float> > F_ext_Vector_;
    std::vector< std::vector <float> > tau_ext_Vector_;
    std::vector< std::vector <float> > tau_VSDS_Vector_;
    std::string x_rob_file_  ;
    std::string F_ext_file_ ;
    std::string tau_ext_file_ ;
    std::string tau_VSDS_file_ ;
    std::vector< std::vector <float> > motion_des_;
    int file_counter_ ;
    int dim_cut_ ;



public:

   cutting_task_planner() ;
   int init(FastResearchInterface	*FRI,ros::NodeHandle nh,string DSname) override  ;

   void  run() override  ;
   void push_data_toVector() override ;
   void save_data_toFile() override  ;


} ;


#endif
