#ifndef TASK_PLANNER_H
#define TASK_PLANNER_H

#include <math.h>
#include <string.h>
#include <fstream>
#include <vector>
#include "dhdc.h"
#include "drdc.h"
#include "utility_fri.h"
#include <TypeIRML.h>
#include "motion_generation.h"
#include "boost/filesystem.hpp"

namespace vsds_orient_control {

using namespace boost::filesystem;


#ifndef LWR_JNT_NUM
#define LWR_JNT_NUM 7
#endif
#define NUMBER_OF_CART_DOFS	6
#define NUMBER_OF_JOINTS		7


using namespace std;
using namespace StiffnessProfiles;
using namespace GeneralFunction ;

class TaskPlanner{

protected:

    FastResearchInterface	*FRI_;
    VSDSQuat MyVSDSQuat_ ;
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

    virtual void InitDataLogging()=0  ;


    std::vector< std::vector <float> > x_rob_Vector_;
    std::vector< std::vector <float> > F_ext_Vector_;
    std::vector< std::vector <float> > tau_ext_Vector_;
    std::vector< std::vector <float> > tau_VSDS_Vector_;

    std::string x_rob_file_  ;
    std::string F_ext_file_ ;
    std::string tau_ext_file_ ;
    std::string tau_VSDS_file_ ;


public:


   virtual int Init(FastResearchInterface	*FRI_,ros::NodeHandle nh,string DSname) =0;
   virtual void  Run() =0;

   virtual void SaveDataToFile() =0;


} ;

}

#endif
