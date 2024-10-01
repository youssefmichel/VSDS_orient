#pragma once
#include <ros/ros.h>
#include <ros/package.h>

#include <FastResearchInterface.h>
#include <FastResearchInterfaceTest.h>
#include <math.h>
#include <string.h>
#include <fstream>
#include <vector>

#include <LWRCartImpedanceController.h>

#include "dhdc.h"
#include "drdc.h"
#include <signal.h>
#include "motion_generation.h"



#ifndef LWR_JNT_NUM
#define LWR_JNT_NUM 7
#endif


using namespace std;

#ifndef PI
#define PI	3.1415926535897932384626433832795
#endif

#ifndef NUMBER_OF_CYCLES_FOR_QUAULITY_CHECK
#define NUMBER_OF_CYCLES_FOR_QUAULITY_CHECK		2000
#endif

#define SIZE_OF_TRANSFER_STRING					32
#define LINEAR_VISCOSITY   30.0
#define MIN_STIFFNESS 0.01

using  namespace  Eigen;

namespace FRI_wrapper{

int startCartImpedanceCtrl(FastResearchInterface *fri, float *commCartPose) ;

int startJointImpedanceCtrl(FastResearchInterface *fri, float *commJointPosition) ;
int loadVectorMatrixFromFile (std::string fileName, int cols, vector<vector<float>> &outMat) ;
Vec SaturationFunc(Vec inp,float max) ;
void saveVectorMatrixToFile (string fileName, vector < vector <float> > outMat) ;

float getSquaredDistance(float a[3], float b[3]) ;
float LowPass(float signal, float prev_filt, float cutt_off, float cycle_time) ;
Vec LowPass(Vec signal, Vec prev_filt, float cutt_off, float cycle_time) ;


float ComputeDampingFactor(float F_ext,float F_ext_min,float F_ext_max) ;
Mat GetRotationMatrix(float *CartPose) ;
Vec GetTranslation(float *CartPose) ; 
Mat ConvertJacobianToMat(float ** Jacobian) ;
Mat ConvertMassMatToMat(float ** Mass) ;
Vec floatToVec(float *inp,int size) ;
Mat ToolToWorldJacobian(Mat Jac_tool,Mat Rot) ;
void SetDesiredPoseFRI(Mat Rot_d,Vec x_d,float *CartPose_d) ;
Vec QuartOrientErr(Mat R_a, Mat R_d);

vector< std::vector <float> > KukaMoveCartesianMinJerk(FastResearchInterface *FRI, float tot_time, float dt, Vec x_df) ;
int MoveCartesianMinJerkFullPose(FastResearchInterface *FRI, float tot_time, float dt, Vec x_df, Quaterniond q_df) ;
int JointGravityCompensation(FastResearchInterface *FRI, float tot_time,string packPath) ;
Mat GetFRIJacobian(FastResearchInterface *FRI,float ** ptr_jacobian, float *currentCartPose ) ;

}




