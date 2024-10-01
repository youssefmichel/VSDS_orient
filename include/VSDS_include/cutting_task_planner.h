#ifndef LASA_TASK_PLANNER_H
#define LASA_TASK_PLANNER_H

#include <math.h>
#include <string.h>
#include <fstream>
#include <vector>

#include "utility_fri.h"
#include "task_planner.h"
#include <TypeIRML.h>
#include "motion_generation.h"

#include "boost/filesystem.hpp"
using namespace boost::filesystem;


#ifndef LWR_JNT_NUM
#define LWR_JNT_NUM 7
#endif
#define NUMBER_OF_CART_DOFS	6
#define NUMBER_OF_JOINTS		7

namespace vsds_orient_control {

using namespace std;
using namespace StiffnessProfiles;
using namespace GeneralFunction ;




class CuttingTaskPlanner: public TaskPlanner{

private:

    void InitDataLogging()  override;
    std::vector< std::vector <float> > motion_des_;
    int file_counter_ ;
    int dim_cut_ ;



public:


   int Init(FastResearchInterface	*FRI,ros::NodeHandle nh,string DSname) override  ;

   void Run() override  ;

   void SaveDataToFile() override  ;


} ;

}

#endif
