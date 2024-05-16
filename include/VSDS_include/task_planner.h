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

class task_planner{

protected:

    virtual void init_datalogging() ;


public:


   virtual int init(FastResearchInterface	*FRI_,ros::NodeHandle nh,string DSname) ;

   virtual void  run() ;
   virtual void push_data_toVector() ;
   virtual void save_data_toFile() ;


} ;


#endif
