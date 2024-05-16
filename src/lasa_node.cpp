#include "lasa_task_planner.h"

using namespace boost::filesystem;




//*******************************************************************************************
// main()

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "lasa_node");

    ros::NodeHandle nh;
    string DS_ModelName ;
    if(! ros::param::get("VSDS_name",DS_ModelName)) {
        ROS_WARN("NO DS Model Found") ;
        return false;
    }

   ROS_INFO("Lasa Node Launched: %s", (DS_ModelName + "").c_str());
   float currentCartPose[FRI_CART_FRM_DIM] ;

    FastResearchInterface	*FRI;
    std::string packPath = ros::package::getPath("vsds_orient");
    std::cout << packPath << "\n";

    bool					Run							=	true;

    char					c							=	0 ;
    int i = 0;
    int						ResultValue					=	0;
    FRI = new FastResearchInterface((packPath + "/data/Control-FRI-Driver_2ms.init").c_str());

    fprintf(stdout, "Check .\n");

    fprintf(stdout, "OK-OK \n");
    fflush(stdout);

    std::string initJointFile ;
    if (DS_ModelName=="Trapezoid") {
         initJointFile    = packPath + "/data/InitAnglePos_Trapezoid3.txt";
    }
    else if (DS_ModelName== "Worm") {
        initJointFile    = packPath + "/data/InitAnglePos_worm2.txt";
    }
    else if (DS_ModelName== "JShape") {
     initJointFile    = packPath + "/data/InitAnglePos_JShape2.txt";
    }
    else {
         initJointFile    = packPath + "/data/InitAnglePos_Normal.txt";
    }
    string initJointFile_nor    = packPath + "/data/InitAnglePos_Normal.txt";




    while (Run)
    {
        std::string DataPathEnding;
        cout<<"\n"<<"Welchen Test möchtest du durchführen?\n"<<"\n";
        cout<<"Bitte ohne Leerzeichen eingeben\n";
        cin>>DataPathEnding;
        cout<<"\n\n";
        c	=	WaitForKBCharacter(NULL);
        printf("\n\n\n");
        printf("Starting.....");

        switch (c)
        {
        case 'h':
        case 'H':{

            printf("Going to home position... (please wait)\n");
            RunJointTrajectory(FRI, initJointFile);
            break ;
        }
        case 'n':
        case 'N':{

            printf("Going to Normal home position... (please wait)\n");
            RunJointTrajectory(FRI, initJointFile_nor);
            break ;
        }
        case 'g':
        case 'G':{

            FRI_wrapper::JointGravityCompensation(FRI, 60, packPath) ;
            break ;

        }

        case 'o':
        case 'O': {
            // move to initial desired orientation
            std::vector <double> v ;
            Quaterniond q_init_VSDS ;

            ROS_INFO("Going to Desired Full Pose") ;

            if (! ros::param::get("q_init",v)){
                ROS_WARN("q_init Param Not Found !!") ;
            }

            q_init_VSDS.w()=v[0] ;
            q_init_VSDS.x()=v[1] ;
            q_init_VSDS.y()=v[2] ;
            q_init_VSDS.z()=v[3] ;

            FRI->GetMeasuredCartPose(currentCartPose);
            Vec x_d=FRI_wrapper::GetTranslation(currentCartPose) ;
            Matrix3d R_init_VSDS= q_init_VSDS.normalized().toRotationMatrix() ;
            FRI_wrapper::MoveCartesian_MinJerk_FullPose( FRI, 2, FRI->GetFRICycleTime(), x_d,  q_init_VSDS) ;


            break ;}

        case 'w':
        case 'W': {
          lasa_task_planner MylasaPlanner ;
          MylasaPlanner.init(FRI,nh,DS_ModelName) ;
          int CycleCounter=0 ;
          int done =0 ;

          while ((FRI->IsMachineOK()) && ((float)CycleCounter * FRI->GetFRICycleTime()< 40.0 )  && (done==0) ){

              MylasaPlanner.run() ;
              CycleCounter++ ;
             if (dhdKbHit() && dhdKbGet()=='q') done = 1;

          }

          MylasaPlanner.save_data_toFile() ;

            break ;
        }
        case 'q':
        case 'Q':

            ResultValue=FRI->StopRobot() ;

            delete FRI;
            sleep(3);
            return(EXIT_SUCCESS);
        }
    }




}







