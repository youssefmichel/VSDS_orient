#include "lasa_task_planner.h"


lasa_task_planner::lasa_task_planner(){

}

int lasa_task_planner::init(FastResearchInterface *FRI,ros::NodeHandle nh, string DS_ModelName) {

    FRI_=FRI ;
    DS_ModelName_=DS_ModelName ;
    init_datalogging() ;


    FRI_->GetMeasuredJointPositions(currentJointPosition_);
    if(FRI_wrapper::startJointImpedanceCtrl(FRI_,currentJointPosition_)==0){
        ptr_jacobian_ = new float *[7];
        for(int i = 0; i <7; i++){
            ptr_jacobian_[i] = new float[7];
        }

        dt_=FRI_->GetFRICycleTime()  ;

        cout<<"dt: "<<dt_<<endl ;
        FRI_->GetMeasuredCartPose(currentCartPose_);
        x_init_=FRI_wrapper::GetTranslation(currentCartPose_) ;

        MyVSDSQuat_=new VSDSQuat() ; ;
        MyVSDSQuat_->Initiliaze() ;

        for (int i=0 ; i< NUMBER_OF_JOINTS ; i++){
            JointStiffnessValues_[i]=MIN_STIFFNESS ;
            JointDampingValues_[i]= 0 ;
            CommandedJointTorques_[i]=0.0 ;
        }
        FRI_->SetCommandedJointStiffness(JointStiffnessValues_);
        FRI_->SetCommandedJointDamping(JointDampingValues_);
        FRI_->SetCommandedJointTorques(CommandedJointTorques_);
        FRI_->SetCommandedJointPositions(currentJointPosition_);
        FRI_->SetCommandedCartPose(currentCartPose_);

        q_prev_=FRI_wrapper::float_2_Vec(currentJointPosition_,NUMBER_OF_JOINTS) ;
        q_dot_prev_=Vec::Zero(NUMBER_OF_JOINTS) ;
        x_prev_=x_init_ ;
        x_dot_prev_=Vec::Zero(3) ;

        std::vector <double> temp ;

        if(!ros::param::get("D_transl_",temp)) {
            ROS_WARN("Translational Damping Param Not found") ;
            D_transl_=Mat::Identity(3,3) *20 ;
        }
        else {
            D_transl_=Vector3d(temp[0],temp[1],temp[2]).asDiagonal();
        }

        if(!ros::param::get("D_orient_",temp)) {
            ROS_WARN("Translational Damping Param Not found") ;
            D_orient_=Mat::Identity(3,3) *20 ;
        }
        else {
            D_orient_=Vector3d(temp[0],temp[1],temp[2]).asDiagonal();
        }

        if(!ros::param::get("K_transl_",temp)) {
            ROS_WARN("Translational Stiffness Param Not found") ;
            K_transl_=Mat::Identity(3,3) *200 ;
        }
        else {
            K_transl_=Vector3d(temp[0],temp[1],temp[2]).asDiagonal();
        }

        CycleCounter_=0 ;
        x_d_= x_init_ ;


        ROS_INFO("Lasa Task Planner Initialized: %s", (DS_ModelName + "").c_str());

        return 1 ;
    }
    else
    {
        ROS_WARN("Failed to initialize Lasa Task Planner Node !!") ;
        return 0 ;
    }

}

void lasa_task_planner::init_datalogging() {

    printf("       please enter Demo Number \n");
    char n ;
    n	=	WaitForKBCharacter(NULL);
    printf("\n\n\n");
    std::string ss; ss.push_back(n);
    std::string packPath = ros::package::getPath("vsds_orient");
    string Save_Data_dir=packPath + "/data/"+DS_ModelName_+"/" ;

    if ( !exists( Save_Data_dir ) ) { // Check if src folder exists
        boost::filesystem::create_directory(Save_Data_dir);
    }

    x_rob_file_ =Save_Data_dir+"x_rob"+ss+".txt";
    F_ext_file_ =Save_Data_dir+"F_ext"+ss+".txt";
    tau_ext_file_ =Save_Data_dir+"tau_ext"+ss+".txt";
    tau_VSDS_file_ =Save_Data_dir+"tau_VSDS"+ss+".txt";

}

void lasa_task_planner::run(){


    FRI_->WaitForKRCTick();
    FRI_->GetMeasuredCartPose(currentCartPose_);
    FRI_->GetMeasuredJointPositions(currentJointPosition_);
    FRI_->GetEstimatedExternalCartForcesAndTorques(EstimatedExternalCartForcesTorques_);
    FRI_->GetEstimatedExternalJointTorques(EstimatedExternalJointTorques_);
    Matrix3d Rot_mat=FRI_wrapper::GetRotationMatrix(currentCartPose_) ;
    //   Mat Jacobian_Matrix_world=FRI_wrapper::GetFRI_Jacobian(FRI_,ptr_jacobian_,currentCartPose_) ;
    FRI_->GetCurrentJacobianMatrix (ptr_jacobian_);
    Mat Jac_temp=FRI_wrapper::Convert_Jacobian_2Mat(ptr_jacobian_) ;
    Mat Jacobian_Matrix_tool=Jac_temp ;
    Jacobian_Matrix_tool.row(3)=Jac_temp.row(5) ;
    Jacobian_Matrix_tool.row(5)=Jac_temp.row(3) ;
    Mat Jacobian_Matrix_world=FRI_wrapper::Tool_2_World_Jacobian(Jacobian_Matrix_tool,Rot_mat) ;


    Vec x=FRI_wrapper::GetTranslation(currentCartPose_) ;

    Vec q=FRI_wrapper::float_2_Vec(currentJointPosition_,NUMBER_OF_JOINTS) ;

    Eigen::Quaterniond quat_curr_eig(Rot_mat);
    Vec quat_curr_vec=Vec::Zero(4)  ;
    quat_curr_vec(0)=quat_curr_eig.w() ;
    quat_curr_vec(1)=quat_curr_eig.x() ;
    quat_curr_vec(2)=quat_curr_eig.y() ;
    quat_curr_vec(3)=quat_curr_eig.z() ;

    Vec q_dot=(q-q_prev_)/dt_ ; q_prev_=q ;
    q_dot=FRI_wrapper::low_pass(  q_dot,   q_dot_prev_,50.0,dt_ ) ;
    q_dot_prev_=q_dot ;

    Vec x_dot=(x-x_prev_)/dt_ ; x_prev_=x ;
    x_dot=FRI_wrapper::low_pass(  x_dot,  x_dot_prev_, 50.0, dt_ ) ;
    x_dot_prev_=x_dot ;

    Vec F_transl= K_transl_*(x_d_-x)-D_transl_*x_dot ;

   Vec tau_orient_VSDS=MyVSDSQuat_->Update(quat_curr_vec) ;
    Mat Jac_o= Jacobian_Matrix_world.block(3, 0, 3, 7);
    Vec omega = Jac_o*q_dot ;
    Vec tau_o = Jac_o.transpose()*( tau_orient_VSDS - D_orient_*omega );
    Vec tau_x= Jacobian_Matrix_world.block(0, 0, 3, 7).transpose()*F_transl;


    for (int i=0; i<LWR_JNT_NUM ; i++){
         CommandedJointTorques_[i]=tau_o(i)+tau_x(i) ;
        // CommandedJointTorques_[i]=0.0 ;
    }


    FRI_->SetCommandedCartPose(currentCartPose_);
    FRI_->SetCommandedJointPositions(currentJointPosition_);
    FRI_->SetCommandedJointTorques(CommandedJointTorques_);

    x_rob_Vector_.push_back( std::vector <float>() );
    F_ext_Vector_.push_back( std::vector <float>() );
    tau_ext_Vector_.push_back( std::vector <float>() );
    tau_VSDS_Vector_.push_back( std::vector <float>() );

    x_rob_Vector_[CycleCounter_].push_back(x(0));
    x_rob_Vector_[CycleCounter_].push_back(x(1));
    x_rob_Vector_[CycleCounter_].push_back(x(2));

    x_rob_Vector_[CycleCounter_].push_back(quat_curr_vec(0));
    x_rob_Vector_[CycleCounter_].push_back(quat_curr_vec(1));
    x_rob_Vector_[CycleCounter_].push_back(quat_curr_vec(2));
    x_rob_Vector_[CycleCounter_].push_back(quat_curr_vec(3));

    tau_VSDS_Vector_[CycleCounter_].push_back(tau_orient_VSDS(0));
    tau_VSDS_Vector_[CycleCounter_].push_back(tau_orient_VSDS(1));
    tau_VSDS_Vector_[CycleCounter_].push_back(tau_orient_VSDS(2));

    for (int i=0;i <LWR_JNT_NUM ; i++ ){
        tau_ext_Vector_[CycleCounter_].push_back(EstimatedExternalJointTorques_[i]);
    }
    for (int i=0;i <FRI_CART_VEC ; i++ ){
        F_ext_Vector_[CycleCounter_].push_back(EstimatedExternalCartForcesTorques_[i]);
    }

    CycleCounter_ ++ ;

    ros::spinOnce() ;

}

void lasa_task_planner::push_data_toVector() {

}

void lasa_task_planner::save_data_toFile() {


    FRI_wrapper::saveVectorMatrixToFile(x_rob_file_, x_rob_Vector_);
    FRI_wrapper::saveVectorMatrixToFile(F_ext_file_,F_ext_Vector_);
    FRI_wrapper::saveVectorMatrixToFile(tau_ext_file_,tau_ext_Vector_);
    FRI_wrapper::saveVectorMatrixToFile(tau_VSDS_file_,tau_VSDS_Vector_);

    for (int i = 0; i < 7; i++)
    {
        CommandedJointTorques_[i]	=	(float)0.0;
        JointStiffnessValues_[i]=500 ;
        JointDampingValues_[i]=0.7 ;
    }
    FRI_->SetCommandedJointTorques(CommandedJointTorques_);
    FRI_->GetMeasuredJointPositions(currentJointPosition_);
    FRI_->SetCommandedJointPositions(currentJointPosition_);
    FRI_->SetCommandedJointStiffness(JointStiffnessValues_);
    FRI_->SetCommandedJointDamping(JointDampingValues_);

}
