
#include "MotionGeneration.h"


VSDSQuat::VSDSQuat(){

}

bool VSDSQuat::Initiliaze(){

    string VSDS_name ;
    if(! ros::param::get("VSDS_name",VSDS_name))
        return false;
    ROS_INFO("Current DS Model Executed:%s ",VSDS_name.c_str() ) ;

    if(! ros::param::get("N_viapoints",N_viapoints_)){
        ROS_WARN("N_via Not Found !!");
        return false;
}

    std::vector <double> v ;


    if (! ros::param::get("q_goal",v)){
        ROS_WARN("q_goal Param Not Found !!");
              return false ;
    }

     q_goal_[0]=v[0] ;
     q_goal_[1]=v[1] ;
     q_goal_[2]=v[2] ;
     q_goal_[3]=v[3] ;

     if (! ros::param::get("q_init",v)){
         ROS_WARN("q_init Param Not Found !!");
               return false ;
     }

      q_init_[0]=v[0] ;
      q_init_[1]=v[1] ;
      q_init_[2]=v[2] ;
      q_init_[3]=v[3] ;




    string packpath=ros::package::getPath("vsds_orient") ;
    string x_len_string = packpath + "/config/" + VSDS_name + "/x_len.txt" ;
    string x_rec_string = packpath + "/config/" + VSDS_name + "/x_rec.txt" ;
    string x_cen_string = packpath + "/config/" + VSDS_name + "/x_cen.txt" ;
    string A_hat_string = packpath + "/config/" + VSDS_name + "/A_hat.txt" ;


    ROS_INFO("Pack Path:%s ",packpath.c_str() ) ;

     ROS_INFO("String X_len:%s ",x_len_string.c_str() ) ;

    x_len_=Vec::Zero(N_viapoints_) ;

    x_rec_=Mat::Zero(3,N_viapoints_+1) ;
    x_cen_=Mat::Zero(3,N_viapoints_) ;
    A_hat_= Mat::Zero(3,N_viapoints_*3) ;


    std::ifstream x_len_file ;
    x_len_file.open(x_len_string.c_str());
    if(x_len_file){
            for(int j=0; j<N_viapoints_; j++){
                x_len_file >> x_len_(j);
            }

        x_len_file.close();
    }
    else {
     ROS_WARN("X_len File Not Found !!");
     return false ;
    }


    std::ifstream x_rec_file ;
    x_rec_file.open(x_rec_string.c_str());
    if(x_rec_file){
        for(int i = 0; i < TS_SIZE; i++){
            for(int j=0; j<N_viapoints_+1 ; j++){
                x_rec_file >> x_rec_(i,j);
            }
        }

        x_rec_file.close();
    }
    else {
     ROS_WARN("X_Rec File Not Found !!");
     return false ;
    }

    std::ifstream x_cen_file ;
    ROS_INFO("String X_cent:%s ",x_cen_string.c_str() ) ;
    x_cen_file.open(x_cen_string.c_str());
    if(x_cen_file){
        for(int i = 0; i < TS_SIZE; i++){
            for(int j=0; j<N_viapoints_; j++){
                x_cen_file >> x_cen_(i,j);
            }
        }

        x_cen_file.close();
    }
    else {
     ROS_WARN("X_Cen File Not Found !!");
     return false ;
    }

    std::ifstream A_hat_file ;
    A_hat_file.open(A_hat_string.c_str());
    if(A_hat_file){
        for(int i = 0; i < TS_SIZE; i++){
            for(int j=0; j<N_viapoints_*3; j++){
                A_hat_file >> A_hat_(i,j);
            }
        }

        A_hat_file.close();
    }
    else {
     ROS_WARN("X_Cen File Not Found !!");
     return false ;
    }

    ROS_INFO("VSDS Initialized properly");

    return true ;


}

Vec VSDSQuat::Omega(Vec x)
{
    Vec omega = Vec::Zero(N_viapoints_);
    realtype sigmascale=1 ;
    Vec delta = sigmascale * x_len_;




    for(int i=0;i <N_viapoints_;i++){



        omega(i) = exp(-(1/(2*delta(i)*delta(i)))*(x-x_cen_.block(0,i,TS_SIZE,1)).transpose()*(x-x_cen_.block(0,i,TS_SIZE,1)));

    }


    realtype omega_sum = omega.sum();
    omega = omega / omega_sum;
    return omega;
}


Vec VSDSQuat::Update(Vec qt_curr_vec){

      // project into tangent space ;
      double qt_curr[4] ; double qt_curr_TS[3];
      GeneralFunction::Vec2double(qt_curr_vec,qt_curr) ;
      QuatUtils::quat_log(qt_curr,q_goal_,qt_curr_TS) ;
      Mat fl = Mat::Zero(TS_SIZE,N_viapoints_);


      Vec omega_curr=Omega(GeneralFunction::double2Vec(qt_curr_TS,3)) ;
      int indx_stf=0 ;

      for(int k=0; k<N_viapoints_; k++){

          //Mat A_hat_temp=280*Mat::Identity(3,3) ;
          Mat A_hat_temp=A_hat_.block(0,indx_stf,3,3) ;
          indx_stf=indx_stf+3  ;
          fl.col(k) =( omega_curr(k)* A_hat_temp * (-GeneralFunction::double2Vec(qt_curr_TS,sizeof(qt_curr_TS)/sizeof(*qt_curr_TS)) + x_rec_.block(0,k+1,TS_SIZE,1)) ) ;
      }


       Vec fvs_sum =  fl.rowwise().sum();

       return fvs_sum ;
}




MinJerk::MinJerk( realtype dt, realtype t_f, Vec x0, Vec xf):

    dt_(dt),
    t_f_(t_f),
    x0_(x0),
    xf_(xf)
{

}

void MinJerk::Update(realtype t)
{
    realtype tau = t / t_f_;
    if (tau>1){
        tau = 1;
    }
    x_d_ = x0_ + ((x0_ - xf_)*((15 * (pow(tau,4))) - (6 * (pow(tau, 5))) - (10 * (pow(tau, 3)))));

}

Vec MinJerk::GetDesiredPos()
{
    return x_d_;
}
