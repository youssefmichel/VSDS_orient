#ifndef MOTIONGENERATION_H_
#define MOTIONGENERATION_H_



#include "utility.h"

#include "quatUtils.h"

#define TS_SIZE 3  //define Tangen space size

namespace vsds_orient_control {


class VSDSQuat {
private:

   double q_goal_[4] ;
   double q_init_[4] ;
   int N_viapoints_ ;
   Vec x_len_  ;
   Mat x_rec_  ;
   Mat x_cen_  ;
   Mat A_hat_ ;

public:
    VSDSQuat() ;
    bool Initiliaze();
    Vec  Update(Vec q_curr) ;
    Vec computeOmega(Vec x) ;
    ~VSDSQuat() ;
};
}

class MinJerk {
private:
    realtype t_tot_;
    realtype dt_;
    realtype t_f_;
    Vec x0_;
    Vec xf_;
    Vec x_d_;




public:
    MinJerk( realtype dt, realtype t_f, Vec x0, Vec xf);
    void Update(realtype t);
    Vec GetDesiredPos();
     MinJerk() ;
    ~MinJerk() ;


};


#endif
