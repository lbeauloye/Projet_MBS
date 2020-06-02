//---------------------------
// UCL-CEREM-MBS
//
// @version MBsysLab_s 1.7.a
//
// Creation : 2006
// Last update : 01/10/2008
//---------------------------

#include "math.h"

#include "mbs_data.h"
#include "user_model.h"
#include "set_output.h"
#include "useful_functions.h"

double* user_JointForces(MbsData *mbs_data, double tsim)
{
/*-- Begin of user code --*/

    // example for a spring-damper in joint number 2
    double K = 100;
    double C = 1000;
    double L0 = 0.0;
    mbs_data->Qq[4] = -(K*(mbs_data->q[4]-L0) + C*mbs_data->qd[4]);
    mbs_data->Qq[3] = -(K*(mbs_data->q[3]-L0) + C*mbs_data->qd[3]);


/*-- End of user code --*/

    return mbs_data->Qq;
}
