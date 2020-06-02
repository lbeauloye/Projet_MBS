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


void user_DrivenJoints(MbsData *mbs_data,double tsim)
{
  if(tsim<=10)
  {
    mbs_data->q[1]=0.1*pow(tsim,2);
    mbs_data->qd[1]=0.1*tsim;
    mbs_data->qdd[1]=0.1;
  }
  else
  {
    mbs_data->q[1]=10+1*(tsim-10);
    mbs_data->qd[1]=1;
    mbs_data->qdd[1]=0.0;
  }
}
