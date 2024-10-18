#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                       Code generated with SymPy 1.12                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_7321584177188704090) {
   out_7321584177188704090[0] = delta_x[0] + nom_x[0];
   out_7321584177188704090[1] = delta_x[1] + nom_x[1];
   out_7321584177188704090[2] = delta_x[2] + nom_x[2];
   out_7321584177188704090[3] = delta_x[3] + nom_x[3];
   out_7321584177188704090[4] = delta_x[4] + nom_x[4];
   out_7321584177188704090[5] = delta_x[5] + nom_x[5];
   out_7321584177188704090[6] = delta_x[6] + nom_x[6];
   out_7321584177188704090[7] = delta_x[7] + nom_x[7];
   out_7321584177188704090[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5388938486158592874) {
   out_5388938486158592874[0] = -nom_x[0] + true_x[0];
   out_5388938486158592874[1] = -nom_x[1] + true_x[1];
   out_5388938486158592874[2] = -nom_x[2] + true_x[2];
   out_5388938486158592874[3] = -nom_x[3] + true_x[3];
   out_5388938486158592874[4] = -nom_x[4] + true_x[4];
   out_5388938486158592874[5] = -nom_x[5] + true_x[5];
   out_5388938486158592874[6] = -nom_x[6] + true_x[6];
   out_5388938486158592874[7] = -nom_x[7] + true_x[7];
   out_5388938486158592874[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_4391130289298168509) {
   out_4391130289298168509[0] = 1.0;
   out_4391130289298168509[1] = 0;
   out_4391130289298168509[2] = 0;
   out_4391130289298168509[3] = 0;
   out_4391130289298168509[4] = 0;
   out_4391130289298168509[5] = 0;
   out_4391130289298168509[6] = 0;
   out_4391130289298168509[7] = 0;
   out_4391130289298168509[8] = 0;
   out_4391130289298168509[9] = 0;
   out_4391130289298168509[10] = 1.0;
   out_4391130289298168509[11] = 0;
   out_4391130289298168509[12] = 0;
   out_4391130289298168509[13] = 0;
   out_4391130289298168509[14] = 0;
   out_4391130289298168509[15] = 0;
   out_4391130289298168509[16] = 0;
   out_4391130289298168509[17] = 0;
   out_4391130289298168509[18] = 0;
   out_4391130289298168509[19] = 0;
   out_4391130289298168509[20] = 1.0;
   out_4391130289298168509[21] = 0;
   out_4391130289298168509[22] = 0;
   out_4391130289298168509[23] = 0;
   out_4391130289298168509[24] = 0;
   out_4391130289298168509[25] = 0;
   out_4391130289298168509[26] = 0;
   out_4391130289298168509[27] = 0;
   out_4391130289298168509[28] = 0;
   out_4391130289298168509[29] = 0;
   out_4391130289298168509[30] = 1.0;
   out_4391130289298168509[31] = 0;
   out_4391130289298168509[32] = 0;
   out_4391130289298168509[33] = 0;
   out_4391130289298168509[34] = 0;
   out_4391130289298168509[35] = 0;
   out_4391130289298168509[36] = 0;
   out_4391130289298168509[37] = 0;
   out_4391130289298168509[38] = 0;
   out_4391130289298168509[39] = 0;
   out_4391130289298168509[40] = 1.0;
   out_4391130289298168509[41] = 0;
   out_4391130289298168509[42] = 0;
   out_4391130289298168509[43] = 0;
   out_4391130289298168509[44] = 0;
   out_4391130289298168509[45] = 0;
   out_4391130289298168509[46] = 0;
   out_4391130289298168509[47] = 0;
   out_4391130289298168509[48] = 0;
   out_4391130289298168509[49] = 0;
   out_4391130289298168509[50] = 1.0;
   out_4391130289298168509[51] = 0;
   out_4391130289298168509[52] = 0;
   out_4391130289298168509[53] = 0;
   out_4391130289298168509[54] = 0;
   out_4391130289298168509[55] = 0;
   out_4391130289298168509[56] = 0;
   out_4391130289298168509[57] = 0;
   out_4391130289298168509[58] = 0;
   out_4391130289298168509[59] = 0;
   out_4391130289298168509[60] = 1.0;
   out_4391130289298168509[61] = 0;
   out_4391130289298168509[62] = 0;
   out_4391130289298168509[63] = 0;
   out_4391130289298168509[64] = 0;
   out_4391130289298168509[65] = 0;
   out_4391130289298168509[66] = 0;
   out_4391130289298168509[67] = 0;
   out_4391130289298168509[68] = 0;
   out_4391130289298168509[69] = 0;
   out_4391130289298168509[70] = 1.0;
   out_4391130289298168509[71] = 0;
   out_4391130289298168509[72] = 0;
   out_4391130289298168509[73] = 0;
   out_4391130289298168509[74] = 0;
   out_4391130289298168509[75] = 0;
   out_4391130289298168509[76] = 0;
   out_4391130289298168509[77] = 0;
   out_4391130289298168509[78] = 0;
   out_4391130289298168509[79] = 0;
   out_4391130289298168509[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_3998517490544307444) {
   out_3998517490544307444[0] = state[0];
   out_3998517490544307444[1] = state[1];
   out_3998517490544307444[2] = state[2];
   out_3998517490544307444[3] = state[3];
   out_3998517490544307444[4] = state[4];
   out_3998517490544307444[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_3998517490544307444[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_3998517490544307444[7] = state[7];
   out_3998517490544307444[8] = state[8];
}
void F_fun(double *state, double dt, double *out_1844626809948872121) {
   out_1844626809948872121[0] = 1;
   out_1844626809948872121[1] = 0;
   out_1844626809948872121[2] = 0;
   out_1844626809948872121[3] = 0;
   out_1844626809948872121[4] = 0;
   out_1844626809948872121[5] = 0;
   out_1844626809948872121[6] = 0;
   out_1844626809948872121[7] = 0;
   out_1844626809948872121[8] = 0;
   out_1844626809948872121[9] = 0;
   out_1844626809948872121[10] = 1;
   out_1844626809948872121[11] = 0;
   out_1844626809948872121[12] = 0;
   out_1844626809948872121[13] = 0;
   out_1844626809948872121[14] = 0;
   out_1844626809948872121[15] = 0;
   out_1844626809948872121[16] = 0;
   out_1844626809948872121[17] = 0;
   out_1844626809948872121[18] = 0;
   out_1844626809948872121[19] = 0;
   out_1844626809948872121[20] = 1;
   out_1844626809948872121[21] = 0;
   out_1844626809948872121[22] = 0;
   out_1844626809948872121[23] = 0;
   out_1844626809948872121[24] = 0;
   out_1844626809948872121[25] = 0;
   out_1844626809948872121[26] = 0;
   out_1844626809948872121[27] = 0;
   out_1844626809948872121[28] = 0;
   out_1844626809948872121[29] = 0;
   out_1844626809948872121[30] = 1;
   out_1844626809948872121[31] = 0;
   out_1844626809948872121[32] = 0;
   out_1844626809948872121[33] = 0;
   out_1844626809948872121[34] = 0;
   out_1844626809948872121[35] = 0;
   out_1844626809948872121[36] = 0;
   out_1844626809948872121[37] = 0;
   out_1844626809948872121[38] = 0;
   out_1844626809948872121[39] = 0;
   out_1844626809948872121[40] = 1;
   out_1844626809948872121[41] = 0;
   out_1844626809948872121[42] = 0;
   out_1844626809948872121[43] = 0;
   out_1844626809948872121[44] = 0;
   out_1844626809948872121[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_1844626809948872121[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_1844626809948872121[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1844626809948872121[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1844626809948872121[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_1844626809948872121[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_1844626809948872121[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_1844626809948872121[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_1844626809948872121[53] = -9.8000000000000007*dt;
   out_1844626809948872121[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_1844626809948872121[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_1844626809948872121[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1844626809948872121[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1844626809948872121[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_1844626809948872121[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_1844626809948872121[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_1844626809948872121[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1844626809948872121[62] = 0;
   out_1844626809948872121[63] = 0;
   out_1844626809948872121[64] = 0;
   out_1844626809948872121[65] = 0;
   out_1844626809948872121[66] = 0;
   out_1844626809948872121[67] = 0;
   out_1844626809948872121[68] = 0;
   out_1844626809948872121[69] = 0;
   out_1844626809948872121[70] = 1;
   out_1844626809948872121[71] = 0;
   out_1844626809948872121[72] = 0;
   out_1844626809948872121[73] = 0;
   out_1844626809948872121[74] = 0;
   out_1844626809948872121[75] = 0;
   out_1844626809948872121[76] = 0;
   out_1844626809948872121[77] = 0;
   out_1844626809948872121[78] = 0;
   out_1844626809948872121[79] = 0;
   out_1844626809948872121[80] = 1;
}
void h_25(double *state, double *unused, double *out_1764050421060515004) {
   out_1764050421060515004[0] = state[6];
}
void H_25(double *state, double *unused, double *out_7474627606809239207) {
   out_7474627606809239207[0] = 0;
   out_7474627606809239207[1] = 0;
   out_7474627606809239207[2] = 0;
   out_7474627606809239207[3] = 0;
   out_7474627606809239207[4] = 0;
   out_7474627606809239207[5] = 0;
   out_7474627606809239207[6] = 1;
   out_7474627606809239207[7] = 0;
   out_7474627606809239207[8] = 0;
}
void h_24(double *state, double *unused, double *out_4230355366619659438) {
   out_4230355366619659438[0] = state[4];
   out_4230355366619659438[1] = state[5];
}
void H_24(double *state, double *unused, double *out_2654306102153250944) {
   out_2654306102153250944[0] = 0;
   out_2654306102153250944[1] = 0;
   out_2654306102153250944[2] = 0;
   out_2654306102153250944[3] = 0;
   out_2654306102153250944[4] = 1;
   out_2654306102153250944[5] = 0;
   out_2654306102153250944[6] = 0;
   out_2654306102153250944[7] = 0;
   out_2654306102153250944[8] = 0;
   out_2654306102153250944[9] = 0;
   out_2654306102153250944[10] = 0;
   out_2654306102153250944[11] = 0;
   out_2654306102153250944[12] = 0;
   out_2654306102153250944[13] = 0;
   out_2654306102153250944[14] = 1;
   out_2654306102153250944[15] = 0;
   out_2654306102153250944[16] = 0;
   out_2654306102153250944[17] = 0;
}
void h_30(double *state, double *unused, double *out_1873399107193507346) {
   out_1873399107193507346[0] = state[4];
}
void H_30(double *state, double *unused, double *out_7345288659665999137) {
   out_7345288659665999137[0] = 0;
   out_7345288659665999137[1] = 0;
   out_7345288659665999137[2] = 0;
   out_7345288659665999137[3] = 0;
   out_7345288659665999137[4] = 1;
   out_7345288659665999137[5] = 0;
   out_7345288659665999137[6] = 0;
   out_7345288659665999137[7] = 0;
   out_7345288659665999137[8] = 0;
}
void h_26(double *state, double *unused, double *out_4910185158637840031) {
   out_4910185158637840031[0] = state[7];
}
void H_26(double *state, double *unused, double *out_3733124287935182983) {
   out_3733124287935182983[0] = 0;
   out_3733124287935182983[1] = 0;
   out_3733124287935182983[2] = 0;
   out_3733124287935182983[3] = 0;
   out_3733124287935182983[4] = 0;
   out_3733124287935182983[5] = 0;
   out_3733124287935182983[6] = 0;
   out_3733124287935182983[7] = 1;
   out_3733124287935182983[8] = 0;
}
void h_27(double *state, double *unused, double *out_2707560273962177439) {
   out_2707560273962177439[0] = state[3];
}
void H_27(double *state, double *unused, double *out_5170525347865574226) {
   out_5170525347865574226[0] = 0;
   out_5170525347865574226[1] = 0;
   out_5170525347865574226[2] = 0;
   out_5170525347865574226[3] = 1;
   out_5170525347865574226[4] = 0;
   out_5170525347865574226[5] = 0;
   out_5170525347865574226[6] = 0;
   out_5170525347865574226[7] = 0;
   out_5170525347865574226[8] = 0;
}
void h_29(double *state, double *unused, double *out_929889254291844911) {
   out_929889254291844911[0] = state[1];
}
void H_29(double *state, double *unused, double *out_7855520003980391321) {
   out_7855520003980391321[0] = 0;
   out_7855520003980391321[1] = 1;
   out_7855520003980391321[2] = 0;
   out_7855520003980391321[3] = 0;
   out_7855520003980391321[4] = 0;
   out_7855520003980391321[5] = 0;
   out_7855520003980391321[6] = 0;
   out_7855520003980391321[7] = 0;
   out_7855520003980391321[8] = 0;
}
void h_28(double *state, double *unused, double *out_6348067511785726757) {
   out_6348067511785726757[0] = state[0];
}
void H_28(double *state, double *unused, double *out_2773120986910860747) {
   out_2773120986910860747[0] = 1;
   out_2773120986910860747[1] = 0;
   out_2773120986910860747[2] = 0;
   out_2773120986910860747[3] = 0;
   out_2773120986910860747[4] = 0;
   out_2773120986910860747[5] = 0;
   out_2773120986910860747[6] = 0;
   out_2773120986910860747[7] = 0;
   out_2773120986910860747[8] = 0;
}
void h_31(double *state, double *unused, double *out_6875827055658026880) {
   out_6875827055658026880[0] = state[8];
}
void H_31(double *state, double *unused, double *out_7505273568686199635) {
   out_7505273568686199635[0] = 0;
   out_7505273568686199635[1] = 0;
   out_7505273568686199635[2] = 0;
   out_7505273568686199635[3] = 0;
   out_7505273568686199635[4] = 0;
   out_7505273568686199635[5] = 0;
   out_7505273568686199635[6] = 0;
   out_7505273568686199635[7] = 0;
   out_7505273568686199635[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_7321584177188704090) {
  err_fun(nom_x, delta_x, out_7321584177188704090);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5388938486158592874) {
  inv_err_fun(nom_x, true_x, out_5388938486158592874);
}
void car_H_mod_fun(double *state, double *out_4391130289298168509) {
  H_mod_fun(state, out_4391130289298168509);
}
void car_f_fun(double *state, double dt, double *out_3998517490544307444) {
  f_fun(state,  dt, out_3998517490544307444);
}
void car_F_fun(double *state, double dt, double *out_1844626809948872121) {
  F_fun(state,  dt, out_1844626809948872121);
}
void car_h_25(double *state, double *unused, double *out_1764050421060515004) {
  h_25(state, unused, out_1764050421060515004);
}
void car_H_25(double *state, double *unused, double *out_7474627606809239207) {
  H_25(state, unused, out_7474627606809239207);
}
void car_h_24(double *state, double *unused, double *out_4230355366619659438) {
  h_24(state, unused, out_4230355366619659438);
}
void car_H_24(double *state, double *unused, double *out_2654306102153250944) {
  H_24(state, unused, out_2654306102153250944);
}
void car_h_30(double *state, double *unused, double *out_1873399107193507346) {
  h_30(state, unused, out_1873399107193507346);
}
void car_H_30(double *state, double *unused, double *out_7345288659665999137) {
  H_30(state, unused, out_7345288659665999137);
}
void car_h_26(double *state, double *unused, double *out_4910185158637840031) {
  h_26(state, unused, out_4910185158637840031);
}
void car_H_26(double *state, double *unused, double *out_3733124287935182983) {
  H_26(state, unused, out_3733124287935182983);
}
void car_h_27(double *state, double *unused, double *out_2707560273962177439) {
  h_27(state, unused, out_2707560273962177439);
}
void car_H_27(double *state, double *unused, double *out_5170525347865574226) {
  H_27(state, unused, out_5170525347865574226);
}
void car_h_29(double *state, double *unused, double *out_929889254291844911) {
  h_29(state, unused, out_929889254291844911);
}
void car_H_29(double *state, double *unused, double *out_7855520003980391321) {
  H_29(state, unused, out_7855520003980391321);
}
void car_h_28(double *state, double *unused, double *out_6348067511785726757) {
  h_28(state, unused, out_6348067511785726757);
}
void car_H_28(double *state, double *unused, double *out_2773120986910860747) {
  H_28(state, unused, out_2773120986910860747);
}
void car_h_31(double *state, double *unused, double *out_6875827055658026880) {
  h_31(state, unused, out_6875827055658026880);
}
void car_H_31(double *state, double *unused, double *out_7505273568686199635) {
  H_31(state, unused, out_7505273568686199635);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_lib_init(car)
