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
 *                      Code generated with SymPy 1.13.2                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_4873356547061745749) {
   out_4873356547061745749[0] = delta_x[0] + nom_x[0];
   out_4873356547061745749[1] = delta_x[1] + nom_x[1];
   out_4873356547061745749[2] = delta_x[2] + nom_x[2];
   out_4873356547061745749[3] = delta_x[3] + nom_x[3];
   out_4873356547061745749[4] = delta_x[4] + nom_x[4];
   out_4873356547061745749[5] = delta_x[5] + nom_x[5];
   out_4873356547061745749[6] = delta_x[6] + nom_x[6];
   out_4873356547061745749[7] = delta_x[7] + nom_x[7];
   out_4873356547061745749[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6992295929644482747) {
   out_6992295929644482747[0] = -nom_x[0] + true_x[0];
   out_6992295929644482747[1] = -nom_x[1] + true_x[1];
   out_6992295929644482747[2] = -nom_x[2] + true_x[2];
   out_6992295929644482747[3] = -nom_x[3] + true_x[3];
   out_6992295929644482747[4] = -nom_x[4] + true_x[4];
   out_6992295929644482747[5] = -nom_x[5] + true_x[5];
   out_6992295929644482747[6] = -nom_x[6] + true_x[6];
   out_6992295929644482747[7] = -nom_x[7] + true_x[7];
   out_6992295929644482747[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_2349472704546962535) {
   out_2349472704546962535[0] = 1.0;
   out_2349472704546962535[1] = 0.0;
   out_2349472704546962535[2] = 0.0;
   out_2349472704546962535[3] = 0.0;
   out_2349472704546962535[4] = 0.0;
   out_2349472704546962535[5] = 0.0;
   out_2349472704546962535[6] = 0.0;
   out_2349472704546962535[7] = 0.0;
   out_2349472704546962535[8] = 0.0;
   out_2349472704546962535[9] = 0.0;
   out_2349472704546962535[10] = 1.0;
   out_2349472704546962535[11] = 0.0;
   out_2349472704546962535[12] = 0.0;
   out_2349472704546962535[13] = 0.0;
   out_2349472704546962535[14] = 0.0;
   out_2349472704546962535[15] = 0.0;
   out_2349472704546962535[16] = 0.0;
   out_2349472704546962535[17] = 0.0;
   out_2349472704546962535[18] = 0.0;
   out_2349472704546962535[19] = 0.0;
   out_2349472704546962535[20] = 1.0;
   out_2349472704546962535[21] = 0.0;
   out_2349472704546962535[22] = 0.0;
   out_2349472704546962535[23] = 0.0;
   out_2349472704546962535[24] = 0.0;
   out_2349472704546962535[25] = 0.0;
   out_2349472704546962535[26] = 0.0;
   out_2349472704546962535[27] = 0.0;
   out_2349472704546962535[28] = 0.0;
   out_2349472704546962535[29] = 0.0;
   out_2349472704546962535[30] = 1.0;
   out_2349472704546962535[31] = 0.0;
   out_2349472704546962535[32] = 0.0;
   out_2349472704546962535[33] = 0.0;
   out_2349472704546962535[34] = 0.0;
   out_2349472704546962535[35] = 0.0;
   out_2349472704546962535[36] = 0.0;
   out_2349472704546962535[37] = 0.0;
   out_2349472704546962535[38] = 0.0;
   out_2349472704546962535[39] = 0.0;
   out_2349472704546962535[40] = 1.0;
   out_2349472704546962535[41] = 0.0;
   out_2349472704546962535[42] = 0.0;
   out_2349472704546962535[43] = 0.0;
   out_2349472704546962535[44] = 0.0;
   out_2349472704546962535[45] = 0.0;
   out_2349472704546962535[46] = 0.0;
   out_2349472704546962535[47] = 0.0;
   out_2349472704546962535[48] = 0.0;
   out_2349472704546962535[49] = 0.0;
   out_2349472704546962535[50] = 1.0;
   out_2349472704546962535[51] = 0.0;
   out_2349472704546962535[52] = 0.0;
   out_2349472704546962535[53] = 0.0;
   out_2349472704546962535[54] = 0.0;
   out_2349472704546962535[55] = 0.0;
   out_2349472704546962535[56] = 0.0;
   out_2349472704546962535[57] = 0.0;
   out_2349472704546962535[58] = 0.0;
   out_2349472704546962535[59] = 0.0;
   out_2349472704546962535[60] = 1.0;
   out_2349472704546962535[61] = 0.0;
   out_2349472704546962535[62] = 0.0;
   out_2349472704546962535[63] = 0.0;
   out_2349472704546962535[64] = 0.0;
   out_2349472704546962535[65] = 0.0;
   out_2349472704546962535[66] = 0.0;
   out_2349472704546962535[67] = 0.0;
   out_2349472704546962535[68] = 0.0;
   out_2349472704546962535[69] = 0.0;
   out_2349472704546962535[70] = 1.0;
   out_2349472704546962535[71] = 0.0;
   out_2349472704546962535[72] = 0.0;
   out_2349472704546962535[73] = 0.0;
   out_2349472704546962535[74] = 0.0;
   out_2349472704546962535[75] = 0.0;
   out_2349472704546962535[76] = 0.0;
   out_2349472704546962535[77] = 0.0;
   out_2349472704546962535[78] = 0.0;
   out_2349472704546962535[79] = 0.0;
   out_2349472704546962535[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_6474478253513842442) {
   out_6474478253513842442[0] = state[0];
   out_6474478253513842442[1] = state[1];
   out_6474478253513842442[2] = state[2];
   out_6474478253513842442[3] = state[3];
   out_6474478253513842442[4] = state[4];
   out_6474478253513842442[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_6474478253513842442[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_6474478253513842442[7] = state[7];
   out_6474478253513842442[8] = state[8];
}
void F_fun(double *state, double dt, double *out_9219203371776498857) {
   out_9219203371776498857[0] = 1;
   out_9219203371776498857[1] = 0;
   out_9219203371776498857[2] = 0;
   out_9219203371776498857[3] = 0;
   out_9219203371776498857[4] = 0;
   out_9219203371776498857[5] = 0;
   out_9219203371776498857[6] = 0;
   out_9219203371776498857[7] = 0;
   out_9219203371776498857[8] = 0;
   out_9219203371776498857[9] = 0;
   out_9219203371776498857[10] = 1;
   out_9219203371776498857[11] = 0;
   out_9219203371776498857[12] = 0;
   out_9219203371776498857[13] = 0;
   out_9219203371776498857[14] = 0;
   out_9219203371776498857[15] = 0;
   out_9219203371776498857[16] = 0;
   out_9219203371776498857[17] = 0;
   out_9219203371776498857[18] = 0;
   out_9219203371776498857[19] = 0;
   out_9219203371776498857[20] = 1;
   out_9219203371776498857[21] = 0;
   out_9219203371776498857[22] = 0;
   out_9219203371776498857[23] = 0;
   out_9219203371776498857[24] = 0;
   out_9219203371776498857[25] = 0;
   out_9219203371776498857[26] = 0;
   out_9219203371776498857[27] = 0;
   out_9219203371776498857[28] = 0;
   out_9219203371776498857[29] = 0;
   out_9219203371776498857[30] = 1;
   out_9219203371776498857[31] = 0;
   out_9219203371776498857[32] = 0;
   out_9219203371776498857[33] = 0;
   out_9219203371776498857[34] = 0;
   out_9219203371776498857[35] = 0;
   out_9219203371776498857[36] = 0;
   out_9219203371776498857[37] = 0;
   out_9219203371776498857[38] = 0;
   out_9219203371776498857[39] = 0;
   out_9219203371776498857[40] = 1;
   out_9219203371776498857[41] = 0;
   out_9219203371776498857[42] = 0;
   out_9219203371776498857[43] = 0;
   out_9219203371776498857[44] = 0;
   out_9219203371776498857[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_9219203371776498857[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_9219203371776498857[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_9219203371776498857[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_9219203371776498857[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_9219203371776498857[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_9219203371776498857[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_9219203371776498857[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_9219203371776498857[53] = -9.8000000000000007*dt;
   out_9219203371776498857[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_9219203371776498857[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_9219203371776498857[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_9219203371776498857[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_9219203371776498857[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_9219203371776498857[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_9219203371776498857[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_9219203371776498857[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_9219203371776498857[62] = 0;
   out_9219203371776498857[63] = 0;
   out_9219203371776498857[64] = 0;
   out_9219203371776498857[65] = 0;
   out_9219203371776498857[66] = 0;
   out_9219203371776498857[67] = 0;
   out_9219203371776498857[68] = 0;
   out_9219203371776498857[69] = 0;
   out_9219203371776498857[70] = 1;
   out_9219203371776498857[71] = 0;
   out_9219203371776498857[72] = 0;
   out_9219203371776498857[73] = 0;
   out_9219203371776498857[74] = 0;
   out_9219203371776498857[75] = 0;
   out_9219203371776498857[76] = 0;
   out_9219203371776498857[77] = 0;
   out_9219203371776498857[78] = 0;
   out_9219203371776498857[79] = 0;
   out_9219203371776498857[80] = 1;
}
void h_25(double *state, double *unused, double *out_2366996280747664513) {
   out_2366996280747664513[0] = state[6];
}
void H_25(double *state, double *unused, double *out_7253164529731627375) {
   out_7253164529731627375[0] = 0;
   out_7253164529731627375[1] = 0;
   out_7253164529731627375[2] = 0;
   out_7253164529731627375[3] = 0;
   out_7253164529731627375[4] = 0;
   out_7253164529731627375[5] = 0;
   out_7253164529731627375[6] = 1;
   out_7253164529731627375[7] = 0;
   out_7253164529731627375[8] = 0;
}
void h_24(double *state, double *unused, double *out_2911288029590514583) {
   out_2911288029590514583[0] = state[4];
   out_2911288029590514583[1] = state[5];
}
void H_24(double *state, double *unused, double *out_2384349664703920523) {
   out_2384349664703920523[0] = 0;
   out_2384349664703920523[1] = 0;
   out_2384349664703920523[2] = 0;
   out_2384349664703920523[3] = 0;
   out_2384349664703920523[4] = 1;
   out_2384349664703920523[5] = 0;
   out_2384349664703920523[6] = 0;
   out_2384349664703920523[7] = 0;
   out_2384349664703920523[8] = 0;
   out_2384349664703920523[9] = 0;
   out_2384349664703920523[10] = 0;
   out_2384349664703920523[11] = 0;
   out_2384349664703920523[12] = 0;
   out_2384349664703920523[13] = 0;
   out_2384349664703920523[14] = 1;
   out_2384349664703920523[15] = 0;
   out_2384349664703920523[16] = 0;
   out_2384349664703920523[17] = 0;
}
void h_30(double *state, double *unused, double *out_2642190343032170402) {
   out_2642190343032170402[0] = state[4];
}
void H_30(double *state, double *unused, double *out_2725468199604019177) {
   out_2725468199604019177[0] = 0;
   out_2725468199604019177[1] = 0;
   out_2725468199604019177[2] = 0;
   out_2725468199604019177[3] = 0;
   out_2725468199604019177[4] = 1;
   out_2725468199604019177[5] = 0;
   out_2725468199604019177[6] = 0;
   out_2725468199604019177[7] = 0;
   out_2725468199604019177[8] = 0;
}
void h_26(double *state, double *unused, double *out_9041231860446019548) {
   out_9041231860446019548[0] = state[7];
}
void H_26(double *state, double *unused, double *out_3511661210857571151) {
   out_3511661210857571151[0] = 0;
   out_3511661210857571151[1] = 0;
   out_3511661210857571151[2] = 0;
   out_3511661210857571151[3] = 0;
   out_3511661210857571151[4] = 0;
   out_3511661210857571151[5] = 0;
   out_3511661210857571151[6] = 0;
   out_3511661210857571151[7] = 1;
   out_3511661210857571151[8] = 0;
}
void h_27(double *state, double *unused, double *out_7368669548408510158) {
   out_7368669548408510158[0] = state[3];
}
void H_27(double *state, double *unused, double *out_4949062270787962394) {
   out_4949062270787962394[0] = 0;
   out_4949062270787962394[1] = 0;
   out_4949062270787962394[2] = 0;
   out_4949062270787962394[3] = 1;
   out_4949062270787962394[4] = 0;
   out_4949062270787962394[5] = 0;
   out_4949062270787962394[6] = 0;
   out_4949062270787962394[7] = 0;
   out_4949062270787962394[8] = 0;
}
void h_29(double *state, double *unused, double *out_7643863610693016047) {
   out_7643863610693016047[0] = state[1];
}
void H_29(double *state, double *unused, double *out_3235699543918411361) {
   out_3235699543918411361[0] = 0;
   out_3235699543918411361[1] = 1;
   out_3235699543918411361[2] = 0;
   out_3235699543918411361[3] = 0;
   out_3235699543918411361[4] = 0;
   out_3235699543918411361[5] = 0;
   out_3235699543918411361[6] = 0;
   out_3235699543918411361[7] = 0;
   out_3235699543918411361[8] = 0;
}
void h_28(double *state, double *unused, double *out_6425159695506847723) {
   out_6425159695506847723[0] = state[0];
}
void H_28(double *state, double *unused, double *out_5199329815483737612) {
   out_5199329815483737612[0] = 1;
   out_5199329815483737612[1] = 0;
   out_5199329815483737612[2] = 0;
   out_5199329815483737612[3] = 0;
   out_5199329815483737612[4] = 0;
   out_5199329815483737612[5] = 0;
   out_5199329815483737612[6] = 0;
   out_5199329815483737612[7] = 0;
   out_5199329815483737612[8] = 0;
}
void h_31(double *state, double *unused, double *out_1270453247506357837) {
   out_1270453247506357837[0] = state[8];
}
void H_31(double *state, double *unused, double *out_2885453108624219675) {
   out_2885453108624219675[0] = 0;
   out_2885453108624219675[1] = 0;
   out_2885453108624219675[2] = 0;
   out_2885453108624219675[3] = 0;
   out_2885453108624219675[4] = 0;
   out_2885453108624219675[5] = 0;
   out_2885453108624219675[6] = 0;
   out_2885453108624219675[7] = 0;
   out_2885453108624219675[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_4873356547061745749) {
  err_fun(nom_x, delta_x, out_4873356547061745749);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6992295929644482747) {
  inv_err_fun(nom_x, true_x, out_6992295929644482747);
}
void car_H_mod_fun(double *state, double *out_2349472704546962535) {
  H_mod_fun(state, out_2349472704546962535);
}
void car_f_fun(double *state, double dt, double *out_6474478253513842442) {
  f_fun(state,  dt, out_6474478253513842442);
}
void car_F_fun(double *state, double dt, double *out_9219203371776498857) {
  F_fun(state,  dt, out_9219203371776498857);
}
void car_h_25(double *state, double *unused, double *out_2366996280747664513) {
  h_25(state, unused, out_2366996280747664513);
}
void car_H_25(double *state, double *unused, double *out_7253164529731627375) {
  H_25(state, unused, out_7253164529731627375);
}
void car_h_24(double *state, double *unused, double *out_2911288029590514583) {
  h_24(state, unused, out_2911288029590514583);
}
void car_H_24(double *state, double *unused, double *out_2384349664703920523) {
  H_24(state, unused, out_2384349664703920523);
}
void car_h_30(double *state, double *unused, double *out_2642190343032170402) {
  h_30(state, unused, out_2642190343032170402);
}
void car_H_30(double *state, double *unused, double *out_2725468199604019177) {
  H_30(state, unused, out_2725468199604019177);
}
void car_h_26(double *state, double *unused, double *out_9041231860446019548) {
  h_26(state, unused, out_9041231860446019548);
}
void car_H_26(double *state, double *unused, double *out_3511661210857571151) {
  H_26(state, unused, out_3511661210857571151);
}
void car_h_27(double *state, double *unused, double *out_7368669548408510158) {
  h_27(state, unused, out_7368669548408510158);
}
void car_H_27(double *state, double *unused, double *out_4949062270787962394) {
  H_27(state, unused, out_4949062270787962394);
}
void car_h_29(double *state, double *unused, double *out_7643863610693016047) {
  h_29(state, unused, out_7643863610693016047);
}
void car_H_29(double *state, double *unused, double *out_3235699543918411361) {
  H_29(state, unused, out_3235699543918411361);
}
void car_h_28(double *state, double *unused, double *out_6425159695506847723) {
  h_28(state, unused, out_6425159695506847723);
}
void car_H_28(double *state, double *unused, double *out_5199329815483737612) {
  H_28(state, unused, out_5199329815483737612);
}
void car_h_31(double *state, double *unused, double *out_1270453247506357837) {
  h_31(state, unused, out_1270453247506357837);
}
void car_H_31(double *state, double *unused, double *out_2885453108624219675) {
  H_31(state, unused, out_2885453108624219675);
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
