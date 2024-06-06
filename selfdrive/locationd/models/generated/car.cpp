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
void err_fun(double *nom_x, double *delta_x, double *out_3708890511663834590) {
   out_3708890511663834590[0] = delta_x[0] + nom_x[0];
   out_3708890511663834590[1] = delta_x[1] + nom_x[1];
   out_3708890511663834590[2] = delta_x[2] + nom_x[2];
   out_3708890511663834590[3] = delta_x[3] + nom_x[3];
   out_3708890511663834590[4] = delta_x[4] + nom_x[4];
   out_3708890511663834590[5] = delta_x[5] + nom_x[5];
   out_3708890511663834590[6] = delta_x[6] + nom_x[6];
   out_3708890511663834590[7] = delta_x[7] + nom_x[7];
   out_3708890511663834590[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_703520109164007484) {
   out_703520109164007484[0] = -nom_x[0] + true_x[0];
   out_703520109164007484[1] = -nom_x[1] + true_x[1];
   out_703520109164007484[2] = -nom_x[2] + true_x[2];
   out_703520109164007484[3] = -nom_x[3] + true_x[3];
   out_703520109164007484[4] = -nom_x[4] + true_x[4];
   out_703520109164007484[5] = -nom_x[5] + true_x[5];
   out_703520109164007484[6] = -nom_x[6] + true_x[6];
   out_703520109164007484[7] = -nom_x[7] + true_x[7];
   out_703520109164007484[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_4152512945030036996) {
   out_4152512945030036996[0] = 1.0;
   out_4152512945030036996[1] = 0;
   out_4152512945030036996[2] = 0;
   out_4152512945030036996[3] = 0;
   out_4152512945030036996[4] = 0;
   out_4152512945030036996[5] = 0;
   out_4152512945030036996[6] = 0;
   out_4152512945030036996[7] = 0;
   out_4152512945030036996[8] = 0;
   out_4152512945030036996[9] = 0;
   out_4152512945030036996[10] = 1.0;
   out_4152512945030036996[11] = 0;
   out_4152512945030036996[12] = 0;
   out_4152512945030036996[13] = 0;
   out_4152512945030036996[14] = 0;
   out_4152512945030036996[15] = 0;
   out_4152512945030036996[16] = 0;
   out_4152512945030036996[17] = 0;
   out_4152512945030036996[18] = 0;
   out_4152512945030036996[19] = 0;
   out_4152512945030036996[20] = 1.0;
   out_4152512945030036996[21] = 0;
   out_4152512945030036996[22] = 0;
   out_4152512945030036996[23] = 0;
   out_4152512945030036996[24] = 0;
   out_4152512945030036996[25] = 0;
   out_4152512945030036996[26] = 0;
   out_4152512945030036996[27] = 0;
   out_4152512945030036996[28] = 0;
   out_4152512945030036996[29] = 0;
   out_4152512945030036996[30] = 1.0;
   out_4152512945030036996[31] = 0;
   out_4152512945030036996[32] = 0;
   out_4152512945030036996[33] = 0;
   out_4152512945030036996[34] = 0;
   out_4152512945030036996[35] = 0;
   out_4152512945030036996[36] = 0;
   out_4152512945030036996[37] = 0;
   out_4152512945030036996[38] = 0;
   out_4152512945030036996[39] = 0;
   out_4152512945030036996[40] = 1.0;
   out_4152512945030036996[41] = 0;
   out_4152512945030036996[42] = 0;
   out_4152512945030036996[43] = 0;
   out_4152512945030036996[44] = 0;
   out_4152512945030036996[45] = 0;
   out_4152512945030036996[46] = 0;
   out_4152512945030036996[47] = 0;
   out_4152512945030036996[48] = 0;
   out_4152512945030036996[49] = 0;
   out_4152512945030036996[50] = 1.0;
   out_4152512945030036996[51] = 0;
   out_4152512945030036996[52] = 0;
   out_4152512945030036996[53] = 0;
   out_4152512945030036996[54] = 0;
   out_4152512945030036996[55] = 0;
   out_4152512945030036996[56] = 0;
   out_4152512945030036996[57] = 0;
   out_4152512945030036996[58] = 0;
   out_4152512945030036996[59] = 0;
   out_4152512945030036996[60] = 1.0;
   out_4152512945030036996[61] = 0;
   out_4152512945030036996[62] = 0;
   out_4152512945030036996[63] = 0;
   out_4152512945030036996[64] = 0;
   out_4152512945030036996[65] = 0;
   out_4152512945030036996[66] = 0;
   out_4152512945030036996[67] = 0;
   out_4152512945030036996[68] = 0;
   out_4152512945030036996[69] = 0;
   out_4152512945030036996[70] = 1.0;
   out_4152512945030036996[71] = 0;
   out_4152512945030036996[72] = 0;
   out_4152512945030036996[73] = 0;
   out_4152512945030036996[74] = 0;
   out_4152512945030036996[75] = 0;
   out_4152512945030036996[76] = 0;
   out_4152512945030036996[77] = 0;
   out_4152512945030036996[78] = 0;
   out_4152512945030036996[79] = 0;
   out_4152512945030036996[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_3540655432221376455) {
   out_3540655432221376455[0] = state[0];
   out_3540655432221376455[1] = state[1];
   out_3540655432221376455[2] = state[2];
   out_3540655432221376455[3] = state[3];
   out_3540655432221376455[4] = state[4];
   out_3540655432221376455[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_3540655432221376455[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_3540655432221376455[7] = state[7];
   out_3540655432221376455[8] = state[8];
}
void F_fun(double *state, double dt, double *out_6921966207091035699) {
   out_6921966207091035699[0] = 1;
   out_6921966207091035699[1] = 0;
   out_6921966207091035699[2] = 0;
   out_6921966207091035699[3] = 0;
   out_6921966207091035699[4] = 0;
   out_6921966207091035699[5] = 0;
   out_6921966207091035699[6] = 0;
   out_6921966207091035699[7] = 0;
   out_6921966207091035699[8] = 0;
   out_6921966207091035699[9] = 0;
   out_6921966207091035699[10] = 1;
   out_6921966207091035699[11] = 0;
   out_6921966207091035699[12] = 0;
   out_6921966207091035699[13] = 0;
   out_6921966207091035699[14] = 0;
   out_6921966207091035699[15] = 0;
   out_6921966207091035699[16] = 0;
   out_6921966207091035699[17] = 0;
   out_6921966207091035699[18] = 0;
   out_6921966207091035699[19] = 0;
   out_6921966207091035699[20] = 1;
   out_6921966207091035699[21] = 0;
   out_6921966207091035699[22] = 0;
   out_6921966207091035699[23] = 0;
   out_6921966207091035699[24] = 0;
   out_6921966207091035699[25] = 0;
   out_6921966207091035699[26] = 0;
   out_6921966207091035699[27] = 0;
   out_6921966207091035699[28] = 0;
   out_6921966207091035699[29] = 0;
   out_6921966207091035699[30] = 1;
   out_6921966207091035699[31] = 0;
   out_6921966207091035699[32] = 0;
   out_6921966207091035699[33] = 0;
   out_6921966207091035699[34] = 0;
   out_6921966207091035699[35] = 0;
   out_6921966207091035699[36] = 0;
   out_6921966207091035699[37] = 0;
   out_6921966207091035699[38] = 0;
   out_6921966207091035699[39] = 0;
   out_6921966207091035699[40] = 1;
   out_6921966207091035699[41] = 0;
   out_6921966207091035699[42] = 0;
   out_6921966207091035699[43] = 0;
   out_6921966207091035699[44] = 0;
   out_6921966207091035699[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_6921966207091035699[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_6921966207091035699[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6921966207091035699[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6921966207091035699[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_6921966207091035699[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_6921966207091035699[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_6921966207091035699[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_6921966207091035699[53] = -9.8000000000000007*dt;
   out_6921966207091035699[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_6921966207091035699[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_6921966207091035699[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6921966207091035699[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6921966207091035699[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_6921966207091035699[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_6921966207091035699[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_6921966207091035699[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6921966207091035699[62] = 0;
   out_6921966207091035699[63] = 0;
   out_6921966207091035699[64] = 0;
   out_6921966207091035699[65] = 0;
   out_6921966207091035699[66] = 0;
   out_6921966207091035699[67] = 0;
   out_6921966207091035699[68] = 0;
   out_6921966207091035699[69] = 0;
   out_6921966207091035699[70] = 1;
   out_6921966207091035699[71] = 0;
   out_6921966207091035699[72] = 0;
   out_6921966207091035699[73] = 0;
   out_6921966207091035699[74] = 0;
   out_6921966207091035699[75] = 0;
   out_6921966207091035699[76] = 0;
   out_6921966207091035699[77] = 0;
   out_6921966207091035699[78] = 0;
   out_6921966207091035699[79] = 0;
   out_6921966207091035699[80] = 1;
}
void h_25(double *state, double *unused, double *out_311349995666808605) {
   out_311349995666808605[0] = state[6];
}
void H_25(double *state, double *unused, double *out_2192895469755813793) {
   out_2192895469755813793[0] = 0;
   out_2192895469755813793[1] = 0;
   out_2192895469755813793[2] = 0;
   out_2192895469755813793[3] = 0;
   out_2192895469755813793[4] = 0;
   out_2192895469755813793[5] = 0;
   out_2192895469755813793[6] = 1;
   out_2192895469755813793[7] = 0;
   out_2192895469755813793[8] = 0;
}
void h_24(double *state, double *unused, double *out_139077460535787258) {
   out_139077460535787258[0] = state[4];
   out_139077460535787258[1] = state[5];
}
void H_24(double *state, double *unused, double *out_1808816608874694840) {
   out_1808816608874694840[0] = 0;
   out_1808816608874694840[1] = 0;
   out_1808816608874694840[2] = 0;
   out_1808816608874694840[3] = 0;
   out_1808816608874694840[4] = 1;
   out_1808816608874694840[5] = 0;
   out_1808816608874694840[6] = 0;
   out_1808816608874694840[7] = 0;
   out_1808816608874694840[8] = 0;
   out_1808816608874694840[9] = 0;
   out_1808816608874694840[10] = 0;
   out_1808816608874694840[11] = 0;
   out_1808816608874694840[12] = 0;
   out_1808816608874694840[13] = 0;
   out_1808816608874694840[14] = 1;
   out_1808816608874694840[15] = 0;
   out_1808816608874694840[16] = 0;
   out_1808816608874694840[17] = 0;
}
void h_30(double *state, double *unused, double *out_7996142708090176232) {
   out_7996142708090176232[0] = state[4];
}
void H_30(double *state, double *unused, double *out_4723794871735802962) {
   out_4723794871735802962[0] = 0;
   out_4723794871735802962[1] = 0;
   out_4723794871735802962[2] = 0;
   out_4723794871735802962[3] = 0;
   out_4723794871735802962[4] = 1;
   out_4723794871735802962[5] = 0;
   out_4723794871735802962[6] = 0;
   out_4723794871735802962[7] = 0;
   out_4723794871735802962[8] = 0;
}
void h_26(double *state, double *unused, double *out_5448928097958529739) {
   out_5448928097958529739[0] = state[7];
}
void H_26(double *state, double *unused, double *out_5934398788629870017) {
   out_5934398788629870017[0] = 0;
   out_5934398788629870017[1] = 0;
   out_5934398788629870017[2] = 0;
   out_5934398788629870017[3] = 0;
   out_5934398788629870017[4] = 0;
   out_5934398788629870017[5] = 0;
   out_5934398788629870017[6] = 0;
   out_5934398788629870017[7] = 1;
   out_5934398788629870017[8] = 0;
}
void h_27(double *state, double *unused, double *out_8484959468599188535) {
   out_8484959468599188535[0] = state[3];
}
void H_27(double *state, double *unused, double *out_4496997728699478774) {
   out_4496997728699478774[0] = 0;
   out_4496997728699478774[1] = 0;
   out_4496997728699478774[2] = 0;
   out_4496997728699478774[3] = 1;
   out_4496997728699478774[4] = 0;
   out_4496997728699478774[5] = 0;
   out_4496997728699478774[6] = 0;
   out_4496997728699478774[7] = 0;
   out_4496997728699478774[8] = 0;
}
void h_29(double *state, double *unused, double *out_3806337409269702952) {
   out_3806337409269702952[0] = state[1];
}
void H_29(double *state, double *unused, double *out_5234026216050195146) {
   out_5234026216050195146[0] = 0;
   out_5234026216050195146[1] = 1;
   out_5234026216050195146[2] = 0;
   out_5234026216050195146[3] = 0;
   out_5234026216050195146[4] = 0;
   out_5234026216050195146[5] = 0;
   out_5234026216050195146[6] = 0;
   out_5234026216050195146[7] = 0;
   out_5234026216050195146[8] = 0;
}
void h_28(double *state, double *unused, double *out_8611530738599867039) {
   out_8611530738599867039[0] = state[0];
}
void H_28(double *state, double *unused, double *out_151627198980664572) {
   out_151627198980664572[0] = 1;
   out_151627198980664572[1] = 0;
   out_151627198980664572[2] = 0;
   out_151627198980664572[3] = 0;
   out_151627198980664572[4] = 0;
   out_151627198980664572[5] = 0;
   out_151627198980664572[6] = 0;
   out_151627198980664572[7] = 0;
   out_151627198980664572[8] = 0;
}
void h_31(double *state, double *unused, double *out_154163327315679460) {
   out_154163327315679460[0] = state[8];
}
void H_31(double *state, double *unused, double *out_2162249507878853365) {
   out_2162249507878853365[0] = 0;
   out_2162249507878853365[1] = 0;
   out_2162249507878853365[2] = 0;
   out_2162249507878853365[3] = 0;
   out_2162249507878853365[4] = 0;
   out_2162249507878853365[5] = 0;
   out_2162249507878853365[6] = 0;
   out_2162249507878853365[7] = 0;
   out_2162249507878853365[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_3708890511663834590) {
  err_fun(nom_x, delta_x, out_3708890511663834590);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_703520109164007484) {
  inv_err_fun(nom_x, true_x, out_703520109164007484);
}
void car_H_mod_fun(double *state, double *out_4152512945030036996) {
  H_mod_fun(state, out_4152512945030036996);
}
void car_f_fun(double *state, double dt, double *out_3540655432221376455) {
  f_fun(state,  dt, out_3540655432221376455);
}
void car_F_fun(double *state, double dt, double *out_6921966207091035699) {
  F_fun(state,  dt, out_6921966207091035699);
}
void car_h_25(double *state, double *unused, double *out_311349995666808605) {
  h_25(state, unused, out_311349995666808605);
}
void car_H_25(double *state, double *unused, double *out_2192895469755813793) {
  H_25(state, unused, out_2192895469755813793);
}
void car_h_24(double *state, double *unused, double *out_139077460535787258) {
  h_24(state, unused, out_139077460535787258);
}
void car_H_24(double *state, double *unused, double *out_1808816608874694840) {
  H_24(state, unused, out_1808816608874694840);
}
void car_h_30(double *state, double *unused, double *out_7996142708090176232) {
  h_30(state, unused, out_7996142708090176232);
}
void car_H_30(double *state, double *unused, double *out_4723794871735802962) {
  H_30(state, unused, out_4723794871735802962);
}
void car_h_26(double *state, double *unused, double *out_5448928097958529739) {
  h_26(state, unused, out_5448928097958529739);
}
void car_H_26(double *state, double *unused, double *out_5934398788629870017) {
  H_26(state, unused, out_5934398788629870017);
}
void car_h_27(double *state, double *unused, double *out_8484959468599188535) {
  h_27(state, unused, out_8484959468599188535);
}
void car_H_27(double *state, double *unused, double *out_4496997728699478774) {
  H_27(state, unused, out_4496997728699478774);
}
void car_h_29(double *state, double *unused, double *out_3806337409269702952) {
  h_29(state, unused, out_3806337409269702952);
}
void car_H_29(double *state, double *unused, double *out_5234026216050195146) {
  H_29(state, unused, out_5234026216050195146);
}
void car_h_28(double *state, double *unused, double *out_8611530738599867039) {
  h_28(state, unused, out_8611530738599867039);
}
void car_H_28(double *state, double *unused, double *out_151627198980664572) {
  H_28(state, unused, out_151627198980664572);
}
void car_h_31(double *state, double *unused, double *out_154163327315679460) {
  h_31(state, unused, out_154163327315679460);
}
void car_H_31(double *state, double *unused, double *out_2162249507878853365) {
  H_31(state, unused, out_2162249507878853365);
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
