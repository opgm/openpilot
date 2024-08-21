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
void err_fun(double *nom_x, double *delta_x, double *out_3860548556518224048) {
   out_3860548556518224048[0] = delta_x[0] + nom_x[0];
   out_3860548556518224048[1] = delta_x[1] + nom_x[1];
   out_3860548556518224048[2] = delta_x[2] + nom_x[2];
   out_3860548556518224048[3] = delta_x[3] + nom_x[3];
   out_3860548556518224048[4] = delta_x[4] + nom_x[4];
   out_3860548556518224048[5] = delta_x[5] + nom_x[5];
   out_3860548556518224048[6] = delta_x[6] + nom_x[6];
   out_3860548556518224048[7] = delta_x[7] + nom_x[7];
   out_3860548556518224048[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4761333446785978408) {
   out_4761333446785978408[0] = -nom_x[0] + true_x[0];
   out_4761333446785978408[1] = -nom_x[1] + true_x[1];
   out_4761333446785978408[2] = -nom_x[2] + true_x[2];
   out_4761333446785978408[3] = -nom_x[3] + true_x[3];
   out_4761333446785978408[4] = -nom_x[4] + true_x[4];
   out_4761333446785978408[5] = -nom_x[5] + true_x[5];
   out_4761333446785978408[6] = -nom_x[6] + true_x[6];
   out_4761333446785978408[7] = -nom_x[7] + true_x[7];
   out_4761333446785978408[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_691743405130441903) {
   out_691743405130441903[0] = 1.0;
   out_691743405130441903[1] = 0;
   out_691743405130441903[2] = 0;
   out_691743405130441903[3] = 0;
   out_691743405130441903[4] = 0;
   out_691743405130441903[5] = 0;
   out_691743405130441903[6] = 0;
   out_691743405130441903[7] = 0;
   out_691743405130441903[8] = 0;
   out_691743405130441903[9] = 0;
   out_691743405130441903[10] = 1.0;
   out_691743405130441903[11] = 0;
   out_691743405130441903[12] = 0;
   out_691743405130441903[13] = 0;
   out_691743405130441903[14] = 0;
   out_691743405130441903[15] = 0;
   out_691743405130441903[16] = 0;
   out_691743405130441903[17] = 0;
   out_691743405130441903[18] = 0;
   out_691743405130441903[19] = 0;
   out_691743405130441903[20] = 1.0;
   out_691743405130441903[21] = 0;
   out_691743405130441903[22] = 0;
   out_691743405130441903[23] = 0;
   out_691743405130441903[24] = 0;
   out_691743405130441903[25] = 0;
   out_691743405130441903[26] = 0;
   out_691743405130441903[27] = 0;
   out_691743405130441903[28] = 0;
   out_691743405130441903[29] = 0;
   out_691743405130441903[30] = 1.0;
   out_691743405130441903[31] = 0;
   out_691743405130441903[32] = 0;
   out_691743405130441903[33] = 0;
   out_691743405130441903[34] = 0;
   out_691743405130441903[35] = 0;
   out_691743405130441903[36] = 0;
   out_691743405130441903[37] = 0;
   out_691743405130441903[38] = 0;
   out_691743405130441903[39] = 0;
   out_691743405130441903[40] = 1.0;
   out_691743405130441903[41] = 0;
   out_691743405130441903[42] = 0;
   out_691743405130441903[43] = 0;
   out_691743405130441903[44] = 0;
   out_691743405130441903[45] = 0;
   out_691743405130441903[46] = 0;
   out_691743405130441903[47] = 0;
   out_691743405130441903[48] = 0;
   out_691743405130441903[49] = 0;
   out_691743405130441903[50] = 1.0;
   out_691743405130441903[51] = 0;
   out_691743405130441903[52] = 0;
   out_691743405130441903[53] = 0;
   out_691743405130441903[54] = 0;
   out_691743405130441903[55] = 0;
   out_691743405130441903[56] = 0;
   out_691743405130441903[57] = 0;
   out_691743405130441903[58] = 0;
   out_691743405130441903[59] = 0;
   out_691743405130441903[60] = 1.0;
   out_691743405130441903[61] = 0;
   out_691743405130441903[62] = 0;
   out_691743405130441903[63] = 0;
   out_691743405130441903[64] = 0;
   out_691743405130441903[65] = 0;
   out_691743405130441903[66] = 0;
   out_691743405130441903[67] = 0;
   out_691743405130441903[68] = 0;
   out_691743405130441903[69] = 0;
   out_691743405130441903[70] = 1.0;
   out_691743405130441903[71] = 0;
   out_691743405130441903[72] = 0;
   out_691743405130441903[73] = 0;
   out_691743405130441903[74] = 0;
   out_691743405130441903[75] = 0;
   out_691743405130441903[76] = 0;
   out_691743405130441903[77] = 0;
   out_691743405130441903[78] = 0;
   out_691743405130441903[79] = 0;
   out_691743405130441903[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_489045563455123421) {
   out_489045563455123421[0] = state[0];
   out_489045563455123421[1] = state[1];
   out_489045563455123421[2] = state[2];
   out_489045563455123421[3] = state[3];
   out_489045563455123421[4] = state[4];
   out_489045563455123421[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_489045563455123421[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_489045563455123421[7] = state[7];
   out_489045563455123421[8] = state[8];
}
void F_fun(double *state, double dt, double *out_7686270435155235008) {
   out_7686270435155235008[0] = 1;
   out_7686270435155235008[1] = 0;
   out_7686270435155235008[2] = 0;
   out_7686270435155235008[3] = 0;
   out_7686270435155235008[4] = 0;
   out_7686270435155235008[5] = 0;
   out_7686270435155235008[6] = 0;
   out_7686270435155235008[7] = 0;
   out_7686270435155235008[8] = 0;
   out_7686270435155235008[9] = 0;
   out_7686270435155235008[10] = 1;
   out_7686270435155235008[11] = 0;
   out_7686270435155235008[12] = 0;
   out_7686270435155235008[13] = 0;
   out_7686270435155235008[14] = 0;
   out_7686270435155235008[15] = 0;
   out_7686270435155235008[16] = 0;
   out_7686270435155235008[17] = 0;
   out_7686270435155235008[18] = 0;
   out_7686270435155235008[19] = 0;
   out_7686270435155235008[20] = 1;
   out_7686270435155235008[21] = 0;
   out_7686270435155235008[22] = 0;
   out_7686270435155235008[23] = 0;
   out_7686270435155235008[24] = 0;
   out_7686270435155235008[25] = 0;
   out_7686270435155235008[26] = 0;
   out_7686270435155235008[27] = 0;
   out_7686270435155235008[28] = 0;
   out_7686270435155235008[29] = 0;
   out_7686270435155235008[30] = 1;
   out_7686270435155235008[31] = 0;
   out_7686270435155235008[32] = 0;
   out_7686270435155235008[33] = 0;
   out_7686270435155235008[34] = 0;
   out_7686270435155235008[35] = 0;
   out_7686270435155235008[36] = 0;
   out_7686270435155235008[37] = 0;
   out_7686270435155235008[38] = 0;
   out_7686270435155235008[39] = 0;
   out_7686270435155235008[40] = 1;
   out_7686270435155235008[41] = 0;
   out_7686270435155235008[42] = 0;
   out_7686270435155235008[43] = 0;
   out_7686270435155235008[44] = 0;
   out_7686270435155235008[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_7686270435155235008[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_7686270435155235008[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7686270435155235008[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7686270435155235008[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_7686270435155235008[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_7686270435155235008[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_7686270435155235008[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_7686270435155235008[53] = -9.8000000000000007*dt;
   out_7686270435155235008[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_7686270435155235008[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_7686270435155235008[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7686270435155235008[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7686270435155235008[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_7686270435155235008[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_7686270435155235008[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_7686270435155235008[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7686270435155235008[62] = 0;
   out_7686270435155235008[63] = 0;
   out_7686270435155235008[64] = 0;
   out_7686270435155235008[65] = 0;
   out_7686270435155235008[66] = 0;
   out_7686270435155235008[67] = 0;
   out_7686270435155235008[68] = 0;
   out_7686270435155235008[69] = 0;
   out_7686270435155235008[70] = 1;
   out_7686270435155235008[71] = 0;
   out_7686270435155235008[72] = 0;
   out_7686270435155235008[73] = 0;
   out_7686270435155235008[74] = 0;
   out_7686270435155235008[75] = 0;
   out_7686270435155235008[76] = 0;
   out_7686270435155235008[77] = 0;
   out_7686270435155235008[78] = 0;
   out_7686270435155235008[79] = 0;
   out_7686270435155235008[80] = 1;
}
void h_25(double *state, double *unused, double *out_9100945793002742709) {
   out_9100945793002742709[0] = state[6];
}
void H_25(double *state, double *unused, double *out_2546563379272368754) {
   out_2546563379272368754[0] = 0;
   out_2546563379272368754[1] = 0;
   out_2546563379272368754[2] = 0;
   out_2546563379272368754[3] = 0;
   out_2546563379272368754[4] = 0;
   out_2546563379272368754[5] = 0;
   out_2546563379272368754[6] = 1;
   out_2546563379272368754[7] = 0;
   out_2546563379272368754[8] = 0;
}
void h_24(double *state, double *unused, double *out_1179022644587177356) {
   out_1179022644587177356[0] = state[4];
   out_1179022644587177356[1] = state[5];
}
void H_24(double *state, double *unused, double *out_1435759772125399144) {
   out_1435759772125399144[0] = 0;
   out_1435759772125399144[1] = 0;
   out_1435759772125399144[2] = 0;
   out_1435759772125399144[3] = 0;
   out_1435759772125399144[4] = 1;
   out_1435759772125399144[5] = 0;
   out_1435759772125399144[6] = 0;
   out_1435759772125399144[7] = 0;
   out_1435759772125399144[8] = 0;
   out_1435759772125399144[9] = 0;
   out_1435759772125399144[10] = 0;
   out_1435759772125399144[11] = 0;
   out_1435759772125399144[12] = 0;
   out_1435759772125399144[13] = 0;
   out_1435759772125399144[14] = 1;
   out_1435759772125399144[15] = 0;
   out_1435759772125399144[16] = 0;
   out_1435759772125399144[17] = 0;
}
void h_30(double *state, double *unused, double *out_1582533023886136466) {
   out_1582533023886136466[0] = state[4];
}
void H_30(double *state, double *unused, double *out_5064896337779617381) {
   out_5064896337779617381[0] = 0;
   out_5064896337779617381[1] = 0;
   out_5064896337779617381[2] = 0;
   out_5064896337779617381[3] = 0;
   out_5064896337779617381[4] = 1;
   out_5064896337779617381[5] = 0;
   out_5064896337779617381[6] = 0;
   out_5064896337779617381[7] = 0;
   out_5064896337779617381[8] = 0;
}
void h_26(double *state, double *unused, double *out_6221346409909539169) {
   out_6221346409909539169[0] = state[7];
}
void H_26(double *state, double *unused, double *out_5851089349033169355) {
   out_5851089349033169355[0] = 0;
   out_5851089349033169355[1] = 0;
   out_5851089349033169355[2] = 0;
   out_5851089349033169355[3] = 0;
   out_5851089349033169355[4] = 0;
   out_5851089349033169355[5] = 0;
   out_5851089349033169355[6] = 0;
   out_5851089349033169355[7] = 1;
   out_5851089349033169355[8] = 0;
}
void h_27(double *state, double *unused, double *out_4003931044820778344) {
   out_4003931044820778344[0] = state[3];
}
void H_27(double *state, double *unused, double *out_7288490408963560598) {
   out_7288490408963560598[0] = 0;
   out_7288490408963560598[1] = 0;
   out_7288490408963560598[2] = 0;
   out_7288490408963560598[3] = 1;
   out_7288490408963560598[4] = 0;
   out_7288490408963560598[5] = 0;
   out_7288490408963560598[6] = 0;
   out_7288490408963560598[7] = 0;
   out_7288490408963560598[8] = 0;
}
void h_29(double *state, double *unused, double *out_2185848908562613983) {
   out_2185848908562613983[0] = state[1];
}
void H_29(double *state, double *unused, double *out_5575127682094009565) {
   out_5575127682094009565[0] = 0;
   out_5575127682094009565[1] = 1;
   out_5575127682094009565[2] = 0;
   out_5575127682094009565[3] = 0;
   out_5575127682094009565[4] = 0;
   out_5575127682094009565[5] = 0;
   out_5575127682094009565[6] = 0;
   out_5575127682094009565[7] = 0;
   out_5575127682094009565[8] = 0;
}
void h_28(double *state, double *unused, double *out_967144993376445659) {
   out_967144993376445659[0] = state[0];
}
void H_28(double *state, double *unused, double *out_492728665024478991) {
   out_492728665024478991[0] = 1;
   out_492728665024478991[1] = 0;
   out_492728665024478991[2] = 0;
   out_492728665024478991[3] = 0;
   out_492728665024478991[4] = 0;
   out_492728665024478991[5] = 0;
   out_492728665024478991[6] = 0;
   out_492728665024478991[7] = 0;
   out_492728665024478991[8] = 0;
}
void h_31(double *state, double *unused, double *out_1640387028753854384) {
   out_1640387028753854384[0] = state[8];
}
void H_31(double *state, double *unused, double *out_2577209341149329182) {
   out_2577209341149329182[0] = 0;
   out_2577209341149329182[1] = 0;
   out_2577209341149329182[2] = 0;
   out_2577209341149329182[3] = 0;
   out_2577209341149329182[4] = 0;
   out_2577209341149329182[5] = 0;
   out_2577209341149329182[6] = 0;
   out_2577209341149329182[7] = 0;
   out_2577209341149329182[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_3860548556518224048) {
  err_fun(nom_x, delta_x, out_3860548556518224048);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4761333446785978408) {
  inv_err_fun(nom_x, true_x, out_4761333446785978408);
}
void car_H_mod_fun(double *state, double *out_691743405130441903) {
  H_mod_fun(state, out_691743405130441903);
}
void car_f_fun(double *state, double dt, double *out_489045563455123421) {
  f_fun(state,  dt, out_489045563455123421);
}
void car_F_fun(double *state, double dt, double *out_7686270435155235008) {
  F_fun(state,  dt, out_7686270435155235008);
}
void car_h_25(double *state, double *unused, double *out_9100945793002742709) {
  h_25(state, unused, out_9100945793002742709);
}
void car_H_25(double *state, double *unused, double *out_2546563379272368754) {
  H_25(state, unused, out_2546563379272368754);
}
void car_h_24(double *state, double *unused, double *out_1179022644587177356) {
  h_24(state, unused, out_1179022644587177356);
}
void car_H_24(double *state, double *unused, double *out_1435759772125399144) {
  H_24(state, unused, out_1435759772125399144);
}
void car_h_30(double *state, double *unused, double *out_1582533023886136466) {
  h_30(state, unused, out_1582533023886136466);
}
void car_H_30(double *state, double *unused, double *out_5064896337779617381) {
  H_30(state, unused, out_5064896337779617381);
}
void car_h_26(double *state, double *unused, double *out_6221346409909539169) {
  h_26(state, unused, out_6221346409909539169);
}
void car_H_26(double *state, double *unused, double *out_5851089349033169355) {
  H_26(state, unused, out_5851089349033169355);
}
void car_h_27(double *state, double *unused, double *out_4003931044820778344) {
  h_27(state, unused, out_4003931044820778344);
}
void car_H_27(double *state, double *unused, double *out_7288490408963560598) {
  H_27(state, unused, out_7288490408963560598);
}
void car_h_29(double *state, double *unused, double *out_2185848908562613983) {
  h_29(state, unused, out_2185848908562613983);
}
void car_H_29(double *state, double *unused, double *out_5575127682094009565) {
  H_29(state, unused, out_5575127682094009565);
}
void car_h_28(double *state, double *unused, double *out_967144993376445659) {
  h_28(state, unused, out_967144993376445659);
}
void car_H_28(double *state, double *unused, double *out_492728665024478991) {
  H_28(state, unused, out_492728665024478991);
}
void car_h_31(double *state, double *unused, double *out_1640387028753854384) {
  h_31(state, unused, out_1640387028753854384);
}
void car_H_31(double *state, double *unused, double *out_2577209341149329182) {
  H_31(state, unused, out_2577209341149329182);
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
