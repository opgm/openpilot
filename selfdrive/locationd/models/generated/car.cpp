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
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_6990374026437360235) {
   out_6990374026437360235[0] = delta_x[0] + nom_x[0];
   out_6990374026437360235[1] = delta_x[1] + nom_x[1];
   out_6990374026437360235[2] = delta_x[2] + nom_x[2];
   out_6990374026437360235[3] = delta_x[3] + nom_x[3];
   out_6990374026437360235[4] = delta_x[4] + nom_x[4];
   out_6990374026437360235[5] = delta_x[5] + nom_x[5];
   out_6990374026437360235[6] = delta_x[6] + nom_x[6];
   out_6990374026437360235[7] = delta_x[7] + nom_x[7];
   out_6990374026437360235[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6840343405846249272) {
   out_6840343405846249272[0] = -nom_x[0] + true_x[0];
   out_6840343405846249272[1] = -nom_x[1] + true_x[1];
   out_6840343405846249272[2] = -nom_x[2] + true_x[2];
   out_6840343405846249272[3] = -nom_x[3] + true_x[3];
   out_6840343405846249272[4] = -nom_x[4] + true_x[4];
   out_6840343405846249272[5] = -nom_x[5] + true_x[5];
   out_6840343405846249272[6] = -nom_x[6] + true_x[6];
   out_6840343405846249272[7] = -nom_x[7] + true_x[7];
   out_6840343405846249272[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_3104495035882877201) {
   out_3104495035882877201[0] = 1.0;
   out_3104495035882877201[1] = 0.0;
   out_3104495035882877201[2] = 0.0;
   out_3104495035882877201[3] = 0.0;
   out_3104495035882877201[4] = 0.0;
   out_3104495035882877201[5] = 0.0;
   out_3104495035882877201[6] = 0.0;
   out_3104495035882877201[7] = 0.0;
   out_3104495035882877201[8] = 0.0;
   out_3104495035882877201[9] = 0.0;
   out_3104495035882877201[10] = 1.0;
   out_3104495035882877201[11] = 0.0;
   out_3104495035882877201[12] = 0.0;
   out_3104495035882877201[13] = 0.0;
   out_3104495035882877201[14] = 0.0;
   out_3104495035882877201[15] = 0.0;
   out_3104495035882877201[16] = 0.0;
   out_3104495035882877201[17] = 0.0;
   out_3104495035882877201[18] = 0.0;
   out_3104495035882877201[19] = 0.0;
   out_3104495035882877201[20] = 1.0;
   out_3104495035882877201[21] = 0.0;
   out_3104495035882877201[22] = 0.0;
   out_3104495035882877201[23] = 0.0;
   out_3104495035882877201[24] = 0.0;
   out_3104495035882877201[25] = 0.0;
   out_3104495035882877201[26] = 0.0;
   out_3104495035882877201[27] = 0.0;
   out_3104495035882877201[28] = 0.0;
   out_3104495035882877201[29] = 0.0;
   out_3104495035882877201[30] = 1.0;
   out_3104495035882877201[31] = 0.0;
   out_3104495035882877201[32] = 0.0;
   out_3104495035882877201[33] = 0.0;
   out_3104495035882877201[34] = 0.0;
   out_3104495035882877201[35] = 0.0;
   out_3104495035882877201[36] = 0.0;
   out_3104495035882877201[37] = 0.0;
   out_3104495035882877201[38] = 0.0;
   out_3104495035882877201[39] = 0.0;
   out_3104495035882877201[40] = 1.0;
   out_3104495035882877201[41] = 0.0;
   out_3104495035882877201[42] = 0.0;
   out_3104495035882877201[43] = 0.0;
   out_3104495035882877201[44] = 0.0;
   out_3104495035882877201[45] = 0.0;
   out_3104495035882877201[46] = 0.0;
   out_3104495035882877201[47] = 0.0;
   out_3104495035882877201[48] = 0.0;
   out_3104495035882877201[49] = 0.0;
   out_3104495035882877201[50] = 1.0;
   out_3104495035882877201[51] = 0.0;
   out_3104495035882877201[52] = 0.0;
   out_3104495035882877201[53] = 0.0;
   out_3104495035882877201[54] = 0.0;
   out_3104495035882877201[55] = 0.0;
   out_3104495035882877201[56] = 0.0;
   out_3104495035882877201[57] = 0.0;
   out_3104495035882877201[58] = 0.0;
   out_3104495035882877201[59] = 0.0;
   out_3104495035882877201[60] = 1.0;
   out_3104495035882877201[61] = 0.0;
   out_3104495035882877201[62] = 0.0;
   out_3104495035882877201[63] = 0.0;
   out_3104495035882877201[64] = 0.0;
   out_3104495035882877201[65] = 0.0;
   out_3104495035882877201[66] = 0.0;
   out_3104495035882877201[67] = 0.0;
   out_3104495035882877201[68] = 0.0;
   out_3104495035882877201[69] = 0.0;
   out_3104495035882877201[70] = 1.0;
   out_3104495035882877201[71] = 0.0;
   out_3104495035882877201[72] = 0.0;
   out_3104495035882877201[73] = 0.0;
   out_3104495035882877201[74] = 0.0;
   out_3104495035882877201[75] = 0.0;
   out_3104495035882877201[76] = 0.0;
   out_3104495035882877201[77] = 0.0;
   out_3104495035882877201[78] = 0.0;
   out_3104495035882877201[79] = 0.0;
   out_3104495035882877201[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_2810783572133327735) {
   out_2810783572133327735[0] = state[0];
   out_2810783572133327735[1] = state[1];
   out_2810783572133327735[2] = state[2];
   out_2810783572133327735[3] = state[3];
   out_2810783572133327735[4] = state[4];
   out_2810783572133327735[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8100000000000005*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_2810783572133327735[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_2810783572133327735[7] = state[7];
   out_2810783572133327735[8] = state[8];
}
void F_fun(double *state, double dt, double *out_7959357771420397480) {
   out_7959357771420397480[0] = 1;
   out_7959357771420397480[1] = 0;
   out_7959357771420397480[2] = 0;
   out_7959357771420397480[3] = 0;
   out_7959357771420397480[4] = 0;
   out_7959357771420397480[5] = 0;
   out_7959357771420397480[6] = 0;
   out_7959357771420397480[7] = 0;
   out_7959357771420397480[8] = 0;
   out_7959357771420397480[9] = 0;
   out_7959357771420397480[10] = 1;
   out_7959357771420397480[11] = 0;
   out_7959357771420397480[12] = 0;
   out_7959357771420397480[13] = 0;
   out_7959357771420397480[14] = 0;
   out_7959357771420397480[15] = 0;
   out_7959357771420397480[16] = 0;
   out_7959357771420397480[17] = 0;
   out_7959357771420397480[18] = 0;
   out_7959357771420397480[19] = 0;
   out_7959357771420397480[20] = 1;
   out_7959357771420397480[21] = 0;
   out_7959357771420397480[22] = 0;
   out_7959357771420397480[23] = 0;
   out_7959357771420397480[24] = 0;
   out_7959357771420397480[25] = 0;
   out_7959357771420397480[26] = 0;
   out_7959357771420397480[27] = 0;
   out_7959357771420397480[28] = 0;
   out_7959357771420397480[29] = 0;
   out_7959357771420397480[30] = 1;
   out_7959357771420397480[31] = 0;
   out_7959357771420397480[32] = 0;
   out_7959357771420397480[33] = 0;
   out_7959357771420397480[34] = 0;
   out_7959357771420397480[35] = 0;
   out_7959357771420397480[36] = 0;
   out_7959357771420397480[37] = 0;
   out_7959357771420397480[38] = 0;
   out_7959357771420397480[39] = 0;
   out_7959357771420397480[40] = 1;
   out_7959357771420397480[41] = 0;
   out_7959357771420397480[42] = 0;
   out_7959357771420397480[43] = 0;
   out_7959357771420397480[44] = 0;
   out_7959357771420397480[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_7959357771420397480[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_7959357771420397480[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7959357771420397480[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7959357771420397480[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_7959357771420397480[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_7959357771420397480[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_7959357771420397480[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_7959357771420397480[53] = -9.8100000000000005*dt;
   out_7959357771420397480[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_7959357771420397480[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_7959357771420397480[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7959357771420397480[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7959357771420397480[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_7959357771420397480[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_7959357771420397480[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_7959357771420397480[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7959357771420397480[62] = 0;
   out_7959357771420397480[63] = 0;
   out_7959357771420397480[64] = 0;
   out_7959357771420397480[65] = 0;
   out_7959357771420397480[66] = 0;
   out_7959357771420397480[67] = 0;
   out_7959357771420397480[68] = 0;
   out_7959357771420397480[69] = 0;
   out_7959357771420397480[70] = 1;
   out_7959357771420397480[71] = 0;
   out_7959357771420397480[72] = 0;
   out_7959357771420397480[73] = 0;
   out_7959357771420397480[74] = 0;
   out_7959357771420397480[75] = 0;
   out_7959357771420397480[76] = 0;
   out_7959357771420397480[77] = 0;
   out_7959357771420397480[78] = 0;
   out_7959357771420397480[79] = 0;
   out_7959357771420397480[80] = 1;
}
void h_25(double *state, double *unused, double *out_3287495957292748730) {
   out_3287495957292748730[0] = state[6];
}
void H_25(double *state, double *unused, double *out_4816842997090982604) {
   out_4816842997090982604[0] = 0;
   out_4816842997090982604[1] = 0;
   out_4816842997090982604[2] = 0;
   out_4816842997090982604[3] = 0;
   out_4816842997090982604[4] = 0;
   out_4816842997090982604[5] = 0;
   out_4816842997090982604[6] = 1;
   out_4816842997090982604[7] = 0;
   out_4816842997090982604[8] = 0;
}
void h_24(double *state, double *unused, double *out_2410264965580228903) {
   out_2410264965580228903[0] = state[4];
   out_2410264965580228903[1] = state[5];
}
void H_24(double *state, double *unused, double *out_8374768595893305740) {
   out_8374768595893305740[0] = 0;
   out_8374768595893305740[1] = 0;
   out_8374768595893305740[2] = 0;
   out_8374768595893305740[3] = 0;
   out_8374768595893305740[4] = 1;
   out_8374768595893305740[5] = 0;
   out_8374768595893305740[6] = 0;
   out_8374768595893305740[7] = 0;
   out_8374768595893305740[8] = 0;
   out_8374768595893305740[9] = 0;
   out_8374768595893305740[10] = 0;
   out_8374768595893305740[11] = 0;
   out_8374768595893305740[12] = 0;
   out_8374768595893305740[13] = 0;
   out_8374768595893305740[14] = 1;
   out_8374768595893305740[15] = 0;
   out_8374768595893305740[16] = 0;
   out_8374768595893305740[17] = 0;
}
void h_30(double *state, double *unused, double *out_6519370639698261002) {
   out_6519370639698261002[0] = state[4];
}
void H_30(double *state, double *unused, double *out_289146666963374406) {
   out_289146666963374406[0] = 0;
   out_289146666963374406[1] = 0;
   out_289146666963374406[2] = 0;
   out_289146666963374406[3] = 0;
   out_289146666963374406[4] = 1;
   out_289146666963374406[5] = 0;
   out_289146666963374406[6] = 0;
   out_289146666963374406[7] = 0;
   out_289146666963374406[8] = 0;
}
void h_26(double *state, double *unused, double *out_5537286546362962890) {
   out_5537286546362962890[0] = state[7];
}
void H_26(double *state, double *unused, double *out_1075339678216926380) {
   out_1075339678216926380[0] = 0;
   out_1075339678216926380[1] = 0;
   out_1075339678216926380[2] = 0;
   out_1075339678216926380[3] = 0;
   out_1075339678216926380[4] = 0;
   out_1075339678216926380[5] = 0;
   out_1075339678216926380[6] = 0;
   out_1075339678216926380[7] = 1;
   out_1075339678216926380[8] = 0;
}
void h_27(double *state, double *unused, double *out_1714177310368096915) {
   out_1714177310368096915[0] = state[3];
}
void H_27(double *state, double *unused, double *out_1885616644837050505) {
   out_1885616644837050505[0] = 0;
   out_1885616644837050505[1] = 0;
   out_1885616644837050505[2] = 0;
   out_1885616644837050505[3] = 1;
   out_1885616644837050505[4] = 0;
   out_1885616644837050505[5] = 0;
   out_1885616644837050505[6] = 0;
   out_1885616644837050505[7] = 0;
   out_1885616644837050505[8] = 0;
}
void h_29(double *state, double *unused, double *out_6049087946452575526) {
   out_6049087946452575526[0] = state[1];
}
void H_29(double *state, double *unused, double *out_799378011277766590) {
   out_799378011277766590[0] = 0;
   out_799378011277766590[1] = 1;
   out_799378011277766590[2] = 0;
   out_799378011277766590[3] = 0;
   out_799378011277766590[4] = 0;
   out_799378011277766590[5] = 0;
   out_799378011277766590[6] = 0;
   out_799378011277766590[7] = 0;
   out_799378011277766590[8] = 0;
}
void h_28(double *state, double *unused, double *out_1322608741076235770) {
   out_1322608741076235770[0] = state[0];
}
void H_28(double *state, double *unused, double *out_2763008282843092841) {
   out_2763008282843092841[0] = 1;
   out_2763008282843092841[1] = 0;
   out_2763008282843092841[2] = 0;
   out_2763008282843092841[3] = 0;
   out_2763008282843092841[4] = 0;
   out_2763008282843092841[5] = 0;
   out_2763008282843092841[6] = 0;
   out_2763008282843092841[7] = 0;
   out_2763008282843092841[8] = 0;
}
void h_31(double *state, double *unused, double *out_3483339269057602206) {
   out_3483339269057602206[0] = state[8];
}
void H_31(double *state, double *unused, double *out_449131575983574904) {
   out_449131575983574904[0] = 0;
   out_449131575983574904[1] = 0;
   out_449131575983574904[2] = 0;
   out_449131575983574904[3] = 0;
   out_449131575983574904[4] = 0;
   out_449131575983574904[5] = 0;
   out_449131575983574904[6] = 0;
   out_449131575983574904[7] = 0;
   out_449131575983574904[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_6990374026437360235) {
  err_fun(nom_x, delta_x, out_6990374026437360235);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6840343405846249272) {
  inv_err_fun(nom_x, true_x, out_6840343405846249272);
}
void car_H_mod_fun(double *state, double *out_3104495035882877201) {
  H_mod_fun(state, out_3104495035882877201);
}
void car_f_fun(double *state, double dt, double *out_2810783572133327735) {
  f_fun(state,  dt, out_2810783572133327735);
}
void car_F_fun(double *state, double dt, double *out_7959357771420397480) {
  F_fun(state,  dt, out_7959357771420397480);
}
void car_h_25(double *state, double *unused, double *out_3287495957292748730) {
  h_25(state, unused, out_3287495957292748730);
}
void car_H_25(double *state, double *unused, double *out_4816842997090982604) {
  H_25(state, unused, out_4816842997090982604);
}
void car_h_24(double *state, double *unused, double *out_2410264965580228903) {
  h_24(state, unused, out_2410264965580228903);
}
void car_H_24(double *state, double *unused, double *out_8374768595893305740) {
  H_24(state, unused, out_8374768595893305740);
}
void car_h_30(double *state, double *unused, double *out_6519370639698261002) {
  h_30(state, unused, out_6519370639698261002);
}
void car_H_30(double *state, double *unused, double *out_289146666963374406) {
  H_30(state, unused, out_289146666963374406);
}
void car_h_26(double *state, double *unused, double *out_5537286546362962890) {
  h_26(state, unused, out_5537286546362962890);
}
void car_H_26(double *state, double *unused, double *out_1075339678216926380) {
  H_26(state, unused, out_1075339678216926380);
}
void car_h_27(double *state, double *unused, double *out_1714177310368096915) {
  h_27(state, unused, out_1714177310368096915);
}
void car_H_27(double *state, double *unused, double *out_1885616644837050505) {
  H_27(state, unused, out_1885616644837050505);
}
void car_h_29(double *state, double *unused, double *out_6049087946452575526) {
  h_29(state, unused, out_6049087946452575526);
}
void car_H_29(double *state, double *unused, double *out_799378011277766590) {
  H_29(state, unused, out_799378011277766590);
}
void car_h_28(double *state, double *unused, double *out_1322608741076235770) {
  h_28(state, unused, out_1322608741076235770);
}
void car_H_28(double *state, double *unused, double *out_2763008282843092841) {
  H_28(state, unused, out_2763008282843092841);
}
void car_h_31(double *state, double *unused, double *out_3483339269057602206) {
  h_31(state, unused, out_3483339269057602206);
}
void car_H_31(double *state, double *unused, double *out_449131575983574904) {
  H_31(state, unused, out_449131575983574904);
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
