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
void err_fun(double *nom_x, double *delta_x, double *out_6967453643969959992) {
   out_6967453643969959992[0] = delta_x[0] + nom_x[0];
   out_6967453643969959992[1] = delta_x[1] + nom_x[1];
   out_6967453643969959992[2] = delta_x[2] + nom_x[2];
   out_6967453643969959992[3] = delta_x[3] + nom_x[3];
   out_6967453643969959992[4] = delta_x[4] + nom_x[4];
   out_6967453643969959992[5] = delta_x[5] + nom_x[5];
   out_6967453643969959992[6] = delta_x[6] + nom_x[6];
   out_6967453643969959992[7] = delta_x[7] + nom_x[7];
   out_6967453643969959992[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4192683843425632104) {
   out_4192683843425632104[0] = -nom_x[0] + true_x[0];
   out_4192683843425632104[1] = -nom_x[1] + true_x[1];
   out_4192683843425632104[2] = -nom_x[2] + true_x[2];
   out_4192683843425632104[3] = -nom_x[3] + true_x[3];
   out_4192683843425632104[4] = -nom_x[4] + true_x[4];
   out_4192683843425632104[5] = -nom_x[5] + true_x[5];
   out_4192683843425632104[6] = -nom_x[6] + true_x[6];
   out_4192683843425632104[7] = -nom_x[7] + true_x[7];
   out_4192683843425632104[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_8434874153757315530) {
   out_8434874153757315530[0] = 1.0;
   out_8434874153757315530[1] = 0.0;
   out_8434874153757315530[2] = 0.0;
   out_8434874153757315530[3] = 0.0;
   out_8434874153757315530[4] = 0.0;
   out_8434874153757315530[5] = 0.0;
   out_8434874153757315530[6] = 0.0;
   out_8434874153757315530[7] = 0.0;
   out_8434874153757315530[8] = 0.0;
   out_8434874153757315530[9] = 0.0;
   out_8434874153757315530[10] = 1.0;
   out_8434874153757315530[11] = 0.0;
   out_8434874153757315530[12] = 0.0;
   out_8434874153757315530[13] = 0.0;
   out_8434874153757315530[14] = 0.0;
   out_8434874153757315530[15] = 0.0;
   out_8434874153757315530[16] = 0.0;
   out_8434874153757315530[17] = 0.0;
   out_8434874153757315530[18] = 0.0;
   out_8434874153757315530[19] = 0.0;
   out_8434874153757315530[20] = 1.0;
   out_8434874153757315530[21] = 0.0;
   out_8434874153757315530[22] = 0.0;
   out_8434874153757315530[23] = 0.0;
   out_8434874153757315530[24] = 0.0;
   out_8434874153757315530[25] = 0.0;
   out_8434874153757315530[26] = 0.0;
   out_8434874153757315530[27] = 0.0;
   out_8434874153757315530[28] = 0.0;
   out_8434874153757315530[29] = 0.0;
   out_8434874153757315530[30] = 1.0;
   out_8434874153757315530[31] = 0.0;
   out_8434874153757315530[32] = 0.0;
   out_8434874153757315530[33] = 0.0;
   out_8434874153757315530[34] = 0.0;
   out_8434874153757315530[35] = 0.0;
   out_8434874153757315530[36] = 0.0;
   out_8434874153757315530[37] = 0.0;
   out_8434874153757315530[38] = 0.0;
   out_8434874153757315530[39] = 0.0;
   out_8434874153757315530[40] = 1.0;
   out_8434874153757315530[41] = 0.0;
   out_8434874153757315530[42] = 0.0;
   out_8434874153757315530[43] = 0.0;
   out_8434874153757315530[44] = 0.0;
   out_8434874153757315530[45] = 0.0;
   out_8434874153757315530[46] = 0.0;
   out_8434874153757315530[47] = 0.0;
   out_8434874153757315530[48] = 0.0;
   out_8434874153757315530[49] = 0.0;
   out_8434874153757315530[50] = 1.0;
   out_8434874153757315530[51] = 0.0;
   out_8434874153757315530[52] = 0.0;
   out_8434874153757315530[53] = 0.0;
   out_8434874153757315530[54] = 0.0;
   out_8434874153757315530[55] = 0.0;
   out_8434874153757315530[56] = 0.0;
   out_8434874153757315530[57] = 0.0;
   out_8434874153757315530[58] = 0.0;
   out_8434874153757315530[59] = 0.0;
   out_8434874153757315530[60] = 1.0;
   out_8434874153757315530[61] = 0.0;
   out_8434874153757315530[62] = 0.0;
   out_8434874153757315530[63] = 0.0;
   out_8434874153757315530[64] = 0.0;
   out_8434874153757315530[65] = 0.0;
   out_8434874153757315530[66] = 0.0;
   out_8434874153757315530[67] = 0.0;
   out_8434874153757315530[68] = 0.0;
   out_8434874153757315530[69] = 0.0;
   out_8434874153757315530[70] = 1.0;
   out_8434874153757315530[71] = 0.0;
   out_8434874153757315530[72] = 0.0;
   out_8434874153757315530[73] = 0.0;
   out_8434874153757315530[74] = 0.0;
   out_8434874153757315530[75] = 0.0;
   out_8434874153757315530[76] = 0.0;
   out_8434874153757315530[77] = 0.0;
   out_8434874153757315530[78] = 0.0;
   out_8434874153757315530[79] = 0.0;
   out_8434874153757315530[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_5018562704091678262) {
   out_5018562704091678262[0] = state[0];
   out_5018562704091678262[1] = state[1];
   out_5018562704091678262[2] = state[2];
   out_5018562704091678262[3] = state[3];
   out_5018562704091678262[4] = state[4];
   out_5018562704091678262[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_5018562704091678262[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_5018562704091678262[7] = state[7];
   out_5018562704091678262[8] = state[8];
}
void F_fun(double *state, double dt, double *out_3150118549457082222) {
   out_3150118549457082222[0] = 1;
   out_3150118549457082222[1] = 0;
   out_3150118549457082222[2] = 0;
   out_3150118549457082222[3] = 0;
   out_3150118549457082222[4] = 0;
   out_3150118549457082222[5] = 0;
   out_3150118549457082222[6] = 0;
   out_3150118549457082222[7] = 0;
   out_3150118549457082222[8] = 0;
   out_3150118549457082222[9] = 0;
   out_3150118549457082222[10] = 1;
   out_3150118549457082222[11] = 0;
   out_3150118549457082222[12] = 0;
   out_3150118549457082222[13] = 0;
   out_3150118549457082222[14] = 0;
   out_3150118549457082222[15] = 0;
   out_3150118549457082222[16] = 0;
   out_3150118549457082222[17] = 0;
   out_3150118549457082222[18] = 0;
   out_3150118549457082222[19] = 0;
   out_3150118549457082222[20] = 1;
   out_3150118549457082222[21] = 0;
   out_3150118549457082222[22] = 0;
   out_3150118549457082222[23] = 0;
   out_3150118549457082222[24] = 0;
   out_3150118549457082222[25] = 0;
   out_3150118549457082222[26] = 0;
   out_3150118549457082222[27] = 0;
   out_3150118549457082222[28] = 0;
   out_3150118549457082222[29] = 0;
   out_3150118549457082222[30] = 1;
   out_3150118549457082222[31] = 0;
   out_3150118549457082222[32] = 0;
   out_3150118549457082222[33] = 0;
   out_3150118549457082222[34] = 0;
   out_3150118549457082222[35] = 0;
   out_3150118549457082222[36] = 0;
   out_3150118549457082222[37] = 0;
   out_3150118549457082222[38] = 0;
   out_3150118549457082222[39] = 0;
   out_3150118549457082222[40] = 1;
   out_3150118549457082222[41] = 0;
   out_3150118549457082222[42] = 0;
   out_3150118549457082222[43] = 0;
   out_3150118549457082222[44] = 0;
   out_3150118549457082222[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_3150118549457082222[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_3150118549457082222[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3150118549457082222[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3150118549457082222[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_3150118549457082222[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_3150118549457082222[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_3150118549457082222[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_3150118549457082222[53] = -9.8000000000000007*dt;
   out_3150118549457082222[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_3150118549457082222[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_3150118549457082222[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3150118549457082222[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3150118549457082222[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_3150118549457082222[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_3150118549457082222[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_3150118549457082222[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3150118549457082222[62] = 0;
   out_3150118549457082222[63] = 0;
   out_3150118549457082222[64] = 0;
   out_3150118549457082222[65] = 0;
   out_3150118549457082222[66] = 0;
   out_3150118549457082222[67] = 0;
   out_3150118549457082222[68] = 0;
   out_3150118549457082222[69] = 0;
   out_3150118549457082222[70] = 1;
   out_3150118549457082222[71] = 0;
   out_3150118549457082222[72] = 0;
   out_3150118549457082222[73] = 0;
   out_3150118549457082222[74] = 0;
   out_3150118549457082222[75] = 0;
   out_3150118549457082222[76] = 0;
   out_3150118549457082222[77] = 0;
   out_3150118549457082222[78] = 0;
   out_3150118549457082222[79] = 0;
   out_3150118549457082222[80] = 1;
}
void h_25(double *state, double *unused, double *out_8627201318607884604) {
   out_8627201318607884604[0] = state[6];
}
void H_25(double *state, double *unused, double *out_7375312994936809892) {
   out_7375312994936809892[0] = 0;
   out_7375312994936809892[1] = 0;
   out_7375312994936809892[2] = 0;
   out_7375312994936809892[3] = 0;
   out_7375312994936809892[4] = 0;
   out_7375312994936809892[5] = 0;
   out_7375312994936809892[6] = 1;
   out_7375312994936809892[7] = 0;
   out_7375312994936809892[8] = 0;
}
void h_24(double *state, double *unused, double *out_6046618710890368303) {
   out_6046618710890368303[0] = state[4];
   out_6046618710890368303[1] = state[5];
}
void H_24(double *state, double *unused, double *out_6534881210754764900) {
   out_6534881210754764900[0] = 0;
   out_6534881210754764900[1] = 0;
   out_6534881210754764900[2] = 0;
   out_6534881210754764900[3] = 0;
   out_6534881210754764900[4] = 1;
   out_6534881210754764900[5] = 0;
   out_6534881210754764900[6] = 0;
   out_6534881210754764900[7] = 0;
   out_6534881210754764900[8] = 0;
   out_6534881210754764900[9] = 0;
   out_6534881210754764900[10] = 0;
   out_6534881210754764900[11] = 0;
   out_6534881210754764900[12] = 0;
   out_6534881210754764900[13] = 0;
   out_6534881210754764900[14] = 1;
   out_6534881210754764900[15] = 0;
   out_6534881210754764900[16] = 0;
   out_6534881210754764900[17] = 0;
}
void h_30(double *state, double *unused, double *out_8368911362472783845) {
   out_8368911362472783845[0] = state[4];
}
void H_30(double *state, double *unused, double *out_2847616664809201694) {
   out_2847616664809201694[0] = 0;
   out_2847616664809201694[1] = 0;
   out_2847616664809201694[2] = 0;
   out_2847616664809201694[3] = 0;
   out_2847616664809201694[4] = 1;
   out_2847616664809201694[5] = 0;
   out_2847616664809201694[6] = 0;
   out_2847616664809201694[7] = 0;
   out_2847616664809201694[8] = 0;
}
void h_26(double *state, double *unused, double *out_7041046659792435086) {
   out_7041046659792435086[0] = state[7];
}
void H_26(double *state, double *unused, double *out_3633809676062753668) {
   out_3633809676062753668[0] = 0;
   out_3633809676062753668[1] = 0;
   out_3633809676062753668[2] = 0;
   out_3633809676062753668[3] = 0;
   out_3633809676062753668[4] = 0;
   out_3633809676062753668[5] = 0;
   out_3633809676062753668[6] = 0;
   out_3633809676062753668[7] = 1;
   out_3633809676062753668[8] = 0;
}
void h_27(double *state, double *unused, double *out_4005015289151776290) {
   out_4005015289151776290[0] = state[3];
}
void H_27(double *state, double *unused, double *out_672853353008776783) {
   out_672853353008776783[0] = 0;
   out_672853353008776783[1] = 0;
   out_672853353008776783[2] = 0;
   out_672853353008776783[3] = 1;
   out_672853353008776783[4] = 0;
   out_672853353008776783[5] = 0;
   out_672853353008776783[6] = 0;
   out_672853353008776783[7] = 0;
   out_672853353008776783[8] = 0;
}
void h_29(double *state, double *unused, double *out_236155425481462693) {
   out_236155425481462693[0] = state[1];
}
void H_29(double *state, double *unused, double *out_3357848009123593878) {
   out_3357848009123593878[0] = 0;
   out_3357848009123593878[1] = 1;
   out_3357848009123593878[2] = 0;
   out_3357848009123593878[3] = 0;
   out_3357848009123593878[4] = 0;
   out_3357848009123593878[5] = 0;
   out_3357848009123593878[6] = 0;
   out_3357848009123593878[7] = 0;
   out_3357848009123593878[8] = 0;
}
void h_28(double *state, double *unused, double *out_1440889467771303764) {
   out_1440889467771303764[0] = state[0];
}
void H_28(double *state, double *unused, double *out_1724551007945936696) {
   out_1724551007945936696[0] = 1;
   out_1724551007945936696[1] = 0;
   out_1724551007945936696[2] = 0;
   out_1724551007945936696[3] = 0;
   out_1724551007945936696[4] = 0;
   out_1724551007945936696[5] = 0;
   out_1724551007945936696[6] = 0;
   out_1724551007945936696[7] = 0;
   out_1724551007945936696[8] = 0;
}
void h_31(double *state, double *unused, double *out_6182093226847644662) {
   out_6182093226847644662[0] = state[8];
}
void H_31(double *state, double *unused, double *out_3007601573829402192) {
   out_3007601573829402192[0] = 0;
   out_3007601573829402192[1] = 0;
   out_3007601573829402192[2] = 0;
   out_3007601573829402192[3] = 0;
   out_3007601573829402192[4] = 0;
   out_3007601573829402192[5] = 0;
   out_3007601573829402192[6] = 0;
   out_3007601573829402192[7] = 0;
   out_3007601573829402192[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_6967453643969959992) {
  err_fun(nom_x, delta_x, out_6967453643969959992);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4192683843425632104) {
  inv_err_fun(nom_x, true_x, out_4192683843425632104);
}
void car_H_mod_fun(double *state, double *out_8434874153757315530) {
  H_mod_fun(state, out_8434874153757315530);
}
void car_f_fun(double *state, double dt, double *out_5018562704091678262) {
  f_fun(state,  dt, out_5018562704091678262);
}
void car_F_fun(double *state, double dt, double *out_3150118549457082222) {
  F_fun(state,  dt, out_3150118549457082222);
}
void car_h_25(double *state, double *unused, double *out_8627201318607884604) {
  h_25(state, unused, out_8627201318607884604);
}
void car_H_25(double *state, double *unused, double *out_7375312994936809892) {
  H_25(state, unused, out_7375312994936809892);
}
void car_h_24(double *state, double *unused, double *out_6046618710890368303) {
  h_24(state, unused, out_6046618710890368303);
}
void car_H_24(double *state, double *unused, double *out_6534881210754764900) {
  H_24(state, unused, out_6534881210754764900);
}
void car_h_30(double *state, double *unused, double *out_8368911362472783845) {
  h_30(state, unused, out_8368911362472783845);
}
void car_H_30(double *state, double *unused, double *out_2847616664809201694) {
  H_30(state, unused, out_2847616664809201694);
}
void car_h_26(double *state, double *unused, double *out_7041046659792435086) {
  h_26(state, unused, out_7041046659792435086);
}
void car_H_26(double *state, double *unused, double *out_3633809676062753668) {
  H_26(state, unused, out_3633809676062753668);
}
void car_h_27(double *state, double *unused, double *out_4005015289151776290) {
  h_27(state, unused, out_4005015289151776290);
}
void car_H_27(double *state, double *unused, double *out_672853353008776783) {
  H_27(state, unused, out_672853353008776783);
}
void car_h_29(double *state, double *unused, double *out_236155425481462693) {
  h_29(state, unused, out_236155425481462693);
}
void car_H_29(double *state, double *unused, double *out_3357848009123593878) {
  H_29(state, unused, out_3357848009123593878);
}
void car_h_28(double *state, double *unused, double *out_1440889467771303764) {
  h_28(state, unused, out_1440889467771303764);
}
void car_H_28(double *state, double *unused, double *out_1724551007945936696) {
  H_28(state, unused, out_1724551007945936696);
}
void car_h_31(double *state, double *unused, double *out_6182093226847644662) {
  h_31(state, unused, out_6182093226847644662);
}
void car_H_31(double *state, double *unused, double *out_3007601573829402192) {
  H_31(state, unused, out_3007601573829402192);
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
