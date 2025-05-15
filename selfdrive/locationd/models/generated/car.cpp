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
void err_fun(double *nom_x, double *delta_x, double *out_2575150693000547317) {
   out_2575150693000547317[0] = delta_x[0] + nom_x[0];
   out_2575150693000547317[1] = delta_x[1] + nom_x[1];
   out_2575150693000547317[2] = delta_x[2] + nom_x[2];
   out_2575150693000547317[3] = delta_x[3] + nom_x[3];
   out_2575150693000547317[4] = delta_x[4] + nom_x[4];
   out_2575150693000547317[5] = delta_x[5] + nom_x[5];
   out_2575150693000547317[6] = delta_x[6] + nom_x[6];
   out_2575150693000547317[7] = delta_x[7] + nom_x[7];
   out_2575150693000547317[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6433197917275169840) {
   out_6433197917275169840[0] = -nom_x[0] + true_x[0];
   out_6433197917275169840[1] = -nom_x[1] + true_x[1];
   out_6433197917275169840[2] = -nom_x[2] + true_x[2];
   out_6433197917275169840[3] = -nom_x[3] + true_x[3];
   out_6433197917275169840[4] = -nom_x[4] + true_x[4];
   out_6433197917275169840[5] = -nom_x[5] + true_x[5];
   out_6433197917275169840[6] = -nom_x[6] + true_x[6];
   out_6433197917275169840[7] = -nom_x[7] + true_x[7];
   out_6433197917275169840[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_6597027900384019102) {
   out_6597027900384019102[0] = 1.0;
   out_6597027900384019102[1] = 0.0;
   out_6597027900384019102[2] = 0.0;
   out_6597027900384019102[3] = 0.0;
   out_6597027900384019102[4] = 0.0;
   out_6597027900384019102[5] = 0.0;
   out_6597027900384019102[6] = 0.0;
   out_6597027900384019102[7] = 0.0;
   out_6597027900384019102[8] = 0.0;
   out_6597027900384019102[9] = 0.0;
   out_6597027900384019102[10] = 1.0;
   out_6597027900384019102[11] = 0.0;
   out_6597027900384019102[12] = 0.0;
   out_6597027900384019102[13] = 0.0;
   out_6597027900384019102[14] = 0.0;
   out_6597027900384019102[15] = 0.0;
   out_6597027900384019102[16] = 0.0;
   out_6597027900384019102[17] = 0.0;
   out_6597027900384019102[18] = 0.0;
   out_6597027900384019102[19] = 0.0;
   out_6597027900384019102[20] = 1.0;
   out_6597027900384019102[21] = 0.0;
   out_6597027900384019102[22] = 0.0;
   out_6597027900384019102[23] = 0.0;
   out_6597027900384019102[24] = 0.0;
   out_6597027900384019102[25] = 0.0;
   out_6597027900384019102[26] = 0.0;
   out_6597027900384019102[27] = 0.0;
   out_6597027900384019102[28] = 0.0;
   out_6597027900384019102[29] = 0.0;
   out_6597027900384019102[30] = 1.0;
   out_6597027900384019102[31] = 0.0;
   out_6597027900384019102[32] = 0.0;
   out_6597027900384019102[33] = 0.0;
   out_6597027900384019102[34] = 0.0;
   out_6597027900384019102[35] = 0.0;
   out_6597027900384019102[36] = 0.0;
   out_6597027900384019102[37] = 0.0;
   out_6597027900384019102[38] = 0.0;
   out_6597027900384019102[39] = 0.0;
   out_6597027900384019102[40] = 1.0;
   out_6597027900384019102[41] = 0.0;
   out_6597027900384019102[42] = 0.0;
   out_6597027900384019102[43] = 0.0;
   out_6597027900384019102[44] = 0.0;
   out_6597027900384019102[45] = 0.0;
   out_6597027900384019102[46] = 0.0;
   out_6597027900384019102[47] = 0.0;
   out_6597027900384019102[48] = 0.0;
   out_6597027900384019102[49] = 0.0;
   out_6597027900384019102[50] = 1.0;
   out_6597027900384019102[51] = 0.0;
   out_6597027900384019102[52] = 0.0;
   out_6597027900384019102[53] = 0.0;
   out_6597027900384019102[54] = 0.0;
   out_6597027900384019102[55] = 0.0;
   out_6597027900384019102[56] = 0.0;
   out_6597027900384019102[57] = 0.0;
   out_6597027900384019102[58] = 0.0;
   out_6597027900384019102[59] = 0.0;
   out_6597027900384019102[60] = 1.0;
   out_6597027900384019102[61] = 0.0;
   out_6597027900384019102[62] = 0.0;
   out_6597027900384019102[63] = 0.0;
   out_6597027900384019102[64] = 0.0;
   out_6597027900384019102[65] = 0.0;
   out_6597027900384019102[66] = 0.0;
   out_6597027900384019102[67] = 0.0;
   out_6597027900384019102[68] = 0.0;
   out_6597027900384019102[69] = 0.0;
   out_6597027900384019102[70] = 1.0;
   out_6597027900384019102[71] = 0.0;
   out_6597027900384019102[72] = 0.0;
   out_6597027900384019102[73] = 0.0;
   out_6597027900384019102[74] = 0.0;
   out_6597027900384019102[75] = 0.0;
   out_6597027900384019102[76] = 0.0;
   out_6597027900384019102[77] = 0.0;
   out_6597027900384019102[78] = 0.0;
   out_6597027900384019102[79] = 0.0;
   out_6597027900384019102[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_8598323179917779170) {
   out_8598323179917779170[0] = state[0];
   out_8598323179917779170[1] = state[1];
   out_8598323179917779170[2] = state[2];
   out_8598323179917779170[3] = state[3];
   out_8598323179917779170[4] = state[4];
   out_8598323179917779170[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_8598323179917779170[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_8598323179917779170[7] = state[7];
   out_8598323179917779170[8] = state[8];
}
void F_fun(double *state, double dt, double *out_3687363393736083317) {
   out_3687363393736083317[0] = 1;
   out_3687363393736083317[1] = 0;
   out_3687363393736083317[2] = 0;
   out_3687363393736083317[3] = 0;
   out_3687363393736083317[4] = 0;
   out_3687363393736083317[5] = 0;
   out_3687363393736083317[6] = 0;
   out_3687363393736083317[7] = 0;
   out_3687363393736083317[8] = 0;
   out_3687363393736083317[9] = 0;
   out_3687363393736083317[10] = 1;
   out_3687363393736083317[11] = 0;
   out_3687363393736083317[12] = 0;
   out_3687363393736083317[13] = 0;
   out_3687363393736083317[14] = 0;
   out_3687363393736083317[15] = 0;
   out_3687363393736083317[16] = 0;
   out_3687363393736083317[17] = 0;
   out_3687363393736083317[18] = 0;
   out_3687363393736083317[19] = 0;
   out_3687363393736083317[20] = 1;
   out_3687363393736083317[21] = 0;
   out_3687363393736083317[22] = 0;
   out_3687363393736083317[23] = 0;
   out_3687363393736083317[24] = 0;
   out_3687363393736083317[25] = 0;
   out_3687363393736083317[26] = 0;
   out_3687363393736083317[27] = 0;
   out_3687363393736083317[28] = 0;
   out_3687363393736083317[29] = 0;
   out_3687363393736083317[30] = 1;
   out_3687363393736083317[31] = 0;
   out_3687363393736083317[32] = 0;
   out_3687363393736083317[33] = 0;
   out_3687363393736083317[34] = 0;
   out_3687363393736083317[35] = 0;
   out_3687363393736083317[36] = 0;
   out_3687363393736083317[37] = 0;
   out_3687363393736083317[38] = 0;
   out_3687363393736083317[39] = 0;
   out_3687363393736083317[40] = 1;
   out_3687363393736083317[41] = 0;
   out_3687363393736083317[42] = 0;
   out_3687363393736083317[43] = 0;
   out_3687363393736083317[44] = 0;
   out_3687363393736083317[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_3687363393736083317[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_3687363393736083317[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3687363393736083317[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3687363393736083317[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_3687363393736083317[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_3687363393736083317[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_3687363393736083317[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_3687363393736083317[53] = -9.8000000000000007*dt;
   out_3687363393736083317[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_3687363393736083317[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_3687363393736083317[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3687363393736083317[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3687363393736083317[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_3687363393736083317[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_3687363393736083317[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_3687363393736083317[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3687363393736083317[62] = 0;
   out_3687363393736083317[63] = 0;
   out_3687363393736083317[64] = 0;
   out_3687363393736083317[65] = 0;
   out_3687363393736083317[66] = 0;
   out_3687363393736083317[67] = 0;
   out_3687363393736083317[68] = 0;
   out_3687363393736083317[69] = 0;
   out_3687363393736083317[70] = 1;
   out_3687363393736083317[71] = 0;
   out_3687363393736083317[72] = 0;
   out_3687363393736083317[73] = 0;
   out_3687363393736083317[74] = 0;
   out_3687363393736083317[75] = 0;
   out_3687363393736083317[76] = 0;
   out_3687363393736083317[77] = 0;
   out_3687363393736083317[78] = 0;
   out_3687363393736083317[79] = 0;
   out_3687363393736083317[80] = 1;
}
void h_25(double *state, double *unused, double *out_6019453859290479602) {
   out_6019453859290479602[0] = state[6];
}
void H_25(double *state, double *unused, double *out_437886407615775220) {
   out_437886407615775220[0] = 0;
   out_437886407615775220[1] = 0;
   out_437886407615775220[2] = 0;
   out_437886407615775220[3] = 0;
   out_437886407615775220[4] = 0;
   out_437886407615775220[5] = 0;
   out_437886407615775220[6] = 1;
   out_437886407615775220[7] = 0;
   out_437886407615775220[8] = 0;
}
void h_24(double *state, double *unused, double *out_2865544647271695197) {
   out_2865544647271695197[0] = state[4];
   out_2865544647271695197[1] = state[5];
}
void H_24(double *state, double *unused, double *out_4768809924533717701) {
   out_4768809924533717701[0] = 0;
   out_4768809924533717701[1] = 0;
   out_4768809924533717701[2] = 0;
   out_4768809924533717701[3] = 0;
   out_4768809924533717701[4] = 1;
   out_4768809924533717701[5] = 0;
   out_4768809924533717701[6] = 0;
   out_4768809924533717701[7] = 0;
   out_4768809924533717701[8] = 0;
   out_4768809924533717701[9] = 0;
   out_4768809924533717701[10] = 0;
   out_4768809924533717701[11] = 0;
   out_4768809924533717701[12] = 0;
   out_4768809924533717701[13] = 0;
   out_4768809924533717701[14] = 1;
   out_4768809924533717701[15] = 0;
   out_4768809924533717701[16] = 0;
   out_4768809924533717701[17] = 0;
}
void h_30(double *state, double *unused, double *out_751381367059871334) {
   out_751381367059871334[0] = state[4];
}
void H_30(double *state, double *unused, double *out_567225354759015290) {
   out_567225354759015290[0] = 0;
   out_567225354759015290[1] = 0;
   out_567225354759015290[2] = 0;
   out_567225354759015290[3] = 0;
   out_567225354759015290[4] = 1;
   out_567225354759015290[5] = 0;
   out_567225354759015290[6] = 0;
   out_567225354759015290[7] = 0;
   out_567225354759015290[8] = 0;
}
void h_26(double *state, double *unused, double *out_1354697251736348851) {
   out_1354697251736348851[0] = state[7];
}
void H_26(double *state, double *unused, double *out_4179389726489831444) {
   out_4179389726489831444[0] = 0;
   out_4179389726489831444[1] = 0;
   out_4179389726489831444[2] = 0;
   out_4179389726489831444[3] = 0;
   out_4179389726489831444[4] = 0;
   out_4179389726489831444[5] = 0;
   out_4179389726489831444[6] = 0;
   out_4179389726489831444[7] = 1;
   out_4179389726489831444[8] = 0;
}
void h_27(double *state, double *unused, double *out_3630980750153074874) {
   out_3630980750153074874[0] = state[3];
}
void H_27(double *state, double *unused, double *out_2741988666559440201) {
   out_2741988666559440201[0] = 0;
   out_2741988666559440201[1] = 0;
   out_2741988666559440201[2] = 0;
   out_2741988666559440201[3] = 1;
   out_2741988666559440201[4] = 0;
   out_2741988666559440201[5] = 0;
   out_2741988666559440201[6] = 0;
   out_2741988666559440201[7] = 0;
   out_2741988666559440201[8] = 0;
}
void h_29(double *state, double *unused, double *out_230702726275426778) {
   out_230702726275426778[0] = state[1];
}
void H_29(double *state, double *unused, double *out_56994010444623106) {
   out_56994010444623106[0] = 0;
   out_56994010444623106[1] = 1;
   out_56994010444623106[2] = 0;
   out_56994010444623106[3] = 0;
   out_56994010444623106[4] = 0;
   out_56994010444623106[5] = 0;
   out_56994010444623106[6] = 0;
   out_56994010444623106[7] = 0;
   out_56994010444623106[8] = 0;
}
void h_28(double *state, double *unused, double *out_4574490603054737309) {
   out_4574490603054737309[0] = state[0];
}
void H_28(double *state, double *unused, double *out_1906636261120703145) {
   out_1906636261120703145[0] = 1;
   out_1906636261120703145[1] = 0;
   out_1906636261120703145[2] = 0;
   out_1906636261120703145[3] = 0;
   out_1906636261120703145[4] = 0;
   out_1906636261120703145[5] = 0;
   out_1906636261120703145[6] = 0;
   out_1906636261120703145[7] = 0;
   out_1906636261120703145[8] = 0;
}
void h_31(double *state, double *unused, double *out_6176640527641608747) {
   out_6176640527641608747[0] = state[8];
}
void H_31(double *state, double *unused, double *out_407240445738814792) {
   out_407240445738814792[0] = 0;
   out_407240445738814792[1] = 0;
   out_407240445738814792[2] = 0;
   out_407240445738814792[3] = 0;
   out_407240445738814792[4] = 0;
   out_407240445738814792[5] = 0;
   out_407240445738814792[6] = 0;
   out_407240445738814792[7] = 0;
   out_407240445738814792[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_2575150693000547317) {
  err_fun(nom_x, delta_x, out_2575150693000547317);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6433197917275169840) {
  inv_err_fun(nom_x, true_x, out_6433197917275169840);
}
void car_H_mod_fun(double *state, double *out_6597027900384019102) {
  H_mod_fun(state, out_6597027900384019102);
}
void car_f_fun(double *state, double dt, double *out_8598323179917779170) {
  f_fun(state,  dt, out_8598323179917779170);
}
void car_F_fun(double *state, double dt, double *out_3687363393736083317) {
  F_fun(state,  dt, out_3687363393736083317);
}
void car_h_25(double *state, double *unused, double *out_6019453859290479602) {
  h_25(state, unused, out_6019453859290479602);
}
void car_H_25(double *state, double *unused, double *out_437886407615775220) {
  H_25(state, unused, out_437886407615775220);
}
void car_h_24(double *state, double *unused, double *out_2865544647271695197) {
  h_24(state, unused, out_2865544647271695197);
}
void car_H_24(double *state, double *unused, double *out_4768809924533717701) {
  H_24(state, unused, out_4768809924533717701);
}
void car_h_30(double *state, double *unused, double *out_751381367059871334) {
  h_30(state, unused, out_751381367059871334);
}
void car_H_30(double *state, double *unused, double *out_567225354759015290) {
  H_30(state, unused, out_567225354759015290);
}
void car_h_26(double *state, double *unused, double *out_1354697251736348851) {
  h_26(state, unused, out_1354697251736348851);
}
void car_H_26(double *state, double *unused, double *out_4179389726489831444) {
  H_26(state, unused, out_4179389726489831444);
}
void car_h_27(double *state, double *unused, double *out_3630980750153074874) {
  h_27(state, unused, out_3630980750153074874);
}
void car_H_27(double *state, double *unused, double *out_2741988666559440201) {
  H_27(state, unused, out_2741988666559440201);
}
void car_h_29(double *state, double *unused, double *out_230702726275426778) {
  h_29(state, unused, out_230702726275426778);
}
void car_H_29(double *state, double *unused, double *out_56994010444623106) {
  H_29(state, unused, out_56994010444623106);
}
void car_h_28(double *state, double *unused, double *out_4574490603054737309) {
  h_28(state, unused, out_4574490603054737309);
}
void car_H_28(double *state, double *unused, double *out_1906636261120703145) {
  H_28(state, unused, out_1906636261120703145);
}
void car_h_31(double *state, double *unused, double *out_6176640527641608747) {
  h_31(state, unused, out_6176640527641608747);
}
void car_H_31(double *state, double *unused, double *out_407240445738814792) {
  H_31(state, unused, out_407240445738814792);
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
