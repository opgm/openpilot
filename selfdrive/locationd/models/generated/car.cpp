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
void err_fun(double *nom_x, double *delta_x, double *out_7932307875604201747) {
   out_7932307875604201747[0] = delta_x[0] + nom_x[0];
   out_7932307875604201747[1] = delta_x[1] + nom_x[1];
   out_7932307875604201747[2] = delta_x[2] + nom_x[2];
   out_7932307875604201747[3] = delta_x[3] + nom_x[3];
   out_7932307875604201747[4] = delta_x[4] + nom_x[4];
   out_7932307875604201747[5] = delta_x[5] + nom_x[5];
   out_7932307875604201747[6] = delta_x[6] + nom_x[6];
   out_7932307875604201747[7] = delta_x[7] + nom_x[7];
   out_7932307875604201747[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5461367951673804832) {
   out_5461367951673804832[0] = -nom_x[0] + true_x[0];
   out_5461367951673804832[1] = -nom_x[1] + true_x[1];
   out_5461367951673804832[2] = -nom_x[2] + true_x[2];
   out_5461367951673804832[3] = -nom_x[3] + true_x[3];
   out_5461367951673804832[4] = -nom_x[4] + true_x[4];
   out_5461367951673804832[5] = -nom_x[5] + true_x[5];
   out_5461367951673804832[6] = -nom_x[6] + true_x[6];
   out_5461367951673804832[7] = -nom_x[7] + true_x[7];
   out_5461367951673804832[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_3432542513437690508) {
   out_3432542513437690508[0] = 1.0;
   out_3432542513437690508[1] = 0;
   out_3432542513437690508[2] = 0;
   out_3432542513437690508[3] = 0;
   out_3432542513437690508[4] = 0;
   out_3432542513437690508[5] = 0;
   out_3432542513437690508[6] = 0;
   out_3432542513437690508[7] = 0;
   out_3432542513437690508[8] = 0;
   out_3432542513437690508[9] = 0;
   out_3432542513437690508[10] = 1.0;
   out_3432542513437690508[11] = 0;
   out_3432542513437690508[12] = 0;
   out_3432542513437690508[13] = 0;
   out_3432542513437690508[14] = 0;
   out_3432542513437690508[15] = 0;
   out_3432542513437690508[16] = 0;
   out_3432542513437690508[17] = 0;
   out_3432542513437690508[18] = 0;
   out_3432542513437690508[19] = 0;
   out_3432542513437690508[20] = 1.0;
   out_3432542513437690508[21] = 0;
   out_3432542513437690508[22] = 0;
   out_3432542513437690508[23] = 0;
   out_3432542513437690508[24] = 0;
   out_3432542513437690508[25] = 0;
   out_3432542513437690508[26] = 0;
   out_3432542513437690508[27] = 0;
   out_3432542513437690508[28] = 0;
   out_3432542513437690508[29] = 0;
   out_3432542513437690508[30] = 1.0;
   out_3432542513437690508[31] = 0;
   out_3432542513437690508[32] = 0;
   out_3432542513437690508[33] = 0;
   out_3432542513437690508[34] = 0;
   out_3432542513437690508[35] = 0;
   out_3432542513437690508[36] = 0;
   out_3432542513437690508[37] = 0;
   out_3432542513437690508[38] = 0;
   out_3432542513437690508[39] = 0;
   out_3432542513437690508[40] = 1.0;
   out_3432542513437690508[41] = 0;
   out_3432542513437690508[42] = 0;
   out_3432542513437690508[43] = 0;
   out_3432542513437690508[44] = 0;
   out_3432542513437690508[45] = 0;
   out_3432542513437690508[46] = 0;
   out_3432542513437690508[47] = 0;
   out_3432542513437690508[48] = 0;
   out_3432542513437690508[49] = 0;
   out_3432542513437690508[50] = 1.0;
   out_3432542513437690508[51] = 0;
   out_3432542513437690508[52] = 0;
   out_3432542513437690508[53] = 0;
   out_3432542513437690508[54] = 0;
   out_3432542513437690508[55] = 0;
   out_3432542513437690508[56] = 0;
   out_3432542513437690508[57] = 0;
   out_3432542513437690508[58] = 0;
   out_3432542513437690508[59] = 0;
   out_3432542513437690508[60] = 1.0;
   out_3432542513437690508[61] = 0;
   out_3432542513437690508[62] = 0;
   out_3432542513437690508[63] = 0;
   out_3432542513437690508[64] = 0;
   out_3432542513437690508[65] = 0;
   out_3432542513437690508[66] = 0;
   out_3432542513437690508[67] = 0;
   out_3432542513437690508[68] = 0;
   out_3432542513437690508[69] = 0;
   out_3432542513437690508[70] = 1.0;
   out_3432542513437690508[71] = 0;
   out_3432542513437690508[72] = 0;
   out_3432542513437690508[73] = 0;
   out_3432542513437690508[74] = 0;
   out_3432542513437690508[75] = 0;
   out_3432542513437690508[76] = 0;
   out_3432542513437690508[77] = 0;
   out_3432542513437690508[78] = 0;
   out_3432542513437690508[79] = 0;
   out_3432542513437690508[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_6891344942215127262) {
   out_6891344942215127262[0] = state[0];
   out_6891344942215127262[1] = state[1];
   out_6891344942215127262[2] = state[2];
   out_6891344942215127262[3] = state[3];
   out_6891344942215127262[4] = state[4];
   out_6891344942215127262[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_6891344942215127262[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_6891344942215127262[7] = state[7];
   out_6891344942215127262[8] = state[8];
}
void F_fun(double *state, double dt, double *out_7243645711963944442) {
   out_7243645711963944442[0] = 1;
   out_7243645711963944442[1] = 0;
   out_7243645711963944442[2] = 0;
   out_7243645711963944442[3] = 0;
   out_7243645711963944442[4] = 0;
   out_7243645711963944442[5] = 0;
   out_7243645711963944442[6] = 0;
   out_7243645711963944442[7] = 0;
   out_7243645711963944442[8] = 0;
   out_7243645711963944442[9] = 0;
   out_7243645711963944442[10] = 1;
   out_7243645711963944442[11] = 0;
   out_7243645711963944442[12] = 0;
   out_7243645711963944442[13] = 0;
   out_7243645711963944442[14] = 0;
   out_7243645711963944442[15] = 0;
   out_7243645711963944442[16] = 0;
   out_7243645711963944442[17] = 0;
   out_7243645711963944442[18] = 0;
   out_7243645711963944442[19] = 0;
   out_7243645711963944442[20] = 1;
   out_7243645711963944442[21] = 0;
   out_7243645711963944442[22] = 0;
   out_7243645711963944442[23] = 0;
   out_7243645711963944442[24] = 0;
   out_7243645711963944442[25] = 0;
   out_7243645711963944442[26] = 0;
   out_7243645711963944442[27] = 0;
   out_7243645711963944442[28] = 0;
   out_7243645711963944442[29] = 0;
   out_7243645711963944442[30] = 1;
   out_7243645711963944442[31] = 0;
   out_7243645711963944442[32] = 0;
   out_7243645711963944442[33] = 0;
   out_7243645711963944442[34] = 0;
   out_7243645711963944442[35] = 0;
   out_7243645711963944442[36] = 0;
   out_7243645711963944442[37] = 0;
   out_7243645711963944442[38] = 0;
   out_7243645711963944442[39] = 0;
   out_7243645711963944442[40] = 1;
   out_7243645711963944442[41] = 0;
   out_7243645711963944442[42] = 0;
   out_7243645711963944442[43] = 0;
   out_7243645711963944442[44] = 0;
   out_7243645711963944442[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_7243645711963944442[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_7243645711963944442[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7243645711963944442[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7243645711963944442[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_7243645711963944442[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_7243645711963944442[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_7243645711963944442[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_7243645711963944442[53] = -9.8000000000000007*dt;
   out_7243645711963944442[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_7243645711963944442[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_7243645711963944442[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7243645711963944442[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7243645711963944442[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_7243645711963944442[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_7243645711963944442[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_7243645711963944442[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7243645711963944442[62] = 0;
   out_7243645711963944442[63] = 0;
   out_7243645711963944442[64] = 0;
   out_7243645711963944442[65] = 0;
   out_7243645711963944442[66] = 0;
   out_7243645711963944442[67] = 0;
   out_7243645711963944442[68] = 0;
   out_7243645711963944442[69] = 0;
   out_7243645711963944442[70] = 1;
   out_7243645711963944442[71] = 0;
   out_7243645711963944442[72] = 0;
   out_7243645711963944442[73] = 0;
   out_7243645711963944442[74] = 0;
   out_7243645711963944442[75] = 0;
   out_7243645711963944442[76] = 0;
   out_7243645711963944442[77] = 0;
   out_7243645711963944442[78] = 0;
   out_7243645711963944442[79] = 0;
   out_7243645711963944442[80] = 1;
}
void h_25(double *state, double *unused, double *out_6326641548529593851) {
   out_6326641548529593851[0] = state[6];
}
void H_25(double *state, double *unused, double *out_6254750036277102905) {
   out_6254750036277102905[0] = 0;
   out_6254750036277102905[1] = 0;
   out_6254750036277102905[2] = 0;
   out_6254750036277102905[3] = 0;
   out_6254750036277102905[4] = 0;
   out_6254750036277102905[5] = 0;
   out_6254750036277102905[6] = 1;
   out_6254750036277102905[7] = 0;
   out_6254750036277102905[8] = 0;
}
void h_24(double *state, double *unused, double *out_3960549167503314560) {
   out_3960549167503314560[0] = state[4];
   out_3960549167503314560[1] = state[5];
}
void H_24(double *state, double *unused, double *out_1163641597399823988) {
   out_1163641597399823988[0] = 0;
   out_1163641597399823988[1] = 0;
   out_1163641597399823988[2] = 0;
   out_1163641597399823988[3] = 0;
   out_1163641597399823988[4] = 1;
   out_1163641597399823988[5] = 0;
   out_1163641597399823988[6] = 0;
   out_1163641597399823988[7] = 0;
   out_1163641597399823988[8] = 0;
   out_1163641597399823988[9] = 0;
   out_1163641597399823988[10] = 0;
   out_1163641597399823988[11] = 0;
   out_1163641597399823988[12] = 0;
   out_1163641597399823988[13] = 0;
   out_1163641597399823988[14] = 1;
   out_1163641597399823988[15] = 0;
   out_1163641597399823988[16] = 0;
   out_1163641597399823988[17] = 0;
}
void h_30(double *state, double *unused, double *out_7777272941158477018) {
   out_7777272941158477018[0] = state[4];
}
void H_30(double *state, double *unused, double *out_661940305214513850) {
   out_661940305214513850[0] = 0;
   out_661940305214513850[1] = 0;
   out_661940305214513850[2] = 0;
   out_661940305214513850[3] = 0;
   out_661940305214513850[4] = 1;
   out_661940305214513850[5] = 0;
   out_661940305214513850[6] = 0;
   out_661940305214513850[7] = 0;
   out_661940305214513850[8] = 0;
}
void h_26(double *state, double *unused, double *out_7039715133565287789) {
   out_7039715133565287789[0] = state[7];
}
void H_26(double *state, double *unused, double *out_8450490718558392487) {
   out_8450490718558392487[0] = 0;
   out_8450490718558392487[1] = 0;
   out_8450490718558392487[2] = 0;
   out_8450490718558392487[3] = 0;
   out_8450490718558392487[4] = 0;
   out_8450490718558392487[5] = 0;
   out_8450490718558392487[6] = 0;
   out_8450490718558392487[7] = 1;
   out_8450490718558392487[8] = 0;
}
void h_27(double *state, double *unused, double *out_5864277803220910511) {
   out_5864277803220910511[0] = state[3];
}
void H_27(double *state, double *unused, double *out_1512823006585911061) {
   out_1512823006585911061[0] = 0;
   out_1512823006585911061[1] = 0;
   out_1512823006585911061[2] = 0;
   out_1512823006585911061[3] = 1;
   out_1512823006585911061[4] = 0;
   out_1512823006585911061[5] = 0;
   out_1512823006585911061[6] = 0;
   out_1512823006585911061[7] = 0;
   out_1512823006585911061[8] = 0;
}
void h_29(double *state, double *unused, double *out_2775599673497631373) {
   out_2775599673497631373[0] = state[1];
}
void H_29(double *state, double *unused, double *out_1172171649528906034) {
   out_1172171649528906034[0] = 0;
   out_1172171649528906034[1] = 1;
   out_1172171649528906034[2] = 0;
   out_1172171649528906034[3] = 0;
   out_1172171649528906034[4] = 0;
   out_1172171649528906034[5] = 0;
   out_1172171649528906034[6] = 0;
   out_1172171649528906034[7] = 0;
   out_1172171649528906034[8] = 0;
}
void h_28(double *state, double *unused, double *out_3741449237849594517) {
   out_3741449237849594517[0] = state[0];
}
void H_28(double *state, double *unused, double *out_8308584750524992668) {
   out_8308584750524992668[0] = 1;
   out_8308584750524992668[1] = 0;
   out_8308584750524992668[2] = 0;
   out_8308584750524992668[3] = 0;
   out_8308584750524992668[4] = 0;
   out_8308584750524992668[5] = 0;
   out_8308584750524992668[6] = 0;
   out_8308584750524992668[7] = 0;
   out_8308584750524992668[8] = 0;
}
void h_31(double *state, double *unused, double *out_6601835610814099740) {
   out_6601835610814099740[0] = state[8];
}
void H_31(double *state, double *unused, double *out_6224104074400142477) {
   out_6224104074400142477[0] = 0;
   out_6224104074400142477[1] = 0;
   out_6224104074400142477[2] = 0;
   out_6224104074400142477[3] = 0;
   out_6224104074400142477[4] = 0;
   out_6224104074400142477[5] = 0;
   out_6224104074400142477[6] = 0;
   out_6224104074400142477[7] = 0;
   out_6224104074400142477[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_7932307875604201747) {
  err_fun(nom_x, delta_x, out_7932307875604201747);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5461367951673804832) {
  inv_err_fun(nom_x, true_x, out_5461367951673804832);
}
void car_H_mod_fun(double *state, double *out_3432542513437690508) {
  H_mod_fun(state, out_3432542513437690508);
}
void car_f_fun(double *state, double dt, double *out_6891344942215127262) {
  f_fun(state,  dt, out_6891344942215127262);
}
void car_F_fun(double *state, double dt, double *out_7243645711963944442) {
  F_fun(state,  dt, out_7243645711963944442);
}
void car_h_25(double *state, double *unused, double *out_6326641548529593851) {
  h_25(state, unused, out_6326641548529593851);
}
void car_H_25(double *state, double *unused, double *out_6254750036277102905) {
  H_25(state, unused, out_6254750036277102905);
}
void car_h_24(double *state, double *unused, double *out_3960549167503314560) {
  h_24(state, unused, out_3960549167503314560);
}
void car_H_24(double *state, double *unused, double *out_1163641597399823988) {
  H_24(state, unused, out_1163641597399823988);
}
void car_h_30(double *state, double *unused, double *out_7777272941158477018) {
  h_30(state, unused, out_7777272941158477018);
}
void car_H_30(double *state, double *unused, double *out_661940305214513850) {
  H_30(state, unused, out_661940305214513850);
}
void car_h_26(double *state, double *unused, double *out_7039715133565287789) {
  h_26(state, unused, out_7039715133565287789);
}
void car_H_26(double *state, double *unused, double *out_8450490718558392487) {
  H_26(state, unused, out_8450490718558392487);
}
void car_h_27(double *state, double *unused, double *out_5864277803220910511) {
  h_27(state, unused, out_5864277803220910511);
}
void car_H_27(double *state, double *unused, double *out_1512823006585911061) {
  H_27(state, unused, out_1512823006585911061);
}
void car_h_29(double *state, double *unused, double *out_2775599673497631373) {
  h_29(state, unused, out_2775599673497631373);
}
void car_H_29(double *state, double *unused, double *out_1172171649528906034) {
  H_29(state, unused, out_1172171649528906034);
}
void car_h_28(double *state, double *unused, double *out_3741449237849594517) {
  h_28(state, unused, out_3741449237849594517);
}
void car_H_28(double *state, double *unused, double *out_8308584750524992668) {
  H_28(state, unused, out_8308584750524992668);
}
void car_h_31(double *state, double *unused, double *out_6601835610814099740) {
  h_31(state, unused, out_6601835610814099740);
}
void car_H_31(double *state, double *unused, double *out_6224104074400142477) {
  H_31(state, unused, out_6224104074400142477);
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
