#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_7932307875604201747);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5461367951673804832);
void car_H_mod_fun(double *state, double *out_3432542513437690508);
void car_f_fun(double *state, double dt, double *out_6891344942215127262);
void car_F_fun(double *state, double dt, double *out_7243645711963944442);
void car_h_25(double *state, double *unused, double *out_6326641548529593851);
void car_H_25(double *state, double *unused, double *out_6254750036277102905);
void car_h_24(double *state, double *unused, double *out_3960549167503314560);
void car_H_24(double *state, double *unused, double *out_1163641597399823988);
void car_h_30(double *state, double *unused, double *out_7777272941158477018);
void car_H_30(double *state, double *unused, double *out_661940305214513850);
void car_h_26(double *state, double *unused, double *out_7039715133565287789);
void car_H_26(double *state, double *unused, double *out_8450490718558392487);
void car_h_27(double *state, double *unused, double *out_5864277803220910511);
void car_H_27(double *state, double *unused, double *out_1512823006585911061);
void car_h_29(double *state, double *unused, double *out_2775599673497631373);
void car_H_29(double *state, double *unused, double *out_1172171649528906034);
void car_h_28(double *state, double *unused, double *out_3741449237849594517);
void car_H_28(double *state, double *unused, double *out_8308584750524992668);
void car_h_31(double *state, double *unused, double *out_6601835610814099740);
void car_H_31(double *state, double *unused, double *out_6224104074400142477);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}