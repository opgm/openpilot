#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_7412749682930917159);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_3324739374567006732);
void pose_H_mod_fun(double *state, double *out_6871722307859650590);
void pose_f_fun(double *state, double dt, double *out_8847044247784726907);
void pose_F_fun(double *state, double dt, double *out_6117558309002218944);
void pose_h_4(double *state, double *unused, double *out_2048679593753205460);
void pose_H_4(double *state, double *unused, double *out_3473479926877488476);
void pose_h_10(double *state, double *unused, double *out_6141237783103343630);
void pose_H_10(double *state, double *unused, double *out_8609451639616367789);
void pose_h_13(double *state, double *unused, double *out_2125878123702031972);
void pose_H_13(double *state, double *unused, double *out_6685753752209821277);
void pose_h_14(double *state, double *unused, double *out_1406625725743516395);
void pose_H_14(double *state, double *unused, double *out_7436720783216973005);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}