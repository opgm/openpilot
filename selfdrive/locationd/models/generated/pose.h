#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_7993045186406776049);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_6214826562014697166);
void pose_H_mod_fun(double *state, double *out_2036493750579969796);
void pose_f_fun(double *state, double dt, double *out_5815055826729932803);
void pose_F_fun(double *state, double dt, double *out_8088927993986614311);
void pose_h_4(double *state, double *unused, double *out_4596209739200211234);
void pose_H_4(double *state, double *unused, double *out_6907867915477851546);
void pose_h_10(double *state, double *unused, double *out_7511182918238442662);
void pose_H_10(double *state, double *unused, double *out_5218983418130581187);
void pose_h_13(double *state, double *unused, double *out_3565323313024088772);
void pose_H_13(double *state, double *unused, double *out_3695594090145518745);
void pose_h_14(double *state, double *unused, double *out_6195396300878839194);
void pose_H_14(double *state, double *unused, double *out_2944627059138367017);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}