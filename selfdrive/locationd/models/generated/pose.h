#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_8874829572354212109);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_1180005862082641495);
void pose_H_mod_fun(double *state, double *out_8262365463737575842);
void pose_f_fun(double *state, double dt, double *out_5327889686526496546);
void pose_F_fun(double *state, double dt, double *out_508639460086939955);
void pose_h_4(double *state, double *unused, double *out_4278134127096839033);
void pose_H_4(double *state, double *unused, double *out_8196960472798219952);
void pose_h_10(double *state, double *unused, double *out_5750841777717275382);
void pose_H_10(double *state, double *unused, double *out_2204051736113766918);
void pose_h_13(double *state, double *unused, double *out_9079423146614688595);
void pose_H_13(double *state, double *unused, double *out_7037509775578998863);
void pose_h_14(double *state, double *unused, double *out_2413440844245158569);
void pose_H_14(double *state, double *unused, double *out_6286542744571847135);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}