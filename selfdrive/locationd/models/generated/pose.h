#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_3880423688108830906);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_530507270830550051);
void pose_H_mod_fun(double *state, double *out_8675870831607907019);
void pose_f_fun(double *state, double dt, double *out_5800429947315892303);
void pose_F_fun(double *state, double dt, double *out_4994234759718378919);
void pose_h_4(double *state, double *unused, double *out_4484292768974138631);
void pose_H_4(double *state, double *unused, double *out_3571845806940544873);
void pose_h_10(double *state, double *unused, double *out_6327118854115285339);
void pose_H_10(double *state, double *unused, double *out_5014155151492872447);
void pose_h_13(double *state, double *unused, double *out_7538235923800904941);
void pose_H_13(double *state, double *unused, double *out_4038785401376156056);
void pose_h_14(double *state, double *unused, double *out_5695071338726805119);
void pose_H_14(double *state, double *unused, double *out_391395049398939656);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}