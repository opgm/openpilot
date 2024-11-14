#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_5861723782991102346);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_8211258457099351294);
void pose_H_mod_fun(double *state, double *out_877341408371094612);
void pose_f_fun(double *state, double dt, double *out_6181153239753365860);
void pose_F_fun(double *state, double dt, double *out_125451314886039473);
void pose_h_4(double *state, double *unused, double *out_5481609484354041996);
void pose_H_4(double *state, double *unused, double *out_7988386606170059074);
void pose_h_10(double *state, double *unused, double *out_1946765286996617825);
void pose_H_10(double *state, double *unused, double *out_9127548125181276924);
void pose_h_13(double *state, double *unused, double *out_8982835759751218704);
void pose_H_13(double *state, double *unused, double *out_4776112780837726273);
void pose_h_14(double *state, double *unused, double *out_5159606875724608362);
void pose_H_14(double *state, double *unused, double *out_4025145749830574545);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}