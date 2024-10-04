#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_3280080996569931798);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_5884406445564509642);
void pose_H_mod_fun(double *state, double *out_8259132217396944238);
void pose_f_fun(double *state, double dt, double *out_2909476989004137244);
void pose_F_fun(double *state, double dt, double *out_7984182452575755119);
void pose_h_4(double *state, double *unused, double *out_9175407422316684506);
void pose_H_4(double *state, double *unused, double *out_6069539240583237125);
void pose_h_10(double *state, double *unused, double *out_8641600486548509028);
void pose_H_10(double *state, double *unused, double *out_5292752278649831974);
void pose_h_13(double *state, double *unused, double *out_878346494641374765);
void pose_H_13(double *state, double *unused, double *out_2857265415250904324);
void pose_h_14(double *state, double *unused, double *out_7346852850300827949);
void pose_H_14(double *state, double *unused, double *out_2106298384243752596);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}