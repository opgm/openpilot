#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_4988085883151629357);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_8628576639675182721);
void pose_H_mod_fun(double *state, double *out_844640693863841425);
void pose_f_fun(double *state, double dt, double *out_986367587733679490);
void pose_F_fun(double *state, double dt, double *out_4594191458309706069);
void pose_h_4(double *state, double *unused, double *out_7900639425322801320);
void pose_H_4(double *state, double *unused, double *out_3300993812880714874);
void pose_h_10(double *state, double *unused, double *out_1277818480136149867);
void pose_H_10(double *state, double *unused, double *out_8852053340215761248);
void pose_h_13(double *state, double *unused, double *out_2885571052644477490);
void pose_H_13(double *state, double *unused, double *out_6513267638213047675);
void pose_h_14(double *state, double *unused, double *out_5381598505700020116);
void pose_H_14(double *state, double *unused, double *out_7264234669220199403);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}