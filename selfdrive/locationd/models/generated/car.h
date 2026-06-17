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
void car_err_fun(double *nom_x, double *delta_x, double *out_3326928322291592435);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_3690896583325655603);
void car_H_mod_fun(double *state, double *out_6684748773688263271);
void car_f_fun(double *state, double dt, double *out_1361046479400392025);
void car_F_fun(double *state, double dt, double *out_6712225080938857681);
void car_h_25(double *state, double *unused, double *out_1643919782939166184);
void car_H_25(double *state, double *unused, double *out_8850650187175166797);
void car_h_24(double *state, double *unused, double *out_975072953934001442);
void car_H_24(double *state, double *unused, double *out_7908277450665280628);
void car_h_30(double *state, double *unused, double *out_2416953653188981371);
void car_H_30(double *state, double *unused, double *out_4322953857047558599);
void car_h_26(double *state, double *unused, double *out_5021358973997679022);
void car_H_26(double *state, double *unused, double *out_5109146868301110573);
void car_h_27(double *state, double *unused, double *out_1198249738002813047);
void car_H_27(double *state, double *unused, double *out_6546547928231501816);
void car_h_29(double *state, double *unused, double *out_203714720147643921);
void car_H_29(double *state, double *unused, double *out_4833185201361950783);
void car_h_28(double *state, double *unused, double *out_1801565622679290564);
void car_H_28(double *state, double *unused, double *out_249213815707579791);
void car_h_31(double *state, double *unused, double *out_7665107006086534250);
void car_H_31(double *state, double *unused, double *out_4482938766067759097);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}