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
void car_err_fun(double *nom_x, double *delta_x, double *out_2575150693000547317);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6433197917275169840);
void car_H_mod_fun(double *state, double *out_6597027900384019102);
void car_f_fun(double *state, double dt, double *out_8598323179917779170);
void car_F_fun(double *state, double dt, double *out_3687363393736083317);
void car_h_25(double *state, double *unused, double *out_6019453859290479602);
void car_H_25(double *state, double *unused, double *out_437886407615775220);
void car_h_24(double *state, double *unused, double *out_2865544647271695197);
void car_H_24(double *state, double *unused, double *out_4768809924533717701);
void car_h_30(double *state, double *unused, double *out_751381367059871334);
void car_H_30(double *state, double *unused, double *out_567225354759015290);
void car_h_26(double *state, double *unused, double *out_1354697251736348851);
void car_H_26(double *state, double *unused, double *out_4179389726489831444);
void car_h_27(double *state, double *unused, double *out_3630980750153074874);
void car_H_27(double *state, double *unused, double *out_2741988666559440201);
void car_h_29(double *state, double *unused, double *out_230702726275426778);
void car_H_29(double *state, double *unused, double *out_56994010444623106);
void car_h_28(double *state, double *unused, double *out_4574490603054737309);
void car_H_28(double *state, double *unused, double *out_1906636261120703145);
void car_h_31(double *state, double *unused, double *out_6176640527641608747);
void car_H_31(double *state, double *unused, double *out_407240445738814792);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}