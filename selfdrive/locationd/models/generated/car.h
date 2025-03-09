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
void car_err_fun(double *nom_x, double *delta_x, double *out_6256476077040462880);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1292508760405742630);
void car_H_mod_fun(double *state, double *out_1413406722416810442);
void car_f_fun(double *state, double dt, double *out_7759816362230074958);
void car_F_fun(double *state, double dt, double *out_8282217656282783122);
void car_h_25(double *state, double *unused, double *out_4065437451152131201);
void car_H_25(double *state, double *unused, double *out_4822502917591308972);
void car_h_24(double *state, double *unused, double *out_6335536629906301254);
void car_H_24(double *state, double *unused, double *out_5667499526375004371);
void car_h_30(double *state, double *unused, double *out_4340631513436637090);
void car_H_30(double *state, double *unused, double *out_6707550814626625889);
void car_h_26(double *state, double *unused, double *out_6031079348172318050);
void car_H_26(double *state, double *unused, double *out_5921357803373073915);
void car_h_27(double *state, double *unused, double *out_8503021134998705327);
void car_H_27(double *state, double *unused, double *out_8882314126427050800);
void car_h_29(double *state, double *unused, double *out_806005654026995107);
void car_H_29(double *state, double *unused, double *out_6197319470312233705);
void car_h_28(double *state, double *unused, double *out_1159636242993191742);
void car_H_28(double *state, double *unused, double *out_7167025586327787337);
void car_h_31(double *state, double *unused, double *out_5741429145838878531);
void car_H_31(double *state, double *unused, double *out_6547565905606425391);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}