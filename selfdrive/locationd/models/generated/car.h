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
void car_err_fun(double *nom_x, double *delta_x, double *out_6990374026437360235);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6840343405846249272);
void car_H_mod_fun(double *state, double *out_3104495035882877201);
void car_f_fun(double *state, double dt, double *out_2810783572133327735);
void car_F_fun(double *state, double dt, double *out_7959357771420397480);
void car_h_25(double *state, double *unused, double *out_3287495957292748730);
void car_H_25(double *state, double *unused, double *out_4816842997090982604);
void car_h_24(double *state, double *unused, double *out_2410264965580228903);
void car_H_24(double *state, double *unused, double *out_8374768595893305740);
void car_h_30(double *state, double *unused, double *out_6519370639698261002);
void car_H_30(double *state, double *unused, double *out_289146666963374406);
void car_h_26(double *state, double *unused, double *out_5537286546362962890);
void car_H_26(double *state, double *unused, double *out_1075339678216926380);
void car_h_27(double *state, double *unused, double *out_1714177310368096915);
void car_H_27(double *state, double *unused, double *out_1885616644837050505);
void car_h_29(double *state, double *unused, double *out_6049087946452575526);
void car_H_29(double *state, double *unused, double *out_799378011277766590);
void car_h_28(double *state, double *unused, double *out_1322608741076235770);
void car_H_28(double *state, double *unused, double *out_2763008282843092841);
void car_h_31(double *state, double *unused, double *out_3483339269057602206);
void car_H_31(double *state, double *unused, double *out_449131575983574904);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}