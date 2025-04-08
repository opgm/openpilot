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
void car_err_fun(double *nom_x, double *delta_x, double *out_6967453643969959992);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4192683843425632104);
void car_H_mod_fun(double *state, double *out_8434874153757315530);
void car_f_fun(double *state, double dt, double *out_5018562704091678262);
void car_F_fun(double *state, double dt, double *out_3150118549457082222);
void car_h_25(double *state, double *unused, double *out_8627201318607884604);
void car_H_25(double *state, double *unused, double *out_7375312994936809892);
void car_h_24(double *state, double *unused, double *out_6046618710890368303);
void car_H_24(double *state, double *unused, double *out_6534881210754764900);
void car_h_30(double *state, double *unused, double *out_8368911362472783845);
void car_H_30(double *state, double *unused, double *out_2847616664809201694);
void car_h_26(double *state, double *unused, double *out_7041046659792435086);
void car_H_26(double *state, double *unused, double *out_3633809676062753668);
void car_h_27(double *state, double *unused, double *out_4005015289151776290);
void car_H_27(double *state, double *unused, double *out_672853353008776783);
void car_h_29(double *state, double *unused, double *out_236155425481462693);
void car_H_29(double *state, double *unused, double *out_3357848009123593878);
void car_h_28(double *state, double *unused, double *out_1440889467771303764);
void car_H_28(double *state, double *unused, double *out_1724551007945936696);
void car_h_31(double *state, double *unused, double *out_6182093226847644662);
void car_H_31(double *state, double *unused, double *out_3007601573829402192);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}