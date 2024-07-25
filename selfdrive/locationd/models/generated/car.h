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
void car_err_fun(double *nom_x, double *delta_x, double *out_3708890511663834590);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_703520109164007484);
void car_H_mod_fun(double *state, double *out_4152512945030036996);
void car_f_fun(double *state, double dt, double *out_3540655432221376455);
void car_F_fun(double *state, double dt, double *out_6921966207091035699);
void car_h_25(double *state, double *unused, double *out_311349995666808605);
void car_H_25(double *state, double *unused, double *out_2192895469755813793);
void car_h_24(double *state, double *unused, double *out_139077460535787258);
void car_H_24(double *state, double *unused, double *out_1808816608874694840);
void car_h_30(double *state, double *unused, double *out_7996142708090176232);
void car_H_30(double *state, double *unused, double *out_4723794871735802962);
void car_h_26(double *state, double *unused, double *out_5448928097958529739);
void car_H_26(double *state, double *unused, double *out_5934398788629870017);
void car_h_27(double *state, double *unused, double *out_8484959468599188535);
void car_H_27(double *state, double *unused, double *out_4496997728699478774);
void car_h_29(double *state, double *unused, double *out_3806337409269702952);
void car_H_29(double *state, double *unused, double *out_5234026216050195146);
void car_h_28(double *state, double *unused, double *out_8611530738599867039);
void car_H_28(double *state, double *unused, double *out_151627198980664572);
void car_h_31(double *state, double *unused, double *out_154163327315679460);
void car_H_31(double *state, double *unused, double *out_2162249507878853365);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}