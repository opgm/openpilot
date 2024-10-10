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
void car_err_fun(double *nom_x, double *delta_x, double *out_7321584177188704090);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5388938486158592874);
void car_H_mod_fun(double *state, double *out_4391130289298168509);
void car_f_fun(double *state, double dt, double *out_3998517490544307444);
void car_F_fun(double *state, double dt, double *out_1844626809948872121);
void car_h_25(double *state, double *unused, double *out_1764050421060515004);
void car_H_25(double *state, double *unused, double *out_7474627606809239207);
void car_h_24(double *state, double *unused, double *out_4230355366619659438);
void car_H_24(double *state, double *unused, double *out_2654306102153250944);
void car_h_30(double *state, double *unused, double *out_1873399107193507346);
void car_H_30(double *state, double *unused, double *out_7345288659665999137);
void car_h_26(double *state, double *unused, double *out_4910185158637840031);
void car_H_26(double *state, double *unused, double *out_3733124287935182983);
void car_h_27(double *state, double *unused, double *out_2707560273962177439);
void car_H_27(double *state, double *unused, double *out_5170525347865574226);
void car_h_29(double *state, double *unused, double *out_929889254291844911);
void car_H_29(double *state, double *unused, double *out_7855520003980391321);
void car_h_28(double *state, double *unused, double *out_6348067511785726757);
void car_H_28(double *state, double *unused, double *out_2773120986910860747);
void car_h_31(double *state, double *unused, double *out_6875827055658026880);
void car_H_31(double *state, double *unused, double *out_7505273568686199635);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}