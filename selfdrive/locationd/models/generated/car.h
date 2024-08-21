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
void car_err_fun(double *nom_x, double *delta_x, double *out_3860548556518224048);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4761333446785978408);
void car_H_mod_fun(double *state, double *out_691743405130441903);
void car_f_fun(double *state, double dt, double *out_489045563455123421);
void car_F_fun(double *state, double dt, double *out_7686270435155235008);
void car_h_25(double *state, double *unused, double *out_9100945793002742709);
void car_H_25(double *state, double *unused, double *out_2546563379272368754);
void car_h_24(double *state, double *unused, double *out_1179022644587177356);
void car_H_24(double *state, double *unused, double *out_1435759772125399144);
void car_h_30(double *state, double *unused, double *out_1582533023886136466);
void car_H_30(double *state, double *unused, double *out_5064896337779617381);
void car_h_26(double *state, double *unused, double *out_6221346409909539169);
void car_H_26(double *state, double *unused, double *out_5851089349033169355);
void car_h_27(double *state, double *unused, double *out_4003931044820778344);
void car_H_27(double *state, double *unused, double *out_7288490408963560598);
void car_h_29(double *state, double *unused, double *out_2185848908562613983);
void car_H_29(double *state, double *unused, double *out_5575127682094009565);
void car_h_28(double *state, double *unused, double *out_967144993376445659);
void car_H_28(double *state, double *unused, double *out_492728665024478991);
void car_h_31(double *state, double *unused, double *out_1640387028753854384);
void car_H_31(double *state, double *unused, double *out_2577209341149329182);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}