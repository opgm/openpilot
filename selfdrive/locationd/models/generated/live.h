#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_7968334240474235674);
void live_err_fun(double *nom_x, double *delta_x, double *out_2210371994950831199);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_6560080521163361192);
void live_H_mod_fun(double *state, double *out_5600525409187939404);
void live_f_fun(double *state, double dt, double *out_1674747573228821627);
void live_F_fun(double *state, double dt, double *out_2662974268483097137);
void live_h_4(double *state, double *unused, double *out_8249341451310559327);
void live_H_4(double *state, double *unused, double *out_734079717095727223);
void live_h_9(double *state, double *unused, double *out_2322300930760657741);
void live_H_9(double *state, double *unused, double *out_8021298652360174693);
void live_h_10(double *state, double *unused, double *out_4615824287832200464);
void live_H_10(double *state, double *unused, double *out_6556685274764548416);
void live_h_12(double *state, double *unused, double *out_5077368063104438257);
void live_H_12(double *state, double *unused, double *out_5647178659947005773);
void live_h_35(double *state, double *unused, double *out_6231900545218429087);
void live_H_35(double *state, double *unused, double *out_2901615627621992064);
void live_h_32(double *state, double *unused, double *out_5814209998727032141);
void live_H_32(double *state, double *unused, double *out_4231906559409075068);
void live_h_13(double *state, double *unused, double *out_7214662002272872579);
void live_H_13(double *state, double *unused, double *out_1895209294482975830);
void live_h_14(double *state, double *unused, double *out_2322300930760657741);
void live_H_14(double *state, double *unused, double *out_8021298652360174693);
void live_h_33(double *state, double *unused, double *out_5269655606650700610);
void live_H_33(double *state, double *unused, double *out_248941377016865540);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}