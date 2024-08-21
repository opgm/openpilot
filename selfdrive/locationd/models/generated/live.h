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
void live_H(double *in_vec, double *out_4029624047422583028);
void live_err_fun(double *nom_x, double *delta_x, double *out_310376691841387558);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_1456132818750440010);
void live_H_mod_fun(double *state, double *out_5922142170418800988);
void live_f_fun(double *state, double dt, double *out_2273442895072544352);
void live_F_fun(double *state, double dt, double *out_4235384512522099094);
void live_h_4(double *state, double *unused, double *out_3685216184752008266);
void live_H_4(double *state, double *unused, double *out_1281723694640841136);
void live_h_9(double *state, double *unused, double *out_2646003671663756402);
void live_H_9(double *state, double *unused, double *out_6005495240623606334);
void live_h_10(double *state, double *unused, double *out_2127462386065991370);
void live_H_10(double *state, double *unused, double *out_1155592061403164186);
void live_h_12(double *state, double *unused, double *out_3690503254606561334);
void live_H_12(double *state, double *unused, double *out_7662982071683574132);
void live_h_35(double *state, double *unused, double *out_5907276418402340129);
void live_H_35(double *state, double *unused, double *out_6483295745716134368);
void live_h_32(double *state, double *unused, double *out_2681780490767782577);
void live_H_32(double *state, double *unused, double *out_2133760874940142830);
void live_h_13(double *state, double *unused, double *out_8503145814356939866);
void live_H_13(double *state, double *unused, double *out_918047224441199883);
void live_h_14(double *state, double *unused, double *out_2646003671663756402);
void live_H_14(double *state, double *unused, double *out_6005495240623606334);
void live_h_33(double *state, double *unused, double *out_5296807586278218245);
void live_H_33(double *state, double *unused, double *out_1766862034719702819);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}