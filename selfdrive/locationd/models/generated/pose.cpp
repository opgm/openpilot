#include "pose.h"

namespace {
#define DIM 18
#define EDIM 18
#define MEDIM 18
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_4 = 7.814727903251177;
const static double MAHA_THRESH_10 = 7.814727903251177;
const static double MAHA_THRESH_13 = 7.814727903251177;
const static double MAHA_THRESH_14 = 7.814727903251177;

/******************************************************************************
 *                      Code generated with SymPy 1.13.2                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_7412749682930917159) {
   out_7412749682930917159[0] = delta_x[0] + nom_x[0];
   out_7412749682930917159[1] = delta_x[1] + nom_x[1];
   out_7412749682930917159[2] = delta_x[2] + nom_x[2];
   out_7412749682930917159[3] = delta_x[3] + nom_x[3];
   out_7412749682930917159[4] = delta_x[4] + nom_x[4];
   out_7412749682930917159[5] = delta_x[5] + nom_x[5];
   out_7412749682930917159[6] = delta_x[6] + nom_x[6];
   out_7412749682930917159[7] = delta_x[7] + nom_x[7];
   out_7412749682930917159[8] = delta_x[8] + nom_x[8];
   out_7412749682930917159[9] = delta_x[9] + nom_x[9];
   out_7412749682930917159[10] = delta_x[10] + nom_x[10];
   out_7412749682930917159[11] = delta_x[11] + nom_x[11];
   out_7412749682930917159[12] = delta_x[12] + nom_x[12];
   out_7412749682930917159[13] = delta_x[13] + nom_x[13];
   out_7412749682930917159[14] = delta_x[14] + nom_x[14];
   out_7412749682930917159[15] = delta_x[15] + nom_x[15];
   out_7412749682930917159[16] = delta_x[16] + nom_x[16];
   out_7412749682930917159[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_3324739374567006732) {
   out_3324739374567006732[0] = -nom_x[0] + true_x[0];
   out_3324739374567006732[1] = -nom_x[1] + true_x[1];
   out_3324739374567006732[2] = -nom_x[2] + true_x[2];
   out_3324739374567006732[3] = -nom_x[3] + true_x[3];
   out_3324739374567006732[4] = -nom_x[4] + true_x[4];
   out_3324739374567006732[5] = -nom_x[5] + true_x[5];
   out_3324739374567006732[6] = -nom_x[6] + true_x[6];
   out_3324739374567006732[7] = -nom_x[7] + true_x[7];
   out_3324739374567006732[8] = -nom_x[8] + true_x[8];
   out_3324739374567006732[9] = -nom_x[9] + true_x[9];
   out_3324739374567006732[10] = -nom_x[10] + true_x[10];
   out_3324739374567006732[11] = -nom_x[11] + true_x[11];
   out_3324739374567006732[12] = -nom_x[12] + true_x[12];
   out_3324739374567006732[13] = -nom_x[13] + true_x[13];
   out_3324739374567006732[14] = -nom_x[14] + true_x[14];
   out_3324739374567006732[15] = -nom_x[15] + true_x[15];
   out_3324739374567006732[16] = -nom_x[16] + true_x[16];
   out_3324739374567006732[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_6871722307859650590) {
   out_6871722307859650590[0] = 1.0;
   out_6871722307859650590[1] = 0.0;
   out_6871722307859650590[2] = 0.0;
   out_6871722307859650590[3] = 0.0;
   out_6871722307859650590[4] = 0.0;
   out_6871722307859650590[5] = 0.0;
   out_6871722307859650590[6] = 0.0;
   out_6871722307859650590[7] = 0.0;
   out_6871722307859650590[8] = 0.0;
   out_6871722307859650590[9] = 0.0;
   out_6871722307859650590[10] = 0.0;
   out_6871722307859650590[11] = 0.0;
   out_6871722307859650590[12] = 0.0;
   out_6871722307859650590[13] = 0.0;
   out_6871722307859650590[14] = 0.0;
   out_6871722307859650590[15] = 0.0;
   out_6871722307859650590[16] = 0.0;
   out_6871722307859650590[17] = 0.0;
   out_6871722307859650590[18] = 0.0;
   out_6871722307859650590[19] = 1.0;
   out_6871722307859650590[20] = 0.0;
   out_6871722307859650590[21] = 0.0;
   out_6871722307859650590[22] = 0.0;
   out_6871722307859650590[23] = 0.0;
   out_6871722307859650590[24] = 0.0;
   out_6871722307859650590[25] = 0.0;
   out_6871722307859650590[26] = 0.0;
   out_6871722307859650590[27] = 0.0;
   out_6871722307859650590[28] = 0.0;
   out_6871722307859650590[29] = 0.0;
   out_6871722307859650590[30] = 0.0;
   out_6871722307859650590[31] = 0.0;
   out_6871722307859650590[32] = 0.0;
   out_6871722307859650590[33] = 0.0;
   out_6871722307859650590[34] = 0.0;
   out_6871722307859650590[35] = 0.0;
   out_6871722307859650590[36] = 0.0;
   out_6871722307859650590[37] = 0.0;
   out_6871722307859650590[38] = 1.0;
   out_6871722307859650590[39] = 0.0;
   out_6871722307859650590[40] = 0.0;
   out_6871722307859650590[41] = 0.0;
   out_6871722307859650590[42] = 0.0;
   out_6871722307859650590[43] = 0.0;
   out_6871722307859650590[44] = 0.0;
   out_6871722307859650590[45] = 0.0;
   out_6871722307859650590[46] = 0.0;
   out_6871722307859650590[47] = 0.0;
   out_6871722307859650590[48] = 0.0;
   out_6871722307859650590[49] = 0.0;
   out_6871722307859650590[50] = 0.0;
   out_6871722307859650590[51] = 0.0;
   out_6871722307859650590[52] = 0.0;
   out_6871722307859650590[53] = 0.0;
   out_6871722307859650590[54] = 0.0;
   out_6871722307859650590[55] = 0.0;
   out_6871722307859650590[56] = 0.0;
   out_6871722307859650590[57] = 1.0;
   out_6871722307859650590[58] = 0.0;
   out_6871722307859650590[59] = 0.0;
   out_6871722307859650590[60] = 0.0;
   out_6871722307859650590[61] = 0.0;
   out_6871722307859650590[62] = 0.0;
   out_6871722307859650590[63] = 0.0;
   out_6871722307859650590[64] = 0.0;
   out_6871722307859650590[65] = 0.0;
   out_6871722307859650590[66] = 0.0;
   out_6871722307859650590[67] = 0.0;
   out_6871722307859650590[68] = 0.0;
   out_6871722307859650590[69] = 0.0;
   out_6871722307859650590[70] = 0.0;
   out_6871722307859650590[71] = 0.0;
   out_6871722307859650590[72] = 0.0;
   out_6871722307859650590[73] = 0.0;
   out_6871722307859650590[74] = 0.0;
   out_6871722307859650590[75] = 0.0;
   out_6871722307859650590[76] = 1.0;
   out_6871722307859650590[77] = 0.0;
   out_6871722307859650590[78] = 0.0;
   out_6871722307859650590[79] = 0.0;
   out_6871722307859650590[80] = 0.0;
   out_6871722307859650590[81] = 0.0;
   out_6871722307859650590[82] = 0.0;
   out_6871722307859650590[83] = 0.0;
   out_6871722307859650590[84] = 0.0;
   out_6871722307859650590[85] = 0.0;
   out_6871722307859650590[86] = 0.0;
   out_6871722307859650590[87] = 0.0;
   out_6871722307859650590[88] = 0.0;
   out_6871722307859650590[89] = 0.0;
   out_6871722307859650590[90] = 0.0;
   out_6871722307859650590[91] = 0.0;
   out_6871722307859650590[92] = 0.0;
   out_6871722307859650590[93] = 0.0;
   out_6871722307859650590[94] = 0.0;
   out_6871722307859650590[95] = 1.0;
   out_6871722307859650590[96] = 0.0;
   out_6871722307859650590[97] = 0.0;
   out_6871722307859650590[98] = 0.0;
   out_6871722307859650590[99] = 0.0;
   out_6871722307859650590[100] = 0.0;
   out_6871722307859650590[101] = 0.0;
   out_6871722307859650590[102] = 0.0;
   out_6871722307859650590[103] = 0.0;
   out_6871722307859650590[104] = 0.0;
   out_6871722307859650590[105] = 0.0;
   out_6871722307859650590[106] = 0.0;
   out_6871722307859650590[107] = 0.0;
   out_6871722307859650590[108] = 0.0;
   out_6871722307859650590[109] = 0.0;
   out_6871722307859650590[110] = 0.0;
   out_6871722307859650590[111] = 0.0;
   out_6871722307859650590[112] = 0.0;
   out_6871722307859650590[113] = 0.0;
   out_6871722307859650590[114] = 1.0;
   out_6871722307859650590[115] = 0.0;
   out_6871722307859650590[116] = 0.0;
   out_6871722307859650590[117] = 0.0;
   out_6871722307859650590[118] = 0.0;
   out_6871722307859650590[119] = 0.0;
   out_6871722307859650590[120] = 0.0;
   out_6871722307859650590[121] = 0.0;
   out_6871722307859650590[122] = 0.0;
   out_6871722307859650590[123] = 0.0;
   out_6871722307859650590[124] = 0.0;
   out_6871722307859650590[125] = 0.0;
   out_6871722307859650590[126] = 0.0;
   out_6871722307859650590[127] = 0.0;
   out_6871722307859650590[128] = 0.0;
   out_6871722307859650590[129] = 0.0;
   out_6871722307859650590[130] = 0.0;
   out_6871722307859650590[131] = 0.0;
   out_6871722307859650590[132] = 0.0;
   out_6871722307859650590[133] = 1.0;
   out_6871722307859650590[134] = 0.0;
   out_6871722307859650590[135] = 0.0;
   out_6871722307859650590[136] = 0.0;
   out_6871722307859650590[137] = 0.0;
   out_6871722307859650590[138] = 0.0;
   out_6871722307859650590[139] = 0.0;
   out_6871722307859650590[140] = 0.0;
   out_6871722307859650590[141] = 0.0;
   out_6871722307859650590[142] = 0.0;
   out_6871722307859650590[143] = 0.0;
   out_6871722307859650590[144] = 0.0;
   out_6871722307859650590[145] = 0.0;
   out_6871722307859650590[146] = 0.0;
   out_6871722307859650590[147] = 0.0;
   out_6871722307859650590[148] = 0.0;
   out_6871722307859650590[149] = 0.0;
   out_6871722307859650590[150] = 0.0;
   out_6871722307859650590[151] = 0.0;
   out_6871722307859650590[152] = 1.0;
   out_6871722307859650590[153] = 0.0;
   out_6871722307859650590[154] = 0.0;
   out_6871722307859650590[155] = 0.0;
   out_6871722307859650590[156] = 0.0;
   out_6871722307859650590[157] = 0.0;
   out_6871722307859650590[158] = 0.0;
   out_6871722307859650590[159] = 0.0;
   out_6871722307859650590[160] = 0.0;
   out_6871722307859650590[161] = 0.0;
   out_6871722307859650590[162] = 0.0;
   out_6871722307859650590[163] = 0.0;
   out_6871722307859650590[164] = 0.0;
   out_6871722307859650590[165] = 0.0;
   out_6871722307859650590[166] = 0.0;
   out_6871722307859650590[167] = 0.0;
   out_6871722307859650590[168] = 0.0;
   out_6871722307859650590[169] = 0.0;
   out_6871722307859650590[170] = 0.0;
   out_6871722307859650590[171] = 1.0;
   out_6871722307859650590[172] = 0.0;
   out_6871722307859650590[173] = 0.0;
   out_6871722307859650590[174] = 0.0;
   out_6871722307859650590[175] = 0.0;
   out_6871722307859650590[176] = 0.0;
   out_6871722307859650590[177] = 0.0;
   out_6871722307859650590[178] = 0.0;
   out_6871722307859650590[179] = 0.0;
   out_6871722307859650590[180] = 0.0;
   out_6871722307859650590[181] = 0.0;
   out_6871722307859650590[182] = 0.0;
   out_6871722307859650590[183] = 0.0;
   out_6871722307859650590[184] = 0.0;
   out_6871722307859650590[185] = 0.0;
   out_6871722307859650590[186] = 0.0;
   out_6871722307859650590[187] = 0.0;
   out_6871722307859650590[188] = 0.0;
   out_6871722307859650590[189] = 0.0;
   out_6871722307859650590[190] = 1.0;
   out_6871722307859650590[191] = 0.0;
   out_6871722307859650590[192] = 0.0;
   out_6871722307859650590[193] = 0.0;
   out_6871722307859650590[194] = 0.0;
   out_6871722307859650590[195] = 0.0;
   out_6871722307859650590[196] = 0.0;
   out_6871722307859650590[197] = 0.0;
   out_6871722307859650590[198] = 0.0;
   out_6871722307859650590[199] = 0.0;
   out_6871722307859650590[200] = 0.0;
   out_6871722307859650590[201] = 0.0;
   out_6871722307859650590[202] = 0.0;
   out_6871722307859650590[203] = 0.0;
   out_6871722307859650590[204] = 0.0;
   out_6871722307859650590[205] = 0.0;
   out_6871722307859650590[206] = 0.0;
   out_6871722307859650590[207] = 0.0;
   out_6871722307859650590[208] = 0.0;
   out_6871722307859650590[209] = 1.0;
   out_6871722307859650590[210] = 0.0;
   out_6871722307859650590[211] = 0.0;
   out_6871722307859650590[212] = 0.0;
   out_6871722307859650590[213] = 0.0;
   out_6871722307859650590[214] = 0.0;
   out_6871722307859650590[215] = 0.0;
   out_6871722307859650590[216] = 0.0;
   out_6871722307859650590[217] = 0.0;
   out_6871722307859650590[218] = 0.0;
   out_6871722307859650590[219] = 0.0;
   out_6871722307859650590[220] = 0.0;
   out_6871722307859650590[221] = 0.0;
   out_6871722307859650590[222] = 0.0;
   out_6871722307859650590[223] = 0.0;
   out_6871722307859650590[224] = 0.0;
   out_6871722307859650590[225] = 0.0;
   out_6871722307859650590[226] = 0.0;
   out_6871722307859650590[227] = 0.0;
   out_6871722307859650590[228] = 1.0;
   out_6871722307859650590[229] = 0.0;
   out_6871722307859650590[230] = 0.0;
   out_6871722307859650590[231] = 0.0;
   out_6871722307859650590[232] = 0.0;
   out_6871722307859650590[233] = 0.0;
   out_6871722307859650590[234] = 0.0;
   out_6871722307859650590[235] = 0.0;
   out_6871722307859650590[236] = 0.0;
   out_6871722307859650590[237] = 0.0;
   out_6871722307859650590[238] = 0.0;
   out_6871722307859650590[239] = 0.0;
   out_6871722307859650590[240] = 0.0;
   out_6871722307859650590[241] = 0.0;
   out_6871722307859650590[242] = 0.0;
   out_6871722307859650590[243] = 0.0;
   out_6871722307859650590[244] = 0.0;
   out_6871722307859650590[245] = 0.0;
   out_6871722307859650590[246] = 0.0;
   out_6871722307859650590[247] = 1.0;
   out_6871722307859650590[248] = 0.0;
   out_6871722307859650590[249] = 0.0;
   out_6871722307859650590[250] = 0.0;
   out_6871722307859650590[251] = 0.0;
   out_6871722307859650590[252] = 0.0;
   out_6871722307859650590[253] = 0.0;
   out_6871722307859650590[254] = 0.0;
   out_6871722307859650590[255] = 0.0;
   out_6871722307859650590[256] = 0.0;
   out_6871722307859650590[257] = 0.0;
   out_6871722307859650590[258] = 0.0;
   out_6871722307859650590[259] = 0.0;
   out_6871722307859650590[260] = 0.0;
   out_6871722307859650590[261] = 0.0;
   out_6871722307859650590[262] = 0.0;
   out_6871722307859650590[263] = 0.0;
   out_6871722307859650590[264] = 0.0;
   out_6871722307859650590[265] = 0.0;
   out_6871722307859650590[266] = 1.0;
   out_6871722307859650590[267] = 0.0;
   out_6871722307859650590[268] = 0.0;
   out_6871722307859650590[269] = 0.0;
   out_6871722307859650590[270] = 0.0;
   out_6871722307859650590[271] = 0.0;
   out_6871722307859650590[272] = 0.0;
   out_6871722307859650590[273] = 0.0;
   out_6871722307859650590[274] = 0.0;
   out_6871722307859650590[275] = 0.0;
   out_6871722307859650590[276] = 0.0;
   out_6871722307859650590[277] = 0.0;
   out_6871722307859650590[278] = 0.0;
   out_6871722307859650590[279] = 0.0;
   out_6871722307859650590[280] = 0.0;
   out_6871722307859650590[281] = 0.0;
   out_6871722307859650590[282] = 0.0;
   out_6871722307859650590[283] = 0.0;
   out_6871722307859650590[284] = 0.0;
   out_6871722307859650590[285] = 1.0;
   out_6871722307859650590[286] = 0.0;
   out_6871722307859650590[287] = 0.0;
   out_6871722307859650590[288] = 0.0;
   out_6871722307859650590[289] = 0.0;
   out_6871722307859650590[290] = 0.0;
   out_6871722307859650590[291] = 0.0;
   out_6871722307859650590[292] = 0.0;
   out_6871722307859650590[293] = 0.0;
   out_6871722307859650590[294] = 0.0;
   out_6871722307859650590[295] = 0.0;
   out_6871722307859650590[296] = 0.0;
   out_6871722307859650590[297] = 0.0;
   out_6871722307859650590[298] = 0.0;
   out_6871722307859650590[299] = 0.0;
   out_6871722307859650590[300] = 0.0;
   out_6871722307859650590[301] = 0.0;
   out_6871722307859650590[302] = 0.0;
   out_6871722307859650590[303] = 0.0;
   out_6871722307859650590[304] = 1.0;
   out_6871722307859650590[305] = 0.0;
   out_6871722307859650590[306] = 0.0;
   out_6871722307859650590[307] = 0.0;
   out_6871722307859650590[308] = 0.0;
   out_6871722307859650590[309] = 0.0;
   out_6871722307859650590[310] = 0.0;
   out_6871722307859650590[311] = 0.0;
   out_6871722307859650590[312] = 0.0;
   out_6871722307859650590[313] = 0.0;
   out_6871722307859650590[314] = 0.0;
   out_6871722307859650590[315] = 0.0;
   out_6871722307859650590[316] = 0.0;
   out_6871722307859650590[317] = 0.0;
   out_6871722307859650590[318] = 0.0;
   out_6871722307859650590[319] = 0.0;
   out_6871722307859650590[320] = 0.0;
   out_6871722307859650590[321] = 0.0;
   out_6871722307859650590[322] = 0.0;
   out_6871722307859650590[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_8847044247784726907) {
   out_8847044247784726907[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_8847044247784726907[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_8847044247784726907[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_8847044247784726907[3] = dt*state[12] + state[3];
   out_8847044247784726907[4] = dt*state[13] + state[4];
   out_8847044247784726907[5] = dt*state[14] + state[5];
   out_8847044247784726907[6] = state[6];
   out_8847044247784726907[7] = state[7];
   out_8847044247784726907[8] = state[8];
   out_8847044247784726907[9] = state[9];
   out_8847044247784726907[10] = state[10];
   out_8847044247784726907[11] = state[11];
   out_8847044247784726907[12] = state[12];
   out_8847044247784726907[13] = state[13];
   out_8847044247784726907[14] = state[14];
   out_8847044247784726907[15] = state[15];
   out_8847044247784726907[16] = state[16];
   out_8847044247784726907[17] = state[17];
}
void F_fun(double *state, double dt, double *out_6117558309002218944) {
   out_6117558309002218944[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6117558309002218944[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6117558309002218944[2] = 0;
   out_6117558309002218944[3] = 0;
   out_6117558309002218944[4] = 0;
   out_6117558309002218944[5] = 0;
   out_6117558309002218944[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6117558309002218944[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6117558309002218944[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6117558309002218944[9] = 0;
   out_6117558309002218944[10] = 0;
   out_6117558309002218944[11] = 0;
   out_6117558309002218944[12] = 0;
   out_6117558309002218944[13] = 0;
   out_6117558309002218944[14] = 0;
   out_6117558309002218944[15] = 0;
   out_6117558309002218944[16] = 0;
   out_6117558309002218944[17] = 0;
   out_6117558309002218944[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_6117558309002218944[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_6117558309002218944[20] = 0;
   out_6117558309002218944[21] = 0;
   out_6117558309002218944[22] = 0;
   out_6117558309002218944[23] = 0;
   out_6117558309002218944[24] = 0;
   out_6117558309002218944[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_6117558309002218944[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_6117558309002218944[27] = 0;
   out_6117558309002218944[28] = 0;
   out_6117558309002218944[29] = 0;
   out_6117558309002218944[30] = 0;
   out_6117558309002218944[31] = 0;
   out_6117558309002218944[32] = 0;
   out_6117558309002218944[33] = 0;
   out_6117558309002218944[34] = 0;
   out_6117558309002218944[35] = 0;
   out_6117558309002218944[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6117558309002218944[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6117558309002218944[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6117558309002218944[39] = 0;
   out_6117558309002218944[40] = 0;
   out_6117558309002218944[41] = 0;
   out_6117558309002218944[42] = 0;
   out_6117558309002218944[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6117558309002218944[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6117558309002218944[45] = 0;
   out_6117558309002218944[46] = 0;
   out_6117558309002218944[47] = 0;
   out_6117558309002218944[48] = 0;
   out_6117558309002218944[49] = 0;
   out_6117558309002218944[50] = 0;
   out_6117558309002218944[51] = 0;
   out_6117558309002218944[52] = 0;
   out_6117558309002218944[53] = 0;
   out_6117558309002218944[54] = 0;
   out_6117558309002218944[55] = 0;
   out_6117558309002218944[56] = 0;
   out_6117558309002218944[57] = 1;
   out_6117558309002218944[58] = 0;
   out_6117558309002218944[59] = 0;
   out_6117558309002218944[60] = 0;
   out_6117558309002218944[61] = 0;
   out_6117558309002218944[62] = 0;
   out_6117558309002218944[63] = 0;
   out_6117558309002218944[64] = 0;
   out_6117558309002218944[65] = 0;
   out_6117558309002218944[66] = dt;
   out_6117558309002218944[67] = 0;
   out_6117558309002218944[68] = 0;
   out_6117558309002218944[69] = 0;
   out_6117558309002218944[70] = 0;
   out_6117558309002218944[71] = 0;
   out_6117558309002218944[72] = 0;
   out_6117558309002218944[73] = 0;
   out_6117558309002218944[74] = 0;
   out_6117558309002218944[75] = 0;
   out_6117558309002218944[76] = 1;
   out_6117558309002218944[77] = 0;
   out_6117558309002218944[78] = 0;
   out_6117558309002218944[79] = 0;
   out_6117558309002218944[80] = 0;
   out_6117558309002218944[81] = 0;
   out_6117558309002218944[82] = 0;
   out_6117558309002218944[83] = 0;
   out_6117558309002218944[84] = 0;
   out_6117558309002218944[85] = dt;
   out_6117558309002218944[86] = 0;
   out_6117558309002218944[87] = 0;
   out_6117558309002218944[88] = 0;
   out_6117558309002218944[89] = 0;
   out_6117558309002218944[90] = 0;
   out_6117558309002218944[91] = 0;
   out_6117558309002218944[92] = 0;
   out_6117558309002218944[93] = 0;
   out_6117558309002218944[94] = 0;
   out_6117558309002218944[95] = 1;
   out_6117558309002218944[96] = 0;
   out_6117558309002218944[97] = 0;
   out_6117558309002218944[98] = 0;
   out_6117558309002218944[99] = 0;
   out_6117558309002218944[100] = 0;
   out_6117558309002218944[101] = 0;
   out_6117558309002218944[102] = 0;
   out_6117558309002218944[103] = 0;
   out_6117558309002218944[104] = dt;
   out_6117558309002218944[105] = 0;
   out_6117558309002218944[106] = 0;
   out_6117558309002218944[107] = 0;
   out_6117558309002218944[108] = 0;
   out_6117558309002218944[109] = 0;
   out_6117558309002218944[110] = 0;
   out_6117558309002218944[111] = 0;
   out_6117558309002218944[112] = 0;
   out_6117558309002218944[113] = 0;
   out_6117558309002218944[114] = 1;
   out_6117558309002218944[115] = 0;
   out_6117558309002218944[116] = 0;
   out_6117558309002218944[117] = 0;
   out_6117558309002218944[118] = 0;
   out_6117558309002218944[119] = 0;
   out_6117558309002218944[120] = 0;
   out_6117558309002218944[121] = 0;
   out_6117558309002218944[122] = 0;
   out_6117558309002218944[123] = 0;
   out_6117558309002218944[124] = 0;
   out_6117558309002218944[125] = 0;
   out_6117558309002218944[126] = 0;
   out_6117558309002218944[127] = 0;
   out_6117558309002218944[128] = 0;
   out_6117558309002218944[129] = 0;
   out_6117558309002218944[130] = 0;
   out_6117558309002218944[131] = 0;
   out_6117558309002218944[132] = 0;
   out_6117558309002218944[133] = 1;
   out_6117558309002218944[134] = 0;
   out_6117558309002218944[135] = 0;
   out_6117558309002218944[136] = 0;
   out_6117558309002218944[137] = 0;
   out_6117558309002218944[138] = 0;
   out_6117558309002218944[139] = 0;
   out_6117558309002218944[140] = 0;
   out_6117558309002218944[141] = 0;
   out_6117558309002218944[142] = 0;
   out_6117558309002218944[143] = 0;
   out_6117558309002218944[144] = 0;
   out_6117558309002218944[145] = 0;
   out_6117558309002218944[146] = 0;
   out_6117558309002218944[147] = 0;
   out_6117558309002218944[148] = 0;
   out_6117558309002218944[149] = 0;
   out_6117558309002218944[150] = 0;
   out_6117558309002218944[151] = 0;
   out_6117558309002218944[152] = 1;
   out_6117558309002218944[153] = 0;
   out_6117558309002218944[154] = 0;
   out_6117558309002218944[155] = 0;
   out_6117558309002218944[156] = 0;
   out_6117558309002218944[157] = 0;
   out_6117558309002218944[158] = 0;
   out_6117558309002218944[159] = 0;
   out_6117558309002218944[160] = 0;
   out_6117558309002218944[161] = 0;
   out_6117558309002218944[162] = 0;
   out_6117558309002218944[163] = 0;
   out_6117558309002218944[164] = 0;
   out_6117558309002218944[165] = 0;
   out_6117558309002218944[166] = 0;
   out_6117558309002218944[167] = 0;
   out_6117558309002218944[168] = 0;
   out_6117558309002218944[169] = 0;
   out_6117558309002218944[170] = 0;
   out_6117558309002218944[171] = 1;
   out_6117558309002218944[172] = 0;
   out_6117558309002218944[173] = 0;
   out_6117558309002218944[174] = 0;
   out_6117558309002218944[175] = 0;
   out_6117558309002218944[176] = 0;
   out_6117558309002218944[177] = 0;
   out_6117558309002218944[178] = 0;
   out_6117558309002218944[179] = 0;
   out_6117558309002218944[180] = 0;
   out_6117558309002218944[181] = 0;
   out_6117558309002218944[182] = 0;
   out_6117558309002218944[183] = 0;
   out_6117558309002218944[184] = 0;
   out_6117558309002218944[185] = 0;
   out_6117558309002218944[186] = 0;
   out_6117558309002218944[187] = 0;
   out_6117558309002218944[188] = 0;
   out_6117558309002218944[189] = 0;
   out_6117558309002218944[190] = 1;
   out_6117558309002218944[191] = 0;
   out_6117558309002218944[192] = 0;
   out_6117558309002218944[193] = 0;
   out_6117558309002218944[194] = 0;
   out_6117558309002218944[195] = 0;
   out_6117558309002218944[196] = 0;
   out_6117558309002218944[197] = 0;
   out_6117558309002218944[198] = 0;
   out_6117558309002218944[199] = 0;
   out_6117558309002218944[200] = 0;
   out_6117558309002218944[201] = 0;
   out_6117558309002218944[202] = 0;
   out_6117558309002218944[203] = 0;
   out_6117558309002218944[204] = 0;
   out_6117558309002218944[205] = 0;
   out_6117558309002218944[206] = 0;
   out_6117558309002218944[207] = 0;
   out_6117558309002218944[208] = 0;
   out_6117558309002218944[209] = 1;
   out_6117558309002218944[210] = 0;
   out_6117558309002218944[211] = 0;
   out_6117558309002218944[212] = 0;
   out_6117558309002218944[213] = 0;
   out_6117558309002218944[214] = 0;
   out_6117558309002218944[215] = 0;
   out_6117558309002218944[216] = 0;
   out_6117558309002218944[217] = 0;
   out_6117558309002218944[218] = 0;
   out_6117558309002218944[219] = 0;
   out_6117558309002218944[220] = 0;
   out_6117558309002218944[221] = 0;
   out_6117558309002218944[222] = 0;
   out_6117558309002218944[223] = 0;
   out_6117558309002218944[224] = 0;
   out_6117558309002218944[225] = 0;
   out_6117558309002218944[226] = 0;
   out_6117558309002218944[227] = 0;
   out_6117558309002218944[228] = 1;
   out_6117558309002218944[229] = 0;
   out_6117558309002218944[230] = 0;
   out_6117558309002218944[231] = 0;
   out_6117558309002218944[232] = 0;
   out_6117558309002218944[233] = 0;
   out_6117558309002218944[234] = 0;
   out_6117558309002218944[235] = 0;
   out_6117558309002218944[236] = 0;
   out_6117558309002218944[237] = 0;
   out_6117558309002218944[238] = 0;
   out_6117558309002218944[239] = 0;
   out_6117558309002218944[240] = 0;
   out_6117558309002218944[241] = 0;
   out_6117558309002218944[242] = 0;
   out_6117558309002218944[243] = 0;
   out_6117558309002218944[244] = 0;
   out_6117558309002218944[245] = 0;
   out_6117558309002218944[246] = 0;
   out_6117558309002218944[247] = 1;
   out_6117558309002218944[248] = 0;
   out_6117558309002218944[249] = 0;
   out_6117558309002218944[250] = 0;
   out_6117558309002218944[251] = 0;
   out_6117558309002218944[252] = 0;
   out_6117558309002218944[253] = 0;
   out_6117558309002218944[254] = 0;
   out_6117558309002218944[255] = 0;
   out_6117558309002218944[256] = 0;
   out_6117558309002218944[257] = 0;
   out_6117558309002218944[258] = 0;
   out_6117558309002218944[259] = 0;
   out_6117558309002218944[260] = 0;
   out_6117558309002218944[261] = 0;
   out_6117558309002218944[262] = 0;
   out_6117558309002218944[263] = 0;
   out_6117558309002218944[264] = 0;
   out_6117558309002218944[265] = 0;
   out_6117558309002218944[266] = 1;
   out_6117558309002218944[267] = 0;
   out_6117558309002218944[268] = 0;
   out_6117558309002218944[269] = 0;
   out_6117558309002218944[270] = 0;
   out_6117558309002218944[271] = 0;
   out_6117558309002218944[272] = 0;
   out_6117558309002218944[273] = 0;
   out_6117558309002218944[274] = 0;
   out_6117558309002218944[275] = 0;
   out_6117558309002218944[276] = 0;
   out_6117558309002218944[277] = 0;
   out_6117558309002218944[278] = 0;
   out_6117558309002218944[279] = 0;
   out_6117558309002218944[280] = 0;
   out_6117558309002218944[281] = 0;
   out_6117558309002218944[282] = 0;
   out_6117558309002218944[283] = 0;
   out_6117558309002218944[284] = 0;
   out_6117558309002218944[285] = 1;
   out_6117558309002218944[286] = 0;
   out_6117558309002218944[287] = 0;
   out_6117558309002218944[288] = 0;
   out_6117558309002218944[289] = 0;
   out_6117558309002218944[290] = 0;
   out_6117558309002218944[291] = 0;
   out_6117558309002218944[292] = 0;
   out_6117558309002218944[293] = 0;
   out_6117558309002218944[294] = 0;
   out_6117558309002218944[295] = 0;
   out_6117558309002218944[296] = 0;
   out_6117558309002218944[297] = 0;
   out_6117558309002218944[298] = 0;
   out_6117558309002218944[299] = 0;
   out_6117558309002218944[300] = 0;
   out_6117558309002218944[301] = 0;
   out_6117558309002218944[302] = 0;
   out_6117558309002218944[303] = 0;
   out_6117558309002218944[304] = 1;
   out_6117558309002218944[305] = 0;
   out_6117558309002218944[306] = 0;
   out_6117558309002218944[307] = 0;
   out_6117558309002218944[308] = 0;
   out_6117558309002218944[309] = 0;
   out_6117558309002218944[310] = 0;
   out_6117558309002218944[311] = 0;
   out_6117558309002218944[312] = 0;
   out_6117558309002218944[313] = 0;
   out_6117558309002218944[314] = 0;
   out_6117558309002218944[315] = 0;
   out_6117558309002218944[316] = 0;
   out_6117558309002218944[317] = 0;
   out_6117558309002218944[318] = 0;
   out_6117558309002218944[319] = 0;
   out_6117558309002218944[320] = 0;
   out_6117558309002218944[321] = 0;
   out_6117558309002218944[322] = 0;
   out_6117558309002218944[323] = 1;
}
void h_4(double *state, double *unused, double *out_2048679593753205460) {
   out_2048679593753205460[0] = state[6] + state[9];
   out_2048679593753205460[1] = state[7] + state[10];
   out_2048679593753205460[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_3473479926877488476) {
   out_3473479926877488476[0] = 0;
   out_3473479926877488476[1] = 0;
   out_3473479926877488476[2] = 0;
   out_3473479926877488476[3] = 0;
   out_3473479926877488476[4] = 0;
   out_3473479926877488476[5] = 0;
   out_3473479926877488476[6] = 1;
   out_3473479926877488476[7] = 0;
   out_3473479926877488476[8] = 0;
   out_3473479926877488476[9] = 1;
   out_3473479926877488476[10] = 0;
   out_3473479926877488476[11] = 0;
   out_3473479926877488476[12] = 0;
   out_3473479926877488476[13] = 0;
   out_3473479926877488476[14] = 0;
   out_3473479926877488476[15] = 0;
   out_3473479926877488476[16] = 0;
   out_3473479926877488476[17] = 0;
   out_3473479926877488476[18] = 0;
   out_3473479926877488476[19] = 0;
   out_3473479926877488476[20] = 0;
   out_3473479926877488476[21] = 0;
   out_3473479926877488476[22] = 0;
   out_3473479926877488476[23] = 0;
   out_3473479926877488476[24] = 0;
   out_3473479926877488476[25] = 1;
   out_3473479926877488476[26] = 0;
   out_3473479926877488476[27] = 0;
   out_3473479926877488476[28] = 1;
   out_3473479926877488476[29] = 0;
   out_3473479926877488476[30] = 0;
   out_3473479926877488476[31] = 0;
   out_3473479926877488476[32] = 0;
   out_3473479926877488476[33] = 0;
   out_3473479926877488476[34] = 0;
   out_3473479926877488476[35] = 0;
   out_3473479926877488476[36] = 0;
   out_3473479926877488476[37] = 0;
   out_3473479926877488476[38] = 0;
   out_3473479926877488476[39] = 0;
   out_3473479926877488476[40] = 0;
   out_3473479926877488476[41] = 0;
   out_3473479926877488476[42] = 0;
   out_3473479926877488476[43] = 0;
   out_3473479926877488476[44] = 1;
   out_3473479926877488476[45] = 0;
   out_3473479926877488476[46] = 0;
   out_3473479926877488476[47] = 1;
   out_3473479926877488476[48] = 0;
   out_3473479926877488476[49] = 0;
   out_3473479926877488476[50] = 0;
   out_3473479926877488476[51] = 0;
   out_3473479926877488476[52] = 0;
   out_3473479926877488476[53] = 0;
}
void h_10(double *state, double *unused, double *out_6141237783103343630) {
   out_6141237783103343630[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_6141237783103343630[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_6141237783103343630[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_8609451639616367789) {
   out_8609451639616367789[0] = 0;
   out_8609451639616367789[1] = 9.8100000000000005*cos(state[1]);
   out_8609451639616367789[2] = 0;
   out_8609451639616367789[3] = 0;
   out_8609451639616367789[4] = -state[8];
   out_8609451639616367789[5] = state[7];
   out_8609451639616367789[6] = 0;
   out_8609451639616367789[7] = state[5];
   out_8609451639616367789[8] = -state[4];
   out_8609451639616367789[9] = 0;
   out_8609451639616367789[10] = 0;
   out_8609451639616367789[11] = 0;
   out_8609451639616367789[12] = 1;
   out_8609451639616367789[13] = 0;
   out_8609451639616367789[14] = 0;
   out_8609451639616367789[15] = 1;
   out_8609451639616367789[16] = 0;
   out_8609451639616367789[17] = 0;
   out_8609451639616367789[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_8609451639616367789[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_8609451639616367789[20] = 0;
   out_8609451639616367789[21] = state[8];
   out_8609451639616367789[22] = 0;
   out_8609451639616367789[23] = -state[6];
   out_8609451639616367789[24] = -state[5];
   out_8609451639616367789[25] = 0;
   out_8609451639616367789[26] = state[3];
   out_8609451639616367789[27] = 0;
   out_8609451639616367789[28] = 0;
   out_8609451639616367789[29] = 0;
   out_8609451639616367789[30] = 0;
   out_8609451639616367789[31] = 1;
   out_8609451639616367789[32] = 0;
   out_8609451639616367789[33] = 0;
   out_8609451639616367789[34] = 1;
   out_8609451639616367789[35] = 0;
   out_8609451639616367789[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_8609451639616367789[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_8609451639616367789[38] = 0;
   out_8609451639616367789[39] = -state[7];
   out_8609451639616367789[40] = state[6];
   out_8609451639616367789[41] = 0;
   out_8609451639616367789[42] = state[4];
   out_8609451639616367789[43] = -state[3];
   out_8609451639616367789[44] = 0;
   out_8609451639616367789[45] = 0;
   out_8609451639616367789[46] = 0;
   out_8609451639616367789[47] = 0;
   out_8609451639616367789[48] = 0;
   out_8609451639616367789[49] = 0;
   out_8609451639616367789[50] = 1;
   out_8609451639616367789[51] = 0;
   out_8609451639616367789[52] = 0;
   out_8609451639616367789[53] = 1;
}
void h_13(double *state, double *unused, double *out_2125878123702031972) {
   out_2125878123702031972[0] = state[3];
   out_2125878123702031972[1] = state[4];
   out_2125878123702031972[2] = state[5];
}
void H_13(double *state, double *unused, double *out_6685753752209821277) {
   out_6685753752209821277[0] = 0;
   out_6685753752209821277[1] = 0;
   out_6685753752209821277[2] = 0;
   out_6685753752209821277[3] = 1;
   out_6685753752209821277[4] = 0;
   out_6685753752209821277[5] = 0;
   out_6685753752209821277[6] = 0;
   out_6685753752209821277[7] = 0;
   out_6685753752209821277[8] = 0;
   out_6685753752209821277[9] = 0;
   out_6685753752209821277[10] = 0;
   out_6685753752209821277[11] = 0;
   out_6685753752209821277[12] = 0;
   out_6685753752209821277[13] = 0;
   out_6685753752209821277[14] = 0;
   out_6685753752209821277[15] = 0;
   out_6685753752209821277[16] = 0;
   out_6685753752209821277[17] = 0;
   out_6685753752209821277[18] = 0;
   out_6685753752209821277[19] = 0;
   out_6685753752209821277[20] = 0;
   out_6685753752209821277[21] = 0;
   out_6685753752209821277[22] = 1;
   out_6685753752209821277[23] = 0;
   out_6685753752209821277[24] = 0;
   out_6685753752209821277[25] = 0;
   out_6685753752209821277[26] = 0;
   out_6685753752209821277[27] = 0;
   out_6685753752209821277[28] = 0;
   out_6685753752209821277[29] = 0;
   out_6685753752209821277[30] = 0;
   out_6685753752209821277[31] = 0;
   out_6685753752209821277[32] = 0;
   out_6685753752209821277[33] = 0;
   out_6685753752209821277[34] = 0;
   out_6685753752209821277[35] = 0;
   out_6685753752209821277[36] = 0;
   out_6685753752209821277[37] = 0;
   out_6685753752209821277[38] = 0;
   out_6685753752209821277[39] = 0;
   out_6685753752209821277[40] = 0;
   out_6685753752209821277[41] = 1;
   out_6685753752209821277[42] = 0;
   out_6685753752209821277[43] = 0;
   out_6685753752209821277[44] = 0;
   out_6685753752209821277[45] = 0;
   out_6685753752209821277[46] = 0;
   out_6685753752209821277[47] = 0;
   out_6685753752209821277[48] = 0;
   out_6685753752209821277[49] = 0;
   out_6685753752209821277[50] = 0;
   out_6685753752209821277[51] = 0;
   out_6685753752209821277[52] = 0;
   out_6685753752209821277[53] = 0;
}
void h_14(double *state, double *unused, double *out_1406625725743516395) {
   out_1406625725743516395[0] = state[6];
   out_1406625725743516395[1] = state[7];
   out_1406625725743516395[2] = state[8];
}
void H_14(double *state, double *unused, double *out_7436720783216973005) {
   out_7436720783216973005[0] = 0;
   out_7436720783216973005[1] = 0;
   out_7436720783216973005[2] = 0;
   out_7436720783216973005[3] = 0;
   out_7436720783216973005[4] = 0;
   out_7436720783216973005[5] = 0;
   out_7436720783216973005[6] = 1;
   out_7436720783216973005[7] = 0;
   out_7436720783216973005[8] = 0;
   out_7436720783216973005[9] = 0;
   out_7436720783216973005[10] = 0;
   out_7436720783216973005[11] = 0;
   out_7436720783216973005[12] = 0;
   out_7436720783216973005[13] = 0;
   out_7436720783216973005[14] = 0;
   out_7436720783216973005[15] = 0;
   out_7436720783216973005[16] = 0;
   out_7436720783216973005[17] = 0;
   out_7436720783216973005[18] = 0;
   out_7436720783216973005[19] = 0;
   out_7436720783216973005[20] = 0;
   out_7436720783216973005[21] = 0;
   out_7436720783216973005[22] = 0;
   out_7436720783216973005[23] = 0;
   out_7436720783216973005[24] = 0;
   out_7436720783216973005[25] = 1;
   out_7436720783216973005[26] = 0;
   out_7436720783216973005[27] = 0;
   out_7436720783216973005[28] = 0;
   out_7436720783216973005[29] = 0;
   out_7436720783216973005[30] = 0;
   out_7436720783216973005[31] = 0;
   out_7436720783216973005[32] = 0;
   out_7436720783216973005[33] = 0;
   out_7436720783216973005[34] = 0;
   out_7436720783216973005[35] = 0;
   out_7436720783216973005[36] = 0;
   out_7436720783216973005[37] = 0;
   out_7436720783216973005[38] = 0;
   out_7436720783216973005[39] = 0;
   out_7436720783216973005[40] = 0;
   out_7436720783216973005[41] = 0;
   out_7436720783216973005[42] = 0;
   out_7436720783216973005[43] = 0;
   out_7436720783216973005[44] = 1;
   out_7436720783216973005[45] = 0;
   out_7436720783216973005[46] = 0;
   out_7436720783216973005[47] = 0;
   out_7436720783216973005[48] = 0;
   out_7436720783216973005[49] = 0;
   out_7436720783216973005[50] = 0;
   out_7436720783216973005[51] = 0;
   out_7436720783216973005[52] = 0;
   out_7436720783216973005[53] = 0;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_4, H_4, NULL, in_z, in_R, in_ea, MAHA_THRESH_4);
}
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_10, H_10, NULL, in_z, in_R, in_ea, MAHA_THRESH_10);
}
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_13, H_13, NULL, in_z, in_R, in_ea, MAHA_THRESH_13);
}
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_14, H_14, NULL, in_z, in_R, in_ea, MAHA_THRESH_14);
}
void pose_err_fun(double *nom_x, double *delta_x, double *out_7412749682930917159) {
  err_fun(nom_x, delta_x, out_7412749682930917159);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_3324739374567006732) {
  inv_err_fun(nom_x, true_x, out_3324739374567006732);
}
void pose_H_mod_fun(double *state, double *out_6871722307859650590) {
  H_mod_fun(state, out_6871722307859650590);
}
void pose_f_fun(double *state, double dt, double *out_8847044247784726907) {
  f_fun(state,  dt, out_8847044247784726907);
}
void pose_F_fun(double *state, double dt, double *out_6117558309002218944) {
  F_fun(state,  dt, out_6117558309002218944);
}
void pose_h_4(double *state, double *unused, double *out_2048679593753205460) {
  h_4(state, unused, out_2048679593753205460);
}
void pose_H_4(double *state, double *unused, double *out_3473479926877488476) {
  H_4(state, unused, out_3473479926877488476);
}
void pose_h_10(double *state, double *unused, double *out_6141237783103343630) {
  h_10(state, unused, out_6141237783103343630);
}
void pose_H_10(double *state, double *unused, double *out_8609451639616367789) {
  H_10(state, unused, out_8609451639616367789);
}
void pose_h_13(double *state, double *unused, double *out_2125878123702031972) {
  h_13(state, unused, out_2125878123702031972);
}
void pose_H_13(double *state, double *unused, double *out_6685753752209821277) {
  H_13(state, unused, out_6685753752209821277);
}
void pose_h_14(double *state, double *unused, double *out_1406625725743516395) {
  h_14(state, unused, out_1406625725743516395);
}
void pose_H_14(double *state, double *unused, double *out_7436720783216973005) {
  H_14(state, unused, out_7436720783216973005);
}
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF pose = {
  .name = "pose",
  .kinds = { 4, 10, 13, 14 },
  .feature_kinds = {  },
  .f_fun = pose_f_fun,
  .F_fun = pose_F_fun,
  .err_fun = pose_err_fun,
  .inv_err_fun = pose_inv_err_fun,
  .H_mod_fun = pose_H_mod_fun,
  .predict = pose_predict,
  .hs = {
    { 4, pose_h_4 },
    { 10, pose_h_10 },
    { 13, pose_h_13 },
    { 14, pose_h_14 },
  },
  .Hs = {
    { 4, pose_H_4 },
    { 10, pose_H_10 },
    { 13, pose_H_13 },
    { 14, pose_H_14 },
  },
  .updates = {
    { 4, pose_update_4 },
    { 10, pose_update_10 },
    { 13, pose_update_13 },
    { 14, pose_update_14 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_lib_init(pose)
