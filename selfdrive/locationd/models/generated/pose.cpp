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
void err_fun(double *nom_x, double *delta_x, double *out_3880423688108830906) {
   out_3880423688108830906[0] = delta_x[0] + nom_x[0];
   out_3880423688108830906[1] = delta_x[1] + nom_x[1];
   out_3880423688108830906[2] = delta_x[2] + nom_x[2];
   out_3880423688108830906[3] = delta_x[3] + nom_x[3];
   out_3880423688108830906[4] = delta_x[4] + nom_x[4];
   out_3880423688108830906[5] = delta_x[5] + nom_x[5];
   out_3880423688108830906[6] = delta_x[6] + nom_x[6];
   out_3880423688108830906[7] = delta_x[7] + nom_x[7];
   out_3880423688108830906[8] = delta_x[8] + nom_x[8];
   out_3880423688108830906[9] = delta_x[9] + nom_x[9];
   out_3880423688108830906[10] = delta_x[10] + nom_x[10];
   out_3880423688108830906[11] = delta_x[11] + nom_x[11];
   out_3880423688108830906[12] = delta_x[12] + nom_x[12];
   out_3880423688108830906[13] = delta_x[13] + nom_x[13];
   out_3880423688108830906[14] = delta_x[14] + nom_x[14];
   out_3880423688108830906[15] = delta_x[15] + nom_x[15];
   out_3880423688108830906[16] = delta_x[16] + nom_x[16];
   out_3880423688108830906[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_530507270830550051) {
   out_530507270830550051[0] = -nom_x[0] + true_x[0];
   out_530507270830550051[1] = -nom_x[1] + true_x[1];
   out_530507270830550051[2] = -nom_x[2] + true_x[2];
   out_530507270830550051[3] = -nom_x[3] + true_x[3];
   out_530507270830550051[4] = -nom_x[4] + true_x[4];
   out_530507270830550051[5] = -nom_x[5] + true_x[5];
   out_530507270830550051[6] = -nom_x[6] + true_x[6];
   out_530507270830550051[7] = -nom_x[7] + true_x[7];
   out_530507270830550051[8] = -nom_x[8] + true_x[8];
   out_530507270830550051[9] = -nom_x[9] + true_x[9];
   out_530507270830550051[10] = -nom_x[10] + true_x[10];
   out_530507270830550051[11] = -nom_x[11] + true_x[11];
   out_530507270830550051[12] = -nom_x[12] + true_x[12];
   out_530507270830550051[13] = -nom_x[13] + true_x[13];
   out_530507270830550051[14] = -nom_x[14] + true_x[14];
   out_530507270830550051[15] = -nom_x[15] + true_x[15];
   out_530507270830550051[16] = -nom_x[16] + true_x[16];
   out_530507270830550051[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_8675870831607907019) {
   out_8675870831607907019[0] = 1.0;
   out_8675870831607907019[1] = 0.0;
   out_8675870831607907019[2] = 0.0;
   out_8675870831607907019[3] = 0.0;
   out_8675870831607907019[4] = 0.0;
   out_8675870831607907019[5] = 0.0;
   out_8675870831607907019[6] = 0.0;
   out_8675870831607907019[7] = 0.0;
   out_8675870831607907019[8] = 0.0;
   out_8675870831607907019[9] = 0.0;
   out_8675870831607907019[10] = 0.0;
   out_8675870831607907019[11] = 0.0;
   out_8675870831607907019[12] = 0.0;
   out_8675870831607907019[13] = 0.0;
   out_8675870831607907019[14] = 0.0;
   out_8675870831607907019[15] = 0.0;
   out_8675870831607907019[16] = 0.0;
   out_8675870831607907019[17] = 0.0;
   out_8675870831607907019[18] = 0.0;
   out_8675870831607907019[19] = 1.0;
   out_8675870831607907019[20] = 0.0;
   out_8675870831607907019[21] = 0.0;
   out_8675870831607907019[22] = 0.0;
   out_8675870831607907019[23] = 0.0;
   out_8675870831607907019[24] = 0.0;
   out_8675870831607907019[25] = 0.0;
   out_8675870831607907019[26] = 0.0;
   out_8675870831607907019[27] = 0.0;
   out_8675870831607907019[28] = 0.0;
   out_8675870831607907019[29] = 0.0;
   out_8675870831607907019[30] = 0.0;
   out_8675870831607907019[31] = 0.0;
   out_8675870831607907019[32] = 0.0;
   out_8675870831607907019[33] = 0.0;
   out_8675870831607907019[34] = 0.0;
   out_8675870831607907019[35] = 0.0;
   out_8675870831607907019[36] = 0.0;
   out_8675870831607907019[37] = 0.0;
   out_8675870831607907019[38] = 1.0;
   out_8675870831607907019[39] = 0.0;
   out_8675870831607907019[40] = 0.0;
   out_8675870831607907019[41] = 0.0;
   out_8675870831607907019[42] = 0.0;
   out_8675870831607907019[43] = 0.0;
   out_8675870831607907019[44] = 0.0;
   out_8675870831607907019[45] = 0.0;
   out_8675870831607907019[46] = 0.0;
   out_8675870831607907019[47] = 0.0;
   out_8675870831607907019[48] = 0.0;
   out_8675870831607907019[49] = 0.0;
   out_8675870831607907019[50] = 0.0;
   out_8675870831607907019[51] = 0.0;
   out_8675870831607907019[52] = 0.0;
   out_8675870831607907019[53] = 0.0;
   out_8675870831607907019[54] = 0.0;
   out_8675870831607907019[55] = 0.0;
   out_8675870831607907019[56] = 0.0;
   out_8675870831607907019[57] = 1.0;
   out_8675870831607907019[58] = 0.0;
   out_8675870831607907019[59] = 0.0;
   out_8675870831607907019[60] = 0.0;
   out_8675870831607907019[61] = 0.0;
   out_8675870831607907019[62] = 0.0;
   out_8675870831607907019[63] = 0.0;
   out_8675870831607907019[64] = 0.0;
   out_8675870831607907019[65] = 0.0;
   out_8675870831607907019[66] = 0.0;
   out_8675870831607907019[67] = 0.0;
   out_8675870831607907019[68] = 0.0;
   out_8675870831607907019[69] = 0.0;
   out_8675870831607907019[70] = 0.0;
   out_8675870831607907019[71] = 0.0;
   out_8675870831607907019[72] = 0.0;
   out_8675870831607907019[73] = 0.0;
   out_8675870831607907019[74] = 0.0;
   out_8675870831607907019[75] = 0.0;
   out_8675870831607907019[76] = 1.0;
   out_8675870831607907019[77] = 0.0;
   out_8675870831607907019[78] = 0.0;
   out_8675870831607907019[79] = 0.0;
   out_8675870831607907019[80] = 0.0;
   out_8675870831607907019[81] = 0.0;
   out_8675870831607907019[82] = 0.0;
   out_8675870831607907019[83] = 0.0;
   out_8675870831607907019[84] = 0.0;
   out_8675870831607907019[85] = 0.0;
   out_8675870831607907019[86] = 0.0;
   out_8675870831607907019[87] = 0.0;
   out_8675870831607907019[88] = 0.0;
   out_8675870831607907019[89] = 0.0;
   out_8675870831607907019[90] = 0.0;
   out_8675870831607907019[91] = 0.0;
   out_8675870831607907019[92] = 0.0;
   out_8675870831607907019[93] = 0.0;
   out_8675870831607907019[94] = 0.0;
   out_8675870831607907019[95] = 1.0;
   out_8675870831607907019[96] = 0.0;
   out_8675870831607907019[97] = 0.0;
   out_8675870831607907019[98] = 0.0;
   out_8675870831607907019[99] = 0.0;
   out_8675870831607907019[100] = 0.0;
   out_8675870831607907019[101] = 0.0;
   out_8675870831607907019[102] = 0.0;
   out_8675870831607907019[103] = 0.0;
   out_8675870831607907019[104] = 0.0;
   out_8675870831607907019[105] = 0.0;
   out_8675870831607907019[106] = 0.0;
   out_8675870831607907019[107] = 0.0;
   out_8675870831607907019[108] = 0.0;
   out_8675870831607907019[109] = 0.0;
   out_8675870831607907019[110] = 0.0;
   out_8675870831607907019[111] = 0.0;
   out_8675870831607907019[112] = 0.0;
   out_8675870831607907019[113] = 0.0;
   out_8675870831607907019[114] = 1.0;
   out_8675870831607907019[115] = 0.0;
   out_8675870831607907019[116] = 0.0;
   out_8675870831607907019[117] = 0.0;
   out_8675870831607907019[118] = 0.0;
   out_8675870831607907019[119] = 0.0;
   out_8675870831607907019[120] = 0.0;
   out_8675870831607907019[121] = 0.0;
   out_8675870831607907019[122] = 0.0;
   out_8675870831607907019[123] = 0.0;
   out_8675870831607907019[124] = 0.0;
   out_8675870831607907019[125] = 0.0;
   out_8675870831607907019[126] = 0.0;
   out_8675870831607907019[127] = 0.0;
   out_8675870831607907019[128] = 0.0;
   out_8675870831607907019[129] = 0.0;
   out_8675870831607907019[130] = 0.0;
   out_8675870831607907019[131] = 0.0;
   out_8675870831607907019[132] = 0.0;
   out_8675870831607907019[133] = 1.0;
   out_8675870831607907019[134] = 0.0;
   out_8675870831607907019[135] = 0.0;
   out_8675870831607907019[136] = 0.0;
   out_8675870831607907019[137] = 0.0;
   out_8675870831607907019[138] = 0.0;
   out_8675870831607907019[139] = 0.0;
   out_8675870831607907019[140] = 0.0;
   out_8675870831607907019[141] = 0.0;
   out_8675870831607907019[142] = 0.0;
   out_8675870831607907019[143] = 0.0;
   out_8675870831607907019[144] = 0.0;
   out_8675870831607907019[145] = 0.0;
   out_8675870831607907019[146] = 0.0;
   out_8675870831607907019[147] = 0.0;
   out_8675870831607907019[148] = 0.0;
   out_8675870831607907019[149] = 0.0;
   out_8675870831607907019[150] = 0.0;
   out_8675870831607907019[151] = 0.0;
   out_8675870831607907019[152] = 1.0;
   out_8675870831607907019[153] = 0.0;
   out_8675870831607907019[154] = 0.0;
   out_8675870831607907019[155] = 0.0;
   out_8675870831607907019[156] = 0.0;
   out_8675870831607907019[157] = 0.0;
   out_8675870831607907019[158] = 0.0;
   out_8675870831607907019[159] = 0.0;
   out_8675870831607907019[160] = 0.0;
   out_8675870831607907019[161] = 0.0;
   out_8675870831607907019[162] = 0.0;
   out_8675870831607907019[163] = 0.0;
   out_8675870831607907019[164] = 0.0;
   out_8675870831607907019[165] = 0.0;
   out_8675870831607907019[166] = 0.0;
   out_8675870831607907019[167] = 0.0;
   out_8675870831607907019[168] = 0.0;
   out_8675870831607907019[169] = 0.0;
   out_8675870831607907019[170] = 0.0;
   out_8675870831607907019[171] = 1.0;
   out_8675870831607907019[172] = 0.0;
   out_8675870831607907019[173] = 0.0;
   out_8675870831607907019[174] = 0.0;
   out_8675870831607907019[175] = 0.0;
   out_8675870831607907019[176] = 0.0;
   out_8675870831607907019[177] = 0.0;
   out_8675870831607907019[178] = 0.0;
   out_8675870831607907019[179] = 0.0;
   out_8675870831607907019[180] = 0.0;
   out_8675870831607907019[181] = 0.0;
   out_8675870831607907019[182] = 0.0;
   out_8675870831607907019[183] = 0.0;
   out_8675870831607907019[184] = 0.0;
   out_8675870831607907019[185] = 0.0;
   out_8675870831607907019[186] = 0.0;
   out_8675870831607907019[187] = 0.0;
   out_8675870831607907019[188] = 0.0;
   out_8675870831607907019[189] = 0.0;
   out_8675870831607907019[190] = 1.0;
   out_8675870831607907019[191] = 0.0;
   out_8675870831607907019[192] = 0.0;
   out_8675870831607907019[193] = 0.0;
   out_8675870831607907019[194] = 0.0;
   out_8675870831607907019[195] = 0.0;
   out_8675870831607907019[196] = 0.0;
   out_8675870831607907019[197] = 0.0;
   out_8675870831607907019[198] = 0.0;
   out_8675870831607907019[199] = 0.0;
   out_8675870831607907019[200] = 0.0;
   out_8675870831607907019[201] = 0.0;
   out_8675870831607907019[202] = 0.0;
   out_8675870831607907019[203] = 0.0;
   out_8675870831607907019[204] = 0.0;
   out_8675870831607907019[205] = 0.0;
   out_8675870831607907019[206] = 0.0;
   out_8675870831607907019[207] = 0.0;
   out_8675870831607907019[208] = 0.0;
   out_8675870831607907019[209] = 1.0;
   out_8675870831607907019[210] = 0.0;
   out_8675870831607907019[211] = 0.0;
   out_8675870831607907019[212] = 0.0;
   out_8675870831607907019[213] = 0.0;
   out_8675870831607907019[214] = 0.0;
   out_8675870831607907019[215] = 0.0;
   out_8675870831607907019[216] = 0.0;
   out_8675870831607907019[217] = 0.0;
   out_8675870831607907019[218] = 0.0;
   out_8675870831607907019[219] = 0.0;
   out_8675870831607907019[220] = 0.0;
   out_8675870831607907019[221] = 0.0;
   out_8675870831607907019[222] = 0.0;
   out_8675870831607907019[223] = 0.0;
   out_8675870831607907019[224] = 0.0;
   out_8675870831607907019[225] = 0.0;
   out_8675870831607907019[226] = 0.0;
   out_8675870831607907019[227] = 0.0;
   out_8675870831607907019[228] = 1.0;
   out_8675870831607907019[229] = 0.0;
   out_8675870831607907019[230] = 0.0;
   out_8675870831607907019[231] = 0.0;
   out_8675870831607907019[232] = 0.0;
   out_8675870831607907019[233] = 0.0;
   out_8675870831607907019[234] = 0.0;
   out_8675870831607907019[235] = 0.0;
   out_8675870831607907019[236] = 0.0;
   out_8675870831607907019[237] = 0.0;
   out_8675870831607907019[238] = 0.0;
   out_8675870831607907019[239] = 0.0;
   out_8675870831607907019[240] = 0.0;
   out_8675870831607907019[241] = 0.0;
   out_8675870831607907019[242] = 0.0;
   out_8675870831607907019[243] = 0.0;
   out_8675870831607907019[244] = 0.0;
   out_8675870831607907019[245] = 0.0;
   out_8675870831607907019[246] = 0.0;
   out_8675870831607907019[247] = 1.0;
   out_8675870831607907019[248] = 0.0;
   out_8675870831607907019[249] = 0.0;
   out_8675870831607907019[250] = 0.0;
   out_8675870831607907019[251] = 0.0;
   out_8675870831607907019[252] = 0.0;
   out_8675870831607907019[253] = 0.0;
   out_8675870831607907019[254] = 0.0;
   out_8675870831607907019[255] = 0.0;
   out_8675870831607907019[256] = 0.0;
   out_8675870831607907019[257] = 0.0;
   out_8675870831607907019[258] = 0.0;
   out_8675870831607907019[259] = 0.0;
   out_8675870831607907019[260] = 0.0;
   out_8675870831607907019[261] = 0.0;
   out_8675870831607907019[262] = 0.0;
   out_8675870831607907019[263] = 0.0;
   out_8675870831607907019[264] = 0.0;
   out_8675870831607907019[265] = 0.0;
   out_8675870831607907019[266] = 1.0;
   out_8675870831607907019[267] = 0.0;
   out_8675870831607907019[268] = 0.0;
   out_8675870831607907019[269] = 0.0;
   out_8675870831607907019[270] = 0.0;
   out_8675870831607907019[271] = 0.0;
   out_8675870831607907019[272] = 0.0;
   out_8675870831607907019[273] = 0.0;
   out_8675870831607907019[274] = 0.0;
   out_8675870831607907019[275] = 0.0;
   out_8675870831607907019[276] = 0.0;
   out_8675870831607907019[277] = 0.0;
   out_8675870831607907019[278] = 0.0;
   out_8675870831607907019[279] = 0.0;
   out_8675870831607907019[280] = 0.0;
   out_8675870831607907019[281] = 0.0;
   out_8675870831607907019[282] = 0.0;
   out_8675870831607907019[283] = 0.0;
   out_8675870831607907019[284] = 0.0;
   out_8675870831607907019[285] = 1.0;
   out_8675870831607907019[286] = 0.0;
   out_8675870831607907019[287] = 0.0;
   out_8675870831607907019[288] = 0.0;
   out_8675870831607907019[289] = 0.0;
   out_8675870831607907019[290] = 0.0;
   out_8675870831607907019[291] = 0.0;
   out_8675870831607907019[292] = 0.0;
   out_8675870831607907019[293] = 0.0;
   out_8675870831607907019[294] = 0.0;
   out_8675870831607907019[295] = 0.0;
   out_8675870831607907019[296] = 0.0;
   out_8675870831607907019[297] = 0.0;
   out_8675870831607907019[298] = 0.0;
   out_8675870831607907019[299] = 0.0;
   out_8675870831607907019[300] = 0.0;
   out_8675870831607907019[301] = 0.0;
   out_8675870831607907019[302] = 0.0;
   out_8675870831607907019[303] = 0.0;
   out_8675870831607907019[304] = 1.0;
   out_8675870831607907019[305] = 0.0;
   out_8675870831607907019[306] = 0.0;
   out_8675870831607907019[307] = 0.0;
   out_8675870831607907019[308] = 0.0;
   out_8675870831607907019[309] = 0.0;
   out_8675870831607907019[310] = 0.0;
   out_8675870831607907019[311] = 0.0;
   out_8675870831607907019[312] = 0.0;
   out_8675870831607907019[313] = 0.0;
   out_8675870831607907019[314] = 0.0;
   out_8675870831607907019[315] = 0.0;
   out_8675870831607907019[316] = 0.0;
   out_8675870831607907019[317] = 0.0;
   out_8675870831607907019[318] = 0.0;
   out_8675870831607907019[319] = 0.0;
   out_8675870831607907019[320] = 0.0;
   out_8675870831607907019[321] = 0.0;
   out_8675870831607907019[322] = 0.0;
   out_8675870831607907019[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_5800429947315892303) {
   out_5800429947315892303[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_5800429947315892303[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_5800429947315892303[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_5800429947315892303[3] = dt*state[12] + state[3];
   out_5800429947315892303[4] = dt*state[13] + state[4];
   out_5800429947315892303[5] = dt*state[14] + state[5];
   out_5800429947315892303[6] = state[6];
   out_5800429947315892303[7] = state[7];
   out_5800429947315892303[8] = state[8];
   out_5800429947315892303[9] = state[9];
   out_5800429947315892303[10] = state[10];
   out_5800429947315892303[11] = state[11];
   out_5800429947315892303[12] = state[12];
   out_5800429947315892303[13] = state[13];
   out_5800429947315892303[14] = state[14];
   out_5800429947315892303[15] = state[15];
   out_5800429947315892303[16] = state[16];
   out_5800429947315892303[17] = state[17];
}
void F_fun(double *state, double dt, double *out_4994234759718378919) {
   out_4994234759718378919[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4994234759718378919[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4994234759718378919[2] = 0;
   out_4994234759718378919[3] = 0;
   out_4994234759718378919[4] = 0;
   out_4994234759718378919[5] = 0;
   out_4994234759718378919[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4994234759718378919[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4994234759718378919[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4994234759718378919[9] = 0;
   out_4994234759718378919[10] = 0;
   out_4994234759718378919[11] = 0;
   out_4994234759718378919[12] = 0;
   out_4994234759718378919[13] = 0;
   out_4994234759718378919[14] = 0;
   out_4994234759718378919[15] = 0;
   out_4994234759718378919[16] = 0;
   out_4994234759718378919[17] = 0;
   out_4994234759718378919[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_4994234759718378919[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_4994234759718378919[20] = 0;
   out_4994234759718378919[21] = 0;
   out_4994234759718378919[22] = 0;
   out_4994234759718378919[23] = 0;
   out_4994234759718378919[24] = 0;
   out_4994234759718378919[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_4994234759718378919[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_4994234759718378919[27] = 0;
   out_4994234759718378919[28] = 0;
   out_4994234759718378919[29] = 0;
   out_4994234759718378919[30] = 0;
   out_4994234759718378919[31] = 0;
   out_4994234759718378919[32] = 0;
   out_4994234759718378919[33] = 0;
   out_4994234759718378919[34] = 0;
   out_4994234759718378919[35] = 0;
   out_4994234759718378919[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4994234759718378919[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4994234759718378919[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4994234759718378919[39] = 0;
   out_4994234759718378919[40] = 0;
   out_4994234759718378919[41] = 0;
   out_4994234759718378919[42] = 0;
   out_4994234759718378919[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4994234759718378919[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4994234759718378919[45] = 0;
   out_4994234759718378919[46] = 0;
   out_4994234759718378919[47] = 0;
   out_4994234759718378919[48] = 0;
   out_4994234759718378919[49] = 0;
   out_4994234759718378919[50] = 0;
   out_4994234759718378919[51] = 0;
   out_4994234759718378919[52] = 0;
   out_4994234759718378919[53] = 0;
   out_4994234759718378919[54] = 0;
   out_4994234759718378919[55] = 0;
   out_4994234759718378919[56] = 0;
   out_4994234759718378919[57] = 1;
   out_4994234759718378919[58] = 0;
   out_4994234759718378919[59] = 0;
   out_4994234759718378919[60] = 0;
   out_4994234759718378919[61] = 0;
   out_4994234759718378919[62] = 0;
   out_4994234759718378919[63] = 0;
   out_4994234759718378919[64] = 0;
   out_4994234759718378919[65] = 0;
   out_4994234759718378919[66] = dt;
   out_4994234759718378919[67] = 0;
   out_4994234759718378919[68] = 0;
   out_4994234759718378919[69] = 0;
   out_4994234759718378919[70] = 0;
   out_4994234759718378919[71] = 0;
   out_4994234759718378919[72] = 0;
   out_4994234759718378919[73] = 0;
   out_4994234759718378919[74] = 0;
   out_4994234759718378919[75] = 0;
   out_4994234759718378919[76] = 1;
   out_4994234759718378919[77] = 0;
   out_4994234759718378919[78] = 0;
   out_4994234759718378919[79] = 0;
   out_4994234759718378919[80] = 0;
   out_4994234759718378919[81] = 0;
   out_4994234759718378919[82] = 0;
   out_4994234759718378919[83] = 0;
   out_4994234759718378919[84] = 0;
   out_4994234759718378919[85] = dt;
   out_4994234759718378919[86] = 0;
   out_4994234759718378919[87] = 0;
   out_4994234759718378919[88] = 0;
   out_4994234759718378919[89] = 0;
   out_4994234759718378919[90] = 0;
   out_4994234759718378919[91] = 0;
   out_4994234759718378919[92] = 0;
   out_4994234759718378919[93] = 0;
   out_4994234759718378919[94] = 0;
   out_4994234759718378919[95] = 1;
   out_4994234759718378919[96] = 0;
   out_4994234759718378919[97] = 0;
   out_4994234759718378919[98] = 0;
   out_4994234759718378919[99] = 0;
   out_4994234759718378919[100] = 0;
   out_4994234759718378919[101] = 0;
   out_4994234759718378919[102] = 0;
   out_4994234759718378919[103] = 0;
   out_4994234759718378919[104] = dt;
   out_4994234759718378919[105] = 0;
   out_4994234759718378919[106] = 0;
   out_4994234759718378919[107] = 0;
   out_4994234759718378919[108] = 0;
   out_4994234759718378919[109] = 0;
   out_4994234759718378919[110] = 0;
   out_4994234759718378919[111] = 0;
   out_4994234759718378919[112] = 0;
   out_4994234759718378919[113] = 0;
   out_4994234759718378919[114] = 1;
   out_4994234759718378919[115] = 0;
   out_4994234759718378919[116] = 0;
   out_4994234759718378919[117] = 0;
   out_4994234759718378919[118] = 0;
   out_4994234759718378919[119] = 0;
   out_4994234759718378919[120] = 0;
   out_4994234759718378919[121] = 0;
   out_4994234759718378919[122] = 0;
   out_4994234759718378919[123] = 0;
   out_4994234759718378919[124] = 0;
   out_4994234759718378919[125] = 0;
   out_4994234759718378919[126] = 0;
   out_4994234759718378919[127] = 0;
   out_4994234759718378919[128] = 0;
   out_4994234759718378919[129] = 0;
   out_4994234759718378919[130] = 0;
   out_4994234759718378919[131] = 0;
   out_4994234759718378919[132] = 0;
   out_4994234759718378919[133] = 1;
   out_4994234759718378919[134] = 0;
   out_4994234759718378919[135] = 0;
   out_4994234759718378919[136] = 0;
   out_4994234759718378919[137] = 0;
   out_4994234759718378919[138] = 0;
   out_4994234759718378919[139] = 0;
   out_4994234759718378919[140] = 0;
   out_4994234759718378919[141] = 0;
   out_4994234759718378919[142] = 0;
   out_4994234759718378919[143] = 0;
   out_4994234759718378919[144] = 0;
   out_4994234759718378919[145] = 0;
   out_4994234759718378919[146] = 0;
   out_4994234759718378919[147] = 0;
   out_4994234759718378919[148] = 0;
   out_4994234759718378919[149] = 0;
   out_4994234759718378919[150] = 0;
   out_4994234759718378919[151] = 0;
   out_4994234759718378919[152] = 1;
   out_4994234759718378919[153] = 0;
   out_4994234759718378919[154] = 0;
   out_4994234759718378919[155] = 0;
   out_4994234759718378919[156] = 0;
   out_4994234759718378919[157] = 0;
   out_4994234759718378919[158] = 0;
   out_4994234759718378919[159] = 0;
   out_4994234759718378919[160] = 0;
   out_4994234759718378919[161] = 0;
   out_4994234759718378919[162] = 0;
   out_4994234759718378919[163] = 0;
   out_4994234759718378919[164] = 0;
   out_4994234759718378919[165] = 0;
   out_4994234759718378919[166] = 0;
   out_4994234759718378919[167] = 0;
   out_4994234759718378919[168] = 0;
   out_4994234759718378919[169] = 0;
   out_4994234759718378919[170] = 0;
   out_4994234759718378919[171] = 1;
   out_4994234759718378919[172] = 0;
   out_4994234759718378919[173] = 0;
   out_4994234759718378919[174] = 0;
   out_4994234759718378919[175] = 0;
   out_4994234759718378919[176] = 0;
   out_4994234759718378919[177] = 0;
   out_4994234759718378919[178] = 0;
   out_4994234759718378919[179] = 0;
   out_4994234759718378919[180] = 0;
   out_4994234759718378919[181] = 0;
   out_4994234759718378919[182] = 0;
   out_4994234759718378919[183] = 0;
   out_4994234759718378919[184] = 0;
   out_4994234759718378919[185] = 0;
   out_4994234759718378919[186] = 0;
   out_4994234759718378919[187] = 0;
   out_4994234759718378919[188] = 0;
   out_4994234759718378919[189] = 0;
   out_4994234759718378919[190] = 1;
   out_4994234759718378919[191] = 0;
   out_4994234759718378919[192] = 0;
   out_4994234759718378919[193] = 0;
   out_4994234759718378919[194] = 0;
   out_4994234759718378919[195] = 0;
   out_4994234759718378919[196] = 0;
   out_4994234759718378919[197] = 0;
   out_4994234759718378919[198] = 0;
   out_4994234759718378919[199] = 0;
   out_4994234759718378919[200] = 0;
   out_4994234759718378919[201] = 0;
   out_4994234759718378919[202] = 0;
   out_4994234759718378919[203] = 0;
   out_4994234759718378919[204] = 0;
   out_4994234759718378919[205] = 0;
   out_4994234759718378919[206] = 0;
   out_4994234759718378919[207] = 0;
   out_4994234759718378919[208] = 0;
   out_4994234759718378919[209] = 1;
   out_4994234759718378919[210] = 0;
   out_4994234759718378919[211] = 0;
   out_4994234759718378919[212] = 0;
   out_4994234759718378919[213] = 0;
   out_4994234759718378919[214] = 0;
   out_4994234759718378919[215] = 0;
   out_4994234759718378919[216] = 0;
   out_4994234759718378919[217] = 0;
   out_4994234759718378919[218] = 0;
   out_4994234759718378919[219] = 0;
   out_4994234759718378919[220] = 0;
   out_4994234759718378919[221] = 0;
   out_4994234759718378919[222] = 0;
   out_4994234759718378919[223] = 0;
   out_4994234759718378919[224] = 0;
   out_4994234759718378919[225] = 0;
   out_4994234759718378919[226] = 0;
   out_4994234759718378919[227] = 0;
   out_4994234759718378919[228] = 1;
   out_4994234759718378919[229] = 0;
   out_4994234759718378919[230] = 0;
   out_4994234759718378919[231] = 0;
   out_4994234759718378919[232] = 0;
   out_4994234759718378919[233] = 0;
   out_4994234759718378919[234] = 0;
   out_4994234759718378919[235] = 0;
   out_4994234759718378919[236] = 0;
   out_4994234759718378919[237] = 0;
   out_4994234759718378919[238] = 0;
   out_4994234759718378919[239] = 0;
   out_4994234759718378919[240] = 0;
   out_4994234759718378919[241] = 0;
   out_4994234759718378919[242] = 0;
   out_4994234759718378919[243] = 0;
   out_4994234759718378919[244] = 0;
   out_4994234759718378919[245] = 0;
   out_4994234759718378919[246] = 0;
   out_4994234759718378919[247] = 1;
   out_4994234759718378919[248] = 0;
   out_4994234759718378919[249] = 0;
   out_4994234759718378919[250] = 0;
   out_4994234759718378919[251] = 0;
   out_4994234759718378919[252] = 0;
   out_4994234759718378919[253] = 0;
   out_4994234759718378919[254] = 0;
   out_4994234759718378919[255] = 0;
   out_4994234759718378919[256] = 0;
   out_4994234759718378919[257] = 0;
   out_4994234759718378919[258] = 0;
   out_4994234759718378919[259] = 0;
   out_4994234759718378919[260] = 0;
   out_4994234759718378919[261] = 0;
   out_4994234759718378919[262] = 0;
   out_4994234759718378919[263] = 0;
   out_4994234759718378919[264] = 0;
   out_4994234759718378919[265] = 0;
   out_4994234759718378919[266] = 1;
   out_4994234759718378919[267] = 0;
   out_4994234759718378919[268] = 0;
   out_4994234759718378919[269] = 0;
   out_4994234759718378919[270] = 0;
   out_4994234759718378919[271] = 0;
   out_4994234759718378919[272] = 0;
   out_4994234759718378919[273] = 0;
   out_4994234759718378919[274] = 0;
   out_4994234759718378919[275] = 0;
   out_4994234759718378919[276] = 0;
   out_4994234759718378919[277] = 0;
   out_4994234759718378919[278] = 0;
   out_4994234759718378919[279] = 0;
   out_4994234759718378919[280] = 0;
   out_4994234759718378919[281] = 0;
   out_4994234759718378919[282] = 0;
   out_4994234759718378919[283] = 0;
   out_4994234759718378919[284] = 0;
   out_4994234759718378919[285] = 1;
   out_4994234759718378919[286] = 0;
   out_4994234759718378919[287] = 0;
   out_4994234759718378919[288] = 0;
   out_4994234759718378919[289] = 0;
   out_4994234759718378919[290] = 0;
   out_4994234759718378919[291] = 0;
   out_4994234759718378919[292] = 0;
   out_4994234759718378919[293] = 0;
   out_4994234759718378919[294] = 0;
   out_4994234759718378919[295] = 0;
   out_4994234759718378919[296] = 0;
   out_4994234759718378919[297] = 0;
   out_4994234759718378919[298] = 0;
   out_4994234759718378919[299] = 0;
   out_4994234759718378919[300] = 0;
   out_4994234759718378919[301] = 0;
   out_4994234759718378919[302] = 0;
   out_4994234759718378919[303] = 0;
   out_4994234759718378919[304] = 1;
   out_4994234759718378919[305] = 0;
   out_4994234759718378919[306] = 0;
   out_4994234759718378919[307] = 0;
   out_4994234759718378919[308] = 0;
   out_4994234759718378919[309] = 0;
   out_4994234759718378919[310] = 0;
   out_4994234759718378919[311] = 0;
   out_4994234759718378919[312] = 0;
   out_4994234759718378919[313] = 0;
   out_4994234759718378919[314] = 0;
   out_4994234759718378919[315] = 0;
   out_4994234759718378919[316] = 0;
   out_4994234759718378919[317] = 0;
   out_4994234759718378919[318] = 0;
   out_4994234759718378919[319] = 0;
   out_4994234759718378919[320] = 0;
   out_4994234759718378919[321] = 0;
   out_4994234759718378919[322] = 0;
   out_4994234759718378919[323] = 1;
}
void h_4(double *state, double *unused, double *out_4484292768974138631) {
   out_4484292768974138631[0] = state[6] + state[9];
   out_4484292768974138631[1] = state[7] + state[10];
   out_4484292768974138631[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_3571845806940544873) {
   out_3571845806940544873[0] = 0;
   out_3571845806940544873[1] = 0;
   out_3571845806940544873[2] = 0;
   out_3571845806940544873[3] = 0;
   out_3571845806940544873[4] = 0;
   out_3571845806940544873[5] = 0;
   out_3571845806940544873[6] = 1;
   out_3571845806940544873[7] = 0;
   out_3571845806940544873[8] = 0;
   out_3571845806940544873[9] = 1;
   out_3571845806940544873[10] = 0;
   out_3571845806940544873[11] = 0;
   out_3571845806940544873[12] = 0;
   out_3571845806940544873[13] = 0;
   out_3571845806940544873[14] = 0;
   out_3571845806940544873[15] = 0;
   out_3571845806940544873[16] = 0;
   out_3571845806940544873[17] = 0;
   out_3571845806940544873[18] = 0;
   out_3571845806940544873[19] = 0;
   out_3571845806940544873[20] = 0;
   out_3571845806940544873[21] = 0;
   out_3571845806940544873[22] = 0;
   out_3571845806940544873[23] = 0;
   out_3571845806940544873[24] = 0;
   out_3571845806940544873[25] = 1;
   out_3571845806940544873[26] = 0;
   out_3571845806940544873[27] = 0;
   out_3571845806940544873[28] = 1;
   out_3571845806940544873[29] = 0;
   out_3571845806940544873[30] = 0;
   out_3571845806940544873[31] = 0;
   out_3571845806940544873[32] = 0;
   out_3571845806940544873[33] = 0;
   out_3571845806940544873[34] = 0;
   out_3571845806940544873[35] = 0;
   out_3571845806940544873[36] = 0;
   out_3571845806940544873[37] = 0;
   out_3571845806940544873[38] = 0;
   out_3571845806940544873[39] = 0;
   out_3571845806940544873[40] = 0;
   out_3571845806940544873[41] = 0;
   out_3571845806940544873[42] = 0;
   out_3571845806940544873[43] = 0;
   out_3571845806940544873[44] = 1;
   out_3571845806940544873[45] = 0;
   out_3571845806940544873[46] = 0;
   out_3571845806940544873[47] = 1;
   out_3571845806940544873[48] = 0;
   out_3571845806940544873[49] = 0;
   out_3571845806940544873[50] = 0;
   out_3571845806940544873[51] = 0;
   out_3571845806940544873[52] = 0;
   out_3571845806940544873[53] = 0;
}
void h_10(double *state, double *unused, double *out_6327118854115285339) {
   out_6327118854115285339[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_6327118854115285339[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_6327118854115285339[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_5014155151492872447) {
   out_5014155151492872447[0] = 0;
   out_5014155151492872447[1] = 9.8100000000000005*cos(state[1]);
   out_5014155151492872447[2] = 0;
   out_5014155151492872447[3] = 0;
   out_5014155151492872447[4] = -state[8];
   out_5014155151492872447[5] = state[7];
   out_5014155151492872447[6] = 0;
   out_5014155151492872447[7] = state[5];
   out_5014155151492872447[8] = -state[4];
   out_5014155151492872447[9] = 0;
   out_5014155151492872447[10] = 0;
   out_5014155151492872447[11] = 0;
   out_5014155151492872447[12] = 1;
   out_5014155151492872447[13] = 0;
   out_5014155151492872447[14] = 0;
   out_5014155151492872447[15] = 1;
   out_5014155151492872447[16] = 0;
   out_5014155151492872447[17] = 0;
   out_5014155151492872447[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_5014155151492872447[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_5014155151492872447[20] = 0;
   out_5014155151492872447[21] = state[8];
   out_5014155151492872447[22] = 0;
   out_5014155151492872447[23] = -state[6];
   out_5014155151492872447[24] = -state[5];
   out_5014155151492872447[25] = 0;
   out_5014155151492872447[26] = state[3];
   out_5014155151492872447[27] = 0;
   out_5014155151492872447[28] = 0;
   out_5014155151492872447[29] = 0;
   out_5014155151492872447[30] = 0;
   out_5014155151492872447[31] = 1;
   out_5014155151492872447[32] = 0;
   out_5014155151492872447[33] = 0;
   out_5014155151492872447[34] = 1;
   out_5014155151492872447[35] = 0;
   out_5014155151492872447[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_5014155151492872447[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_5014155151492872447[38] = 0;
   out_5014155151492872447[39] = -state[7];
   out_5014155151492872447[40] = state[6];
   out_5014155151492872447[41] = 0;
   out_5014155151492872447[42] = state[4];
   out_5014155151492872447[43] = -state[3];
   out_5014155151492872447[44] = 0;
   out_5014155151492872447[45] = 0;
   out_5014155151492872447[46] = 0;
   out_5014155151492872447[47] = 0;
   out_5014155151492872447[48] = 0;
   out_5014155151492872447[49] = 0;
   out_5014155151492872447[50] = 1;
   out_5014155151492872447[51] = 0;
   out_5014155151492872447[52] = 0;
   out_5014155151492872447[53] = 1;
}
void h_13(double *state, double *unused, double *out_7538235923800904941) {
   out_7538235923800904941[0] = state[3];
   out_7538235923800904941[1] = state[4];
   out_7538235923800904941[2] = state[5];
}
void H_13(double *state, double *unused, double *out_4038785401376156056) {
   out_4038785401376156056[0] = 0;
   out_4038785401376156056[1] = 0;
   out_4038785401376156056[2] = 0;
   out_4038785401376156056[3] = 1;
   out_4038785401376156056[4] = 0;
   out_4038785401376156056[5] = 0;
   out_4038785401376156056[6] = 0;
   out_4038785401376156056[7] = 0;
   out_4038785401376156056[8] = 0;
   out_4038785401376156056[9] = 0;
   out_4038785401376156056[10] = 0;
   out_4038785401376156056[11] = 0;
   out_4038785401376156056[12] = 0;
   out_4038785401376156056[13] = 0;
   out_4038785401376156056[14] = 0;
   out_4038785401376156056[15] = 0;
   out_4038785401376156056[16] = 0;
   out_4038785401376156056[17] = 0;
   out_4038785401376156056[18] = 0;
   out_4038785401376156056[19] = 0;
   out_4038785401376156056[20] = 0;
   out_4038785401376156056[21] = 0;
   out_4038785401376156056[22] = 1;
   out_4038785401376156056[23] = 0;
   out_4038785401376156056[24] = 0;
   out_4038785401376156056[25] = 0;
   out_4038785401376156056[26] = 0;
   out_4038785401376156056[27] = 0;
   out_4038785401376156056[28] = 0;
   out_4038785401376156056[29] = 0;
   out_4038785401376156056[30] = 0;
   out_4038785401376156056[31] = 0;
   out_4038785401376156056[32] = 0;
   out_4038785401376156056[33] = 0;
   out_4038785401376156056[34] = 0;
   out_4038785401376156056[35] = 0;
   out_4038785401376156056[36] = 0;
   out_4038785401376156056[37] = 0;
   out_4038785401376156056[38] = 0;
   out_4038785401376156056[39] = 0;
   out_4038785401376156056[40] = 0;
   out_4038785401376156056[41] = 1;
   out_4038785401376156056[42] = 0;
   out_4038785401376156056[43] = 0;
   out_4038785401376156056[44] = 0;
   out_4038785401376156056[45] = 0;
   out_4038785401376156056[46] = 0;
   out_4038785401376156056[47] = 0;
   out_4038785401376156056[48] = 0;
   out_4038785401376156056[49] = 0;
   out_4038785401376156056[50] = 0;
   out_4038785401376156056[51] = 0;
   out_4038785401376156056[52] = 0;
   out_4038785401376156056[53] = 0;
}
void h_14(double *state, double *unused, double *out_5695071338726805119) {
   out_5695071338726805119[0] = state[6];
   out_5695071338726805119[1] = state[7];
   out_5695071338726805119[2] = state[8];
}
void H_14(double *state, double *unused, double *out_391395049398939656) {
   out_391395049398939656[0] = 0;
   out_391395049398939656[1] = 0;
   out_391395049398939656[2] = 0;
   out_391395049398939656[3] = 0;
   out_391395049398939656[4] = 0;
   out_391395049398939656[5] = 0;
   out_391395049398939656[6] = 1;
   out_391395049398939656[7] = 0;
   out_391395049398939656[8] = 0;
   out_391395049398939656[9] = 0;
   out_391395049398939656[10] = 0;
   out_391395049398939656[11] = 0;
   out_391395049398939656[12] = 0;
   out_391395049398939656[13] = 0;
   out_391395049398939656[14] = 0;
   out_391395049398939656[15] = 0;
   out_391395049398939656[16] = 0;
   out_391395049398939656[17] = 0;
   out_391395049398939656[18] = 0;
   out_391395049398939656[19] = 0;
   out_391395049398939656[20] = 0;
   out_391395049398939656[21] = 0;
   out_391395049398939656[22] = 0;
   out_391395049398939656[23] = 0;
   out_391395049398939656[24] = 0;
   out_391395049398939656[25] = 1;
   out_391395049398939656[26] = 0;
   out_391395049398939656[27] = 0;
   out_391395049398939656[28] = 0;
   out_391395049398939656[29] = 0;
   out_391395049398939656[30] = 0;
   out_391395049398939656[31] = 0;
   out_391395049398939656[32] = 0;
   out_391395049398939656[33] = 0;
   out_391395049398939656[34] = 0;
   out_391395049398939656[35] = 0;
   out_391395049398939656[36] = 0;
   out_391395049398939656[37] = 0;
   out_391395049398939656[38] = 0;
   out_391395049398939656[39] = 0;
   out_391395049398939656[40] = 0;
   out_391395049398939656[41] = 0;
   out_391395049398939656[42] = 0;
   out_391395049398939656[43] = 0;
   out_391395049398939656[44] = 1;
   out_391395049398939656[45] = 0;
   out_391395049398939656[46] = 0;
   out_391395049398939656[47] = 0;
   out_391395049398939656[48] = 0;
   out_391395049398939656[49] = 0;
   out_391395049398939656[50] = 0;
   out_391395049398939656[51] = 0;
   out_391395049398939656[52] = 0;
   out_391395049398939656[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_3880423688108830906) {
  err_fun(nom_x, delta_x, out_3880423688108830906);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_530507270830550051) {
  inv_err_fun(nom_x, true_x, out_530507270830550051);
}
void pose_H_mod_fun(double *state, double *out_8675870831607907019) {
  H_mod_fun(state, out_8675870831607907019);
}
void pose_f_fun(double *state, double dt, double *out_5800429947315892303) {
  f_fun(state,  dt, out_5800429947315892303);
}
void pose_F_fun(double *state, double dt, double *out_4994234759718378919) {
  F_fun(state,  dt, out_4994234759718378919);
}
void pose_h_4(double *state, double *unused, double *out_4484292768974138631) {
  h_4(state, unused, out_4484292768974138631);
}
void pose_H_4(double *state, double *unused, double *out_3571845806940544873) {
  H_4(state, unused, out_3571845806940544873);
}
void pose_h_10(double *state, double *unused, double *out_6327118854115285339) {
  h_10(state, unused, out_6327118854115285339);
}
void pose_H_10(double *state, double *unused, double *out_5014155151492872447) {
  H_10(state, unused, out_5014155151492872447);
}
void pose_h_13(double *state, double *unused, double *out_7538235923800904941) {
  h_13(state, unused, out_7538235923800904941);
}
void pose_H_13(double *state, double *unused, double *out_4038785401376156056) {
  H_13(state, unused, out_4038785401376156056);
}
void pose_h_14(double *state, double *unused, double *out_5695071338726805119) {
  h_14(state, unused, out_5695071338726805119);
}
void pose_H_14(double *state, double *unused, double *out_391395049398939656) {
  H_14(state, unused, out_391395049398939656);
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
