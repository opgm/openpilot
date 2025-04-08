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
void err_fun(double *nom_x, double *delta_x, double *out_7993045186406776049) {
   out_7993045186406776049[0] = delta_x[0] + nom_x[0];
   out_7993045186406776049[1] = delta_x[1] + nom_x[1];
   out_7993045186406776049[2] = delta_x[2] + nom_x[2];
   out_7993045186406776049[3] = delta_x[3] + nom_x[3];
   out_7993045186406776049[4] = delta_x[4] + nom_x[4];
   out_7993045186406776049[5] = delta_x[5] + nom_x[5];
   out_7993045186406776049[6] = delta_x[6] + nom_x[6];
   out_7993045186406776049[7] = delta_x[7] + nom_x[7];
   out_7993045186406776049[8] = delta_x[8] + nom_x[8];
   out_7993045186406776049[9] = delta_x[9] + nom_x[9];
   out_7993045186406776049[10] = delta_x[10] + nom_x[10];
   out_7993045186406776049[11] = delta_x[11] + nom_x[11];
   out_7993045186406776049[12] = delta_x[12] + nom_x[12];
   out_7993045186406776049[13] = delta_x[13] + nom_x[13];
   out_7993045186406776049[14] = delta_x[14] + nom_x[14];
   out_7993045186406776049[15] = delta_x[15] + nom_x[15];
   out_7993045186406776049[16] = delta_x[16] + nom_x[16];
   out_7993045186406776049[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6214826562014697166) {
   out_6214826562014697166[0] = -nom_x[0] + true_x[0];
   out_6214826562014697166[1] = -nom_x[1] + true_x[1];
   out_6214826562014697166[2] = -nom_x[2] + true_x[2];
   out_6214826562014697166[3] = -nom_x[3] + true_x[3];
   out_6214826562014697166[4] = -nom_x[4] + true_x[4];
   out_6214826562014697166[5] = -nom_x[5] + true_x[5];
   out_6214826562014697166[6] = -nom_x[6] + true_x[6];
   out_6214826562014697166[7] = -nom_x[7] + true_x[7];
   out_6214826562014697166[8] = -nom_x[8] + true_x[8];
   out_6214826562014697166[9] = -nom_x[9] + true_x[9];
   out_6214826562014697166[10] = -nom_x[10] + true_x[10];
   out_6214826562014697166[11] = -nom_x[11] + true_x[11];
   out_6214826562014697166[12] = -nom_x[12] + true_x[12];
   out_6214826562014697166[13] = -nom_x[13] + true_x[13];
   out_6214826562014697166[14] = -nom_x[14] + true_x[14];
   out_6214826562014697166[15] = -nom_x[15] + true_x[15];
   out_6214826562014697166[16] = -nom_x[16] + true_x[16];
   out_6214826562014697166[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_2036493750579969796) {
   out_2036493750579969796[0] = 1.0;
   out_2036493750579969796[1] = 0.0;
   out_2036493750579969796[2] = 0.0;
   out_2036493750579969796[3] = 0.0;
   out_2036493750579969796[4] = 0.0;
   out_2036493750579969796[5] = 0.0;
   out_2036493750579969796[6] = 0.0;
   out_2036493750579969796[7] = 0.0;
   out_2036493750579969796[8] = 0.0;
   out_2036493750579969796[9] = 0.0;
   out_2036493750579969796[10] = 0.0;
   out_2036493750579969796[11] = 0.0;
   out_2036493750579969796[12] = 0.0;
   out_2036493750579969796[13] = 0.0;
   out_2036493750579969796[14] = 0.0;
   out_2036493750579969796[15] = 0.0;
   out_2036493750579969796[16] = 0.0;
   out_2036493750579969796[17] = 0.0;
   out_2036493750579969796[18] = 0.0;
   out_2036493750579969796[19] = 1.0;
   out_2036493750579969796[20] = 0.0;
   out_2036493750579969796[21] = 0.0;
   out_2036493750579969796[22] = 0.0;
   out_2036493750579969796[23] = 0.0;
   out_2036493750579969796[24] = 0.0;
   out_2036493750579969796[25] = 0.0;
   out_2036493750579969796[26] = 0.0;
   out_2036493750579969796[27] = 0.0;
   out_2036493750579969796[28] = 0.0;
   out_2036493750579969796[29] = 0.0;
   out_2036493750579969796[30] = 0.0;
   out_2036493750579969796[31] = 0.0;
   out_2036493750579969796[32] = 0.0;
   out_2036493750579969796[33] = 0.0;
   out_2036493750579969796[34] = 0.0;
   out_2036493750579969796[35] = 0.0;
   out_2036493750579969796[36] = 0.0;
   out_2036493750579969796[37] = 0.0;
   out_2036493750579969796[38] = 1.0;
   out_2036493750579969796[39] = 0.0;
   out_2036493750579969796[40] = 0.0;
   out_2036493750579969796[41] = 0.0;
   out_2036493750579969796[42] = 0.0;
   out_2036493750579969796[43] = 0.0;
   out_2036493750579969796[44] = 0.0;
   out_2036493750579969796[45] = 0.0;
   out_2036493750579969796[46] = 0.0;
   out_2036493750579969796[47] = 0.0;
   out_2036493750579969796[48] = 0.0;
   out_2036493750579969796[49] = 0.0;
   out_2036493750579969796[50] = 0.0;
   out_2036493750579969796[51] = 0.0;
   out_2036493750579969796[52] = 0.0;
   out_2036493750579969796[53] = 0.0;
   out_2036493750579969796[54] = 0.0;
   out_2036493750579969796[55] = 0.0;
   out_2036493750579969796[56] = 0.0;
   out_2036493750579969796[57] = 1.0;
   out_2036493750579969796[58] = 0.0;
   out_2036493750579969796[59] = 0.0;
   out_2036493750579969796[60] = 0.0;
   out_2036493750579969796[61] = 0.0;
   out_2036493750579969796[62] = 0.0;
   out_2036493750579969796[63] = 0.0;
   out_2036493750579969796[64] = 0.0;
   out_2036493750579969796[65] = 0.0;
   out_2036493750579969796[66] = 0.0;
   out_2036493750579969796[67] = 0.0;
   out_2036493750579969796[68] = 0.0;
   out_2036493750579969796[69] = 0.0;
   out_2036493750579969796[70] = 0.0;
   out_2036493750579969796[71] = 0.0;
   out_2036493750579969796[72] = 0.0;
   out_2036493750579969796[73] = 0.0;
   out_2036493750579969796[74] = 0.0;
   out_2036493750579969796[75] = 0.0;
   out_2036493750579969796[76] = 1.0;
   out_2036493750579969796[77] = 0.0;
   out_2036493750579969796[78] = 0.0;
   out_2036493750579969796[79] = 0.0;
   out_2036493750579969796[80] = 0.0;
   out_2036493750579969796[81] = 0.0;
   out_2036493750579969796[82] = 0.0;
   out_2036493750579969796[83] = 0.0;
   out_2036493750579969796[84] = 0.0;
   out_2036493750579969796[85] = 0.0;
   out_2036493750579969796[86] = 0.0;
   out_2036493750579969796[87] = 0.0;
   out_2036493750579969796[88] = 0.0;
   out_2036493750579969796[89] = 0.0;
   out_2036493750579969796[90] = 0.0;
   out_2036493750579969796[91] = 0.0;
   out_2036493750579969796[92] = 0.0;
   out_2036493750579969796[93] = 0.0;
   out_2036493750579969796[94] = 0.0;
   out_2036493750579969796[95] = 1.0;
   out_2036493750579969796[96] = 0.0;
   out_2036493750579969796[97] = 0.0;
   out_2036493750579969796[98] = 0.0;
   out_2036493750579969796[99] = 0.0;
   out_2036493750579969796[100] = 0.0;
   out_2036493750579969796[101] = 0.0;
   out_2036493750579969796[102] = 0.0;
   out_2036493750579969796[103] = 0.0;
   out_2036493750579969796[104] = 0.0;
   out_2036493750579969796[105] = 0.0;
   out_2036493750579969796[106] = 0.0;
   out_2036493750579969796[107] = 0.0;
   out_2036493750579969796[108] = 0.0;
   out_2036493750579969796[109] = 0.0;
   out_2036493750579969796[110] = 0.0;
   out_2036493750579969796[111] = 0.0;
   out_2036493750579969796[112] = 0.0;
   out_2036493750579969796[113] = 0.0;
   out_2036493750579969796[114] = 1.0;
   out_2036493750579969796[115] = 0.0;
   out_2036493750579969796[116] = 0.0;
   out_2036493750579969796[117] = 0.0;
   out_2036493750579969796[118] = 0.0;
   out_2036493750579969796[119] = 0.0;
   out_2036493750579969796[120] = 0.0;
   out_2036493750579969796[121] = 0.0;
   out_2036493750579969796[122] = 0.0;
   out_2036493750579969796[123] = 0.0;
   out_2036493750579969796[124] = 0.0;
   out_2036493750579969796[125] = 0.0;
   out_2036493750579969796[126] = 0.0;
   out_2036493750579969796[127] = 0.0;
   out_2036493750579969796[128] = 0.0;
   out_2036493750579969796[129] = 0.0;
   out_2036493750579969796[130] = 0.0;
   out_2036493750579969796[131] = 0.0;
   out_2036493750579969796[132] = 0.0;
   out_2036493750579969796[133] = 1.0;
   out_2036493750579969796[134] = 0.0;
   out_2036493750579969796[135] = 0.0;
   out_2036493750579969796[136] = 0.0;
   out_2036493750579969796[137] = 0.0;
   out_2036493750579969796[138] = 0.0;
   out_2036493750579969796[139] = 0.0;
   out_2036493750579969796[140] = 0.0;
   out_2036493750579969796[141] = 0.0;
   out_2036493750579969796[142] = 0.0;
   out_2036493750579969796[143] = 0.0;
   out_2036493750579969796[144] = 0.0;
   out_2036493750579969796[145] = 0.0;
   out_2036493750579969796[146] = 0.0;
   out_2036493750579969796[147] = 0.0;
   out_2036493750579969796[148] = 0.0;
   out_2036493750579969796[149] = 0.0;
   out_2036493750579969796[150] = 0.0;
   out_2036493750579969796[151] = 0.0;
   out_2036493750579969796[152] = 1.0;
   out_2036493750579969796[153] = 0.0;
   out_2036493750579969796[154] = 0.0;
   out_2036493750579969796[155] = 0.0;
   out_2036493750579969796[156] = 0.0;
   out_2036493750579969796[157] = 0.0;
   out_2036493750579969796[158] = 0.0;
   out_2036493750579969796[159] = 0.0;
   out_2036493750579969796[160] = 0.0;
   out_2036493750579969796[161] = 0.0;
   out_2036493750579969796[162] = 0.0;
   out_2036493750579969796[163] = 0.0;
   out_2036493750579969796[164] = 0.0;
   out_2036493750579969796[165] = 0.0;
   out_2036493750579969796[166] = 0.0;
   out_2036493750579969796[167] = 0.0;
   out_2036493750579969796[168] = 0.0;
   out_2036493750579969796[169] = 0.0;
   out_2036493750579969796[170] = 0.0;
   out_2036493750579969796[171] = 1.0;
   out_2036493750579969796[172] = 0.0;
   out_2036493750579969796[173] = 0.0;
   out_2036493750579969796[174] = 0.0;
   out_2036493750579969796[175] = 0.0;
   out_2036493750579969796[176] = 0.0;
   out_2036493750579969796[177] = 0.0;
   out_2036493750579969796[178] = 0.0;
   out_2036493750579969796[179] = 0.0;
   out_2036493750579969796[180] = 0.0;
   out_2036493750579969796[181] = 0.0;
   out_2036493750579969796[182] = 0.0;
   out_2036493750579969796[183] = 0.0;
   out_2036493750579969796[184] = 0.0;
   out_2036493750579969796[185] = 0.0;
   out_2036493750579969796[186] = 0.0;
   out_2036493750579969796[187] = 0.0;
   out_2036493750579969796[188] = 0.0;
   out_2036493750579969796[189] = 0.0;
   out_2036493750579969796[190] = 1.0;
   out_2036493750579969796[191] = 0.0;
   out_2036493750579969796[192] = 0.0;
   out_2036493750579969796[193] = 0.0;
   out_2036493750579969796[194] = 0.0;
   out_2036493750579969796[195] = 0.0;
   out_2036493750579969796[196] = 0.0;
   out_2036493750579969796[197] = 0.0;
   out_2036493750579969796[198] = 0.0;
   out_2036493750579969796[199] = 0.0;
   out_2036493750579969796[200] = 0.0;
   out_2036493750579969796[201] = 0.0;
   out_2036493750579969796[202] = 0.0;
   out_2036493750579969796[203] = 0.0;
   out_2036493750579969796[204] = 0.0;
   out_2036493750579969796[205] = 0.0;
   out_2036493750579969796[206] = 0.0;
   out_2036493750579969796[207] = 0.0;
   out_2036493750579969796[208] = 0.0;
   out_2036493750579969796[209] = 1.0;
   out_2036493750579969796[210] = 0.0;
   out_2036493750579969796[211] = 0.0;
   out_2036493750579969796[212] = 0.0;
   out_2036493750579969796[213] = 0.0;
   out_2036493750579969796[214] = 0.0;
   out_2036493750579969796[215] = 0.0;
   out_2036493750579969796[216] = 0.0;
   out_2036493750579969796[217] = 0.0;
   out_2036493750579969796[218] = 0.0;
   out_2036493750579969796[219] = 0.0;
   out_2036493750579969796[220] = 0.0;
   out_2036493750579969796[221] = 0.0;
   out_2036493750579969796[222] = 0.0;
   out_2036493750579969796[223] = 0.0;
   out_2036493750579969796[224] = 0.0;
   out_2036493750579969796[225] = 0.0;
   out_2036493750579969796[226] = 0.0;
   out_2036493750579969796[227] = 0.0;
   out_2036493750579969796[228] = 1.0;
   out_2036493750579969796[229] = 0.0;
   out_2036493750579969796[230] = 0.0;
   out_2036493750579969796[231] = 0.0;
   out_2036493750579969796[232] = 0.0;
   out_2036493750579969796[233] = 0.0;
   out_2036493750579969796[234] = 0.0;
   out_2036493750579969796[235] = 0.0;
   out_2036493750579969796[236] = 0.0;
   out_2036493750579969796[237] = 0.0;
   out_2036493750579969796[238] = 0.0;
   out_2036493750579969796[239] = 0.0;
   out_2036493750579969796[240] = 0.0;
   out_2036493750579969796[241] = 0.0;
   out_2036493750579969796[242] = 0.0;
   out_2036493750579969796[243] = 0.0;
   out_2036493750579969796[244] = 0.0;
   out_2036493750579969796[245] = 0.0;
   out_2036493750579969796[246] = 0.0;
   out_2036493750579969796[247] = 1.0;
   out_2036493750579969796[248] = 0.0;
   out_2036493750579969796[249] = 0.0;
   out_2036493750579969796[250] = 0.0;
   out_2036493750579969796[251] = 0.0;
   out_2036493750579969796[252] = 0.0;
   out_2036493750579969796[253] = 0.0;
   out_2036493750579969796[254] = 0.0;
   out_2036493750579969796[255] = 0.0;
   out_2036493750579969796[256] = 0.0;
   out_2036493750579969796[257] = 0.0;
   out_2036493750579969796[258] = 0.0;
   out_2036493750579969796[259] = 0.0;
   out_2036493750579969796[260] = 0.0;
   out_2036493750579969796[261] = 0.0;
   out_2036493750579969796[262] = 0.0;
   out_2036493750579969796[263] = 0.0;
   out_2036493750579969796[264] = 0.0;
   out_2036493750579969796[265] = 0.0;
   out_2036493750579969796[266] = 1.0;
   out_2036493750579969796[267] = 0.0;
   out_2036493750579969796[268] = 0.0;
   out_2036493750579969796[269] = 0.0;
   out_2036493750579969796[270] = 0.0;
   out_2036493750579969796[271] = 0.0;
   out_2036493750579969796[272] = 0.0;
   out_2036493750579969796[273] = 0.0;
   out_2036493750579969796[274] = 0.0;
   out_2036493750579969796[275] = 0.0;
   out_2036493750579969796[276] = 0.0;
   out_2036493750579969796[277] = 0.0;
   out_2036493750579969796[278] = 0.0;
   out_2036493750579969796[279] = 0.0;
   out_2036493750579969796[280] = 0.0;
   out_2036493750579969796[281] = 0.0;
   out_2036493750579969796[282] = 0.0;
   out_2036493750579969796[283] = 0.0;
   out_2036493750579969796[284] = 0.0;
   out_2036493750579969796[285] = 1.0;
   out_2036493750579969796[286] = 0.0;
   out_2036493750579969796[287] = 0.0;
   out_2036493750579969796[288] = 0.0;
   out_2036493750579969796[289] = 0.0;
   out_2036493750579969796[290] = 0.0;
   out_2036493750579969796[291] = 0.0;
   out_2036493750579969796[292] = 0.0;
   out_2036493750579969796[293] = 0.0;
   out_2036493750579969796[294] = 0.0;
   out_2036493750579969796[295] = 0.0;
   out_2036493750579969796[296] = 0.0;
   out_2036493750579969796[297] = 0.0;
   out_2036493750579969796[298] = 0.0;
   out_2036493750579969796[299] = 0.0;
   out_2036493750579969796[300] = 0.0;
   out_2036493750579969796[301] = 0.0;
   out_2036493750579969796[302] = 0.0;
   out_2036493750579969796[303] = 0.0;
   out_2036493750579969796[304] = 1.0;
   out_2036493750579969796[305] = 0.0;
   out_2036493750579969796[306] = 0.0;
   out_2036493750579969796[307] = 0.0;
   out_2036493750579969796[308] = 0.0;
   out_2036493750579969796[309] = 0.0;
   out_2036493750579969796[310] = 0.0;
   out_2036493750579969796[311] = 0.0;
   out_2036493750579969796[312] = 0.0;
   out_2036493750579969796[313] = 0.0;
   out_2036493750579969796[314] = 0.0;
   out_2036493750579969796[315] = 0.0;
   out_2036493750579969796[316] = 0.0;
   out_2036493750579969796[317] = 0.0;
   out_2036493750579969796[318] = 0.0;
   out_2036493750579969796[319] = 0.0;
   out_2036493750579969796[320] = 0.0;
   out_2036493750579969796[321] = 0.0;
   out_2036493750579969796[322] = 0.0;
   out_2036493750579969796[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_5815055826729932803) {
   out_5815055826729932803[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_5815055826729932803[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_5815055826729932803[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_5815055826729932803[3] = dt*state[12] + state[3];
   out_5815055826729932803[4] = dt*state[13] + state[4];
   out_5815055826729932803[5] = dt*state[14] + state[5];
   out_5815055826729932803[6] = state[6];
   out_5815055826729932803[7] = state[7];
   out_5815055826729932803[8] = state[8];
   out_5815055826729932803[9] = state[9];
   out_5815055826729932803[10] = state[10];
   out_5815055826729932803[11] = state[11];
   out_5815055826729932803[12] = state[12];
   out_5815055826729932803[13] = state[13];
   out_5815055826729932803[14] = state[14];
   out_5815055826729932803[15] = state[15];
   out_5815055826729932803[16] = state[16];
   out_5815055826729932803[17] = state[17];
}
void F_fun(double *state, double dt, double *out_8088927993986614311) {
   out_8088927993986614311[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_8088927993986614311[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_8088927993986614311[2] = 0;
   out_8088927993986614311[3] = 0;
   out_8088927993986614311[4] = 0;
   out_8088927993986614311[5] = 0;
   out_8088927993986614311[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_8088927993986614311[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_8088927993986614311[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_8088927993986614311[9] = 0;
   out_8088927993986614311[10] = 0;
   out_8088927993986614311[11] = 0;
   out_8088927993986614311[12] = 0;
   out_8088927993986614311[13] = 0;
   out_8088927993986614311[14] = 0;
   out_8088927993986614311[15] = 0;
   out_8088927993986614311[16] = 0;
   out_8088927993986614311[17] = 0;
   out_8088927993986614311[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_8088927993986614311[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_8088927993986614311[20] = 0;
   out_8088927993986614311[21] = 0;
   out_8088927993986614311[22] = 0;
   out_8088927993986614311[23] = 0;
   out_8088927993986614311[24] = 0;
   out_8088927993986614311[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_8088927993986614311[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_8088927993986614311[27] = 0;
   out_8088927993986614311[28] = 0;
   out_8088927993986614311[29] = 0;
   out_8088927993986614311[30] = 0;
   out_8088927993986614311[31] = 0;
   out_8088927993986614311[32] = 0;
   out_8088927993986614311[33] = 0;
   out_8088927993986614311[34] = 0;
   out_8088927993986614311[35] = 0;
   out_8088927993986614311[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_8088927993986614311[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_8088927993986614311[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_8088927993986614311[39] = 0;
   out_8088927993986614311[40] = 0;
   out_8088927993986614311[41] = 0;
   out_8088927993986614311[42] = 0;
   out_8088927993986614311[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_8088927993986614311[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_8088927993986614311[45] = 0;
   out_8088927993986614311[46] = 0;
   out_8088927993986614311[47] = 0;
   out_8088927993986614311[48] = 0;
   out_8088927993986614311[49] = 0;
   out_8088927993986614311[50] = 0;
   out_8088927993986614311[51] = 0;
   out_8088927993986614311[52] = 0;
   out_8088927993986614311[53] = 0;
   out_8088927993986614311[54] = 0;
   out_8088927993986614311[55] = 0;
   out_8088927993986614311[56] = 0;
   out_8088927993986614311[57] = 1;
   out_8088927993986614311[58] = 0;
   out_8088927993986614311[59] = 0;
   out_8088927993986614311[60] = 0;
   out_8088927993986614311[61] = 0;
   out_8088927993986614311[62] = 0;
   out_8088927993986614311[63] = 0;
   out_8088927993986614311[64] = 0;
   out_8088927993986614311[65] = 0;
   out_8088927993986614311[66] = dt;
   out_8088927993986614311[67] = 0;
   out_8088927993986614311[68] = 0;
   out_8088927993986614311[69] = 0;
   out_8088927993986614311[70] = 0;
   out_8088927993986614311[71] = 0;
   out_8088927993986614311[72] = 0;
   out_8088927993986614311[73] = 0;
   out_8088927993986614311[74] = 0;
   out_8088927993986614311[75] = 0;
   out_8088927993986614311[76] = 1;
   out_8088927993986614311[77] = 0;
   out_8088927993986614311[78] = 0;
   out_8088927993986614311[79] = 0;
   out_8088927993986614311[80] = 0;
   out_8088927993986614311[81] = 0;
   out_8088927993986614311[82] = 0;
   out_8088927993986614311[83] = 0;
   out_8088927993986614311[84] = 0;
   out_8088927993986614311[85] = dt;
   out_8088927993986614311[86] = 0;
   out_8088927993986614311[87] = 0;
   out_8088927993986614311[88] = 0;
   out_8088927993986614311[89] = 0;
   out_8088927993986614311[90] = 0;
   out_8088927993986614311[91] = 0;
   out_8088927993986614311[92] = 0;
   out_8088927993986614311[93] = 0;
   out_8088927993986614311[94] = 0;
   out_8088927993986614311[95] = 1;
   out_8088927993986614311[96] = 0;
   out_8088927993986614311[97] = 0;
   out_8088927993986614311[98] = 0;
   out_8088927993986614311[99] = 0;
   out_8088927993986614311[100] = 0;
   out_8088927993986614311[101] = 0;
   out_8088927993986614311[102] = 0;
   out_8088927993986614311[103] = 0;
   out_8088927993986614311[104] = dt;
   out_8088927993986614311[105] = 0;
   out_8088927993986614311[106] = 0;
   out_8088927993986614311[107] = 0;
   out_8088927993986614311[108] = 0;
   out_8088927993986614311[109] = 0;
   out_8088927993986614311[110] = 0;
   out_8088927993986614311[111] = 0;
   out_8088927993986614311[112] = 0;
   out_8088927993986614311[113] = 0;
   out_8088927993986614311[114] = 1;
   out_8088927993986614311[115] = 0;
   out_8088927993986614311[116] = 0;
   out_8088927993986614311[117] = 0;
   out_8088927993986614311[118] = 0;
   out_8088927993986614311[119] = 0;
   out_8088927993986614311[120] = 0;
   out_8088927993986614311[121] = 0;
   out_8088927993986614311[122] = 0;
   out_8088927993986614311[123] = 0;
   out_8088927993986614311[124] = 0;
   out_8088927993986614311[125] = 0;
   out_8088927993986614311[126] = 0;
   out_8088927993986614311[127] = 0;
   out_8088927993986614311[128] = 0;
   out_8088927993986614311[129] = 0;
   out_8088927993986614311[130] = 0;
   out_8088927993986614311[131] = 0;
   out_8088927993986614311[132] = 0;
   out_8088927993986614311[133] = 1;
   out_8088927993986614311[134] = 0;
   out_8088927993986614311[135] = 0;
   out_8088927993986614311[136] = 0;
   out_8088927993986614311[137] = 0;
   out_8088927993986614311[138] = 0;
   out_8088927993986614311[139] = 0;
   out_8088927993986614311[140] = 0;
   out_8088927993986614311[141] = 0;
   out_8088927993986614311[142] = 0;
   out_8088927993986614311[143] = 0;
   out_8088927993986614311[144] = 0;
   out_8088927993986614311[145] = 0;
   out_8088927993986614311[146] = 0;
   out_8088927993986614311[147] = 0;
   out_8088927993986614311[148] = 0;
   out_8088927993986614311[149] = 0;
   out_8088927993986614311[150] = 0;
   out_8088927993986614311[151] = 0;
   out_8088927993986614311[152] = 1;
   out_8088927993986614311[153] = 0;
   out_8088927993986614311[154] = 0;
   out_8088927993986614311[155] = 0;
   out_8088927993986614311[156] = 0;
   out_8088927993986614311[157] = 0;
   out_8088927993986614311[158] = 0;
   out_8088927993986614311[159] = 0;
   out_8088927993986614311[160] = 0;
   out_8088927993986614311[161] = 0;
   out_8088927993986614311[162] = 0;
   out_8088927993986614311[163] = 0;
   out_8088927993986614311[164] = 0;
   out_8088927993986614311[165] = 0;
   out_8088927993986614311[166] = 0;
   out_8088927993986614311[167] = 0;
   out_8088927993986614311[168] = 0;
   out_8088927993986614311[169] = 0;
   out_8088927993986614311[170] = 0;
   out_8088927993986614311[171] = 1;
   out_8088927993986614311[172] = 0;
   out_8088927993986614311[173] = 0;
   out_8088927993986614311[174] = 0;
   out_8088927993986614311[175] = 0;
   out_8088927993986614311[176] = 0;
   out_8088927993986614311[177] = 0;
   out_8088927993986614311[178] = 0;
   out_8088927993986614311[179] = 0;
   out_8088927993986614311[180] = 0;
   out_8088927993986614311[181] = 0;
   out_8088927993986614311[182] = 0;
   out_8088927993986614311[183] = 0;
   out_8088927993986614311[184] = 0;
   out_8088927993986614311[185] = 0;
   out_8088927993986614311[186] = 0;
   out_8088927993986614311[187] = 0;
   out_8088927993986614311[188] = 0;
   out_8088927993986614311[189] = 0;
   out_8088927993986614311[190] = 1;
   out_8088927993986614311[191] = 0;
   out_8088927993986614311[192] = 0;
   out_8088927993986614311[193] = 0;
   out_8088927993986614311[194] = 0;
   out_8088927993986614311[195] = 0;
   out_8088927993986614311[196] = 0;
   out_8088927993986614311[197] = 0;
   out_8088927993986614311[198] = 0;
   out_8088927993986614311[199] = 0;
   out_8088927993986614311[200] = 0;
   out_8088927993986614311[201] = 0;
   out_8088927993986614311[202] = 0;
   out_8088927993986614311[203] = 0;
   out_8088927993986614311[204] = 0;
   out_8088927993986614311[205] = 0;
   out_8088927993986614311[206] = 0;
   out_8088927993986614311[207] = 0;
   out_8088927993986614311[208] = 0;
   out_8088927993986614311[209] = 1;
   out_8088927993986614311[210] = 0;
   out_8088927993986614311[211] = 0;
   out_8088927993986614311[212] = 0;
   out_8088927993986614311[213] = 0;
   out_8088927993986614311[214] = 0;
   out_8088927993986614311[215] = 0;
   out_8088927993986614311[216] = 0;
   out_8088927993986614311[217] = 0;
   out_8088927993986614311[218] = 0;
   out_8088927993986614311[219] = 0;
   out_8088927993986614311[220] = 0;
   out_8088927993986614311[221] = 0;
   out_8088927993986614311[222] = 0;
   out_8088927993986614311[223] = 0;
   out_8088927993986614311[224] = 0;
   out_8088927993986614311[225] = 0;
   out_8088927993986614311[226] = 0;
   out_8088927993986614311[227] = 0;
   out_8088927993986614311[228] = 1;
   out_8088927993986614311[229] = 0;
   out_8088927993986614311[230] = 0;
   out_8088927993986614311[231] = 0;
   out_8088927993986614311[232] = 0;
   out_8088927993986614311[233] = 0;
   out_8088927993986614311[234] = 0;
   out_8088927993986614311[235] = 0;
   out_8088927993986614311[236] = 0;
   out_8088927993986614311[237] = 0;
   out_8088927993986614311[238] = 0;
   out_8088927993986614311[239] = 0;
   out_8088927993986614311[240] = 0;
   out_8088927993986614311[241] = 0;
   out_8088927993986614311[242] = 0;
   out_8088927993986614311[243] = 0;
   out_8088927993986614311[244] = 0;
   out_8088927993986614311[245] = 0;
   out_8088927993986614311[246] = 0;
   out_8088927993986614311[247] = 1;
   out_8088927993986614311[248] = 0;
   out_8088927993986614311[249] = 0;
   out_8088927993986614311[250] = 0;
   out_8088927993986614311[251] = 0;
   out_8088927993986614311[252] = 0;
   out_8088927993986614311[253] = 0;
   out_8088927993986614311[254] = 0;
   out_8088927993986614311[255] = 0;
   out_8088927993986614311[256] = 0;
   out_8088927993986614311[257] = 0;
   out_8088927993986614311[258] = 0;
   out_8088927993986614311[259] = 0;
   out_8088927993986614311[260] = 0;
   out_8088927993986614311[261] = 0;
   out_8088927993986614311[262] = 0;
   out_8088927993986614311[263] = 0;
   out_8088927993986614311[264] = 0;
   out_8088927993986614311[265] = 0;
   out_8088927993986614311[266] = 1;
   out_8088927993986614311[267] = 0;
   out_8088927993986614311[268] = 0;
   out_8088927993986614311[269] = 0;
   out_8088927993986614311[270] = 0;
   out_8088927993986614311[271] = 0;
   out_8088927993986614311[272] = 0;
   out_8088927993986614311[273] = 0;
   out_8088927993986614311[274] = 0;
   out_8088927993986614311[275] = 0;
   out_8088927993986614311[276] = 0;
   out_8088927993986614311[277] = 0;
   out_8088927993986614311[278] = 0;
   out_8088927993986614311[279] = 0;
   out_8088927993986614311[280] = 0;
   out_8088927993986614311[281] = 0;
   out_8088927993986614311[282] = 0;
   out_8088927993986614311[283] = 0;
   out_8088927993986614311[284] = 0;
   out_8088927993986614311[285] = 1;
   out_8088927993986614311[286] = 0;
   out_8088927993986614311[287] = 0;
   out_8088927993986614311[288] = 0;
   out_8088927993986614311[289] = 0;
   out_8088927993986614311[290] = 0;
   out_8088927993986614311[291] = 0;
   out_8088927993986614311[292] = 0;
   out_8088927993986614311[293] = 0;
   out_8088927993986614311[294] = 0;
   out_8088927993986614311[295] = 0;
   out_8088927993986614311[296] = 0;
   out_8088927993986614311[297] = 0;
   out_8088927993986614311[298] = 0;
   out_8088927993986614311[299] = 0;
   out_8088927993986614311[300] = 0;
   out_8088927993986614311[301] = 0;
   out_8088927993986614311[302] = 0;
   out_8088927993986614311[303] = 0;
   out_8088927993986614311[304] = 1;
   out_8088927993986614311[305] = 0;
   out_8088927993986614311[306] = 0;
   out_8088927993986614311[307] = 0;
   out_8088927993986614311[308] = 0;
   out_8088927993986614311[309] = 0;
   out_8088927993986614311[310] = 0;
   out_8088927993986614311[311] = 0;
   out_8088927993986614311[312] = 0;
   out_8088927993986614311[313] = 0;
   out_8088927993986614311[314] = 0;
   out_8088927993986614311[315] = 0;
   out_8088927993986614311[316] = 0;
   out_8088927993986614311[317] = 0;
   out_8088927993986614311[318] = 0;
   out_8088927993986614311[319] = 0;
   out_8088927993986614311[320] = 0;
   out_8088927993986614311[321] = 0;
   out_8088927993986614311[322] = 0;
   out_8088927993986614311[323] = 1;
}
void h_4(double *state, double *unused, double *out_4596209739200211234) {
   out_4596209739200211234[0] = state[6] + state[9];
   out_4596209739200211234[1] = state[7] + state[10];
   out_4596209739200211234[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_6907867915477851546) {
   out_6907867915477851546[0] = 0;
   out_6907867915477851546[1] = 0;
   out_6907867915477851546[2] = 0;
   out_6907867915477851546[3] = 0;
   out_6907867915477851546[4] = 0;
   out_6907867915477851546[5] = 0;
   out_6907867915477851546[6] = 1;
   out_6907867915477851546[7] = 0;
   out_6907867915477851546[8] = 0;
   out_6907867915477851546[9] = 1;
   out_6907867915477851546[10] = 0;
   out_6907867915477851546[11] = 0;
   out_6907867915477851546[12] = 0;
   out_6907867915477851546[13] = 0;
   out_6907867915477851546[14] = 0;
   out_6907867915477851546[15] = 0;
   out_6907867915477851546[16] = 0;
   out_6907867915477851546[17] = 0;
   out_6907867915477851546[18] = 0;
   out_6907867915477851546[19] = 0;
   out_6907867915477851546[20] = 0;
   out_6907867915477851546[21] = 0;
   out_6907867915477851546[22] = 0;
   out_6907867915477851546[23] = 0;
   out_6907867915477851546[24] = 0;
   out_6907867915477851546[25] = 1;
   out_6907867915477851546[26] = 0;
   out_6907867915477851546[27] = 0;
   out_6907867915477851546[28] = 1;
   out_6907867915477851546[29] = 0;
   out_6907867915477851546[30] = 0;
   out_6907867915477851546[31] = 0;
   out_6907867915477851546[32] = 0;
   out_6907867915477851546[33] = 0;
   out_6907867915477851546[34] = 0;
   out_6907867915477851546[35] = 0;
   out_6907867915477851546[36] = 0;
   out_6907867915477851546[37] = 0;
   out_6907867915477851546[38] = 0;
   out_6907867915477851546[39] = 0;
   out_6907867915477851546[40] = 0;
   out_6907867915477851546[41] = 0;
   out_6907867915477851546[42] = 0;
   out_6907867915477851546[43] = 0;
   out_6907867915477851546[44] = 1;
   out_6907867915477851546[45] = 0;
   out_6907867915477851546[46] = 0;
   out_6907867915477851546[47] = 1;
   out_6907867915477851546[48] = 0;
   out_6907867915477851546[49] = 0;
   out_6907867915477851546[50] = 0;
   out_6907867915477851546[51] = 0;
   out_6907867915477851546[52] = 0;
   out_6907867915477851546[53] = 0;
}
void h_10(double *state, double *unused, double *out_7511182918238442662) {
   out_7511182918238442662[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_7511182918238442662[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_7511182918238442662[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_5218983418130581187) {
   out_5218983418130581187[0] = 0;
   out_5218983418130581187[1] = 9.8100000000000005*cos(state[1]);
   out_5218983418130581187[2] = 0;
   out_5218983418130581187[3] = 0;
   out_5218983418130581187[4] = -state[8];
   out_5218983418130581187[5] = state[7];
   out_5218983418130581187[6] = 0;
   out_5218983418130581187[7] = state[5];
   out_5218983418130581187[8] = -state[4];
   out_5218983418130581187[9] = 0;
   out_5218983418130581187[10] = 0;
   out_5218983418130581187[11] = 0;
   out_5218983418130581187[12] = 1;
   out_5218983418130581187[13] = 0;
   out_5218983418130581187[14] = 0;
   out_5218983418130581187[15] = 1;
   out_5218983418130581187[16] = 0;
   out_5218983418130581187[17] = 0;
   out_5218983418130581187[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_5218983418130581187[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_5218983418130581187[20] = 0;
   out_5218983418130581187[21] = state[8];
   out_5218983418130581187[22] = 0;
   out_5218983418130581187[23] = -state[6];
   out_5218983418130581187[24] = -state[5];
   out_5218983418130581187[25] = 0;
   out_5218983418130581187[26] = state[3];
   out_5218983418130581187[27] = 0;
   out_5218983418130581187[28] = 0;
   out_5218983418130581187[29] = 0;
   out_5218983418130581187[30] = 0;
   out_5218983418130581187[31] = 1;
   out_5218983418130581187[32] = 0;
   out_5218983418130581187[33] = 0;
   out_5218983418130581187[34] = 1;
   out_5218983418130581187[35] = 0;
   out_5218983418130581187[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_5218983418130581187[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_5218983418130581187[38] = 0;
   out_5218983418130581187[39] = -state[7];
   out_5218983418130581187[40] = state[6];
   out_5218983418130581187[41] = 0;
   out_5218983418130581187[42] = state[4];
   out_5218983418130581187[43] = -state[3];
   out_5218983418130581187[44] = 0;
   out_5218983418130581187[45] = 0;
   out_5218983418130581187[46] = 0;
   out_5218983418130581187[47] = 0;
   out_5218983418130581187[48] = 0;
   out_5218983418130581187[49] = 0;
   out_5218983418130581187[50] = 1;
   out_5218983418130581187[51] = 0;
   out_5218983418130581187[52] = 0;
   out_5218983418130581187[53] = 1;
}
void h_13(double *state, double *unused, double *out_3565323313024088772) {
   out_3565323313024088772[0] = state[3];
   out_3565323313024088772[1] = state[4];
   out_3565323313024088772[2] = state[5];
}
void H_13(double *state, double *unused, double *out_3695594090145518745) {
   out_3695594090145518745[0] = 0;
   out_3695594090145518745[1] = 0;
   out_3695594090145518745[2] = 0;
   out_3695594090145518745[3] = 1;
   out_3695594090145518745[4] = 0;
   out_3695594090145518745[5] = 0;
   out_3695594090145518745[6] = 0;
   out_3695594090145518745[7] = 0;
   out_3695594090145518745[8] = 0;
   out_3695594090145518745[9] = 0;
   out_3695594090145518745[10] = 0;
   out_3695594090145518745[11] = 0;
   out_3695594090145518745[12] = 0;
   out_3695594090145518745[13] = 0;
   out_3695594090145518745[14] = 0;
   out_3695594090145518745[15] = 0;
   out_3695594090145518745[16] = 0;
   out_3695594090145518745[17] = 0;
   out_3695594090145518745[18] = 0;
   out_3695594090145518745[19] = 0;
   out_3695594090145518745[20] = 0;
   out_3695594090145518745[21] = 0;
   out_3695594090145518745[22] = 1;
   out_3695594090145518745[23] = 0;
   out_3695594090145518745[24] = 0;
   out_3695594090145518745[25] = 0;
   out_3695594090145518745[26] = 0;
   out_3695594090145518745[27] = 0;
   out_3695594090145518745[28] = 0;
   out_3695594090145518745[29] = 0;
   out_3695594090145518745[30] = 0;
   out_3695594090145518745[31] = 0;
   out_3695594090145518745[32] = 0;
   out_3695594090145518745[33] = 0;
   out_3695594090145518745[34] = 0;
   out_3695594090145518745[35] = 0;
   out_3695594090145518745[36] = 0;
   out_3695594090145518745[37] = 0;
   out_3695594090145518745[38] = 0;
   out_3695594090145518745[39] = 0;
   out_3695594090145518745[40] = 0;
   out_3695594090145518745[41] = 1;
   out_3695594090145518745[42] = 0;
   out_3695594090145518745[43] = 0;
   out_3695594090145518745[44] = 0;
   out_3695594090145518745[45] = 0;
   out_3695594090145518745[46] = 0;
   out_3695594090145518745[47] = 0;
   out_3695594090145518745[48] = 0;
   out_3695594090145518745[49] = 0;
   out_3695594090145518745[50] = 0;
   out_3695594090145518745[51] = 0;
   out_3695594090145518745[52] = 0;
   out_3695594090145518745[53] = 0;
}
void h_14(double *state, double *unused, double *out_6195396300878839194) {
   out_6195396300878839194[0] = state[6];
   out_6195396300878839194[1] = state[7];
   out_6195396300878839194[2] = state[8];
}
void H_14(double *state, double *unused, double *out_2944627059138367017) {
   out_2944627059138367017[0] = 0;
   out_2944627059138367017[1] = 0;
   out_2944627059138367017[2] = 0;
   out_2944627059138367017[3] = 0;
   out_2944627059138367017[4] = 0;
   out_2944627059138367017[5] = 0;
   out_2944627059138367017[6] = 1;
   out_2944627059138367017[7] = 0;
   out_2944627059138367017[8] = 0;
   out_2944627059138367017[9] = 0;
   out_2944627059138367017[10] = 0;
   out_2944627059138367017[11] = 0;
   out_2944627059138367017[12] = 0;
   out_2944627059138367017[13] = 0;
   out_2944627059138367017[14] = 0;
   out_2944627059138367017[15] = 0;
   out_2944627059138367017[16] = 0;
   out_2944627059138367017[17] = 0;
   out_2944627059138367017[18] = 0;
   out_2944627059138367017[19] = 0;
   out_2944627059138367017[20] = 0;
   out_2944627059138367017[21] = 0;
   out_2944627059138367017[22] = 0;
   out_2944627059138367017[23] = 0;
   out_2944627059138367017[24] = 0;
   out_2944627059138367017[25] = 1;
   out_2944627059138367017[26] = 0;
   out_2944627059138367017[27] = 0;
   out_2944627059138367017[28] = 0;
   out_2944627059138367017[29] = 0;
   out_2944627059138367017[30] = 0;
   out_2944627059138367017[31] = 0;
   out_2944627059138367017[32] = 0;
   out_2944627059138367017[33] = 0;
   out_2944627059138367017[34] = 0;
   out_2944627059138367017[35] = 0;
   out_2944627059138367017[36] = 0;
   out_2944627059138367017[37] = 0;
   out_2944627059138367017[38] = 0;
   out_2944627059138367017[39] = 0;
   out_2944627059138367017[40] = 0;
   out_2944627059138367017[41] = 0;
   out_2944627059138367017[42] = 0;
   out_2944627059138367017[43] = 0;
   out_2944627059138367017[44] = 1;
   out_2944627059138367017[45] = 0;
   out_2944627059138367017[46] = 0;
   out_2944627059138367017[47] = 0;
   out_2944627059138367017[48] = 0;
   out_2944627059138367017[49] = 0;
   out_2944627059138367017[50] = 0;
   out_2944627059138367017[51] = 0;
   out_2944627059138367017[52] = 0;
   out_2944627059138367017[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_7993045186406776049) {
  err_fun(nom_x, delta_x, out_7993045186406776049);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_6214826562014697166) {
  inv_err_fun(nom_x, true_x, out_6214826562014697166);
}
void pose_H_mod_fun(double *state, double *out_2036493750579969796) {
  H_mod_fun(state, out_2036493750579969796);
}
void pose_f_fun(double *state, double dt, double *out_5815055826729932803) {
  f_fun(state,  dt, out_5815055826729932803);
}
void pose_F_fun(double *state, double dt, double *out_8088927993986614311) {
  F_fun(state,  dt, out_8088927993986614311);
}
void pose_h_4(double *state, double *unused, double *out_4596209739200211234) {
  h_4(state, unused, out_4596209739200211234);
}
void pose_H_4(double *state, double *unused, double *out_6907867915477851546) {
  H_4(state, unused, out_6907867915477851546);
}
void pose_h_10(double *state, double *unused, double *out_7511182918238442662) {
  h_10(state, unused, out_7511182918238442662);
}
void pose_H_10(double *state, double *unused, double *out_5218983418130581187) {
  H_10(state, unused, out_5218983418130581187);
}
void pose_h_13(double *state, double *unused, double *out_3565323313024088772) {
  h_13(state, unused, out_3565323313024088772);
}
void pose_H_13(double *state, double *unused, double *out_3695594090145518745) {
  H_13(state, unused, out_3695594090145518745);
}
void pose_h_14(double *state, double *unused, double *out_6195396300878839194) {
  h_14(state, unused, out_6195396300878839194);
}
void pose_H_14(double *state, double *unused, double *out_2944627059138367017) {
  H_14(state, unused, out_2944627059138367017);
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
