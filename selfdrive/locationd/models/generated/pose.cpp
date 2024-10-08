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
 *                       Code generated with SymPy 1.12                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_8874829572354212109) {
   out_8874829572354212109[0] = delta_x[0] + nom_x[0];
   out_8874829572354212109[1] = delta_x[1] + nom_x[1];
   out_8874829572354212109[2] = delta_x[2] + nom_x[2];
   out_8874829572354212109[3] = delta_x[3] + nom_x[3];
   out_8874829572354212109[4] = delta_x[4] + nom_x[4];
   out_8874829572354212109[5] = delta_x[5] + nom_x[5];
   out_8874829572354212109[6] = delta_x[6] + nom_x[6];
   out_8874829572354212109[7] = delta_x[7] + nom_x[7];
   out_8874829572354212109[8] = delta_x[8] + nom_x[8];
   out_8874829572354212109[9] = delta_x[9] + nom_x[9];
   out_8874829572354212109[10] = delta_x[10] + nom_x[10];
   out_8874829572354212109[11] = delta_x[11] + nom_x[11];
   out_8874829572354212109[12] = delta_x[12] + nom_x[12];
   out_8874829572354212109[13] = delta_x[13] + nom_x[13];
   out_8874829572354212109[14] = delta_x[14] + nom_x[14];
   out_8874829572354212109[15] = delta_x[15] + nom_x[15];
   out_8874829572354212109[16] = delta_x[16] + nom_x[16];
   out_8874829572354212109[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_1180005862082641495) {
   out_1180005862082641495[0] = -nom_x[0] + true_x[0];
   out_1180005862082641495[1] = -nom_x[1] + true_x[1];
   out_1180005862082641495[2] = -nom_x[2] + true_x[2];
   out_1180005862082641495[3] = -nom_x[3] + true_x[3];
   out_1180005862082641495[4] = -nom_x[4] + true_x[4];
   out_1180005862082641495[5] = -nom_x[5] + true_x[5];
   out_1180005862082641495[6] = -nom_x[6] + true_x[6];
   out_1180005862082641495[7] = -nom_x[7] + true_x[7];
   out_1180005862082641495[8] = -nom_x[8] + true_x[8];
   out_1180005862082641495[9] = -nom_x[9] + true_x[9];
   out_1180005862082641495[10] = -nom_x[10] + true_x[10];
   out_1180005862082641495[11] = -nom_x[11] + true_x[11];
   out_1180005862082641495[12] = -nom_x[12] + true_x[12];
   out_1180005862082641495[13] = -nom_x[13] + true_x[13];
   out_1180005862082641495[14] = -nom_x[14] + true_x[14];
   out_1180005862082641495[15] = -nom_x[15] + true_x[15];
   out_1180005862082641495[16] = -nom_x[16] + true_x[16];
   out_1180005862082641495[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_8262365463737575842) {
   out_8262365463737575842[0] = 1.0;
   out_8262365463737575842[1] = 0;
   out_8262365463737575842[2] = 0;
   out_8262365463737575842[3] = 0;
   out_8262365463737575842[4] = 0;
   out_8262365463737575842[5] = 0;
   out_8262365463737575842[6] = 0;
   out_8262365463737575842[7] = 0;
   out_8262365463737575842[8] = 0;
   out_8262365463737575842[9] = 0;
   out_8262365463737575842[10] = 0;
   out_8262365463737575842[11] = 0;
   out_8262365463737575842[12] = 0;
   out_8262365463737575842[13] = 0;
   out_8262365463737575842[14] = 0;
   out_8262365463737575842[15] = 0;
   out_8262365463737575842[16] = 0;
   out_8262365463737575842[17] = 0;
   out_8262365463737575842[18] = 0;
   out_8262365463737575842[19] = 1.0;
   out_8262365463737575842[20] = 0;
   out_8262365463737575842[21] = 0;
   out_8262365463737575842[22] = 0;
   out_8262365463737575842[23] = 0;
   out_8262365463737575842[24] = 0;
   out_8262365463737575842[25] = 0;
   out_8262365463737575842[26] = 0;
   out_8262365463737575842[27] = 0;
   out_8262365463737575842[28] = 0;
   out_8262365463737575842[29] = 0;
   out_8262365463737575842[30] = 0;
   out_8262365463737575842[31] = 0;
   out_8262365463737575842[32] = 0;
   out_8262365463737575842[33] = 0;
   out_8262365463737575842[34] = 0;
   out_8262365463737575842[35] = 0;
   out_8262365463737575842[36] = 0;
   out_8262365463737575842[37] = 0;
   out_8262365463737575842[38] = 1.0;
   out_8262365463737575842[39] = 0;
   out_8262365463737575842[40] = 0;
   out_8262365463737575842[41] = 0;
   out_8262365463737575842[42] = 0;
   out_8262365463737575842[43] = 0;
   out_8262365463737575842[44] = 0;
   out_8262365463737575842[45] = 0;
   out_8262365463737575842[46] = 0;
   out_8262365463737575842[47] = 0;
   out_8262365463737575842[48] = 0;
   out_8262365463737575842[49] = 0;
   out_8262365463737575842[50] = 0;
   out_8262365463737575842[51] = 0;
   out_8262365463737575842[52] = 0;
   out_8262365463737575842[53] = 0;
   out_8262365463737575842[54] = 0;
   out_8262365463737575842[55] = 0;
   out_8262365463737575842[56] = 0;
   out_8262365463737575842[57] = 1.0;
   out_8262365463737575842[58] = 0;
   out_8262365463737575842[59] = 0;
   out_8262365463737575842[60] = 0;
   out_8262365463737575842[61] = 0;
   out_8262365463737575842[62] = 0;
   out_8262365463737575842[63] = 0;
   out_8262365463737575842[64] = 0;
   out_8262365463737575842[65] = 0;
   out_8262365463737575842[66] = 0;
   out_8262365463737575842[67] = 0;
   out_8262365463737575842[68] = 0;
   out_8262365463737575842[69] = 0;
   out_8262365463737575842[70] = 0;
   out_8262365463737575842[71] = 0;
   out_8262365463737575842[72] = 0;
   out_8262365463737575842[73] = 0;
   out_8262365463737575842[74] = 0;
   out_8262365463737575842[75] = 0;
   out_8262365463737575842[76] = 1.0;
   out_8262365463737575842[77] = 0;
   out_8262365463737575842[78] = 0;
   out_8262365463737575842[79] = 0;
   out_8262365463737575842[80] = 0;
   out_8262365463737575842[81] = 0;
   out_8262365463737575842[82] = 0;
   out_8262365463737575842[83] = 0;
   out_8262365463737575842[84] = 0;
   out_8262365463737575842[85] = 0;
   out_8262365463737575842[86] = 0;
   out_8262365463737575842[87] = 0;
   out_8262365463737575842[88] = 0;
   out_8262365463737575842[89] = 0;
   out_8262365463737575842[90] = 0;
   out_8262365463737575842[91] = 0;
   out_8262365463737575842[92] = 0;
   out_8262365463737575842[93] = 0;
   out_8262365463737575842[94] = 0;
   out_8262365463737575842[95] = 1.0;
   out_8262365463737575842[96] = 0;
   out_8262365463737575842[97] = 0;
   out_8262365463737575842[98] = 0;
   out_8262365463737575842[99] = 0;
   out_8262365463737575842[100] = 0;
   out_8262365463737575842[101] = 0;
   out_8262365463737575842[102] = 0;
   out_8262365463737575842[103] = 0;
   out_8262365463737575842[104] = 0;
   out_8262365463737575842[105] = 0;
   out_8262365463737575842[106] = 0;
   out_8262365463737575842[107] = 0;
   out_8262365463737575842[108] = 0;
   out_8262365463737575842[109] = 0;
   out_8262365463737575842[110] = 0;
   out_8262365463737575842[111] = 0;
   out_8262365463737575842[112] = 0;
   out_8262365463737575842[113] = 0;
   out_8262365463737575842[114] = 1.0;
   out_8262365463737575842[115] = 0;
   out_8262365463737575842[116] = 0;
   out_8262365463737575842[117] = 0;
   out_8262365463737575842[118] = 0;
   out_8262365463737575842[119] = 0;
   out_8262365463737575842[120] = 0;
   out_8262365463737575842[121] = 0;
   out_8262365463737575842[122] = 0;
   out_8262365463737575842[123] = 0;
   out_8262365463737575842[124] = 0;
   out_8262365463737575842[125] = 0;
   out_8262365463737575842[126] = 0;
   out_8262365463737575842[127] = 0;
   out_8262365463737575842[128] = 0;
   out_8262365463737575842[129] = 0;
   out_8262365463737575842[130] = 0;
   out_8262365463737575842[131] = 0;
   out_8262365463737575842[132] = 0;
   out_8262365463737575842[133] = 1.0;
   out_8262365463737575842[134] = 0;
   out_8262365463737575842[135] = 0;
   out_8262365463737575842[136] = 0;
   out_8262365463737575842[137] = 0;
   out_8262365463737575842[138] = 0;
   out_8262365463737575842[139] = 0;
   out_8262365463737575842[140] = 0;
   out_8262365463737575842[141] = 0;
   out_8262365463737575842[142] = 0;
   out_8262365463737575842[143] = 0;
   out_8262365463737575842[144] = 0;
   out_8262365463737575842[145] = 0;
   out_8262365463737575842[146] = 0;
   out_8262365463737575842[147] = 0;
   out_8262365463737575842[148] = 0;
   out_8262365463737575842[149] = 0;
   out_8262365463737575842[150] = 0;
   out_8262365463737575842[151] = 0;
   out_8262365463737575842[152] = 1.0;
   out_8262365463737575842[153] = 0;
   out_8262365463737575842[154] = 0;
   out_8262365463737575842[155] = 0;
   out_8262365463737575842[156] = 0;
   out_8262365463737575842[157] = 0;
   out_8262365463737575842[158] = 0;
   out_8262365463737575842[159] = 0;
   out_8262365463737575842[160] = 0;
   out_8262365463737575842[161] = 0;
   out_8262365463737575842[162] = 0;
   out_8262365463737575842[163] = 0;
   out_8262365463737575842[164] = 0;
   out_8262365463737575842[165] = 0;
   out_8262365463737575842[166] = 0;
   out_8262365463737575842[167] = 0;
   out_8262365463737575842[168] = 0;
   out_8262365463737575842[169] = 0;
   out_8262365463737575842[170] = 0;
   out_8262365463737575842[171] = 1.0;
   out_8262365463737575842[172] = 0;
   out_8262365463737575842[173] = 0;
   out_8262365463737575842[174] = 0;
   out_8262365463737575842[175] = 0;
   out_8262365463737575842[176] = 0;
   out_8262365463737575842[177] = 0;
   out_8262365463737575842[178] = 0;
   out_8262365463737575842[179] = 0;
   out_8262365463737575842[180] = 0;
   out_8262365463737575842[181] = 0;
   out_8262365463737575842[182] = 0;
   out_8262365463737575842[183] = 0;
   out_8262365463737575842[184] = 0;
   out_8262365463737575842[185] = 0;
   out_8262365463737575842[186] = 0;
   out_8262365463737575842[187] = 0;
   out_8262365463737575842[188] = 0;
   out_8262365463737575842[189] = 0;
   out_8262365463737575842[190] = 1.0;
   out_8262365463737575842[191] = 0;
   out_8262365463737575842[192] = 0;
   out_8262365463737575842[193] = 0;
   out_8262365463737575842[194] = 0;
   out_8262365463737575842[195] = 0;
   out_8262365463737575842[196] = 0;
   out_8262365463737575842[197] = 0;
   out_8262365463737575842[198] = 0;
   out_8262365463737575842[199] = 0;
   out_8262365463737575842[200] = 0;
   out_8262365463737575842[201] = 0;
   out_8262365463737575842[202] = 0;
   out_8262365463737575842[203] = 0;
   out_8262365463737575842[204] = 0;
   out_8262365463737575842[205] = 0;
   out_8262365463737575842[206] = 0;
   out_8262365463737575842[207] = 0;
   out_8262365463737575842[208] = 0;
   out_8262365463737575842[209] = 1.0;
   out_8262365463737575842[210] = 0;
   out_8262365463737575842[211] = 0;
   out_8262365463737575842[212] = 0;
   out_8262365463737575842[213] = 0;
   out_8262365463737575842[214] = 0;
   out_8262365463737575842[215] = 0;
   out_8262365463737575842[216] = 0;
   out_8262365463737575842[217] = 0;
   out_8262365463737575842[218] = 0;
   out_8262365463737575842[219] = 0;
   out_8262365463737575842[220] = 0;
   out_8262365463737575842[221] = 0;
   out_8262365463737575842[222] = 0;
   out_8262365463737575842[223] = 0;
   out_8262365463737575842[224] = 0;
   out_8262365463737575842[225] = 0;
   out_8262365463737575842[226] = 0;
   out_8262365463737575842[227] = 0;
   out_8262365463737575842[228] = 1.0;
   out_8262365463737575842[229] = 0;
   out_8262365463737575842[230] = 0;
   out_8262365463737575842[231] = 0;
   out_8262365463737575842[232] = 0;
   out_8262365463737575842[233] = 0;
   out_8262365463737575842[234] = 0;
   out_8262365463737575842[235] = 0;
   out_8262365463737575842[236] = 0;
   out_8262365463737575842[237] = 0;
   out_8262365463737575842[238] = 0;
   out_8262365463737575842[239] = 0;
   out_8262365463737575842[240] = 0;
   out_8262365463737575842[241] = 0;
   out_8262365463737575842[242] = 0;
   out_8262365463737575842[243] = 0;
   out_8262365463737575842[244] = 0;
   out_8262365463737575842[245] = 0;
   out_8262365463737575842[246] = 0;
   out_8262365463737575842[247] = 1.0;
   out_8262365463737575842[248] = 0;
   out_8262365463737575842[249] = 0;
   out_8262365463737575842[250] = 0;
   out_8262365463737575842[251] = 0;
   out_8262365463737575842[252] = 0;
   out_8262365463737575842[253] = 0;
   out_8262365463737575842[254] = 0;
   out_8262365463737575842[255] = 0;
   out_8262365463737575842[256] = 0;
   out_8262365463737575842[257] = 0;
   out_8262365463737575842[258] = 0;
   out_8262365463737575842[259] = 0;
   out_8262365463737575842[260] = 0;
   out_8262365463737575842[261] = 0;
   out_8262365463737575842[262] = 0;
   out_8262365463737575842[263] = 0;
   out_8262365463737575842[264] = 0;
   out_8262365463737575842[265] = 0;
   out_8262365463737575842[266] = 1.0;
   out_8262365463737575842[267] = 0;
   out_8262365463737575842[268] = 0;
   out_8262365463737575842[269] = 0;
   out_8262365463737575842[270] = 0;
   out_8262365463737575842[271] = 0;
   out_8262365463737575842[272] = 0;
   out_8262365463737575842[273] = 0;
   out_8262365463737575842[274] = 0;
   out_8262365463737575842[275] = 0;
   out_8262365463737575842[276] = 0;
   out_8262365463737575842[277] = 0;
   out_8262365463737575842[278] = 0;
   out_8262365463737575842[279] = 0;
   out_8262365463737575842[280] = 0;
   out_8262365463737575842[281] = 0;
   out_8262365463737575842[282] = 0;
   out_8262365463737575842[283] = 0;
   out_8262365463737575842[284] = 0;
   out_8262365463737575842[285] = 1.0;
   out_8262365463737575842[286] = 0;
   out_8262365463737575842[287] = 0;
   out_8262365463737575842[288] = 0;
   out_8262365463737575842[289] = 0;
   out_8262365463737575842[290] = 0;
   out_8262365463737575842[291] = 0;
   out_8262365463737575842[292] = 0;
   out_8262365463737575842[293] = 0;
   out_8262365463737575842[294] = 0;
   out_8262365463737575842[295] = 0;
   out_8262365463737575842[296] = 0;
   out_8262365463737575842[297] = 0;
   out_8262365463737575842[298] = 0;
   out_8262365463737575842[299] = 0;
   out_8262365463737575842[300] = 0;
   out_8262365463737575842[301] = 0;
   out_8262365463737575842[302] = 0;
   out_8262365463737575842[303] = 0;
   out_8262365463737575842[304] = 1.0;
   out_8262365463737575842[305] = 0;
   out_8262365463737575842[306] = 0;
   out_8262365463737575842[307] = 0;
   out_8262365463737575842[308] = 0;
   out_8262365463737575842[309] = 0;
   out_8262365463737575842[310] = 0;
   out_8262365463737575842[311] = 0;
   out_8262365463737575842[312] = 0;
   out_8262365463737575842[313] = 0;
   out_8262365463737575842[314] = 0;
   out_8262365463737575842[315] = 0;
   out_8262365463737575842[316] = 0;
   out_8262365463737575842[317] = 0;
   out_8262365463737575842[318] = 0;
   out_8262365463737575842[319] = 0;
   out_8262365463737575842[320] = 0;
   out_8262365463737575842[321] = 0;
   out_8262365463737575842[322] = 0;
   out_8262365463737575842[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_5327889686526496546) {
   out_5327889686526496546[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_5327889686526496546[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_5327889686526496546[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_5327889686526496546[3] = dt*state[12] + state[3];
   out_5327889686526496546[4] = dt*state[13] + state[4];
   out_5327889686526496546[5] = dt*state[14] + state[5];
   out_5327889686526496546[6] = state[6];
   out_5327889686526496546[7] = state[7];
   out_5327889686526496546[8] = state[8];
   out_5327889686526496546[9] = state[9];
   out_5327889686526496546[10] = state[10];
   out_5327889686526496546[11] = state[11];
   out_5327889686526496546[12] = state[12];
   out_5327889686526496546[13] = state[13];
   out_5327889686526496546[14] = state[14];
   out_5327889686526496546[15] = state[15];
   out_5327889686526496546[16] = state[16];
   out_5327889686526496546[17] = state[17];
}
void F_fun(double *state, double dt, double *out_508639460086939955) {
   out_508639460086939955[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_508639460086939955[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_508639460086939955[2] = 0;
   out_508639460086939955[3] = 0;
   out_508639460086939955[4] = 0;
   out_508639460086939955[5] = 0;
   out_508639460086939955[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_508639460086939955[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_508639460086939955[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_508639460086939955[9] = 0;
   out_508639460086939955[10] = 0;
   out_508639460086939955[11] = 0;
   out_508639460086939955[12] = 0;
   out_508639460086939955[13] = 0;
   out_508639460086939955[14] = 0;
   out_508639460086939955[15] = 0;
   out_508639460086939955[16] = 0;
   out_508639460086939955[17] = 0;
   out_508639460086939955[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_508639460086939955[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_508639460086939955[20] = 0;
   out_508639460086939955[21] = 0;
   out_508639460086939955[22] = 0;
   out_508639460086939955[23] = 0;
   out_508639460086939955[24] = 0;
   out_508639460086939955[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_508639460086939955[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_508639460086939955[27] = 0;
   out_508639460086939955[28] = 0;
   out_508639460086939955[29] = 0;
   out_508639460086939955[30] = 0;
   out_508639460086939955[31] = 0;
   out_508639460086939955[32] = 0;
   out_508639460086939955[33] = 0;
   out_508639460086939955[34] = 0;
   out_508639460086939955[35] = 0;
   out_508639460086939955[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_508639460086939955[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_508639460086939955[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_508639460086939955[39] = 0;
   out_508639460086939955[40] = 0;
   out_508639460086939955[41] = 0;
   out_508639460086939955[42] = 0;
   out_508639460086939955[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_508639460086939955[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_508639460086939955[45] = 0;
   out_508639460086939955[46] = 0;
   out_508639460086939955[47] = 0;
   out_508639460086939955[48] = 0;
   out_508639460086939955[49] = 0;
   out_508639460086939955[50] = 0;
   out_508639460086939955[51] = 0;
   out_508639460086939955[52] = 0;
   out_508639460086939955[53] = 0;
   out_508639460086939955[54] = 0;
   out_508639460086939955[55] = 0;
   out_508639460086939955[56] = 0;
   out_508639460086939955[57] = 1;
   out_508639460086939955[58] = 0;
   out_508639460086939955[59] = 0;
   out_508639460086939955[60] = 0;
   out_508639460086939955[61] = 0;
   out_508639460086939955[62] = 0;
   out_508639460086939955[63] = 0;
   out_508639460086939955[64] = 0;
   out_508639460086939955[65] = 0;
   out_508639460086939955[66] = dt;
   out_508639460086939955[67] = 0;
   out_508639460086939955[68] = 0;
   out_508639460086939955[69] = 0;
   out_508639460086939955[70] = 0;
   out_508639460086939955[71] = 0;
   out_508639460086939955[72] = 0;
   out_508639460086939955[73] = 0;
   out_508639460086939955[74] = 0;
   out_508639460086939955[75] = 0;
   out_508639460086939955[76] = 1;
   out_508639460086939955[77] = 0;
   out_508639460086939955[78] = 0;
   out_508639460086939955[79] = 0;
   out_508639460086939955[80] = 0;
   out_508639460086939955[81] = 0;
   out_508639460086939955[82] = 0;
   out_508639460086939955[83] = 0;
   out_508639460086939955[84] = 0;
   out_508639460086939955[85] = dt;
   out_508639460086939955[86] = 0;
   out_508639460086939955[87] = 0;
   out_508639460086939955[88] = 0;
   out_508639460086939955[89] = 0;
   out_508639460086939955[90] = 0;
   out_508639460086939955[91] = 0;
   out_508639460086939955[92] = 0;
   out_508639460086939955[93] = 0;
   out_508639460086939955[94] = 0;
   out_508639460086939955[95] = 1;
   out_508639460086939955[96] = 0;
   out_508639460086939955[97] = 0;
   out_508639460086939955[98] = 0;
   out_508639460086939955[99] = 0;
   out_508639460086939955[100] = 0;
   out_508639460086939955[101] = 0;
   out_508639460086939955[102] = 0;
   out_508639460086939955[103] = 0;
   out_508639460086939955[104] = dt;
   out_508639460086939955[105] = 0;
   out_508639460086939955[106] = 0;
   out_508639460086939955[107] = 0;
   out_508639460086939955[108] = 0;
   out_508639460086939955[109] = 0;
   out_508639460086939955[110] = 0;
   out_508639460086939955[111] = 0;
   out_508639460086939955[112] = 0;
   out_508639460086939955[113] = 0;
   out_508639460086939955[114] = 1;
   out_508639460086939955[115] = 0;
   out_508639460086939955[116] = 0;
   out_508639460086939955[117] = 0;
   out_508639460086939955[118] = 0;
   out_508639460086939955[119] = 0;
   out_508639460086939955[120] = 0;
   out_508639460086939955[121] = 0;
   out_508639460086939955[122] = 0;
   out_508639460086939955[123] = 0;
   out_508639460086939955[124] = 0;
   out_508639460086939955[125] = 0;
   out_508639460086939955[126] = 0;
   out_508639460086939955[127] = 0;
   out_508639460086939955[128] = 0;
   out_508639460086939955[129] = 0;
   out_508639460086939955[130] = 0;
   out_508639460086939955[131] = 0;
   out_508639460086939955[132] = 0;
   out_508639460086939955[133] = 1;
   out_508639460086939955[134] = 0;
   out_508639460086939955[135] = 0;
   out_508639460086939955[136] = 0;
   out_508639460086939955[137] = 0;
   out_508639460086939955[138] = 0;
   out_508639460086939955[139] = 0;
   out_508639460086939955[140] = 0;
   out_508639460086939955[141] = 0;
   out_508639460086939955[142] = 0;
   out_508639460086939955[143] = 0;
   out_508639460086939955[144] = 0;
   out_508639460086939955[145] = 0;
   out_508639460086939955[146] = 0;
   out_508639460086939955[147] = 0;
   out_508639460086939955[148] = 0;
   out_508639460086939955[149] = 0;
   out_508639460086939955[150] = 0;
   out_508639460086939955[151] = 0;
   out_508639460086939955[152] = 1;
   out_508639460086939955[153] = 0;
   out_508639460086939955[154] = 0;
   out_508639460086939955[155] = 0;
   out_508639460086939955[156] = 0;
   out_508639460086939955[157] = 0;
   out_508639460086939955[158] = 0;
   out_508639460086939955[159] = 0;
   out_508639460086939955[160] = 0;
   out_508639460086939955[161] = 0;
   out_508639460086939955[162] = 0;
   out_508639460086939955[163] = 0;
   out_508639460086939955[164] = 0;
   out_508639460086939955[165] = 0;
   out_508639460086939955[166] = 0;
   out_508639460086939955[167] = 0;
   out_508639460086939955[168] = 0;
   out_508639460086939955[169] = 0;
   out_508639460086939955[170] = 0;
   out_508639460086939955[171] = 1;
   out_508639460086939955[172] = 0;
   out_508639460086939955[173] = 0;
   out_508639460086939955[174] = 0;
   out_508639460086939955[175] = 0;
   out_508639460086939955[176] = 0;
   out_508639460086939955[177] = 0;
   out_508639460086939955[178] = 0;
   out_508639460086939955[179] = 0;
   out_508639460086939955[180] = 0;
   out_508639460086939955[181] = 0;
   out_508639460086939955[182] = 0;
   out_508639460086939955[183] = 0;
   out_508639460086939955[184] = 0;
   out_508639460086939955[185] = 0;
   out_508639460086939955[186] = 0;
   out_508639460086939955[187] = 0;
   out_508639460086939955[188] = 0;
   out_508639460086939955[189] = 0;
   out_508639460086939955[190] = 1;
   out_508639460086939955[191] = 0;
   out_508639460086939955[192] = 0;
   out_508639460086939955[193] = 0;
   out_508639460086939955[194] = 0;
   out_508639460086939955[195] = 0;
   out_508639460086939955[196] = 0;
   out_508639460086939955[197] = 0;
   out_508639460086939955[198] = 0;
   out_508639460086939955[199] = 0;
   out_508639460086939955[200] = 0;
   out_508639460086939955[201] = 0;
   out_508639460086939955[202] = 0;
   out_508639460086939955[203] = 0;
   out_508639460086939955[204] = 0;
   out_508639460086939955[205] = 0;
   out_508639460086939955[206] = 0;
   out_508639460086939955[207] = 0;
   out_508639460086939955[208] = 0;
   out_508639460086939955[209] = 1;
   out_508639460086939955[210] = 0;
   out_508639460086939955[211] = 0;
   out_508639460086939955[212] = 0;
   out_508639460086939955[213] = 0;
   out_508639460086939955[214] = 0;
   out_508639460086939955[215] = 0;
   out_508639460086939955[216] = 0;
   out_508639460086939955[217] = 0;
   out_508639460086939955[218] = 0;
   out_508639460086939955[219] = 0;
   out_508639460086939955[220] = 0;
   out_508639460086939955[221] = 0;
   out_508639460086939955[222] = 0;
   out_508639460086939955[223] = 0;
   out_508639460086939955[224] = 0;
   out_508639460086939955[225] = 0;
   out_508639460086939955[226] = 0;
   out_508639460086939955[227] = 0;
   out_508639460086939955[228] = 1;
   out_508639460086939955[229] = 0;
   out_508639460086939955[230] = 0;
   out_508639460086939955[231] = 0;
   out_508639460086939955[232] = 0;
   out_508639460086939955[233] = 0;
   out_508639460086939955[234] = 0;
   out_508639460086939955[235] = 0;
   out_508639460086939955[236] = 0;
   out_508639460086939955[237] = 0;
   out_508639460086939955[238] = 0;
   out_508639460086939955[239] = 0;
   out_508639460086939955[240] = 0;
   out_508639460086939955[241] = 0;
   out_508639460086939955[242] = 0;
   out_508639460086939955[243] = 0;
   out_508639460086939955[244] = 0;
   out_508639460086939955[245] = 0;
   out_508639460086939955[246] = 0;
   out_508639460086939955[247] = 1;
   out_508639460086939955[248] = 0;
   out_508639460086939955[249] = 0;
   out_508639460086939955[250] = 0;
   out_508639460086939955[251] = 0;
   out_508639460086939955[252] = 0;
   out_508639460086939955[253] = 0;
   out_508639460086939955[254] = 0;
   out_508639460086939955[255] = 0;
   out_508639460086939955[256] = 0;
   out_508639460086939955[257] = 0;
   out_508639460086939955[258] = 0;
   out_508639460086939955[259] = 0;
   out_508639460086939955[260] = 0;
   out_508639460086939955[261] = 0;
   out_508639460086939955[262] = 0;
   out_508639460086939955[263] = 0;
   out_508639460086939955[264] = 0;
   out_508639460086939955[265] = 0;
   out_508639460086939955[266] = 1;
   out_508639460086939955[267] = 0;
   out_508639460086939955[268] = 0;
   out_508639460086939955[269] = 0;
   out_508639460086939955[270] = 0;
   out_508639460086939955[271] = 0;
   out_508639460086939955[272] = 0;
   out_508639460086939955[273] = 0;
   out_508639460086939955[274] = 0;
   out_508639460086939955[275] = 0;
   out_508639460086939955[276] = 0;
   out_508639460086939955[277] = 0;
   out_508639460086939955[278] = 0;
   out_508639460086939955[279] = 0;
   out_508639460086939955[280] = 0;
   out_508639460086939955[281] = 0;
   out_508639460086939955[282] = 0;
   out_508639460086939955[283] = 0;
   out_508639460086939955[284] = 0;
   out_508639460086939955[285] = 1;
   out_508639460086939955[286] = 0;
   out_508639460086939955[287] = 0;
   out_508639460086939955[288] = 0;
   out_508639460086939955[289] = 0;
   out_508639460086939955[290] = 0;
   out_508639460086939955[291] = 0;
   out_508639460086939955[292] = 0;
   out_508639460086939955[293] = 0;
   out_508639460086939955[294] = 0;
   out_508639460086939955[295] = 0;
   out_508639460086939955[296] = 0;
   out_508639460086939955[297] = 0;
   out_508639460086939955[298] = 0;
   out_508639460086939955[299] = 0;
   out_508639460086939955[300] = 0;
   out_508639460086939955[301] = 0;
   out_508639460086939955[302] = 0;
   out_508639460086939955[303] = 0;
   out_508639460086939955[304] = 1;
   out_508639460086939955[305] = 0;
   out_508639460086939955[306] = 0;
   out_508639460086939955[307] = 0;
   out_508639460086939955[308] = 0;
   out_508639460086939955[309] = 0;
   out_508639460086939955[310] = 0;
   out_508639460086939955[311] = 0;
   out_508639460086939955[312] = 0;
   out_508639460086939955[313] = 0;
   out_508639460086939955[314] = 0;
   out_508639460086939955[315] = 0;
   out_508639460086939955[316] = 0;
   out_508639460086939955[317] = 0;
   out_508639460086939955[318] = 0;
   out_508639460086939955[319] = 0;
   out_508639460086939955[320] = 0;
   out_508639460086939955[321] = 0;
   out_508639460086939955[322] = 0;
   out_508639460086939955[323] = 1;
}
void h_4(double *state, double *unused, double *out_4278134127096839033) {
   out_4278134127096839033[0] = state[6] + state[9];
   out_4278134127096839033[1] = state[7] + state[10];
   out_4278134127096839033[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_8196960472798219952) {
   out_8196960472798219952[0] = 0;
   out_8196960472798219952[1] = 0;
   out_8196960472798219952[2] = 0;
   out_8196960472798219952[3] = 0;
   out_8196960472798219952[4] = 0;
   out_8196960472798219952[5] = 0;
   out_8196960472798219952[6] = 1;
   out_8196960472798219952[7] = 0;
   out_8196960472798219952[8] = 0;
   out_8196960472798219952[9] = 1;
   out_8196960472798219952[10] = 0;
   out_8196960472798219952[11] = 0;
   out_8196960472798219952[12] = 0;
   out_8196960472798219952[13] = 0;
   out_8196960472798219952[14] = 0;
   out_8196960472798219952[15] = 0;
   out_8196960472798219952[16] = 0;
   out_8196960472798219952[17] = 0;
   out_8196960472798219952[18] = 0;
   out_8196960472798219952[19] = 0;
   out_8196960472798219952[20] = 0;
   out_8196960472798219952[21] = 0;
   out_8196960472798219952[22] = 0;
   out_8196960472798219952[23] = 0;
   out_8196960472798219952[24] = 0;
   out_8196960472798219952[25] = 1;
   out_8196960472798219952[26] = 0;
   out_8196960472798219952[27] = 0;
   out_8196960472798219952[28] = 1;
   out_8196960472798219952[29] = 0;
   out_8196960472798219952[30] = 0;
   out_8196960472798219952[31] = 0;
   out_8196960472798219952[32] = 0;
   out_8196960472798219952[33] = 0;
   out_8196960472798219952[34] = 0;
   out_8196960472798219952[35] = 0;
   out_8196960472798219952[36] = 0;
   out_8196960472798219952[37] = 0;
   out_8196960472798219952[38] = 0;
   out_8196960472798219952[39] = 0;
   out_8196960472798219952[40] = 0;
   out_8196960472798219952[41] = 0;
   out_8196960472798219952[42] = 0;
   out_8196960472798219952[43] = 0;
   out_8196960472798219952[44] = 1;
   out_8196960472798219952[45] = 0;
   out_8196960472798219952[46] = 0;
   out_8196960472798219952[47] = 1;
   out_8196960472798219952[48] = 0;
   out_8196960472798219952[49] = 0;
   out_8196960472798219952[50] = 0;
   out_8196960472798219952[51] = 0;
   out_8196960472798219952[52] = 0;
   out_8196960472798219952[53] = 0;
}
void h_10(double *state, double *unused, double *out_5750841777717275382) {
   out_5750841777717275382[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_5750841777717275382[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_5750841777717275382[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_2204051736113766918) {
   out_2204051736113766918[0] = 0;
   out_2204051736113766918[1] = 9.8100000000000005*cos(state[1]);
   out_2204051736113766918[2] = 0;
   out_2204051736113766918[3] = 0;
   out_2204051736113766918[4] = -state[8];
   out_2204051736113766918[5] = state[7];
   out_2204051736113766918[6] = 0;
   out_2204051736113766918[7] = state[5];
   out_2204051736113766918[8] = -state[4];
   out_2204051736113766918[9] = 0;
   out_2204051736113766918[10] = 0;
   out_2204051736113766918[11] = 0;
   out_2204051736113766918[12] = 1;
   out_2204051736113766918[13] = 0;
   out_2204051736113766918[14] = 0;
   out_2204051736113766918[15] = 1;
   out_2204051736113766918[16] = 0;
   out_2204051736113766918[17] = 0;
   out_2204051736113766918[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_2204051736113766918[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_2204051736113766918[20] = 0;
   out_2204051736113766918[21] = state[8];
   out_2204051736113766918[22] = 0;
   out_2204051736113766918[23] = -state[6];
   out_2204051736113766918[24] = -state[5];
   out_2204051736113766918[25] = 0;
   out_2204051736113766918[26] = state[3];
   out_2204051736113766918[27] = 0;
   out_2204051736113766918[28] = 0;
   out_2204051736113766918[29] = 0;
   out_2204051736113766918[30] = 0;
   out_2204051736113766918[31] = 1;
   out_2204051736113766918[32] = 0;
   out_2204051736113766918[33] = 0;
   out_2204051736113766918[34] = 1;
   out_2204051736113766918[35] = 0;
   out_2204051736113766918[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_2204051736113766918[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_2204051736113766918[38] = 0;
   out_2204051736113766918[39] = -state[7];
   out_2204051736113766918[40] = state[6];
   out_2204051736113766918[41] = 0;
   out_2204051736113766918[42] = state[4];
   out_2204051736113766918[43] = -state[3];
   out_2204051736113766918[44] = 0;
   out_2204051736113766918[45] = 0;
   out_2204051736113766918[46] = 0;
   out_2204051736113766918[47] = 0;
   out_2204051736113766918[48] = 0;
   out_2204051736113766918[49] = 0;
   out_2204051736113766918[50] = 1;
   out_2204051736113766918[51] = 0;
   out_2204051736113766918[52] = 0;
   out_2204051736113766918[53] = 1;
}
void h_13(double *state, double *unused, double *out_9079423146614688595) {
   out_9079423146614688595[0] = state[3];
   out_9079423146614688595[1] = state[4];
   out_9079423146614688595[2] = state[5];
}
void H_13(double *state, double *unused, double *out_7037509775578998863) {
   out_7037509775578998863[0] = 0;
   out_7037509775578998863[1] = 0;
   out_7037509775578998863[2] = 0;
   out_7037509775578998863[3] = 1;
   out_7037509775578998863[4] = 0;
   out_7037509775578998863[5] = 0;
   out_7037509775578998863[6] = 0;
   out_7037509775578998863[7] = 0;
   out_7037509775578998863[8] = 0;
   out_7037509775578998863[9] = 0;
   out_7037509775578998863[10] = 0;
   out_7037509775578998863[11] = 0;
   out_7037509775578998863[12] = 0;
   out_7037509775578998863[13] = 0;
   out_7037509775578998863[14] = 0;
   out_7037509775578998863[15] = 0;
   out_7037509775578998863[16] = 0;
   out_7037509775578998863[17] = 0;
   out_7037509775578998863[18] = 0;
   out_7037509775578998863[19] = 0;
   out_7037509775578998863[20] = 0;
   out_7037509775578998863[21] = 0;
   out_7037509775578998863[22] = 1;
   out_7037509775578998863[23] = 0;
   out_7037509775578998863[24] = 0;
   out_7037509775578998863[25] = 0;
   out_7037509775578998863[26] = 0;
   out_7037509775578998863[27] = 0;
   out_7037509775578998863[28] = 0;
   out_7037509775578998863[29] = 0;
   out_7037509775578998863[30] = 0;
   out_7037509775578998863[31] = 0;
   out_7037509775578998863[32] = 0;
   out_7037509775578998863[33] = 0;
   out_7037509775578998863[34] = 0;
   out_7037509775578998863[35] = 0;
   out_7037509775578998863[36] = 0;
   out_7037509775578998863[37] = 0;
   out_7037509775578998863[38] = 0;
   out_7037509775578998863[39] = 0;
   out_7037509775578998863[40] = 0;
   out_7037509775578998863[41] = 1;
   out_7037509775578998863[42] = 0;
   out_7037509775578998863[43] = 0;
   out_7037509775578998863[44] = 0;
   out_7037509775578998863[45] = 0;
   out_7037509775578998863[46] = 0;
   out_7037509775578998863[47] = 0;
   out_7037509775578998863[48] = 0;
   out_7037509775578998863[49] = 0;
   out_7037509775578998863[50] = 0;
   out_7037509775578998863[51] = 0;
   out_7037509775578998863[52] = 0;
   out_7037509775578998863[53] = 0;
}
void h_14(double *state, double *unused, double *out_2413440844245158569) {
   out_2413440844245158569[0] = state[6];
   out_2413440844245158569[1] = state[7];
   out_2413440844245158569[2] = state[8];
}
void H_14(double *state, double *unused, double *out_6286542744571847135) {
   out_6286542744571847135[0] = 0;
   out_6286542744571847135[1] = 0;
   out_6286542744571847135[2] = 0;
   out_6286542744571847135[3] = 0;
   out_6286542744571847135[4] = 0;
   out_6286542744571847135[5] = 0;
   out_6286542744571847135[6] = 1;
   out_6286542744571847135[7] = 0;
   out_6286542744571847135[8] = 0;
   out_6286542744571847135[9] = 0;
   out_6286542744571847135[10] = 0;
   out_6286542744571847135[11] = 0;
   out_6286542744571847135[12] = 0;
   out_6286542744571847135[13] = 0;
   out_6286542744571847135[14] = 0;
   out_6286542744571847135[15] = 0;
   out_6286542744571847135[16] = 0;
   out_6286542744571847135[17] = 0;
   out_6286542744571847135[18] = 0;
   out_6286542744571847135[19] = 0;
   out_6286542744571847135[20] = 0;
   out_6286542744571847135[21] = 0;
   out_6286542744571847135[22] = 0;
   out_6286542744571847135[23] = 0;
   out_6286542744571847135[24] = 0;
   out_6286542744571847135[25] = 1;
   out_6286542744571847135[26] = 0;
   out_6286542744571847135[27] = 0;
   out_6286542744571847135[28] = 0;
   out_6286542744571847135[29] = 0;
   out_6286542744571847135[30] = 0;
   out_6286542744571847135[31] = 0;
   out_6286542744571847135[32] = 0;
   out_6286542744571847135[33] = 0;
   out_6286542744571847135[34] = 0;
   out_6286542744571847135[35] = 0;
   out_6286542744571847135[36] = 0;
   out_6286542744571847135[37] = 0;
   out_6286542744571847135[38] = 0;
   out_6286542744571847135[39] = 0;
   out_6286542744571847135[40] = 0;
   out_6286542744571847135[41] = 0;
   out_6286542744571847135[42] = 0;
   out_6286542744571847135[43] = 0;
   out_6286542744571847135[44] = 1;
   out_6286542744571847135[45] = 0;
   out_6286542744571847135[46] = 0;
   out_6286542744571847135[47] = 0;
   out_6286542744571847135[48] = 0;
   out_6286542744571847135[49] = 0;
   out_6286542744571847135[50] = 0;
   out_6286542744571847135[51] = 0;
   out_6286542744571847135[52] = 0;
   out_6286542744571847135[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_8874829572354212109) {
  err_fun(nom_x, delta_x, out_8874829572354212109);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_1180005862082641495) {
  inv_err_fun(nom_x, true_x, out_1180005862082641495);
}
void pose_H_mod_fun(double *state, double *out_8262365463737575842) {
  H_mod_fun(state, out_8262365463737575842);
}
void pose_f_fun(double *state, double dt, double *out_5327889686526496546) {
  f_fun(state,  dt, out_5327889686526496546);
}
void pose_F_fun(double *state, double dt, double *out_508639460086939955) {
  F_fun(state,  dt, out_508639460086939955);
}
void pose_h_4(double *state, double *unused, double *out_4278134127096839033) {
  h_4(state, unused, out_4278134127096839033);
}
void pose_H_4(double *state, double *unused, double *out_8196960472798219952) {
  H_4(state, unused, out_8196960472798219952);
}
void pose_h_10(double *state, double *unused, double *out_5750841777717275382) {
  h_10(state, unused, out_5750841777717275382);
}
void pose_H_10(double *state, double *unused, double *out_2204051736113766918) {
  H_10(state, unused, out_2204051736113766918);
}
void pose_h_13(double *state, double *unused, double *out_9079423146614688595) {
  h_13(state, unused, out_9079423146614688595);
}
void pose_H_13(double *state, double *unused, double *out_7037509775578998863) {
  H_13(state, unused, out_7037509775578998863);
}
void pose_h_14(double *state, double *unused, double *out_2413440844245158569) {
  h_14(state, unused, out_2413440844245158569);
}
void pose_H_14(double *state, double *unused, double *out_6286542744571847135) {
  H_14(state, unused, out_6286542744571847135);
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
