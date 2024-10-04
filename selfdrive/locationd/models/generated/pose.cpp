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
void err_fun(double *nom_x, double *delta_x, double *out_3280080996569931798) {
   out_3280080996569931798[0] = delta_x[0] + nom_x[0];
   out_3280080996569931798[1] = delta_x[1] + nom_x[1];
   out_3280080996569931798[2] = delta_x[2] + nom_x[2];
   out_3280080996569931798[3] = delta_x[3] + nom_x[3];
   out_3280080996569931798[4] = delta_x[4] + nom_x[4];
   out_3280080996569931798[5] = delta_x[5] + nom_x[5];
   out_3280080996569931798[6] = delta_x[6] + nom_x[6];
   out_3280080996569931798[7] = delta_x[7] + nom_x[7];
   out_3280080996569931798[8] = delta_x[8] + nom_x[8];
   out_3280080996569931798[9] = delta_x[9] + nom_x[9];
   out_3280080996569931798[10] = delta_x[10] + nom_x[10];
   out_3280080996569931798[11] = delta_x[11] + nom_x[11];
   out_3280080996569931798[12] = delta_x[12] + nom_x[12];
   out_3280080996569931798[13] = delta_x[13] + nom_x[13];
   out_3280080996569931798[14] = delta_x[14] + nom_x[14];
   out_3280080996569931798[15] = delta_x[15] + nom_x[15];
   out_3280080996569931798[16] = delta_x[16] + nom_x[16];
   out_3280080996569931798[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5884406445564509642) {
   out_5884406445564509642[0] = -nom_x[0] + true_x[0];
   out_5884406445564509642[1] = -nom_x[1] + true_x[1];
   out_5884406445564509642[2] = -nom_x[2] + true_x[2];
   out_5884406445564509642[3] = -nom_x[3] + true_x[3];
   out_5884406445564509642[4] = -nom_x[4] + true_x[4];
   out_5884406445564509642[5] = -nom_x[5] + true_x[5];
   out_5884406445564509642[6] = -nom_x[6] + true_x[6];
   out_5884406445564509642[7] = -nom_x[7] + true_x[7];
   out_5884406445564509642[8] = -nom_x[8] + true_x[8];
   out_5884406445564509642[9] = -nom_x[9] + true_x[9];
   out_5884406445564509642[10] = -nom_x[10] + true_x[10];
   out_5884406445564509642[11] = -nom_x[11] + true_x[11];
   out_5884406445564509642[12] = -nom_x[12] + true_x[12];
   out_5884406445564509642[13] = -nom_x[13] + true_x[13];
   out_5884406445564509642[14] = -nom_x[14] + true_x[14];
   out_5884406445564509642[15] = -nom_x[15] + true_x[15];
   out_5884406445564509642[16] = -nom_x[16] + true_x[16];
   out_5884406445564509642[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_8259132217396944238) {
   out_8259132217396944238[0] = 1.0;
   out_8259132217396944238[1] = 0;
   out_8259132217396944238[2] = 0;
   out_8259132217396944238[3] = 0;
   out_8259132217396944238[4] = 0;
   out_8259132217396944238[5] = 0;
   out_8259132217396944238[6] = 0;
   out_8259132217396944238[7] = 0;
   out_8259132217396944238[8] = 0;
   out_8259132217396944238[9] = 0;
   out_8259132217396944238[10] = 0;
   out_8259132217396944238[11] = 0;
   out_8259132217396944238[12] = 0;
   out_8259132217396944238[13] = 0;
   out_8259132217396944238[14] = 0;
   out_8259132217396944238[15] = 0;
   out_8259132217396944238[16] = 0;
   out_8259132217396944238[17] = 0;
   out_8259132217396944238[18] = 0;
   out_8259132217396944238[19] = 1.0;
   out_8259132217396944238[20] = 0;
   out_8259132217396944238[21] = 0;
   out_8259132217396944238[22] = 0;
   out_8259132217396944238[23] = 0;
   out_8259132217396944238[24] = 0;
   out_8259132217396944238[25] = 0;
   out_8259132217396944238[26] = 0;
   out_8259132217396944238[27] = 0;
   out_8259132217396944238[28] = 0;
   out_8259132217396944238[29] = 0;
   out_8259132217396944238[30] = 0;
   out_8259132217396944238[31] = 0;
   out_8259132217396944238[32] = 0;
   out_8259132217396944238[33] = 0;
   out_8259132217396944238[34] = 0;
   out_8259132217396944238[35] = 0;
   out_8259132217396944238[36] = 0;
   out_8259132217396944238[37] = 0;
   out_8259132217396944238[38] = 1.0;
   out_8259132217396944238[39] = 0;
   out_8259132217396944238[40] = 0;
   out_8259132217396944238[41] = 0;
   out_8259132217396944238[42] = 0;
   out_8259132217396944238[43] = 0;
   out_8259132217396944238[44] = 0;
   out_8259132217396944238[45] = 0;
   out_8259132217396944238[46] = 0;
   out_8259132217396944238[47] = 0;
   out_8259132217396944238[48] = 0;
   out_8259132217396944238[49] = 0;
   out_8259132217396944238[50] = 0;
   out_8259132217396944238[51] = 0;
   out_8259132217396944238[52] = 0;
   out_8259132217396944238[53] = 0;
   out_8259132217396944238[54] = 0;
   out_8259132217396944238[55] = 0;
   out_8259132217396944238[56] = 0;
   out_8259132217396944238[57] = 1.0;
   out_8259132217396944238[58] = 0;
   out_8259132217396944238[59] = 0;
   out_8259132217396944238[60] = 0;
   out_8259132217396944238[61] = 0;
   out_8259132217396944238[62] = 0;
   out_8259132217396944238[63] = 0;
   out_8259132217396944238[64] = 0;
   out_8259132217396944238[65] = 0;
   out_8259132217396944238[66] = 0;
   out_8259132217396944238[67] = 0;
   out_8259132217396944238[68] = 0;
   out_8259132217396944238[69] = 0;
   out_8259132217396944238[70] = 0;
   out_8259132217396944238[71] = 0;
   out_8259132217396944238[72] = 0;
   out_8259132217396944238[73] = 0;
   out_8259132217396944238[74] = 0;
   out_8259132217396944238[75] = 0;
   out_8259132217396944238[76] = 1.0;
   out_8259132217396944238[77] = 0;
   out_8259132217396944238[78] = 0;
   out_8259132217396944238[79] = 0;
   out_8259132217396944238[80] = 0;
   out_8259132217396944238[81] = 0;
   out_8259132217396944238[82] = 0;
   out_8259132217396944238[83] = 0;
   out_8259132217396944238[84] = 0;
   out_8259132217396944238[85] = 0;
   out_8259132217396944238[86] = 0;
   out_8259132217396944238[87] = 0;
   out_8259132217396944238[88] = 0;
   out_8259132217396944238[89] = 0;
   out_8259132217396944238[90] = 0;
   out_8259132217396944238[91] = 0;
   out_8259132217396944238[92] = 0;
   out_8259132217396944238[93] = 0;
   out_8259132217396944238[94] = 0;
   out_8259132217396944238[95] = 1.0;
   out_8259132217396944238[96] = 0;
   out_8259132217396944238[97] = 0;
   out_8259132217396944238[98] = 0;
   out_8259132217396944238[99] = 0;
   out_8259132217396944238[100] = 0;
   out_8259132217396944238[101] = 0;
   out_8259132217396944238[102] = 0;
   out_8259132217396944238[103] = 0;
   out_8259132217396944238[104] = 0;
   out_8259132217396944238[105] = 0;
   out_8259132217396944238[106] = 0;
   out_8259132217396944238[107] = 0;
   out_8259132217396944238[108] = 0;
   out_8259132217396944238[109] = 0;
   out_8259132217396944238[110] = 0;
   out_8259132217396944238[111] = 0;
   out_8259132217396944238[112] = 0;
   out_8259132217396944238[113] = 0;
   out_8259132217396944238[114] = 1.0;
   out_8259132217396944238[115] = 0;
   out_8259132217396944238[116] = 0;
   out_8259132217396944238[117] = 0;
   out_8259132217396944238[118] = 0;
   out_8259132217396944238[119] = 0;
   out_8259132217396944238[120] = 0;
   out_8259132217396944238[121] = 0;
   out_8259132217396944238[122] = 0;
   out_8259132217396944238[123] = 0;
   out_8259132217396944238[124] = 0;
   out_8259132217396944238[125] = 0;
   out_8259132217396944238[126] = 0;
   out_8259132217396944238[127] = 0;
   out_8259132217396944238[128] = 0;
   out_8259132217396944238[129] = 0;
   out_8259132217396944238[130] = 0;
   out_8259132217396944238[131] = 0;
   out_8259132217396944238[132] = 0;
   out_8259132217396944238[133] = 1.0;
   out_8259132217396944238[134] = 0;
   out_8259132217396944238[135] = 0;
   out_8259132217396944238[136] = 0;
   out_8259132217396944238[137] = 0;
   out_8259132217396944238[138] = 0;
   out_8259132217396944238[139] = 0;
   out_8259132217396944238[140] = 0;
   out_8259132217396944238[141] = 0;
   out_8259132217396944238[142] = 0;
   out_8259132217396944238[143] = 0;
   out_8259132217396944238[144] = 0;
   out_8259132217396944238[145] = 0;
   out_8259132217396944238[146] = 0;
   out_8259132217396944238[147] = 0;
   out_8259132217396944238[148] = 0;
   out_8259132217396944238[149] = 0;
   out_8259132217396944238[150] = 0;
   out_8259132217396944238[151] = 0;
   out_8259132217396944238[152] = 1.0;
   out_8259132217396944238[153] = 0;
   out_8259132217396944238[154] = 0;
   out_8259132217396944238[155] = 0;
   out_8259132217396944238[156] = 0;
   out_8259132217396944238[157] = 0;
   out_8259132217396944238[158] = 0;
   out_8259132217396944238[159] = 0;
   out_8259132217396944238[160] = 0;
   out_8259132217396944238[161] = 0;
   out_8259132217396944238[162] = 0;
   out_8259132217396944238[163] = 0;
   out_8259132217396944238[164] = 0;
   out_8259132217396944238[165] = 0;
   out_8259132217396944238[166] = 0;
   out_8259132217396944238[167] = 0;
   out_8259132217396944238[168] = 0;
   out_8259132217396944238[169] = 0;
   out_8259132217396944238[170] = 0;
   out_8259132217396944238[171] = 1.0;
   out_8259132217396944238[172] = 0;
   out_8259132217396944238[173] = 0;
   out_8259132217396944238[174] = 0;
   out_8259132217396944238[175] = 0;
   out_8259132217396944238[176] = 0;
   out_8259132217396944238[177] = 0;
   out_8259132217396944238[178] = 0;
   out_8259132217396944238[179] = 0;
   out_8259132217396944238[180] = 0;
   out_8259132217396944238[181] = 0;
   out_8259132217396944238[182] = 0;
   out_8259132217396944238[183] = 0;
   out_8259132217396944238[184] = 0;
   out_8259132217396944238[185] = 0;
   out_8259132217396944238[186] = 0;
   out_8259132217396944238[187] = 0;
   out_8259132217396944238[188] = 0;
   out_8259132217396944238[189] = 0;
   out_8259132217396944238[190] = 1.0;
   out_8259132217396944238[191] = 0;
   out_8259132217396944238[192] = 0;
   out_8259132217396944238[193] = 0;
   out_8259132217396944238[194] = 0;
   out_8259132217396944238[195] = 0;
   out_8259132217396944238[196] = 0;
   out_8259132217396944238[197] = 0;
   out_8259132217396944238[198] = 0;
   out_8259132217396944238[199] = 0;
   out_8259132217396944238[200] = 0;
   out_8259132217396944238[201] = 0;
   out_8259132217396944238[202] = 0;
   out_8259132217396944238[203] = 0;
   out_8259132217396944238[204] = 0;
   out_8259132217396944238[205] = 0;
   out_8259132217396944238[206] = 0;
   out_8259132217396944238[207] = 0;
   out_8259132217396944238[208] = 0;
   out_8259132217396944238[209] = 1.0;
   out_8259132217396944238[210] = 0;
   out_8259132217396944238[211] = 0;
   out_8259132217396944238[212] = 0;
   out_8259132217396944238[213] = 0;
   out_8259132217396944238[214] = 0;
   out_8259132217396944238[215] = 0;
   out_8259132217396944238[216] = 0;
   out_8259132217396944238[217] = 0;
   out_8259132217396944238[218] = 0;
   out_8259132217396944238[219] = 0;
   out_8259132217396944238[220] = 0;
   out_8259132217396944238[221] = 0;
   out_8259132217396944238[222] = 0;
   out_8259132217396944238[223] = 0;
   out_8259132217396944238[224] = 0;
   out_8259132217396944238[225] = 0;
   out_8259132217396944238[226] = 0;
   out_8259132217396944238[227] = 0;
   out_8259132217396944238[228] = 1.0;
   out_8259132217396944238[229] = 0;
   out_8259132217396944238[230] = 0;
   out_8259132217396944238[231] = 0;
   out_8259132217396944238[232] = 0;
   out_8259132217396944238[233] = 0;
   out_8259132217396944238[234] = 0;
   out_8259132217396944238[235] = 0;
   out_8259132217396944238[236] = 0;
   out_8259132217396944238[237] = 0;
   out_8259132217396944238[238] = 0;
   out_8259132217396944238[239] = 0;
   out_8259132217396944238[240] = 0;
   out_8259132217396944238[241] = 0;
   out_8259132217396944238[242] = 0;
   out_8259132217396944238[243] = 0;
   out_8259132217396944238[244] = 0;
   out_8259132217396944238[245] = 0;
   out_8259132217396944238[246] = 0;
   out_8259132217396944238[247] = 1.0;
   out_8259132217396944238[248] = 0;
   out_8259132217396944238[249] = 0;
   out_8259132217396944238[250] = 0;
   out_8259132217396944238[251] = 0;
   out_8259132217396944238[252] = 0;
   out_8259132217396944238[253] = 0;
   out_8259132217396944238[254] = 0;
   out_8259132217396944238[255] = 0;
   out_8259132217396944238[256] = 0;
   out_8259132217396944238[257] = 0;
   out_8259132217396944238[258] = 0;
   out_8259132217396944238[259] = 0;
   out_8259132217396944238[260] = 0;
   out_8259132217396944238[261] = 0;
   out_8259132217396944238[262] = 0;
   out_8259132217396944238[263] = 0;
   out_8259132217396944238[264] = 0;
   out_8259132217396944238[265] = 0;
   out_8259132217396944238[266] = 1.0;
   out_8259132217396944238[267] = 0;
   out_8259132217396944238[268] = 0;
   out_8259132217396944238[269] = 0;
   out_8259132217396944238[270] = 0;
   out_8259132217396944238[271] = 0;
   out_8259132217396944238[272] = 0;
   out_8259132217396944238[273] = 0;
   out_8259132217396944238[274] = 0;
   out_8259132217396944238[275] = 0;
   out_8259132217396944238[276] = 0;
   out_8259132217396944238[277] = 0;
   out_8259132217396944238[278] = 0;
   out_8259132217396944238[279] = 0;
   out_8259132217396944238[280] = 0;
   out_8259132217396944238[281] = 0;
   out_8259132217396944238[282] = 0;
   out_8259132217396944238[283] = 0;
   out_8259132217396944238[284] = 0;
   out_8259132217396944238[285] = 1.0;
   out_8259132217396944238[286] = 0;
   out_8259132217396944238[287] = 0;
   out_8259132217396944238[288] = 0;
   out_8259132217396944238[289] = 0;
   out_8259132217396944238[290] = 0;
   out_8259132217396944238[291] = 0;
   out_8259132217396944238[292] = 0;
   out_8259132217396944238[293] = 0;
   out_8259132217396944238[294] = 0;
   out_8259132217396944238[295] = 0;
   out_8259132217396944238[296] = 0;
   out_8259132217396944238[297] = 0;
   out_8259132217396944238[298] = 0;
   out_8259132217396944238[299] = 0;
   out_8259132217396944238[300] = 0;
   out_8259132217396944238[301] = 0;
   out_8259132217396944238[302] = 0;
   out_8259132217396944238[303] = 0;
   out_8259132217396944238[304] = 1.0;
   out_8259132217396944238[305] = 0;
   out_8259132217396944238[306] = 0;
   out_8259132217396944238[307] = 0;
   out_8259132217396944238[308] = 0;
   out_8259132217396944238[309] = 0;
   out_8259132217396944238[310] = 0;
   out_8259132217396944238[311] = 0;
   out_8259132217396944238[312] = 0;
   out_8259132217396944238[313] = 0;
   out_8259132217396944238[314] = 0;
   out_8259132217396944238[315] = 0;
   out_8259132217396944238[316] = 0;
   out_8259132217396944238[317] = 0;
   out_8259132217396944238[318] = 0;
   out_8259132217396944238[319] = 0;
   out_8259132217396944238[320] = 0;
   out_8259132217396944238[321] = 0;
   out_8259132217396944238[322] = 0;
   out_8259132217396944238[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_2909476989004137244) {
   out_2909476989004137244[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_2909476989004137244[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_2909476989004137244[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_2909476989004137244[3] = dt*state[12] + state[3];
   out_2909476989004137244[4] = dt*state[13] + state[4];
   out_2909476989004137244[5] = dt*state[14] + state[5];
   out_2909476989004137244[6] = state[6];
   out_2909476989004137244[7] = state[7];
   out_2909476989004137244[8] = state[8];
   out_2909476989004137244[9] = state[9];
   out_2909476989004137244[10] = state[10];
   out_2909476989004137244[11] = state[11];
   out_2909476989004137244[12] = state[12];
   out_2909476989004137244[13] = state[13];
   out_2909476989004137244[14] = state[14];
   out_2909476989004137244[15] = state[15];
   out_2909476989004137244[16] = state[16];
   out_2909476989004137244[17] = state[17];
}
void F_fun(double *state, double dt, double *out_7984182452575755119) {
   out_7984182452575755119[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7984182452575755119[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7984182452575755119[2] = 0;
   out_7984182452575755119[3] = 0;
   out_7984182452575755119[4] = 0;
   out_7984182452575755119[5] = 0;
   out_7984182452575755119[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7984182452575755119[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7984182452575755119[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7984182452575755119[9] = 0;
   out_7984182452575755119[10] = 0;
   out_7984182452575755119[11] = 0;
   out_7984182452575755119[12] = 0;
   out_7984182452575755119[13] = 0;
   out_7984182452575755119[14] = 0;
   out_7984182452575755119[15] = 0;
   out_7984182452575755119[16] = 0;
   out_7984182452575755119[17] = 0;
   out_7984182452575755119[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_7984182452575755119[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_7984182452575755119[20] = 0;
   out_7984182452575755119[21] = 0;
   out_7984182452575755119[22] = 0;
   out_7984182452575755119[23] = 0;
   out_7984182452575755119[24] = 0;
   out_7984182452575755119[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_7984182452575755119[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_7984182452575755119[27] = 0;
   out_7984182452575755119[28] = 0;
   out_7984182452575755119[29] = 0;
   out_7984182452575755119[30] = 0;
   out_7984182452575755119[31] = 0;
   out_7984182452575755119[32] = 0;
   out_7984182452575755119[33] = 0;
   out_7984182452575755119[34] = 0;
   out_7984182452575755119[35] = 0;
   out_7984182452575755119[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7984182452575755119[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7984182452575755119[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7984182452575755119[39] = 0;
   out_7984182452575755119[40] = 0;
   out_7984182452575755119[41] = 0;
   out_7984182452575755119[42] = 0;
   out_7984182452575755119[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7984182452575755119[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7984182452575755119[45] = 0;
   out_7984182452575755119[46] = 0;
   out_7984182452575755119[47] = 0;
   out_7984182452575755119[48] = 0;
   out_7984182452575755119[49] = 0;
   out_7984182452575755119[50] = 0;
   out_7984182452575755119[51] = 0;
   out_7984182452575755119[52] = 0;
   out_7984182452575755119[53] = 0;
   out_7984182452575755119[54] = 0;
   out_7984182452575755119[55] = 0;
   out_7984182452575755119[56] = 0;
   out_7984182452575755119[57] = 1;
   out_7984182452575755119[58] = 0;
   out_7984182452575755119[59] = 0;
   out_7984182452575755119[60] = 0;
   out_7984182452575755119[61] = 0;
   out_7984182452575755119[62] = 0;
   out_7984182452575755119[63] = 0;
   out_7984182452575755119[64] = 0;
   out_7984182452575755119[65] = 0;
   out_7984182452575755119[66] = dt;
   out_7984182452575755119[67] = 0;
   out_7984182452575755119[68] = 0;
   out_7984182452575755119[69] = 0;
   out_7984182452575755119[70] = 0;
   out_7984182452575755119[71] = 0;
   out_7984182452575755119[72] = 0;
   out_7984182452575755119[73] = 0;
   out_7984182452575755119[74] = 0;
   out_7984182452575755119[75] = 0;
   out_7984182452575755119[76] = 1;
   out_7984182452575755119[77] = 0;
   out_7984182452575755119[78] = 0;
   out_7984182452575755119[79] = 0;
   out_7984182452575755119[80] = 0;
   out_7984182452575755119[81] = 0;
   out_7984182452575755119[82] = 0;
   out_7984182452575755119[83] = 0;
   out_7984182452575755119[84] = 0;
   out_7984182452575755119[85] = dt;
   out_7984182452575755119[86] = 0;
   out_7984182452575755119[87] = 0;
   out_7984182452575755119[88] = 0;
   out_7984182452575755119[89] = 0;
   out_7984182452575755119[90] = 0;
   out_7984182452575755119[91] = 0;
   out_7984182452575755119[92] = 0;
   out_7984182452575755119[93] = 0;
   out_7984182452575755119[94] = 0;
   out_7984182452575755119[95] = 1;
   out_7984182452575755119[96] = 0;
   out_7984182452575755119[97] = 0;
   out_7984182452575755119[98] = 0;
   out_7984182452575755119[99] = 0;
   out_7984182452575755119[100] = 0;
   out_7984182452575755119[101] = 0;
   out_7984182452575755119[102] = 0;
   out_7984182452575755119[103] = 0;
   out_7984182452575755119[104] = dt;
   out_7984182452575755119[105] = 0;
   out_7984182452575755119[106] = 0;
   out_7984182452575755119[107] = 0;
   out_7984182452575755119[108] = 0;
   out_7984182452575755119[109] = 0;
   out_7984182452575755119[110] = 0;
   out_7984182452575755119[111] = 0;
   out_7984182452575755119[112] = 0;
   out_7984182452575755119[113] = 0;
   out_7984182452575755119[114] = 1;
   out_7984182452575755119[115] = 0;
   out_7984182452575755119[116] = 0;
   out_7984182452575755119[117] = 0;
   out_7984182452575755119[118] = 0;
   out_7984182452575755119[119] = 0;
   out_7984182452575755119[120] = 0;
   out_7984182452575755119[121] = 0;
   out_7984182452575755119[122] = 0;
   out_7984182452575755119[123] = 0;
   out_7984182452575755119[124] = 0;
   out_7984182452575755119[125] = 0;
   out_7984182452575755119[126] = 0;
   out_7984182452575755119[127] = 0;
   out_7984182452575755119[128] = 0;
   out_7984182452575755119[129] = 0;
   out_7984182452575755119[130] = 0;
   out_7984182452575755119[131] = 0;
   out_7984182452575755119[132] = 0;
   out_7984182452575755119[133] = 1;
   out_7984182452575755119[134] = 0;
   out_7984182452575755119[135] = 0;
   out_7984182452575755119[136] = 0;
   out_7984182452575755119[137] = 0;
   out_7984182452575755119[138] = 0;
   out_7984182452575755119[139] = 0;
   out_7984182452575755119[140] = 0;
   out_7984182452575755119[141] = 0;
   out_7984182452575755119[142] = 0;
   out_7984182452575755119[143] = 0;
   out_7984182452575755119[144] = 0;
   out_7984182452575755119[145] = 0;
   out_7984182452575755119[146] = 0;
   out_7984182452575755119[147] = 0;
   out_7984182452575755119[148] = 0;
   out_7984182452575755119[149] = 0;
   out_7984182452575755119[150] = 0;
   out_7984182452575755119[151] = 0;
   out_7984182452575755119[152] = 1;
   out_7984182452575755119[153] = 0;
   out_7984182452575755119[154] = 0;
   out_7984182452575755119[155] = 0;
   out_7984182452575755119[156] = 0;
   out_7984182452575755119[157] = 0;
   out_7984182452575755119[158] = 0;
   out_7984182452575755119[159] = 0;
   out_7984182452575755119[160] = 0;
   out_7984182452575755119[161] = 0;
   out_7984182452575755119[162] = 0;
   out_7984182452575755119[163] = 0;
   out_7984182452575755119[164] = 0;
   out_7984182452575755119[165] = 0;
   out_7984182452575755119[166] = 0;
   out_7984182452575755119[167] = 0;
   out_7984182452575755119[168] = 0;
   out_7984182452575755119[169] = 0;
   out_7984182452575755119[170] = 0;
   out_7984182452575755119[171] = 1;
   out_7984182452575755119[172] = 0;
   out_7984182452575755119[173] = 0;
   out_7984182452575755119[174] = 0;
   out_7984182452575755119[175] = 0;
   out_7984182452575755119[176] = 0;
   out_7984182452575755119[177] = 0;
   out_7984182452575755119[178] = 0;
   out_7984182452575755119[179] = 0;
   out_7984182452575755119[180] = 0;
   out_7984182452575755119[181] = 0;
   out_7984182452575755119[182] = 0;
   out_7984182452575755119[183] = 0;
   out_7984182452575755119[184] = 0;
   out_7984182452575755119[185] = 0;
   out_7984182452575755119[186] = 0;
   out_7984182452575755119[187] = 0;
   out_7984182452575755119[188] = 0;
   out_7984182452575755119[189] = 0;
   out_7984182452575755119[190] = 1;
   out_7984182452575755119[191] = 0;
   out_7984182452575755119[192] = 0;
   out_7984182452575755119[193] = 0;
   out_7984182452575755119[194] = 0;
   out_7984182452575755119[195] = 0;
   out_7984182452575755119[196] = 0;
   out_7984182452575755119[197] = 0;
   out_7984182452575755119[198] = 0;
   out_7984182452575755119[199] = 0;
   out_7984182452575755119[200] = 0;
   out_7984182452575755119[201] = 0;
   out_7984182452575755119[202] = 0;
   out_7984182452575755119[203] = 0;
   out_7984182452575755119[204] = 0;
   out_7984182452575755119[205] = 0;
   out_7984182452575755119[206] = 0;
   out_7984182452575755119[207] = 0;
   out_7984182452575755119[208] = 0;
   out_7984182452575755119[209] = 1;
   out_7984182452575755119[210] = 0;
   out_7984182452575755119[211] = 0;
   out_7984182452575755119[212] = 0;
   out_7984182452575755119[213] = 0;
   out_7984182452575755119[214] = 0;
   out_7984182452575755119[215] = 0;
   out_7984182452575755119[216] = 0;
   out_7984182452575755119[217] = 0;
   out_7984182452575755119[218] = 0;
   out_7984182452575755119[219] = 0;
   out_7984182452575755119[220] = 0;
   out_7984182452575755119[221] = 0;
   out_7984182452575755119[222] = 0;
   out_7984182452575755119[223] = 0;
   out_7984182452575755119[224] = 0;
   out_7984182452575755119[225] = 0;
   out_7984182452575755119[226] = 0;
   out_7984182452575755119[227] = 0;
   out_7984182452575755119[228] = 1;
   out_7984182452575755119[229] = 0;
   out_7984182452575755119[230] = 0;
   out_7984182452575755119[231] = 0;
   out_7984182452575755119[232] = 0;
   out_7984182452575755119[233] = 0;
   out_7984182452575755119[234] = 0;
   out_7984182452575755119[235] = 0;
   out_7984182452575755119[236] = 0;
   out_7984182452575755119[237] = 0;
   out_7984182452575755119[238] = 0;
   out_7984182452575755119[239] = 0;
   out_7984182452575755119[240] = 0;
   out_7984182452575755119[241] = 0;
   out_7984182452575755119[242] = 0;
   out_7984182452575755119[243] = 0;
   out_7984182452575755119[244] = 0;
   out_7984182452575755119[245] = 0;
   out_7984182452575755119[246] = 0;
   out_7984182452575755119[247] = 1;
   out_7984182452575755119[248] = 0;
   out_7984182452575755119[249] = 0;
   out_7984182452575755119[250] = 0;
   out_7984182452575755119[251] = 0;
   out_7984182452575755119[252] = 0;
   out_7984182452575755119[253] = 0;
   out_7984182452575755119[254] = 0;
   out_7984182452575755119[255] = 0;
   out_7984182452575755119[256] = 0;
   out_7984182452575755119[257] = 0;
   out_7984182452575755119[258] = 0;
   out_7984182452575755119[259] = 0;
   out_7984182452575755119[260] = 0;
   out_7984182452575755119[261] = 0;
   out_7984182452575755119[262] = 0;
   out_7984182452575755119[263] = 0;
   out_7984182452575755119[264] = 0;
   out_7984182452575755119[265] = 0;
   out_7984182452575755119[266] = 1;
   out_7984182452575755119[267] = 0;
   out_7984182452575755119[268] = 0;
   out_7984182452575755119[269] = 0;
   out_7984182452575755119[270] = 0;
   out_7984182452575755119[271] = 0;
   out_7984182452575755119[272] = 0;
   out_7984182452575755119[273] = 0;
   out_7984182452575755119[274] = 0;
   out_7984182452575755119[275] = 0;
   out_7984182452575755119[276] = 0;
   out_7984182452575755119[277] = 0;
   out_7984182452575755119[278] = 0;
   out_7984182452575755119[279] = 0;
   out_7984182452575755119[280] = 0;
   out_7984182452575755119[281] = 0;
   out_7984182452575755119[282] = 0;
   out_7984182452575755119[283] = 0;
   out_7984182452575755119[284] = 0;
   out_7984182452575755119[285] = 1;
   out_7984182452575755119[286] = 0;
   out_7984182452575755119[287] = 0;
   out_7984182452575755119[288] = 0;
   out_7984182452575755119[289] = 0;
   out_7984182452575755119[290] = 0;
   out_7984182452575755119[291] = 0;
   out_7984182452575755119[292] = 0;
   out_7984182452575755119[293] = 0;
   out_7984182452575755119[294] = 0;
   out_7984182452575755119[295] = 0;
   out_7984182452575755119[296] = 0;
   out_7984182452575755119[297] = 0;
   out_7984182452575755119[298] = 0;
   out_7984182452575755119[299] = 0;
   out_7984182452575755119[300] = 0;
   out_7984182452575755119[301] = 0;
   out_7984182452575755119[302] = 0;
   out_7984182452575755119[303] = 0;
   out_7984182452575755119[304] = 1;
   out_7984182452575755119[305] = 0;
   out_7984182452575755119[306] = 0;
   out_7984182452575755119[307] = 0;
   out_7984182452575755119[308] = 0;
   out_7984182452575755119[309] = 0;
   out_7984182452575755119[310] = 0;
   out_7984182452575755119[311] = 0;
   out_7984182452575755119[312] = 0;
   out_7984182452575755119[313] = 0;
   out_7984182452575755119[314] = 0;
   out_7984182452575755119[315] = 0;
   out_7984182452575755119[316] = 0;
   out_7984182452575755119[317] = 0;
   out_7984182452575755119[318] = 0;
   out_7984182452575755119[319] = 0;
   out_7984182452575755119[320] = 0;
   out_7984182452575755119[321] = 0;
   out_7984182452575755119[322] = 0;
   out_7984182452575755119[323] = 1;
}
void h_4(double *state, double *unused, double *out_9175407422316684506) {
   out_9175407422316684506[0] = state[6] + state[9];
   out_9175407422316684506[1] = state[7] + state[10];
   out_9175407422316684506[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_6069539240583237125) {
   out_6069539240583237125[0] = 0;
   out_6069539240583237125[1] = 0;
   out_6069539240583237125[2] = 0;
   out_6069539240583237125[3] = 0;
   out_6069539240583237125[4] = 0;
   out_6069539240583237125[5] = 0;
   out_6069539240583237125[6] = 1;
   out_6069539240583237125[7] = 0;
   out_6069539240583237125[8] = 0;
   out_6069539240583237125[9] = 1;
   out_6069539240583237125[10] = 0;
   out_6069539240583237125[11] = 0;
   out_6069539240583237125[12] = 0;
   out_6069539240583237125[13] = 0;
   out_6069539240583237125[14] = 0;
   out_6069539240583237125[15] = 0;
   out_6069539240583237125[16] = 0;
   out_6069539240583237125[17] = 0;
   out_6069539240583237125[18] = 0;
   out_6069539240583237125[19] = 0;
   out_6069539240583237125[20] = 0;
   out_6069539240583237125[21] = 0;
   out_6069539240583237125[22] = 0;
   out_6069539240583237125[23] = 0;
   out_6069539240583237125[24] = 0;
   out_6069539240583237125[25] = 1;
   out_6069539240583237125[26] = 0;
   out_6069539240583237125[27] = 0;
   out_6069539240583237125[28] = 1;
   out_6069539240583237125[29] = 0;
   out_6069539240583237125[30] = 0;
   out_6069539240583237125[31] = 0;
   out_6069539240583237125[32] = 0;
   out_6069539240583237125[33] = 0;
   out_6069539240583237125[34] = 0;
   out_6069539240583237125[35] = 0;
   out_6069539240583237125[36] = 0;
   out_6069539240583237125[37] = 0;
   out_6069539240583237125[38] = 0;
   out_6069539240583237125[39] = 0;
   out_6069539240583237125[40] = 0;
   out_6069539240583237125[41] = 0;
   out_6069539240583237125[42] = 0;
   out_6069539240583237125[43] = 0;
   out_6069539240583237125[44] = 1;
   out_6069539240583237125[45] = 0;
   out_6069539240583237125[46] = 0;
   out_6069539240583237125[47] = 1;
   out_6069539240583237125[48] = 0;
   out_6069539240583237125[49] = 0;
   out_6069539240583237125[50] = 0;
   out_6069539240583237125[51] = 0;
   out_6069539240583237125[52] = 0;
   out_6069539240583237125[53] = 0;
}
void h_10(double *state, double *unused, double *out_8641600486548509028) {
   out_8641600486548509028[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_8641600486548509028[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_8641600486548509028[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_5292752278649831974) {
   out_5292752278649831974[0] = 0;
   out_5292752278649831974[1] = 9.8100000000000005*cos(state[1]);
   out_5292752278649831974[2] = 0;
   out_5292752278649831974[3] = 0;
   out_5292752278649831974[4] = -state[8];
   out_5292752278649831974[5] = state[7];
   out_5292752278649831974[6] = 0;
   out_5292752278649831974[7] = state[5];
   out_5292752278649831974[8] = -state[4];
   out_5292752278649831974[9] = 0;
   out_5292752278649831974[10] = 0;
   out_5292752278649831974[11] = 0;
   out_5292752278649831974[12] = 1;
   out_5292752278649831974[13] = 0;
   out_5292752278649831974[14] = 0;
   out_5292752278649831974[15] = 1;
   out_5292752278649831974[16] = 0;
   out_5292752278649831974[17] = 0;
   out_5292752278649831974[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_5292752278649831974[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_5292752278649831974[20] = 0;
   out_5292752278649831974[21] = state[8];
   out_5292752278649831974[22] = 0;
   out_5292752278649831974[23] = -state[6];
   out_5292752278649831974[24] = -state[5];
   out_5292752278649831974[25] = 0;
   out_5292752278649831974[26] = state[3];
   out_5292752278649831974[27] = 0;
   out_5292752278649831974[28] = 0;
   out_5292752278649831974[29] = 0;
   out_5292752278649831974[30] = 0;
   out_5292752278649831974[31] = 1;
   out_5292752278649831974[32] = 0;
   out_5292752278649831974[33] = 0;
   out_5292752278649831974[34] = 1;
   out_5292752278649831974[35] = 0;
   out_5292752278649831974[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_5292752278649831974[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_5292752278649831974[38] = 0;
   out_5292752278649831974[39] = -state[7];
   out_5292752278649831974[40] = state[6];
   out_5292752278649831974[41] = 0;
   out_5292752278649831974[42] = state[4];
   out_5292752278649831974[43] = -state[3];
   out_5292752278649831974[44] = 0;
   out_5292752278649831974[45] = 0;
   out_5292752278649831974[46] = 0;
   out_5292752278649831974[47] = 0;
   out_5292752278649831974[48] = 0;
   out_5292752278649831974[49] = 0;
   out_5292752278649831974[50] = 1;
   out_5292752278649831974[51] = 0;
   out_5292752278649831974[52] = 0;
   out_5292752278649831974[53] = 1;
}
void h_13(double *state, double *unused, double *out_878346494641374765) {
   out_878346494641374765[0] = state[3];
   out_878346494641374765[1] = state[4];
   out_878346494641374765[2] = state[5];
}
void H_13(double *state, double *unused, double *out_2857265415250904324) {
   out_2857265415250904324[0] = 0;
   out_2857265415250904324[1] = 0;
   out_2857265415250904324[2] = 0;
   out_2857265415250904324[3] = 1;
   out_2857265415250904324[4] = 0;
   out_2857265415250904324[5] = 0;
   out_2857265415250904324[6] = 0;
   out_2857265415250904324[7] = 0;
   out_2857265415250904324[8] = 0;
   out_2857265415250904324[9] = 0;
   out_2857265415250904324[10] = 0;
   out_2857265415250904324[11] = 0;
   out_2857265415250904324[12] = 0;
   out_2857265415250904324[13] = 0;
   out_2857265415250904324[14] = 0;
   out_2857265415250904324[15] = 0;
   out_2857265415250904324[16] = 0;
   out_2857265415250904324[17] = 0;
   out_2857265415250904324[18] = 0;
   out_2857265415250904324[19] = 0;
   out_2857265415250904324[20] = 0;
   out_2857265415250904324[21] = 0;
   out_2857265415250904324[22] = 1;
   out_2857265415250904324[23] = 0;
   out_2857265415250904324[24] = 0;
   out_2857265415250904324[25] = 0;
   out_2857265415250904324[26] = 0;
   out_2857265415250904324[27] = 0;
   out_2857265415250904324[28] = 0;
   out_2857265415250904324[29] = 0;
   out_2857265415250904324[30] = 0;
   out_2857265415250904324[31] = 0;
   out_2857265415250904324[32] = 0;
   out_2857265415250904324[33] = 0;
   out_2857265415250904324[34] = 0;
   out_2857265415250904324[35] = 0;
   out_2857265415250904324[36] = 0;
   out_2857265415250904324[37] = 0;
   out_2857265415250904324[38] = 0;
   out_2857265415250904324[39] = 0;
   out_2857265415250904324[40] = 0;
   out_2857265415250904324[41] = 1;
   out_2857265415250904324[42] = 0;
   out_2857265415250904324[43] = 0;
   out_2857265415250904324[44] = 0;
   out_2857265415250904324[45] = 0;
   out_2857265415250904324[46] = 0;
   out_2857265415250904324[47] = 0;
   out_2857265415250904324[48] = 0;
   out_2857265415250904324[49] = 0;
   out_2857265415250904324[50] = 0;
   out_2857265415250904324[51] = 0;
   out_2857265415250904324[52] = 0;
   out_2857265415250904324[53] = 0;
}
void h_14(double *state, double *unused, double *out_7346852850300827949) {
   out_7346852850300827949[0] = state[6];
   out_7346852850300827949[1] = state[7];
   out_7346852850300827949[2] = state[8];
}
void H_14(double *state, double *unused, double *out_2106298384243752596) {
   out_2106298384243752596[0] = 0;
   out_2106298384243752596[1] = 0;
   out_2106298384243752596[2] = 0;
   out_2106298384243752596[3] = 0;
   out_2106298384243752596[4] = 0;
   out_2106298384243752596[5] = 0;
   out_2106298384243752596[6] = 1;
   out_2106298384243752596[7] = 0;
   out_2106298384243752596[8] = 0;
   out_2106298384243752596[9] = 0;
   out_2106298384243752596[10] = 0;
   out_2106298384243752596[11] = 0;
   out_2106298384243752596[12] = 0;
   out_2106298384243752596[13] = 0;
   out_2106298384243752596[14] = 0;
   out_2106298384243752596[15] = 0;
   out_2106298384243752596[16] = 0;
   out_2106298384243752596[17] = 0;
   out_2106298384243752596[18] = 0;
   out_2106298384243752596[19] = 0;
   out_2106298384243752596[20] = 0;
   out_2106298384243752596[21] = 0;
   out_2106298384243752596[22] = 0;
   out_2106298384243752596[23] = 0;
   out_2106298384243752596[24] = 0;
   out_2106298384243752596[25] = 1;
   out_2106298384243752596[26] = 0;
   out_2106298384243752596[27] = 0;
   out_2106298384243752596[28] = 0;
   out_2106298384243752596[29] = 0;
   out_2106298384243752596[30] = 0;
   out_2106298384243752596[31] = 0;
   out_2106298384243752596[32] = 0;
   out_2106298384243752596[33] = 0;
   out_2106298384243752596[34] = 0;
   out_2106298384243752596[35] = 0;
   out_2106298384243752596[36] = 0;
   out_2106298384243752596[37] = 0;
   out_2106298384243752596[38] = 0;
   out_2106298384243752596[39] = 0;
   out_2106298384243752596[40] = 0;
   out_2106298384243752596[41] = 0;
   out_2106298384243752596[42] = 0;
   out_2106298384243752596[43] = 0;
   out_2106298384243752596[44] = 1;
   out_2106298384243752596[45] = 0;
   out_2106298384243752596[46] = 0;
   out_2106298384243752596[47] = 0;
   out_2106298384243752596[48] = 0;
   out_2106298384243752596[49] = 0;
   out_2106298384243752596[50] = 0;
   out_2106298384243752596[51] = 0;
   out_2106298384243752596[52] = 0;
   out_2106298384243752596[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_3280080996569931798) {
  err_fun(nom_x, delta_x, out_3280080996569931798);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_5884406445564509642) {
  inv_err_fun(nom_x, true_x, out_5884406445564509642);
}
void pose_H_mod_fun(double *state, double *out_8259132217396944238) {
  H_mod_fun(state, out_8259132217396944238);
}
void pose_f_fun(double *state, double dt, double *out_2909476989004137244) {
  f_fun(state,  dt, out_2909476989004137244);
}
void pose_F_fun(double *state, double dt, double *out_7984182452575755119) {
  F_fun(state,  dt, out_7984182452575755119);
}
void pose_h_4(double *state, double *unused, double *out_9175407422316684506) {
  h_4(state, unused, out_9175407422316684506);
}
void pose_H_4(double *state, double *unused, double *out_6069539240583237125) {
  H_4(state, unused, out_6069539240583237125);
}
void pose_h_10(double *state, double *unused, double *out_8641600486548509028) {
  h_10(state, unused, out_8641600486548509028);
}
void pose_H_10(double *state, double *unused, double *out_5292752278649831974) {
  H_10(state, unused, out_5292752278649831974);
}
void pose_h_13(double *state, double *unused, double *out_878346494641374765) {
  h_13(state, unused, out_878346494641374765);
}
void pose_H_13(double *state, double *unused, double *out_2857265415250904324) {
  H_13(state, unused, out_2857265415250904324);
}
void pose_h_14(double *state, double *unused, double *out_7346852850300827949) {
  h_14(state, unused, out_7346852850300827949);
}
void pose_H_14(double *state, double *unused, double *out_2106298384243752596) {
  H_14(state, unused, out_2106298384243752596);
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
