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
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_4988085883151629357) {
   out_4988085883151629357[0] = delta_x[0] + nom_x[0];
   out_4988085883151629357[1] = delta_x[1] + nom_x[1];
   out_4988085883151629357[2] = delta_x[2] + nom_x[2];
   out_4988085883151629357[3] = delta_x[3] + nom_x[3];
   out_4988085883151629357[4] = delta_x[4] + nom_x[4];
   out_4988085883151629357[5] = delta_x[5] + nom_x[5];
   out_4988085883151629357[6] = delta_x[6] + nom_x[6];
   out_4988085883151629357[7] = delta_x[7] + nom_x[7];
   out_4988085883151629357[8] = delta_x[8] + nom_x[8];
   out_4988085883151629357[9] = delta_x[9] + nom_x[9];
   out_4988085883151629357[10] = delta_x[10] + nom_x[10];
   out_4988085883151629357[11] = delta_x[11] + nom_x[11];
   out_4988085883151629357[12] = delta_x[12] + nom_x[12];
   out_4988085883151629357[13] = delta_x[13] + nom_x[13];
   out_4988085883151629357[14] = delta_x[14] + nom_x[14];
   out_4988085883151629357[15] = delta_x[15] + nom_x[15];
   out_4988085883151629357[16] = delta_x[16] + nom_x[16];
   out_4988085883151629357[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_8628576639675182721) {
   out_8628576639675182721[0] = -nom_x[0] + true_x[0];
   out_8628576639675182721[1] = -nom_x[1] + true_x[1];
   out_8628576639675182721[2] = -nom_x[2] + true_x[2];
   out_8628576639675182721[3] = -nom_x[3] + true_x[3];
   out_8628576639675182721[4] = -nom_x[4] + true_x[4];
   out_8628576639675182721[5] = -nom_x[5] + true_x[5];
   out_8628576639675182721[6] = -nom_x[6] + true_x[6];
   out_8628576639675182721[7] = -nom_x[7] + true_x[7];
   out_8628576639675182721[8] = -nom_x[8] + true_x[8];
   out_8628576639675182721[9] = -nom_x[9] + true_x[9];
   out_8628576639675182721[10] = -nom_x[10] + true_x[10];
   out_8628576639675182721[11] = -nom_x[11] + true_x[11];
   out_8628576639675182721[12] = -nom_x[12] + true_x[12];
   out_8628576639675182721[13] = -nom_x[13] + true_x[13];
   out_8628576639675182721[14] = -nom_x[14] + true_x[14];
   out_8628576639675182721[15] = -nom_x[15] + true_x[15];
   out_8628576639675182721[16] = -nom_x[16] + true_x[16];
   out_8628576639675182721[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_844640693863841425) {
   out_844640693863841425[0] = 1.0;
   out_844640693863841425[1] = 0.0;
   out_844640693863841425[2] = 0.0;
   out_844640693863841425[3] = 0.0;
   out_844640693863841425[4] = 0.0;
   out_844640693863841425[5] = 0.0;
   out_844640693863841425[6] = 0.0;
   out_844640693863841425[7] = 0.0;
   out_844640693863841425[8] = 0.0;
   out_844640693863841425[9] = 0.0;
   out_844640693863841425[10] = 0.0;
   out_844640693863841425[11] = 0.0;
   out_844640693863841425[12] = 0.0;
   out_844640693863841425[13] = 0.0;
   out_844640693863841425[14] = 0.0;
   out_844640693863841425[15] = 0.0;
   out_844640693863841425[16] = 0.0;
   out_844640693863841425[17] = 0.0;
   out_844640693863841425[18] = 0.0;
   out_844640693863841425[19] = 1.0;
   out_844640693863841425[20] = 0.0;
   out_844640693863841425[21] = 0.0;
   out_844640693863841425[22] = 0.0;
   out_844640693863841425[23] = 0.0;
   out_844640693863841425[24] = 0.0;
   out_844640693863841425[25] = 0.0;
   out_844640693863841425[26] = 0.0;
   out_844640693863841425[27] = 0.0;
   out_844640693863841425[28] = 0.0;
   out_844640693863841425[29] = 0.0;
   out_844640693863841425[30] = 0.0;
   out_844640693863841425[31] = 0.0;
   out_844640693863841425[32] = 0.0;
   out_844640693863841425[33] = 0.0;
   out_844640693863841425[34] = 0.0;
   out_844640693863841425[35] = 0.0;
   out_844640693863841425[36] = 0.0;
   out_844640693863841425[37] = 0.0;
   out_844640693863841425[38] = 1.0;
   out_844640693863841425[39] = 0.0;
   out_844640693863841425[40] = 0.0;
   out_844640693863841425[41] = 0.0;
   out_844640693863841425[42] = 0.0;
   out_844640693863841425[43] = 0.0;
   out_844640693863841425[44] = 0.0;
   out_844640693863841425[45] = 0.0;
   out_844640693863841425[46] = 0.0;
   out_844640693863841425[47] = 0.0;
   out_844640693863841425[48] = 0.0;
   out_844640693863841425[49] = 0.0;
   out_844640693863841425[50] = 0.0;
   out_844640693863841425[51] = 0.0;
   out_844640693863841425[52] = 0.0;
   out_844640693863841425[53] = 0.0;
   out_844640693863841425[54] = 0.0;
   out_844640693863841425[55] = 0.0;
   out_844640693863841425[56] = 0.0;
   out_844640693863841425[57] = 1.0;
   out_844640693863841425[58] = 0.0;
   out_844640693863841425[59] = 0.0;
   out_844640693863841425[60] = 0.0;
   out_844640693863841425[61] = 0.0;
   out_844640693863841425[62] = 0.0;
   out_844640693863841425[63] = 0.0;
   out_844640693863841425[64] = 0.0;
   out_844640693863841425[65] = 0.0;
   out_844640693863841425[66] = 0.0;
   out_844640693863841425[67] = 0.0;
   out_844640693863841425[68] = 0.0;
   out_844640693863841425[69] = 0.0;
   out_844640693863841425[70] = 0.0;
   out_844640693863841425[71] = 0.0;
   out_844640693863841425[72] = 0.0;
   out_844640693863841425[73] = 0.0;
   out_844640693863841425[74] = 0.0;
   out_844640693863841425[75] = 0.0;
   out_844640693863841425[76] = 1.0;
   out_844640693863841425[77] = 0.0;
   out_844640693863841425[78] = 0.0;
   out_844640693863841425[79] = 0.0;
   out_844640693863841425[80] = 0.0;
   out_844640693863841425[81] = 0.0;
   out_844640693863841425[82] = 0.0;
   out_844640693863841425[83] = 0.0;
   out_844640693863841425[84] = 0.0;
   out_844640693863841425[85] = 0.0;
   out_844640693863841425[86] = 0.0;
   out_844640693863841425[87] = 0.0;
   out_844640693863841425[88] = 0.0;
   out_844640693863841425[89] = 0.0;
   out_844640693863841425[90] = 0.0;
   out_844640693863841425[91] = 0.0;
   out_844640693863841425[92] = 0.0;
   out_844640693863841425[93] = 0.0;
   out_844640693863841425[94] = 0.0;
   out_844640693863841425[95] = 1.0;
   out_844640693863841425[96] = 0.0;
   out_844640693863841425[97] = 0.0;
   out_844640693863841425[98] = 0.0;
   out_844640693863841425[99] = 0.0;
   out_844640693863841425[100] = 0.0;
   out_844640693863841425[101] = 0.0;
   out_844640693863841425[102] = 0.0;
   out_844640693863841425[103] = 0.0;
   out_844640693863841425[104] = 0.0;
   out_844640693863841425[105] = 0.0;
   out_844640693863841425[106] = 0.0;
   out_844640693863841425[107] = 0.0;
   out_844640693863841425[108] = 0.0;
   out_844640693863841425[109] = 0.0;
   out_844640693863841425[110] = 0.0;
   out_844640693863841425[111] = 0.0;
   out_844640693863841425[112] = 0.0;
   out_844640693863841425[113] = 0.0;
   out_844640693863841425[114] = 1.0;
   out_844640693863841425[115] = 0.0;
   out_844640693863841425[116] = 0.0;
   out_844640693863841425[117] = 0.0;
   out_844640693863841425[118] = 0.0;
   out_844640693863841425[119] = 0.0;
   out_844640693863841425[120] = 0.0;
   out_844640693863841425[121] = 0.0;
   out_844640693863841425[122] = 0.0;
   out_844640693863841425[123] = 0.0;
   out_844640693863841425[124] = 0.0;
   out_844640693863841425[125] = 0.0;
   out_844640693863841425[126] = 0.0;
   out_844640693863841425[127] = 0.0;
   out_844640693863841425[128] = 0.0;
   out_844640693863841425[129] = 0.0;
   out_844640693863841425[130] = 0.0;
   out_844640693863841425[131] = 0.0;
   out_844640693863841425[132] = 0.0;
   out_844640693863841425[133] = 1.0;
   out_844640693863841425[134] = 0.0;
   out_844640693863841425[135] = 0.0;
   out_844640693863841425[136] = 0.0;
   out_844640693863841425[137] = 0.0;
   out_844640693863841425[138] = 0.0;
   out_844640693863841425[139] = 0.0;
   out_844640693863841425[140] = 0.0;
   out_844640693863841425[141] = 0.0;
   out_844640693863841425[142] = 0.0;
   out_844640693863841425[143] = 0.0;
   out_844640693863841425[144] = 0.0;
   out_844640693863841425[145] = 0.0;
   out_844640693863841425[146] = 0.0;
   out_844640693863841425[147] = 0.0;
   out_844640693863841425[148] = 0.0;
   out_844640693863841425[149] = 0.0;
   out_844640693863841425[150] = 0.0;
   out_844640693863841425[151] = 0.0;
   out_844640693863841425[152] = 1.0;
   out_844640693863841425[153] = 0.0;
   out_844640693863841425[154] = 0.0;
   out_844640693863841425[155] = 0.0;
   out_844640693863841425[156] = 0.0;
   out_844640693863841425[157] = 0.0;
   out_844640693863841425[158] = 0.0;
   out_844640693863841425[159] = 0.0;
   out_844640693863841425[160] = 0.0;
   out_844640693863841425[161] = 0.0;
   out_844640693863841425[162] = 0.0;
   out_844640693863841425[163] = 0.0;
   out_844640693863841425[164] = 0.0;
   out_844640693863841425[165] = 0.0;
   out_844640693863841425[166] = 0.0;
   out_844640693863841425[167] = 0.0;
   out_844640693863841425[168] = 0.0;
   out_844640693863841425[169] = 0.0;
   out_844640693863841425[170] = 0.0;
   out_844640693863841425[171] = 1.0;
   out_844640693863841425[172] = 0.0;
   out_844640693863841425[173] = 0.0;
   out_844640693863841425[174] = 0.0;
   out_844640693863841425[175] = 0.0;
   out_844640693863841425[176] = 0.0;
   out_844640693863841425[177] = 0.0;
   out_844640693863841425[178] = 0.0;
   out_844640693863841425[179] = 0.0;
   out_844640693863841425[180] = 0.0;
   out_844640693863841425[181] = 0.0;
   out_844640693863841425[182] = 0.0;
   out_844640693863841425[183] = 0.0;
   out_844640693863841425[184] = 0.0;
   out_844640693863841425[185] = 0.0;
   out_844640693863841425[186] = 0.0;
   out_844640693863841425[187] = 0.0;
   out_844640693863841425[188] = 0.0;
   out_844640693863841425[189] = 0.0;
   out_844640693863841425[190] = 1.0;
   out_844640693863841425[191] = 0.0;
   out_844640693863841425[192] = 0.0;
   out_844640693863841425[193] = 0.0;
   out_844640693863841425[194] = 0.0;
   out_844640693863841425[195] = 0.0;
   out_844640693863841425[196] = 0.0;
   out_844640693863841425[197] = 0.0;
   out_844640693863841425[198] = 0.0;
   out_844640693863841425[199] = 0.0;
   out_844640693863841425[200] = 0.0;
   out_844640693863841425[201] = 0.0;
   out_844640693863841425[202] = 0.0;
   out_844640693863841425[203] = 0.0;
   out_844640693863841425[204] = 0.0;
   out_844640693863841425[205] = 0.0;
   out_844640693863841425[206] = 0.0;
   out_844640693863841425[207] = 0.0;
   out_844640693863841425[208] = 0.0;
   out_844640693863841425[209] = 1.0;
   out_844640693863841425[210] = 0.0;
   out_844640693863841425[211] = 0.0;
   out_844640693863841425[212] = 0.0;
   out_844640693863841425[213] = 0.0;
   out_844640693863841425[214] = 0.0;
   out_844640693863841425[215] = 0.0;
   out_844640693863841425[216] = 0.0;
   out_844640693863841425[217] = 0.0;
   out_844640693863841425[218] = 0.0;
   out_844640693863841425[219] = 0.0;
   out_844640693863841425[220] = 0.0;
   out_844640693863841425[221] = 0.0;
   out_844640693863841425[222] = 0.0;
   out_844640693863841425[223] = 0.0;
   out_844640693863841425[224] = 0.0;
   out_844640693863841425[225] = 0.0;
   out_844640693863841425[226] = 0.0;
   out_844640693863841425[227] = 0.0;
   out_844640693863841425[228] = 1.0;
   out_844640693863841425[229] = 0.0;
   out_844640693863841425[230] = 0.0;
   out_844640693863841425[231] = 0.0;
   out_844640693863841425[232] = 0.0;
   out_844640693863841425[233] = 0.0;
   out_844640693863841425[234] = 0.0;
   out_844640693863841425[235] = 0.0;
   out_844640693863841425[236] = 0.0;
   out_844640693863841425[237] = 0.0;
   out_844640693863841425[238] = 0.0;
   out_844640693863841425[239] = 0.0;
   out_844640693863841425[240] = 0.0;
   out_844640693863841425[241] = 0.0;
   out_844640693863841425[242] = 0.0;
   out_844640693863841425[243] = 0.0;
   out_844640693863841425[244] = 0.0;
   out_844640693863841425[245] = 0.0;
   out_844640693863841425[246] = 0.0;
   out_844640693863841425[247] = 1.0;
   out_844640693863841425[248] = 0.0;
   out_844640693863841425[249] = 0.0;
   out_844640693863841425[250] = 0.0;
   out_844640693863841425[251] = 0.0;
   out_844640693863841425[252] = 0.0;
   out_844640693863841425[253] = 0.0;
   out_844640693863841425[254] = 0.0;
   out_844640693863841425[255] = 0.0;
   out_844640693863841425[256] = 0.0;
   out_844640693863841425[257] = 0.0;
   out_844640693863841425[258] = 0.0;
   out_844640693863841425[259] = 0.0;
   out_844640693863841425[260] = 0.0;
   out_844640693863841425[261] = 0.0;
   out_844640693863841425[262] = 0.0;
   out_844640693863841425[263] = 0.0;
   out_844640693863841425[264] = 0.0;
   out_844640693863841425[265] = 0.0;
   out_844640693863841425[266] = 1.0;
   out_844640693863841425[267] = 0.0;
   out_844640693863841425[268] = 0.0;
   out_844640693863841425[269] = 0.0;
   out_844640693863841425[270] = 0.0;
   out_844640693863841425[271] = 0.0;
   out_844640693863841425[272] = 0.0;
   out_844640693863841425[273] = 0.0;
   out_844640693863841425[274] = 0.0;
   out_844640693863841425[275] = 0.0;
   out_844640693863841425[276] = 0.0;
   out_844640693863841425[277] = 0.0;
   out_844640693863841425[278] = 0.0;
   out_844640693863841425[279] = 0.0;
   out_844640693863841425[280] = 0.0;
   out_844640693863841425[281] = 0.0;
   out_844640693863841425[282] = 0.0;
   out_844640693863841425[283] = 0.0;
   out_844640693863841425[284] = 0.0;
   out_844640693863841425[285] = 1.0;
   out_844640693863841425[286] = 0.0;
   out_844640693863841425[287] = 0.0;
   out_844640693863841425[288] = 0.0;
   out_844640693863841425[289] = 0.0;
   out_844640693863841425[290] = 0.0;
   out_844640693863841425[291] = 0.0;
   out_844640693863841425[292] = 0.0;
   out_844640693863841425[293] = 0.0;
   out_844640693863841425[294] = 0.0;
   out_844640693863841425[295] = 0.0;
   out_844640693863841425[296] = 0.0;
   out_844640693863841425[297] = 0.0;
   out_844640693863841425[298] = 0.0;
   out_844640693863841425[299] = 0.0;
   out_844640693863841425[300] = 0.0;
   out_844640693863841425[301] = 0.0;
   out_844640693863841425[302] = 0.0;
   out_844640693863841425[303] = 0.0;
   out_844640693863841425[304] = 1.0;
   out_844640693863841425[305] = 0.0;
   out_844640693863841425[306] = 0.0;
   out_844640693863841425[307] = 0.0;
   out_844640693863841425[308] = 0.0;
   out_844640693863841425[309] = 0.0;
   out_844640693863841425[310] = 0.0;
   out_844640693863841425[311] = 0.0;
   out_844640693863841425[312] = 0.0;
   out_844640693863841425[313] = 0.0;
   out_844640693863841425[314] = 0.0;
   out_844640693863841425[315] = 0.0;
   out_844640693863841425[316] = 0.0;
   out_844640693863841425[317] = 0.0;
   out_844640693863841425[318] = 0.0;
   out_844640693863841425[319] = 0.0;
   out_844640693863841425[320] = 0.0;
   out_844640693863841425[321] = 0.0;
   out_844640693863841425[322] = 0.0;
   out_844640693863841425[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_986367587733679490) {
   out_986367587733679490[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_986367587733679490[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_986367587733679490[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_986367587733679490[3] = dt*state[12] + state[3];
   out_986367587733679490[4] = dt*state[13] + state[4];
   out_986367587733679490[5] = dt*state[14] + state[5];
   out_986367587733679490[6] = state[6];
   out_986367587733679490[7] = state[7];
   out_986367587733679490[8] = state[8];
   out_986367587733679490[9] = state[9];
   out_986367587733679490[10] = state[10];
   out_986367587733679490[11] = state[11];
   out_986367587733679490[12] = state[12];
   out_986367587733679490[13] = state[13];
   out_986367587733679490[14] = state[14];
   out_986367587733679490[15] = state[15];
   out_986367587733679490[16] = state[16];
   out_986367587733679490[17] = state[17];
}
void F_fun(double *state, double dt, double *out_4594191458309706069) {
   out_4594191458309706069[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4594191458309706069[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4594191458309706069[2] = 0;
   out_4594191458309706069[3] = 0;
   out_4594191458309706069[4] = 0;
   out_4594191458309706069[5] = 0;
   out_4594191458309706069[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4594191458309706069[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4594191458309706069[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4594191458309706069[9] = 0;
   out_4594191458309706069[10] = 0;
   out_4594191458309706069[11] = 0;
   out_4594191458309706069[12] = 0;
   out_4594191458309706069[13] = 0;
   out_4594191458309706069[14] = 0;
   out_4594191458309706069[15] = 0;
   out_4594191458309706069[16] = 0;
   out_4594191458309706069[17] = 0;
   out_4594191458309706069[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_4594191458309706069[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_4594191458309706069[20] = 0;
   out_4594191458309706069[21] = 0;
   out_4594191458309706069[22] = 0;
   out_4594191458309706069[23] = 0;
   out_4594191458309706069[24] = 0;
   out_4594191458309706069[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_4594191458309706069[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_4594191458309706069[27] = 0;
   out_4594191458309706069[28] = 0;
   out_4594191458309706069[29] = 0;
   out_4594191458309706069[30] = 0;
   out_4594191458309706069[31] = 0;
   out_4594191458309706069[32] = 0;
   out_4594191458309706069[33] = 0;
   out_4594191458309706069[34] = 0;
   out_4594191458309706069[35] = 0;
   out_4594191458309706069[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4594191458309706069[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4594191458309706069[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4594191458309706069[39] = 0;
   out_4594191458309706069[40] = 0;
   out_4594191458309706069[41] = 0;
   out_4594191458309706069[42] = 0;
   out_4594191458309706069[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4594191458309706069[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4594191458309706069[45] = 0;
   out_4594191458309706069[46] = 0;
   out_4594191458309706069[47] = 0;
   out_4594191458309706069[48] = 0;
   out_4594191458309706069[49] = 0;
   out_4594191458309706069[50] = 0;
   out_4594191458309706069[51] = 0;
   out_4594191458309706069[52] = 0;
   out_4594191458309706069[53] = 0;
   out_4594191458309706069[54] = 0;
   out_4594191458309706069[55] = 0;
   out_4594191458309706069[56] = 0;
   out_4594191458309706069[57] = 1;
   out_4594191458309706069[58] = 0;
   out_4594191458309706069[59] = 0;
   out_4594191458309706069[60] = 0;
   out_4594191458309706069[61] = 0;
   out_4594191458309706069[62] = 0;
   out_4594191458309706069[63] = 0;
   out_4594191458309706069[64] = 0;
   out_4594191458309706069[65] = 0;
   out_4594191458309706069[66] = dt;
   out_4594191458309706069[67] = 0;
   out_4594191458309706069[68] = 0;
   out_4594191458309706069[69] = 0;
   out_4594191458309706069[70] = 0;
   out_4594191458309706069[71] = 0;
   out_4594191458309706069[72] = 0;
   out_4594191458309706069[73] = 0;
   out_4594191458309706069[74] = 0;
   out_4594191458309706069[75] = 0;
   out_4594191458309706069[76] = 1;
   out_4594191458309706069[77] = 0;
   out_4594191458309706069[78] = 0;
   out_4594191458309706069[79] = 0;
   out_4594191458309706069[80] = 0;
   out_4594191458309706069[81] = 0;
   out_4594191458309706069[82] = 0;
   out_4594191458309706069[83] = 0;
   out_4594191458309706069[84] = 0;
   out_4594191458309706069[85] = dt;
   out_4594191458309706069[86] = 0;
   out_4594191458309706069[87] = 0;
   out_4594191458309706069[88] = 0;
   out_4594191458309706069[89] = 0;
   out_4594191458309706069[90] = 0;
   out_4594191458309706069[91] = 0;
   out_4594191458309706069[92] = 0;
   out_4594191458309706069[93] = 0;
   out_4594191458309706069[94] = 0;
   out_4594191458309706069[95] = 1;
   out_4594191458309706069[96] = 0;
   out_4594191458309706069[97] = 0;
   out_4594191458309706069[98] = 0;
   out_4594191458309706069[99] = 0;
   out_4594191458309706069[100] = 0;
   out_4594191458309706069[101] = 0;
   out_4594191458309706069[102] = 0;
   out_4594191458309706069[103] = 0;
   out_4594191458309706069[104] = dt;
   out_4594191458309706069[105] = 0;
   out_4594191458309706069[106] = 0;
   out_4594191458309706069[107] = 0;
   out_4594191458309706069[108] = 0;
   out_4594191458309706069[109] = 0;
   out_4594191458309706069[110] = 0;
   out_4594191458309706069[111] = 0;
   out_4594191458309706069[112] = 0;
   out_4594191458309706069[113] = 0;
   out_4594191458309706069[114] = 1;
   out_4594191458309706069[115] = 0;
   out_4594191458309706069[116] = 0;
   out_4594191458309706069[117] = 0;
   out_4594191458309706069[118] = 0;
   out_4594191458309706069[119] = 0;
   out_4594191458309706069[120] = 0;
   out_4594191458309706069[121] = 0;
   out_4594191458309706069[122] = 0;
   out_4594191458309706069[123] = 0;
   out_4594191458309706069[124] = 0;
   out_4594191458309706069[125] = 0;
   out_4594191458309706069[126] = 0;
   out_4594191458309706069[127] = 0;
   out_4594191458309706069[128] = 0;
   out_4594191458309706069[129] = 0;
   out_4594191458309706069[130] = 0;
   out_4594191458309706069[131] = 0;
   out_4594191458309706069[132] = 0;
   out_4594191458309706069[133] = 1;
   out_4594191458309706069[134] = 0;
   out_4594191458309706069[135] = 0;
   out_4594191458309706069[136] = 0;
   out_4594191458309706069[137] = 0;
   out_4594191458309706069[138] = 0;
   out_4594191458309706069[139] = 0;
   out_4594191458309706069[140] = 0;
   out_4594191458309706069[141] = 0;
   out_4594191458309706069[142] = 0;
   out_4594191458309706069[143] = 0;
   out_4594191458309706069[144] = 0;
   out_4594191458309706069[145] = 0;
   out_4594191458309706069[146] = 0;
   out_4594191458309706069[147] = 0;
   out_4594191458309706069[148] = 0;
   out_4594191458309706069[149] = 0;
   out_4594191458309706069[150] = 0;
   out_4594191458309706069[151] = 0;
   out_4594191458309706069[152] = 1;
   out_4594191458309706069[153] = 0;
   out_4594191458309706069[154] = 0;
   out_4594191458309706069[155] = 0;
   out_4594191458309706069[156] = 0;
   out_4594191458309706069[157] = 0;
   out_4594191458309706069[158] = 0;
   out_4594191458309706069[159] = 0;
   out_4594191458309706069[160] = 0;
   out_4594191458309706069[161] = 0;
   out_4594191458309706069[162] = 0;
   out_4594191458309706069[163] = 0;
   out_4594191458309706069[164] = 0;
   out_4594191458309706069[165] = 0;
   out_4594191458309706069[166] = 0;
   out_4594191458309706069[167] = 0;
   out_4594191458309706069[168] = 0;
   out_4594191458309706069[169] = 0;
   out_4594191458309706069[170] = 0;
   out_4594191458309706069[171] = 1;
   out_4594191458309706069[172] = 0;
   out_4594191458309706069[173] = 0;
   out_4594191458309706069[174] = 0;
   out_4594191458309706069[175] = 0;
   out_4594191458309706069[176] = 0;
   out_4594191458309706069[177] = 0;
   out_4594191458309706069[178] = 0;
   out_4594191458309706069[179] = 0;
   out_4594191458309706069[180] = 0;
   out_4594191458309706069[181] = 0;
   out_4594191458309706069[182] = 0;
   out_4594191458309706069[183] = 0;
   out_4594191458309706069[184] = 0;
   out_4594191458309706069[185] = 0;
   out_4594191458309706069[186] = 0;
   out_4594191458309706069[187] = 0;
   out_4594191458309706069[188] = 0;
   out_4594191458309706069[189] = 0;
   out_4594191458309706069[190] = 1;
   out_4594191458309706069[191] = 0;
   out_4594191458309706069[192] = 0;
   out_4594191458309706069[193] = 0;
   out_4594191458309706069[194] = 0;
   out_4594191458309706069[195] = 0;
   out_4594191458309706069[196] = 0;
   out_4594191458309706069[197] = 0;
   out_4594191458309706069[198] = 0;
   out_4594191458309706069[199] = 0;
   out_4594191458309706069[200] = 0;
   out_4594191458309706069[201] = 0;
   out_4594191458309706069[202] = 0;
   out_4594191458309706069[203] = 0;
   out_4594191458309706069[204] = 0;
   out_4594191458309706069[205] = 0;
   out_4594191458309706069[206] = 0;
   out_4594191458309706069[207] = 0;
   out_4594191458309706069[208] = 0;
   out_4594191458309706069[209] = 1;
   out_4594191458309706069[210] = 0;
   out_4594191458309706069[211] = 0;
   out_4594191458309706069[212] = 0;
   out_4594191458309706069[213] = 0;
   out_4594191458309706069[214] = 0;
   out_4594191458309706069[215] = 0;
   out_4594191458309706069[216] = 0;
   out_4594191458309706069[217] = 0;
   out_4594191458309706069[218] = 0;
   out_4594191458309706069[219] = 0;
   out_4594191458309706069[220] = 0;
   out_4594191458309706069[221] = 0;
   out_4594191458309706069[222] = 0;
   out_4594191458309706069[223] = 0;
   out_4594191458309706069[224] = 0;
   out_4594191458309706069[225] = 0;
   out_4594191458309706069[226] = 0;
   out_4594191458309706069[227] = 0;
   out_4594191458309706069[228] = 1;
   out_4594191458309706069[229] = 0;
   out_4594191458309706069[230] = 0;
   out_4594191458309706069[231] = 0;
   out_4594191458309706069[232] = 0;
   out_4594191458309706069[233] = 0;
   out_4594191458309706069[234] = 0;
   out_4594191458309706069[235] = 0;
   out_4594191458309706069[236] = 0;
   out_4594191458309706069[237] = 0;
   out_4594191458309706069[238] = 0;
   out_4594191458309706069[239] = 0;
   out_4594191458309706069[240] = 0;
   out_4594191458309706069[241] = 0;
   out_4594191458309706069[242] = 0;
   out_4594191458309706069[243] = 0;
   out_4594191458309706069[244] = 0;
   out_4594191458309706069[245] = 0;
   out_4594191458309706069[246] = 0;
   out_4594191458309706069[247] = 1;
   out_4594191458309706069[248] = 0;
   out_4594191458309706069[249] = 0;
   out_4594191458309706069[250] = 0;
   out_4594191458309706069[251] = 0;
   out_4594191458309706069[252] = 0;
   out_4594191458309706069[253] = 0;
   out_4594191458309706069[254] = 0;
   out_4594191458309706069[255] = 0;
   out_4594191458309706069[256] = 0;
   out_4594191458309706069[257] = 0;
   out_4594191458309706069[258] = 0;
   out_4594191458309706069[259] = 0;
   out_4594191458309706069[260] = 0;
   out_4594191458309706069[261] = 0;
   out_4594191458309706069[262] = 0;
   out_4594191458309706069[263] = 0;
   out_4594191458309706069[264] = 0;
   out_4594191458309706069[265] = 0;
   out_4594191458309706069[266] = 1;
   out_4594191458309706069[267] = 0;
   out_4594191458309706069[268] = 0;
   out_4594191458309706069[269] = 0;
   out_4594191458309706069[270] = 0;
   out_4594191458309706069[271] = 0;
   out_4594191458309706069[272] = 0;
   out_4594191458309706069[273] = 0;
   out_4594191458309706069[274] = 0;
   out_4594191458309706069[275] = 0;
   out_4594191458309706069[276] = 0;
   out_4594191458309706069[277] = 0;
   out_4594191458309706069[278] = 0;
   out_4594191458309706069[279] = 0;
   out_4594191458309706069[280] = 0;
   out_4594191458309706069[281] = 0;
   out_4594191458309706069[282] = 0;
   out_4594191458309706069[283] = 0;
   out_4594191458309706069[284] = 0;
   out_4594191458309706069[285] = 1;
   out_4594191458309706069[286] = 0;
   out_4594191458309706069[287] = 0;
   out_4594191458309706069[288] = 0;
   out_4594191458309706069[289] = 0;
   out_4594191458309706069[290] = 0;
   out_4594191458309706069[291] = 0;
   out_4594191458309706069[292] = 0;
   out_4594191458309706069[293] = 0;
   out_4594191458309706069[294] = 0;
   out_4594191458309706069[295] = 0;
   out_4594191458309706069[296] = 0;
   out_4594191458309706069[297] = 0;
   out_4594191458309706069[298] = 0;
   out_4594191458309706069[299] = 0;
   out_4594191458309706069[300] = 0;
   out_4594191458309706069[301] = 0;
   out_4594191458309706069[302] = 0;
   out_4594191458309706069[303] = 0;
   out_4594191458309706069[304] = 1;
   out_4594191458309706069[305] = 0;
   out_4594191458309706069[306] = 0;
   out_4594191458309706069[307] = 0;
   out_4594191458309706069[308] = 0;
   out_4594191458309706069[309] = 0;
   out_4594191458309706069[310] = 0;
   out_4594191458309706069[311] = 0;
   out_4594191458309706069[312] = 0;
   out_4594191458309706069[313] = 0;
   out_4594191458309706069[314] = 0;
   out_4594191458309706069[315] = 0;
   out_4594191458309706069[316] = 0;
   out_4594191458309706069[317] = 0;
   out_4594191458309706069[318] = 0;
   out_4594191458309706069[319] = 0;
   out_4594191458309706069[320] = 0;
   out_4594191458309706069[321] = 0;
   out_4594191458309706069[322] = 0;
   out_4594191458309706069[323] = 1;
}
void h_4(double *state, double *unused, double *out_7900639425322801320) {
   out_7900639425322801320[0] = state[6] + state[9];
   out_7900639425322801320[1] = state[7] + state[10];
   out_7900639425322801320[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_3300993812880714874) {
   out_3300993812880714874[0] = 0;
   out_3300993812880714874[1] = 0;
   out_3300993812880714874[2] = 0;
   out_3300993812880714874[3] = 0;
   out_3300993812880714874[4] = 0;
   out_3300993812880714874[5] = 0;
   out_3300993812880714874[6] = 1;
   out_3300993812880714874[7] = 0;
   out_3300993812880714874[8] = 0;
   out_3300993812880714874[9] = 1;
   out_3300993812880714874[10] = 0;
   out_3300993812880714874[11] = 0;
   out_3300993812880714874[12] = 0;
   out_3300993812880714874[13] = 0;
   out_3300993812880714874[14] = 0;
   out_3300993812880714874[15] = 0;
   out_3300993812880714874[16] = 0;
   out_3300993812880714874[17] = 0;
   out_3300993812880714874[18] = 0;
   out_3300993812880714874[19] = 0;
   out_3300993812880714874[20] = 0;
   out_3300993812880714874[21] = 0;
   out_3300993812880714874[22] = 0;
   out_3300993812880714874[23] = 0;
   out_3300993812880714874[24] = 0;
   out_3300993812880714874[25] = 1;
   out_3300993812880714874[26] = 0;
   out_3300993812880714874[27] = 0;
   out_3300993812880714874[28] = 1;
   out_3300993812880714874[29] = 0;
   out_3300993812880714874[30] = 0;
   out_3300993812880714874[31] = 0;
   out_3300993812880714874[32] = 0;
   out_3300993812880714874[33] = 0;
   out_3300993812880714874[34] = 0;
   out_3300993812880714874[35] = 0;
   out_3300993812880714874[36] = 0;
   out_3300993812880714874[37] = 0;
   out_3300993812880714874[38] = 0;
   out_3300993812880714874[39] = 0;
   out_3300993812880714874[40] = 0;
   out_3300993812880714874[41] = 0;
   out_3300993812880714874[42] = 0;
   out_3300993812880714874[43] = 0;
   out_3300993812880714874[44] = 1;
   out_3300993812880714874[45] = 0;
   out_3300993812880714874[46] = 0;
   out_3300993812880714874[47] = 1;
   out_3300993812880714874[48] = 0;
   out_3300993812880714874[49] = 0;
   out_3300993812880714874[50] = 0;
   out_3300993812880714874[51] = 0;
   out_3300993812880714874[52] = 0;
   out_3300993812880714874[53] = 0;
}
void h_10(double *state, double *unused, double *out_1277818480136149867) {
   out_1277818480136149867[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_1277818480136149867[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_1277818480136149867[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_8852053340215761248) {
   out_8852053340215761248[0] = 0;
   out_8852053340215761248[1] = 9.8100000000000005*cos(state[1]);
   out_8852053340215761248[2] = 0;
   out_8852053340215761248[3] = 0;
   out_8852053340215761248[4] = -state[8];
   out_8852053340215761248[5] = state[7];
   out_8852053340215761248[6] = 0;
   out_8852053340215761248[7] = state[5];
   out_8852053340215761248[8] = -state[4];
   out_8852053340215761248[9] = 0;
   out_8852053340215761248[10] = 0;
   out_8852053340215761248[11] = 0;
   out_8852053340215761248[12] = 1;
   out_8852053340215761248[13] = 0;
   out_8852053340215761248[14] = 0;
   out_8852053340215761248[15] = 1;
   out_8852053340215761248[16] = 0;
   out_8852053340215761248[17] = 0;
   out_8852053340215761248[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_8852053340215761248[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_8852053340215761248[20] = 0;
   out_8852053340215761248[21] = state[8];
   out_8852053340215761248[22] = 0;
   out_8852053340215761248[23] = -state[6];
   out_8852053340215761248[24] = -state[5];
   out_8852053340215761248[25] = 0;
   out_8852053340215761248[26] = state[3];
   out_8852053340215761248[27] = 0;
   out_8852053340215761248[28] = 0;
   out_8852053340215761248[29] = 0;
   out_8852053340215761248[30] = 0;
   out_8852053340215761248[31] = 1;
   out_8852053340215761248[32] = 0;
   out_8852053340215761248[33] = 0;
   out_8852053340215761248[34] = 1;
   out_8852053340215761248[35] = 0;
   out_8852053340215761248[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_8852053340215761248[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_8852053340215761248[38] = 0;
   out_8852053340215761248[39] = -state[7];
   out_8852053340215761248[40] = state[6];
   out_8852053340215761248[41] = 0;
   out_8852053340215761248[42] = state[4];
   out_8852053340215761248[43] = -state[3];
   out_8852053340215761248[44] = 0;
   out_8852053340215761248[45] = 0;
   out_8852053340215761248[46] = 0;
   out_8852053340215761248[47] = 0;
   out_8852053340215761248[48] = 0;
   out_8852053340215761248[49] = 0;
   out_8852053340215761248[50] = 1;
   out_8852053340215761248[51] = 0;
   out_8852053340215761248[52] = 0;
   out_8852053340215761248[53] = 1;
}
void h_13(double *state, double *unused, double *out_2885571052644477490) {
   out_2885571052644477490[0] = state[3];
   out_2885571052644477490[1] = state[4];
   out_2885571052644477490[2] = state[5];
}
void H_13(double *state, double *unused, double *out_6513267638213047675) {
   out_6513267638213047675[0] = 0;
   out_6513267638213047675[1] = 0;
   out_6513267638213047675[2] = 0;
   out_6513267638213047675[3] = 1;
   out_6513267638213047675[4] = 0;
   out_6513267638213047675[5] = 0;
   out_6513267638213047675[6] = 0;
   out_6513267638213047675[7] = 0;
   out_6513267638213047675[8] = 0;
   out_6513267638213047675[9] = 0;
   out_6513267638213047675[10] = 0;
   out_6513267638213047675[11] = 0;
   out_6513267638213047675[12] = 0;
   out_6513267638213047675[13] = 0;
   out_6513267638213047675[14] = 0;
   out_6513267638213047675[15] = 0;
   out_6513267638213047675[16] = 0;
   out_6513267638213047675[17] = 0;
   out_6513267638213047675[18] = 0;
   out_6513267638213047675[19] = 0;
   out_6513267638213047675[20] = 0;
   out_6513267638213047675[21] = 0;
   out_6513267638213047675[22] = 1;
   out_6513267638213047675[23] = 0;
   out_6513267638213047675[24] = 0;
   out_6513267638213047675[25] = 0;
   out_6513267638213047675[26] = 0;
   out_6513267638213047675[27] = 0;
   out_6513267638213047675[28] = 0;
   out_6513267638213047675[29] = 0;
   out_6513267638213047675[30] = 0;
   out_6513267638213047675[31] = 0;
   out_6513267638213047675[32] = 0;
   out_6513267638213047675[33] = 0;
   out_6513267638213047675[34] = 0;
   out_6513267638213047675[35] = 0;
   out_6513267638213047675[36] = 0;
   out_6513267638213047675[37] = 0;
   out_6513267638213047675[38] = 0;
   out_6513267638213047675[39] = 0;
   out_6513267638213047675[40] = 0;
   out_6513267638213047675[41] = 1;
   out_6513267638213047675[42] = 0;
   out_6513267638213047675[43] = 0;
   out_6513267638213047675[44] = 0;
   out_6513267638213047675[45] = 0;
   out_6513267638213047675[46] = 0;
   out_6513267638213047675[47] = 0;
   out_6513267638213047675[48] = 0;
   out_6513267638213047675[49] = 0;
   out_6513267638213047675[50] = 0;
   out_6513267638213047675[51] = 0;
   out_6513267638213047675[52] = 0;
   out_6513267638213047675[53] = 0;
}
void h_14(double *state, double *unused, double *out_5381598505700020116) {
   out_5381598505700020116[0] = state[6];
   out_5381598505700020116[1] = state[7];
   out_5381598505700020116[2] = state[8];
}
void H_14(double *state, double *unused, double *out_7264234669220199403) {
   out_7264234669220199403[0] = 0;
   out_7264234669220199403[1] = 0;
   out_7264234669220199403[2] = 0;
   out_7264234669220199403[3] = 0;
   out_7264234669220199403[4] = 0;
   out_7264234669220199403[5] = 0;
   out_7264234669220199403[6] = 1;
   out_7264234669220199403[7] = 0;
   out_7264234669220199403[8] = 0;
   out_7264234669220199403[9] = 0;
   out_7264234669220199403[10] = 0;
   out_7264234669220199403[11] = 0;
   out_7264234669220199403[12] = 0;
   out_7264234669220199403[13] = 0;
   out_7264234669220199403[14] = 0;
   out_7264234669220199403[15] = 0;
   out_7264234669220199403[16] = 0;
   out_7264234669220199403[17] = 0;
   out_7264234669220199403[18] = 0;
   out_7264234669220199403[19] = 0;
   out_7264234669220199403[20] = 0;
   out_7264234669220199403[21] = 0;
   out_7264234669220199403[22] = 0;
   out_7264234669220199403[23] = 0;
   out_7264234669220199403[24] = 0;
   out_7264234669220199403[25] = 1;
   out_7264234669220199403[26] = 0;
   out_7264234669220199403[27] = 0;
   out_7264234669220199403[28] = 0;
   out_7264234669220199403[29] = 0;
   out_7264234669220199403[30] = 0;
   out_7264234669220199403[31] = 0;
   out_7264234669220199403[32] = 0;
   out_7264234669220199403[33] = 0;
   out_7264234669220199403[34] = 0;
   out_7264234669220199403[35] = 0;
   out_7264234669220199403[36] = 0;
   out_7264234669220199403[37] = 0;
   out_7264234669220199403[38] = 0;
   out_7264234669220199403[39] = 0;
   out_7264234669220199403[40] = 0;
   out_7264234669220199403[41] = 0;
   out_7264234669220199403[42] = 0;
   out_7264234669220199403[43] = 0;
   out_7264234669220199403[44] = 1;
   out_7264234669220199403[45] = 0;
   out_7264234669220199403[46] = 0;
   out_7264234669220199403[47] = 0;
   out_7264234669220199403[48] = 0;
   out_7264234669220199403[49] = 0;
   out_7264234669220199403[50] = 0;
   out_7264234669220199403[51] = 0;
   out_7264234669220199403[52] = 0;
   out_7264234669220199403[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_4988085883151629357) {
  err_fun(nom_x, delta_x, out_4988085883151629357);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_8628576639675182721) {
  inv_err_fun(nom_x, true_x, out_8628576639675182721);
}
void pose_H_mod_fun(double *state, double *out_844640693863841425) {
  H_mod_fun(state, out_844640693863841425);
}
void pose_f_fun(double *state, double dt, double *out_986367587733679490) {
  f_fun(state,  dt, out_986367587733679490);
}
void pose_F_fun(double *state, double dt, double *out_4594191458309706069) {
  F_fun(state,  dt, out_4594191458309706069);
}
void pose_h_4(double *state, double *unused, double *out_7900639425322801320) {
  h_4(state, unused, out_7900639425322801320);
}
void pose_H_4(double *state, double *unused, double *out_3300993812880714874) {
  H_4(state, unused, out_3300993812880714874);
}
void pose_h_10(double *state, double *unused, double *out_1277818480136149867) {
  h_10(state, unused, out_1277818480136149867);
}
void pose_H_10(double *state, double *unused, double *out_8852053340215761248) {
  H_10(state, unused, out_8852053340215761248);
}
void pose_h_13(double *state, double *unused, double *out_2885571052644477490) {
  h_13(state, unused, out_2885571052644477490);
}
void pose_H_13(double *state, double *unused, double *out_6513267638213047675) {
  H_13(state, unused, out_6513267638213047675);
}
void pose_h_14(double *state, double *unused, double *out_5381598505700020116) {
  h_14(state, unused, out_5381598505700020116);
}
void pose_H_14(double *state, double *unused, double *out_7264234669220199403) {
  H_14(state, unused, out_7264234669220199403);
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
