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
void err_fun(double *nom_x, double *delta_x, double *out_5861723782991102346) {
   out_5861723782991102346[0] = delta_x[0] + nom_x[0];
   out_5861723782991102346[1] = delta_x[1] + nom_x[1];
   out_5861723782991102346[2] = delta_x[2] + nom_x[2];
   out_5861723782991102346[3] = delta_x[3] + nom_x[3];
   out_5861723782991102346[4] = delta_x[4] + nom_x[4];
   out_5861723782991102346[5] = delta_x[5] + nom_x[5];
   out_5861723782991102346[6] = delta_x[6] + nom_x[6];
   out_5861723782991102346[7] = delta_x[7] + nom_x[7];
   out_5861723782991102346[8] = delta_x[8] + nom_x[8];
   out_5861723782991102346[9] = delta_x[9] + nom_x[9];
   out_5861723782991102346[10] = delta_x[10] + nom_x[10];
   out_5861723782991102346[11] = delta_x[11] + nom_x[11];
   out_5861723782991102346[12] = delta_x[12] + nom_x[12];
   out_5861723782991102346[13] = delta_x[13] + nom_x[13];
   out_5861723782991102346[14] = delta_x[14] + nom_x[14];
   out_5861723782991102346[15] = delta_x[15] + nom_x[15];
   out_5861723782991102346[16] = delta_x[16] + nom_x[16];
   out_5861723782991102346[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_8211258457099351294) {
   out_8211258457099351294[0] = -nom_x[0] + true_x[0];
   out_8211258457099351294[1] = -nom_x[1] + true_x[1];
   out_8211258457099351294[2] = -nom_x[2] + true_x[2];
   out_8211258457099351294[3] = -nom_x[3] + true_x[3];
   out_8211258457099351294[4] = -nom_x[4] + true_x[4];
   out_8211258457099351294[5] = -nom_x[5] + true_x[5];
   out_8211258457099351294[6] = -nom_x[6] + true_x[6];
   out_8211258457099351294[7] = -nom_x[7] + true_x[7];
   out_8211258457099351294[8] = -nom_x[8] + true_x[8];
   out_8211258457099351294[9] = -nom_x[9] + true_x[9];
   out_8211258457099351294[10] = -nom_x[10] + true_x[10];
   out_8211258457099351294[11] = -nom_x[11] + true_x[11];
   out_8211258457099351294[12] = -nom_x[12] + true_x[12];
   out_8211258457099351294[13] = -nom_x[13] + true_x[13];
   out_8211258457099351294[14] = -nom_x[14] + true_x[14];
   out_8211258457099351294[15] = -nom_x[15] + true_x[15];
   out_8211258457099351294[16] = -nom_x[16] + true_x[16];
   out_8211258457099351294[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_877341408371094612) {
   out_877341408371094612[0] = 1.0;
   out_877341408371094612[1] = 0.0;
   out_877341408371094612[2] = 0.0;
   out_877341408371094612[3] = 0.0;
   out_877341408371094612[4] = 0.0;
   out_877341408371094612[5] = 0.0;
   out_877341408371094612[6] = 0.0;
   out_877341408371094612[7] = 0.0;
   out_877341408371094612[8] = 0.0;
   out_877341408371094612[9] = 0.0;
   out_877341408371094612[10] = 0.0;
   out_877341408371094612[11] = 0.0;
   out_877341408371094612[12] = 0.0;
   out_877341408371094612[13] = 0.0;
   out_877341408371094612[14] = 0.0;
   out_877341408371094612[15] = 0.0;
   out_877341408371094612[16] = 0.0;
   out_877341408371094612[17] = 0.0;
   out_877341408371094612[18] = 0.0;
   out_877341408371094612[19] = 1.0;
   out_877341408371094612[20] = 0.0;
   out_877341408371094612[21] = 0.0;
   out_877341408371094612[22] = 0.0;
   out_877341408371094612[23] = 0.0;
   out_877341408371094612[24] = 0.0;
   out_877341408371094612[25] = 0.0;
   out_877341408371094612[26] = 0.0;
   out_877341408371094612[27] = 0.0;
   out_877341408371094612[28] = 0.0;
   out_877341408371094612[29] = 0.0;
   out_877341408371094612[30] = 0.0;
   out_877341408371094612[31] = 0.0;
   out_877341408371094612[32] = 0.0;
   out_877341408371094612[33] = 0.0;
   out_877341408371094612[34] = 0.0;
   out_877341408371094612[35] = 0.0;
   out_877341408371094612[36] = 0.0;
   out_877341408371094612[37] = 0.0;
   out_877341408371094612[38] = 1.0;
   out_877341408371094612[39] = 0.0;
   out_877341408371094612[40] = 0.0;
   out_877341408371094612[41] = 0.0;
   out_877341408371094612[42] = 0.0;
   out_877341408371094612[43] = 0.0;
   out_877341408371094612[44] = 0.0;
   out_877341408371094612[45] = 0.0;
   out_877341408371094612[46] = 0.0;
   out_877341408371094612[47] = 0.0;
   out_877341408371094612[48] = 0.0;
   out_877341408371094612[49] = 0.0;
   out_877341408371094612[50] = 0.0;
   out_877341408371094612[51] = 0.0;
   out_877341408371094612[52] = 0.0;
   out_877341408371094612[53] = 0.0;
   out_877341408371094612[54] = 0.0;
   out_877341408371094612[55] = 0.0;
   out_877341408371094612[56] = 0.0;
   out_877341408371094612[57] = 1.0;
   out_877341408371094612[58] = 0.0;
   out_877341408371094612[59] = 0.0;
   out_877341408371094612[60] = 0.0;
   out_877341408371094612[61] = 0.0;
   out_877341408371094612[62] = 0.0;
   out_877341408371094612[63] = 0.0;
   out_877341408371094612[64] = 0.0;
   out_877341408371094612[65] = 0.0;
   out_877341408371094612[66] = 0.0;
   out_877341408371094612[67] = 0.0;
   out_877341408371094612[68] = 0.0;
   out_877341408371094612[69] = 0.0;
   out_877341408371094612[70] = 0.0;
   out_877341408371094612[71] = 0.0;
   out_877341408371094612[72] = 0.0;
   out_877341408371094612[73] = 0.0;
   out_877341408371094612[74] = 0.0;
   out_877341408371094612[75] = 0.0;
   out_877341408371094612[76] = 1.0;
   out_877341408371094612[77] = 0.0;
   out_877341408371094612[78] = 0.0;
   out_877341408371094612[79] = 0.0;
   out_877341408371094612[80] = 0.0;
   out_877341408371094612[81] = 0.0;
   out_877341408371094612[82] = 0.0;
   out_877341408371094612[83] = 0.0;
   out_877341408371094612[84] = 0.0;
   out_877341408371094612[85] = 0.0;
   out_877341408371094612[86] = 0.0;
   out_877341408371094612[87] = 0.0;
   out_877341408371094612[88] = 0.0;
   out_877341408371094612[89] = 0.0;
   out_877341408371094612[90] = 0.0;
   out_877341408371094612[91] = 0.0;
   out_877341408371094612[92] = 0.0;
   out_877341408371094612[93] = 0.0;
   out_877341408371094612[94] = 0.0;
   out_877341408371094612[95] = 1.0;
   out_877341408371094612[96] = 0.0;
   out_877341408371094612[97] = 0.0;
   out_877341408371094612[98] = 0.0;
   out_877341408371094612[99] = 0.0;
   out_877341408371094612[100] = 0.0;
   out_877341408371094612[101] = 0.0;
   out_877341408371094612[102] = 0.0;
   out_877341408371094612[103] = 0.0;
   out_877341408371094612[104] = 0.0;
   out_877341408371094612[105] = 0.0;
   out_877341408371094612[106] = 0.0;
   out_877341408371094612[107] = 0.0;
   out_877341408371094612[108] = 0.0;
   out_877341408371094612[109] = 0.0;
   out_877341408371094612[110] = 0.0;
   out_877341408371094612[111] = 0.0;
   out_877341408371094612[112] = 0.0;
   out_877341408371094612[113] = 0.0;
   out_877341408371094612[114] = 1.0;
   out_877341408371094612[115] = 0.0;
   out_877341408371094612[116] = 0.0;
   out_877341408371094612[117] = 0.0;
   out_877341408371094612[118] = 0.0;
   out_877341408371094612[119] = 0.0;
   out_877341408371094612[120] = 0.0;
   out_877341408371094612[121] = 0.0;
   out_877341408371094612[122] = 0.0;
   out_877341408371094612[123] = 0.0;
   out_877341408371094612[124] = 0.0;
   out_877341408371094612[125] = 0.0;
   out_877341408371094612[126] = 0.0;
   out_877341408371094612[127] = 0.0;
   out_877341408371094612[128] = 0.0;
   out_877341408371094612[129] = 0.0;
   out_877341408371094612[130] = 0.0;
   out_877341408371094612[131] = 0.0;
   out_877341408371094612[132] = 0.0;
   out_877341408371094612[133] = 1.0;
   out_877341408371094612[134] = 0.0;
   out_877341408371094612[135] = 0.0;
   out_877341408371094612[136] = 0.0;
   out_877341408371094612[137] = 0.0;
   out_877341408371094612[138] = 0.0;
   out_877341408371094612[139] = 0.0;
   out_877341408371094612[140] = 0.0;
   out_877341408371094612[141] = 0.0;
   out_877341408371094612[142] = 0.0;
   out_877341408371094612[143] = 0.0;
   out_877341408371094612[144] = 0.0;
   out_877341408371094612[145] = 0.0;
   out_877341408371094612[146] = 0.0;
   out_877341408371094612[147] = 0.0;
   out_877341408371094612[148] = 0.0;
   out_877341408371094612[149] = 0.0;
   out_877341408371094612[150] = 0.0;
   out_877341408371094612[151] = 0.0;
   out_877341408371094612[152] = 1.0;
   out_877341408371094612[153] = 0.0;
   out_877341408371094612[154] = 0.0;
   out_877341408371094612[155] = 0.0;
   out_877341408371094612[156] = 0.0;
   out_877341408371094612[157] = 0.0;
   out_877341408371094612[158] = 0.0;
   out_877341408371094612[159] = 0.0;
   out_877341408371094612[160] = 0.0;
   out_877341408371094612[161] = 0.0;
   out_877341408371094612[162] = 0.0;
   out_877341408371094612[163] = 0.0;
   out_877341408371094612[164] = 0.0;
   out_877341408371094612[165] = 0.0;
   out_877341408371094612[166] = 0.0;
   out_877341408371094612[167] = 0.0;
   out_877341408371094612[168] = 0.0;
   out_877341408371094612[169] = 0.0;
   out_877341408371094612[170] = 0.0;
   out_877341408371094612[171] = 1.0;
   out_877341408371094612[172] = 0.0;
   out_877341408371094612[173] = 0.0;
   out_877341408371094612[174] = 0.0;
   out_877341408371094612[175] = 0.0;
   out_877341408371094612[176] = 0.0;
   out_877341408371094612[177] = 0.0;
   out_877341408371094612[178] = 0.0;
   out_877341408371094612[179] = 0.0;
   out_877341408371094612[180] = 0.0;
   out_877341408371094612[181] = 0.0;
   out_877341408371094612[182] = 0.0;
   out_877341408371094612[183] = 0.0;
   out_877341408371094612[184] = 0.0;
   out_877341408371094612[185] = 0.0;
   out_877341408371094612[186] = 0.0;
   out_877341408371094612[187] = 0.0;
   out_877341408371094612[188] = 0.0;
   out_877341408371094612[189] = 0.0;
   out_877341408371094612[190] = 1.0;
   out_877341408371094612[191] = 0.0;
   out_877341408371094612[192] = 0.0;
   out_877341408371094612[193] = 0.0;
   out_877341408371094612[194] = 0.0;
   out_877341408371094612[195] = 0.0;
   out_877341408371094612[196] = 0.0;
   out_877341408371094612[197] = 0.0;
   out_877341408371094612[198] = 0.0;
   out_877341408371094612[199] = 0.0;
   out_877341408371094612[200] = 0.0;
   out_877341408371094612[201] = 0.0;
   out_877341408371094612[202] = 0.0;
   out_877341408371094612[203] = 0.0;
   out_877341408371094612[204] = 0.0;
   out_877341408371094612[205] = 0.0;
   out_877341408371094612[206] = 0.0;
   out_877341408371094612[207] = 0.0;
   out_877341408371094612[208] = 0.0;
   out_877341408371094612[209] = 1.0;
   out_877341408371094612[210] = 0.0;
   out_877341408371094612[211] = 0.0;
   out_877341408371094612[212] = 0.0;
   out_877341408371094612[213] = 0.0;
   out_877341408371094612[214] = 0.0;
   out_877341408371094612[215] = 0.0;
   out_877341408371094612[216] = 0.0;
   out_877341408371094612[217] = 0.0;
   out_877341408371094612[218] = 0.0;
   out_877341408371094612[219] = 0.0;
   out_877341408371094612[220] = 0.0;
   out_877341408371094612[221] = 0.0;
   out_877341408371094612[222] = 0.0;
   out_877341408371094612[223] = 0.0;
   out_877341408371094612[224] = 0.0;
   out_877341408371094612[225] = 0.0;
   out_877341408371094612[226] = 0.0;
   out_877341408371094612[227] = 0.0;
   out_877341408371094612[228] = 1.0;
   out_877341408371094612[229] = 0.0;
   out_877341408371094612[230] = 0.0;
   out_877341408371094612[231] = 0.0;
   out_877341408371094612[232] = 0.0;
   out_877341408371094612[233] = 0.0;
   out_877341408371094612[234] = 0.0;
   out_877341408371094612[235] = 0.0;
   out_877341408371094612[236] = 0.0;
   out_877341408371094612[237] = 0.0;
   out_877341408371094612[238] = 0.0;
   out_877341408371094612[239] = 0.0;
   out_877341408371094612[240] = 0.0;
   out_877341408371094612[241] = 0.0;
   out_877341408371094612[242] = 0.0;
   out_877341408371094612[243] = 0.0;
   out_877341408371094612[244] = 0.0;
   out_877341408371094612[245] = 0.0;
   out_877341408371094612[246] = 0.0;
   out_877341408371094612[247] = 1.0;
   out_877341408371094612[248] = 0.0;
   out_877341408371094612[249] = 0.0;
   out_877341408371094612[250] = 0.0;
   out_877341408371094612[251] = 0.0;
   out_877341408371094612[252] = 0.0;
   out_877341408371094612[253] = 0.0;
   out_877341408371094612[254] = 0.0;
   out_877341408371094612[255] = 0.0;
   out_877341408371094612[256] = 0.0;
   out_877341408371094612[257] = 0.0;
   out_877341408371094612[258] = 0.0;
   out_877341408371094612[259] = 0.0;
   out_877341408371094612[260] = 0.0;
   out_877341408371094612[261] = 0.0;
   out_877341408371094612[262] = 0.0;
   out_877341408371094612[263] = 0.0;
   out_877341408371094612[264] = 0.0;
   out_877341408371094612[265] = 0.0;
   out_877341408371094612[266] = 1.0;
   out_877341408371094612[267] = 0.0;
   out_877341408371094612[268] = 0.0;
   out_877341408371094612[269] = 0.0;
   out_877341408371094612[270] = 0.0;
   out_877341408371094612[271] = 0.0;
   out_877341408371094612[272] = 0.0;
   out_877341408371094612[273] = 0.0;
   out_877341408371094612[274] = 0.0;
   out_877341408371094612[275] = 0.0;
   out_877341408371094612[276] = 0.0;
   out_877341408371094612[277] = 0.0;
   out_877341408371094612[278] = 0.0;
   out_877341408371094612[279] = 0.0;
   out_877341408371094612[280] = 0.0;
   out_877341408371094612[281] = 0.0;
   out_877341408371094612[282] = 0.0;
   out_877341408371094612[283] = 0.0;
   out_877341408371094612[284] = 0.0;
   out_877341408371094612[285] = 1.0;
   out_877341408371094612[286] = 0.0;
   out_877341408371094612[287] = 0.0;
   out_877341408371094612[288] = 0.0;
   out_877341408371094612[289] = 0.0;
   out_877341408371094612[290] = 0.0;
   out_877341408371094612[291] = 0.0;
   out_877341408371094612[292] = 0.0;
   out_877341408371094612[293] = 0.0;
   out_877341408371094612[294] = 0.0;
   out_877341408371094612[295] = 0.0;
   out_877341408371094612[296] = 0.0;
   out_877341408371094612[297] = 0.0;
   out_877341408371094612[298] = 0.0;
   out_877341408371094612[299] = 0.0;
   out_877341408371094612[300] = 0.0;
   out_877341408371094612[301] = 0.0;
   out_877341408371094612[302] = 0.0;
   out_877341408371094612[303] = 0.0;
   out_877341408371094612[304] = 1.0;
   out_877341408371094612[305] = 0.0;
   out_877341408371094612[306] = 0.0;
   out_877341408371094612[307] = 0.0;
   out_877341408371094612[308] = 0.0;
   out_877341408371094612[309] = 0.0;
   out_877341408371094612[310] = 0.0;
   out_877341408371094612[311] = 0.0;
   out_877341408371094612[312] = 0.0;
   out_877341408371094612[313] = 0.0;
   out_877341408371094612[314] = 0.0;
   out_877341408371094612[315] = 0.0;
   out_877341408371094612[316] = 0.0;
   out_877341408371094612[317] = 0.0;
   out_877341408371094612[318] = 0.0;
   out_877341408371094612[319] = 0.0;
   out_877341408371094612[320] = 0.0;
   out_877341408371094612[321] = 0.0;
   out_877341408371094612[322] = 0.0;
   out_877341408371094612[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_6181153239753365860) {
   out_6181153239753365860[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_6181153239753365860[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_6181153239753365860[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_6181153239753365860[3] = dt*state[12] + state[3];
   out_6181153239753365860[4] = dt*state[13] + state[4];
   out_6181153239753365860[5] = dt*state[14] + state[5];
   out_6181153239753365860[6] = state[6];
   out_6181153239753365860[7] = state[7];
   out_6181153239753365860[8] = state[8];
   out_6181153239753365860[9] = state[9];
   out_6181153239753365860[10] = state[10];
   out_6181153239753365860[11] = state[11];
   out_6181153239753365860[12] = state[12];
   out_6181153239753365860[13] = state[13];
   out_6181153239753365860[14] = state[14];
   out_6181153239753365860[15] = state[15];
   out_6181153239753365860[16] = state[16];
   out_6181153239753365860[17] = state[17];
}
void F_fun(double *state, double dt, double *out_125451314886039473) {
   out_125451314886039473[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_125451314886039473[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_125451314886039473[2] = 0;
   out_125451314886039473[3] = 0;
   out_125451314886039473[4] = 0;
   out_125451314886039473[5] = 0;
   out_125451314886039473[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_125451314886039473[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_125451314886039473[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_125451314886039473[9] = 0;
   out_125451314886039473[10] = 0;
   out_125451314886039473[11] = 0;
   out_125451314886039473[12] = 0;
   out_125451314886039473[13] = 0;
   out_125451314886039473[14] = 0;
   out_125451314886039473[15] = 0;
   out_125451314886039473[16] = 0;
   out_125451314886039473[17] = 0;
   out_125451314886039473[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_125451314886039473[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_125451314886039473[20] = 0;
   out_125451314886039473[21] = 0;
   out_125451314886039473[22] = 0;
   out_125451314886039473[23] = 0;
   out_125451314886039473[24] = 0;
   out_125451314886039473[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_125451314886039473[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_125451314886039473[27] = 0;
   out_125451314886039473[28] = 0;
   out_125451314886039473[29] = 0;
   out_125451314886039473[30] = 0;
   out_125451314886039473[31] = 0;
   out_125451314886039473[32] = 0;
   out_125451314886039473[33] = 0;
   out_125451314886039473[34] = 0;
   out_125451314886039473[35] = 0;
   out_125451314886039473[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_125451314886039473[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_125451314886039473[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_125451314886039473[39] = 0;
   out_125451314886039473[40] = 0;
   out_125451314886039473[41] = 0;
   out_125451314886039473[42] = 0;
   out_125451314886039473[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_125451314886039473[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_125451314886039473[45] = 0;
   out_125451314886039473[46] = 0;
   out_125451314886039473[47] = 0;
   out_125451314886039473[48] = 0;
   out_125451314886039473[49] = 0;
   out_125451314886039473[50] = 0;
   out_125451314886039473[51] = 0;
   out_125451314886039473[52] = 0;
   out_125451314886039473[53] = 0;
   out_125451314886039473[54] = 0;
   out_125451314886039473[55] = 0;
   out_125451314886039473[56] = 0;
   out_125451314886039473[57] = 1;
   out_125451314886039473[58] = 0;
   out_125451314886039473[59] = 0;
   out_125451314886039473[60] = 0;
   out_125451314886039473[61] = 0;
   out_125451314886039473[62] = 0;
   out_125451314886039473[63] = 0;
   out_125451314886039473[64] = 0;
   out_125451314886039473[65] = 0;
   out_125451314886039473[66] = dt;
   out_125451314886039473[67] = 0;
   out_125451314886039473[68] = 0;
   out_125451314886039473[69] = 0;
   out_125451314886039473[70] = 0;
   out_125451314886039473[71] = 0;
   out_125451314886039473[72] = 0;
   out_125451314886039473[73] = 0;
   out_125451314886039473[74] = 0;
   out_125451314886039473[75] = 0;
   out_125451314886039473[76] = 1;
   out_125451314886039473[77] = 0;
   out_125451314886039473[78] = 0;
   out_125451314886039473[79] = 0;
   out_125451314886039473[80] = 0;
   out_125451314886039473[81] = 0;
   out_125451314886039473[82] = 0;
   out_125451314886039473[83] = 0;
   out_125451314886039473[84] = 0;
   out_125451314886039473[85] = dt;
   out_125451314886039473[86] = 0;
   out_125451314886039473[87] = 0;
   out_125451314886039473[88] = 0;
   out_125451314886039473[89] = 0;
   out_125451314886039473[90] = 0;
   out_125451314886039473[91] = 0;
   out_125451314886039473[92] = 0;
   out_125451314886039473[93] = 0;
   out_125451314886039473[94] = 0;
   out_125451314886039473[95] = 1;
   out_125451314886039473[96] = 0;
   out_125451314886039473[97] = 0;
   out_125451314886039473[98] = 0;
   out_125451314886039473[99] = 0;
   out_125451314886039473[100] = 0;
   out_125451314886039473[101] = 0;
   out_125451314886039473[102] = 0;
   out_125451314886039473[103] = 0;
   out_125451314886039473[104] = dt;
   out_125451314886039473[105] = 0;
   out_125451314886039473[106] = 0;
   out_125451314886039473[107] = 0;
   out_125451314886039473[108] = 0;
   out_125451314886039473[109] = 0;
   out_125451314886039473[110] = 0;
   out_125451314886039473[111] = 0;
   out_125451314886039473[112] = 0;
   out_125451314886039473[113] = 0;
   out_125451314886039473[114] = 1;
   out_125451314886039473[115] = 0;
   out_125451314886039473[116] = 0;
   out_125451314886039473[117] = 0;
   out_125451314886039473[118] = 0;
   out_125451314886039473[119] = 0;
   out_125451314886039473[120] = 0;
   out_125451314886039473[121] = 0;
   out_125451314886039473[122] = 0;
   out_125451314886039473[123] = 0;
   out_125451314886039473[124] = 0;
   out_125451314886039473[125] = 0;
   out_125451314886039473[126] = 0;
   out_125451314886039473[127] = 0;
   out_125451314886039473[128] = 0;
   out_125451314886039473[129] = 0;
   out_125451314886039473[130] = 0;
   out_125451314886039473[131] = 0;
   out_125451314886039473[132] = 0;
   out_125451314886039473[133] = 1;
   out_125451314886039473[134] = 0;
   out_125451314886039473[135] = 0;
   out_125451314886039473[136] = 0;
   out_125451314886039473[137] = 0;
   out_125451314886039473[138] = 0;
   out_125451314886039473[139] = 0;
   out_125451314886039473[140] = 0;
   out_125451314886039473[141] = 0;
   out_125451314886039473[142] = 0;
   out_125451314886039473[143] = 0;
   out_125451314886039473[144] = 0;
   out_125451314886039473[145] = 0;
   out_125451314886039473[146] = 0;
   out_125451314886039473[147] = 0;
   out_125451314886039473[148] = 0;
   out_125451314886039473[149] = 0;
   out_125451314886039473[150] = 0;
   out_125451314886039473[151] = 0;
   out_125451314886039473[152] = 1;
   out_125451314886039473[153] = 0;
   out_125451314886039473[154] = 0;
   out_125451314886039473[155] = 0;
   out_125451314886039473[156] = 0;
   out_125451314886039473[157] = 0;
   out_125451314886039473[158] = 0;
   out_125451314886039473[159] = 0;
   out_125451314886039473[160] = 0;
   out_125451314886039473[161] = 0;
   out_125451314886039473[162] = 0;
   out_125451314886039473[163] = 0;
   out_125451314886039473[164] = 0;
   out_125451314886039473[165] = 0;
   out_125451314886039473[166] = 0;
   out_125451314886039473[167] = 0;
   out_125451314886039473[168] = 0;
   out_125451314886039473[169] = 0;
   out_125451314886039473[170] = 0;
   out_125451314886039473[171] = 1;
   out_125451314886039473[172] = 0;
   out_125451314886039473[173] = 0;
   out_125451314886039473[174] = 0;
   out_125451314886039473[175] = 0;
   out_125451314886039473[176] = 0;
   out_125451314886039473[177] = 0;
   out_125451314886039473[178] = 0;
   out_125451314886039473[179] = 0;
   out_125451314886039473[180] = 0;
   out_125451314886039473[181] = 0;
   out_125451314886039473[182] = 0;
   out_125451314886039473[183] = 0;
   out_125451314886039473[184] = 0;
   out_125451314886039473[185] = 0;
   out_125451314886039473[186] = 0;
   out_125451314886039473[187] = 0;
   out_125451314886039473[188] = 0;
   out_125451314886039473[189] = 0;
   out_125451314886039473[190] = 1;
   out_125451314886039473[191] = 0;
   out_125451314886039473[192] = 0;
   out_125451314886039473[193] = 0;
   out_125451314886039473[194] = 0;
   out_125451314886039473[195] = 0;
   out_125451314886039473[196] = 0;
   out_125451314886039473[197] = 0;
   out_125451314886039473[198] = 0;
   out_125451314886039473[199] = 0;
   out_125451314886039473[200] = 0;
   out_125451314886039473[201] = 0;
   out_125451314886039473[202] = 0;
   out_125451314886039473[203] = 0;
   out_125451314886039473[204] = 0;
   out_125451314886039473[205] = 0;
   out_125451314886039473[206] = 0;
   out_125451314886039473[207] = 0;
   out_125451314886039473[208] = 0;
   out_125451314886039473[209] = 1;
   out_125451314886039473[210] = 0;
   out_125451314886039473[211] = 0;
   out_125451314886039473[212] = 0;
   out_125451314886039473[213] = 0;
   out_125451314886039473[214] = 0;
   out_125451314886039473[215] = 0;
   out_125451314886039473[216] = 0;
   out_125451314886039473[217] = 0;
   out_125451314886039473[218] = 0;
   out_125451314886039473[219] = 0;
   out_125451314886039473[220] = 0;
   out_125451314886039473[221] = 0;
   out_125451314886039473[222] = 0;
   out_125451314886039473[223] = 0;
   out_125451314886039473[224] = 0;
   out_125451314886039473[225] = 0;
   out_125451314886039473[226] = 0;
   out_125451314886039473[227] = 0;
   out_125451314886039473[228] = 1;
   out_125451314886039473[229] = 0;
   out_125451314886039473[230] = 0;
   out_125451314886039473[231] = 0;
   out_125451314886039473[232] = 0;
   out_125451314886039473[233] = 0;
   out_125451314886039473[234] = 0;
   out_125451314886039473[235] = 0;
   out_125451314886039473[236] = 0;
   out_125451314886039473[237] = 0;
   out_125451314886039473[238] = 0;
   out_125451314886039473[239] = 0;
   out_125451314886039473[240] = 0;
   out_125451314886039473[241] = 0;
   out_125451314886039473[242] = 0;
   out_125451314886039473[243] = 0;
   out_125451314886039473[244] = 0;
   out_125451314886039473[245] = 0;
   out_125451314886039473[246] = 0;
   out_125451314886039473[247] = 1;
   out_125451314886039473[248] = 0;
   out_125451314886039473[249] = 0;
   out_125451314886039473[250] = 0;
   out_125451314886039473[251] = 0;
   out_125451314886039473[252] = 0;
   out_125451314886039473[253] = 0;
   out_125451314886039473[254] = 0;
   out_125451314886039473[255] = 0;
   out_125451314886039473[256] = 0;
   out_125451314886039473[257] = 0;
   out_125451314886039473[258] = 0;
   out_125451314886039473[259] = 0;
   out_125451314886039473[260] = 0;
   out_125451314886039473[261] = 0;
   out_125451314886039473[262] = 0;
   out_125451314886039473[263] = 0;
   out_125451314886039473[264] = 0;
   out_125451314886039473[265] = 0;
   out_125451314886039473[266] = 1;
   out_125451314886039473[267] = 0;
   out_125451314886039473[268] = 0;
   out_125451314886039473[269] = 0;
   out_125451314886039473[270] = 0;
   out_125451314886039473[271] = 0;
   out_125451314886039473[272] = 0;
   out_125451314886039473[273] = 0;
   out_125451314886039473[274] = 0;
   out_125451314886039473[275] = 0;
   out_125451314886039473[276] = 0;
   out_125451314886039473[277] = 0;
   out_125451314886039473[278] = 0;
   out_125451314886039473[279] = 0;
   out_125451314886039473[280] = 0;
   out_125451314886039473[281] = 0;
   out_125451314886039473[282] = 0;
   out_125451314886039473[283] = 0;
   out_125451314886039473[284] = 0;
   out_125451314886039473[285] = 1;
   out_125451314886039473[286] = 0;
   out_125451314886039473[287] = 0;
   out_125451314886039473[288] = 0;
   out_125451314886039473[289] = 0;
   out_125451314886039473[290] = 0;
   out_125451314886039473[291] = 0;
   out_125451314886039473[292] = 0;
   out_125451314886039473[293] = 0;
   out_125451314886039473[294] = 0;
   out_125451314886039473[295] = 0;
   out_125451314886039473[296] = 0;
   out_125451314886039473[297] = 0;
   out_125451314886039473[298] = 0;
   out_125451314886039473[299] = 0;
   out_125451314886039473[300] = 0;
   out_125451314886039473[301] = 0;
   out_125451314886039473[302] = 0;
   out_125451314886039473[303] = 0;
   out_125451314886039473[304] = 1;
   out_125451314886039473[305] = 0;
   out_125451314886039473[306] = 0;
   out_125451314886039473[307] = 0;
   out_125451314886039473[308] = 0;
   out_125451314886039473[309] = 0;
   out_125451314886039473[310] = 0;
   out_125451314886039473[311] = 0;
   out_125451314886039473[312] = 0;
   out_125451314886039473[313] = 0;
   out_125451314886039473[314] = 0;
   out_125451314886039473[315] = 0;
   out_125451314886039473[316] = 0;
   out_125451314886039473[317] = 0;
   out_125451314886039473[318] = 0;
   out_125451314886039473[319] = 0;
   out_125451314886039473[320] = 0;
   out_125451314886039473[321] = 0;
   out_125451314886039473[322] = 0;
   out_125451314886039473[323] = 1;
}
void h_4(double *state, double *unused, double *out_5481609484354041996) {
   out_5481609484354041996[0] = state[6] + state[9];
   out_5481609484354041996[1] = state[7] + state[10];
   out_5481609484354041996[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_7988386606170059074) {
   out_7988386606170059074[0] = 0;
   out_7988386606170059074[1] = 0;
   out_7988386606170059074[2] = 0;
   out_7988386606170059074[3] = 0;
   out_7988386606170059074[4] = 0;
   out_7988386606170059074[5] = 0;
   out_7988386606170059074[6] = 1;
   out_7988386606170059074[7] = 0;
   out_7988386606170059074[8] = 0;
   out_7988386606170059074[9] = 1;
   out_7988386606170059074[10] = 0;
   out_7988386606170059074[11] = 0;
   out_7988386606170059074[12] = 0;
   out_7988386606170059074[13] = 0;
   out_7988386606170059074[14] = 0;
   out_7988386606170059074[15] = 0;
   out_7988386606170059074[16] = 0;
   out_7988386606170059074[17] = 0;
   out_7988386606170059074[18] = 0;
   out_7988386606170059074[19] = 0;
   out_7988386606170059074[20] = 0;
   out_7988386606170059074[21] = 0;
   out_7988386606170059074[22] = 0;
   out_7988386606170059074[23] = 0;
   out_7988386606170059074[24] = 0;
   out_7988386606170059074[25] = 1;
   out_7988386606170059074[26] = 0;
   out_7988386606170059074[27] = 0;
   out_7988386606170059074[28] = 1;
   out_7988386606170059074[29] = 0;
   out_7988386606170059074[30] = 0;
   out_7988386606170059074[31] = 0;
   out_7988386606170059074[32] = 0;
   out_7988386606170059074[33] = 0;
   out_7988386606170059074[34] = 0;
   out_7988386606170059074[35] = 0;
   out_7988386606170059074[36] = 0;
   out_7988386606170059074[37] = 0;
   out_7988386606170059074[38] = 0;
   out_7988386606170059074[39] = 0;
   out_7988386606170059074[40] = 0;
   out_7988386606170059074[41] = 0;
   out_7988386606170059074[42] = 0;
   out_7988386606170059074[43] = 0;
   out_7988386606170059074[44] = 1;
   out_7988386606170059074[45] = 0;
   out_7988386606170059074[46] = 0;
   out_7988386606170059074[47] = 1;
   out_7988386606170059074[48] = 0;
   out_7988386606170059074[49] = 0;
   out_7988386606170059074[50] = 0;
   out_7988386606170059074[51] = 0;
   out_7988386606170059074[52] = 0;
   out_7988386606170059074[53] = 0;
}
void h_10(double *state, double *unused, double *out_1946765286996617825) {
   out_1946765286996617825[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_1946765286996617825[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_1946765286996617825[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_9127548125181276924) {
   out_9127548125181276924[0] = 0;
   out_9127548125181276924[1] = 9.8100000000000005*cos(state[1]);
   out_9127548125181276924[2] = 0;
   out_9127548125181276924[3] = 0;
   out_9127548125181276924[4] = -state[8];
   out_9127548125181276924[5] = state[7];
   out_9127548125181276924[6] = 0;
   out_9127548125181276924[7] = state[5];
   out_9127548125181276924[8] = -state[4];
   out_9127548125181276924[9] = 0;
   out_9127548125181276924[10] = 0;
   out_9127548125181276924[11] = 0;
   out_9127548125181276924[12] = 1;
   out_9127548125181276924[13] = 0;
   out_9127548125181276924[14] = 0;
   out_9127548125181276924[15] = 1;
   out_9127548125181276924[16] = 0;
   out_9127548125181276924[17] = 0;
   out_9127548125181276924[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_9127548125181276924[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_9127548125181276924[20] = 0;
   out_9127548125181276924[21] = state[8];
   out_9127548125181276924[22] = 0;
   out_9127548125181276924[23] = -state[6];
   out_9127548125181276924[24] = -state[5];
   out_9127548125181276924[25] = 0;
   out_9127548125181276924[26] = state[3];
   out_9127548125181276924[27] = 0;
   out_9127548125181276924[28] = 0;
   out_9127548125181276924[29] = 0;
   out_9127548125181276924[30] = 0;
   out_9127548125181276924[31] = 1;
   out_9127548125181276924[32] = 0;
   out_9127548125181276924[33] = 0;
   out_9127548125181276924[34] = 1;
   out_9127548125181276924[35] = 0;
   out_9127548125181276924[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_9127548125181276924[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_9127548125181276924[38] = 0;
   out_9127548125181276924[39] = -state[7];
   out_9127548125181276924[40] = state[6];
   out_9127548125181276924[41] = 0;
   out_9127548125181276924[42] = state[4];
   out_9127548125181276924[43] = -state[3];
   out_9127548125181276924[44] = 0;
   out_9127548125181276924[45] = 0;
   out_9127548125181276924[46] = 0;
   out_9127548125181276924[47] = 0;
   out_9127548125181276924[48] = 0;
   out_9127548125181276924[49] = 0;
   out_9127548125181276924[50] = 1;
   out_9127548125181276924[51] = 0;
   out_9127548125181276924[52] = 0;
   out_9127548125181276924[53] = 1;
}
void h_13(double *state, double *unused, double *out_8982835759751218704) {
   out_8982835759751218704[0] = state[3];
   out_8982835759751218704[1] = state[4];
   out_8982835759751218704[2] = state[5];
}
void H_13(double *state, double *unused, double *out_4776112780837726273) {
   out_4776112780837726273[0] = 0;
   out_4776112780837726273[1] = 0;
   out_4776112780837726273[2] = 0;
   out_4776112780837726273[3] = 1;
   out_4776112780837726273[4] = 0;
   out_4776112780837726273[5] = 0;
   out_4776112780837726273[6] = 0;
   out_4776112780837726273[7] = 0;
   out_4776112780837726273[8] = 0;
   out_4776112780837726273[9] = 0;
   out_4776112780837726273[10] = 0;
   out_4776112780837726273[11] = 0;
   out_4776112780837726273[12] = 0;
   out_4776112780837726273[13] = 0;
   out_4776112780837726273[14] = 0;
   out_4776112780837726273[15] = 0;
   out_4776112780837726273[16] = 0;
   out_4776112780837726273[17] = 0;
   out_4776112780837726273[18] = 0;
   out_4776112780837726273[19] = 0;
   out_4776112780837726273[20] = 0;
   out_4776112780837726273[21] = 0;
   out_4776112780837726273[22] = 1;
   out_4776112780837726273[23] = 0;
   out_4776112780837726273[24] = 0;
   out_4776112780837726273[25] = 0;
   out_4776112780837726273[26] = 0;
   out_4776112780837726273[27] = 0;
   out_4776112780837726273[28] = 0;
   out_4776112780837726273[29] = 0;
   out_4776112780837726273[30] = 0;
   out_4776112780837726273[31] = 0;
   out_4776112780837726273[32] = 0;
   out_4776112780837726273[33] = 0;
   out_4776112780837726273[34] = 0;
   out_4776112780837726273[35] = 0;
   out_4776112780837726273[36] = 0;
   out_4776112780837726273[37] = 0;
   out_4776112780837726273[38] = 0;
   out_4776112780837726273[39] = 0;
   out_4776112780837726273[40] = 0;
   out_4776112780837726273[41] = 1;
   out_4776112780837726273[42] = 0;
   out_4776112780837726273[43] = 0;
   out_4776112780837726273[44] = 0;
   out_4776112780837726273[45] = 0;
   out_4776112780837726273[46] = 0;
   out_4776112780837726273[47] = 0;
   out_4776112780837726273[48] = 0;
   out_4776112780837726273[49] = 0;
   out_4776112780837726273[50] = 0;
   out_4776112780837726273[51] = 0;
   out_4776112780837726273[52] = 0;
   out_4776112780837726273[53] = 0;
}
void h_14(double *state, double *unused, double *out_5159606875724608362) {
   out_5159606875724608362[0] = state[6];
   out_5159606875724608362[1] = state[7];
   out_5159606875724608362[2] = state[8];
}
void H_14(double *state, double *unused, double *out_4025145749830574545) {
   out_4025145749830574545[0] = 0;
   out_4025145749830574545[1] = 0;
   out_4025145749830574545[2] = 0;
   out_4025145749830574545[3] = 0;
   out_4025145749830574545[4] = 0;
   out_4025145749830574545[5] = 0;
   out_4025145749830574545[6] = 1;
   out_4025145749830574545[7] = 0;
   out_4025145749830574545[8] = 0;
   out_4025145749830574545[9] = 0;
   out_4025145749830574545[10] = 0;
   out_4025145749830574545[11] = 0;
   out_4025145749830574545[12] = 0;
   out_4025145749830574545[13] = 0;
   out_4025145749830574545[14] = 0;
   out_4025145749830574545[15] = 0;
   out_4025145749830574545[16] = 0;
   out_4025145749830574545[17] = 0;
   out_4025145749830574545[18] = 0;
   out_4025145749830574545[19] = 0;
   out_4025145749830574545[20] = 0;
   out_4025145749830574545[21] = 0;
   out_4025145749830574545[22] = 0;
   out_4025145749830574545[23] = 0;
   out_4025145749830574545[24] = 0;
   out_4025145749830574545[25] = 1;
   out_4025145749830574545[26] = 0;
   out_4025145749830574545[27] = 0;
   out_4025145749830574545[28] = 0;
   out_4025145749830574545[29] = 0;
   out_4025145749830574545[30] = 0;
   out_4025145749830574545[31] = 0;
   out_4025145749830574545[32] = 0;
   out_4025145749830574545[33] = 0;
   out_4025145749830574545[34] = 0;
   out_4025145749830574545[35] = 0;
   out_4025145749830574545[36] = 0;
   out_4025145749830574545[37] = 0;
   out_4025145749830574545[38] = 0;
   out_4025145749830574545[39] = 0;
   out_4025145749830574545[40] = 0;
   out_4025145749830574545[41] = 0;
   out_4025145749830574545[42] = 0;
   out_4025145749830574545[43] = 0;
   out_4025145749830574545[44] = 1;
   out_4025145749830574545[45] = 0;
   out_4025145749830574545[46] = 0;
   out_4025145749830574545[47] = 0;
   out_4025145749830574545[48] = 0;
   out_4025145749830574545[49] = 0;
   out_4025145749830574545[50] = 0;
   out_4025145749830574545[51] = 0;
   out_4025145749830574545[52] = 0;
   out_4025145749830574545[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_5861723782991102346) {
  err_fun(nom_x, delta_x, out_5861723782991102346);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_8211258457099351294) {
  inv_err_fun(nom_x, true_x, out_8211258457099351294);
}
void pose_H_mod_fun(double *state, double *out_877341408371094612) {
  H_mod_fun(state, out_877341408371094612);
}
void pose_f_fun(double *state, double dt, double *out_6181153239753365860) {
  f_fun(state,  dt, out_6181153239753365860);
}
void pose_F_fun(double *state, double dt, double *out_125451314886039473) {
  F_fun(state,  dt, out_125451314886039473);
}
void pose_h_4(double *state, double *unused, double *out_5481609484354041996) {
  h_4(state, unused, out_5481609484354041996);
}
void pose_H_4(double *state, double *unused, double *out_7988386606170059074) {
  H_4(state, unused, out_7988386606170059074);
}
void pose_h_10(double *state, double *unused, double *out_1946765286996617825) {
  h_10(state, unused, out_1946765286996617825);
}
void pose_H_10(double *state, double *unused, double *out_9127548125181276924) {
  H_10(state, unused, out_9127548125181276924);
}
void pose_h_13(double *state, double *unused, double *out_8982835759751218704) {
  h_13(state, unused, out_8982835759751218704);
}
void pose_H_13(double *state, double *unused, double *out_4776112780837726273) {
  H_13(state, unused, out_4776112780837726273);
}
void pose_h_14(double *state, double *unused, double *out_5159606875724608362) {
  h_14(state, unused, out_5159606875724608362);
}
void pose_H_14(double *state, double *unused, double *out_4025145749830574545) {
  H_14(state, unused, out_4025145749830574545);
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
