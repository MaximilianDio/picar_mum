/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <gazebo/gazebo.hh>
#include <ignition/math.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>

using namespace std;

double fields[1038][3] ={
{3.98268,2.15568,-8.70577e-07},
{3.965,2.16504,-8.70392e-07},
{3.94731,2.17436,-8.70064e-07},
{3.92975,2.18393,-8.6954e-07},
{3.91236,2.1938,-8.68772e-07},
{3.89502,2.20376,-8.67931e-07},
{3.87787,2.21403,-8.6684e-07},
{3.86088,2.22458,-8.65523e-07},
{3.84397,2.23524,-8.64105e-07},
{3.82726,2.24621,-8.62433e-07},
{3.81067,2.25737,-8.60608e-07},
{3.79422,2.26873,-8.58604e-07},
{3.77797,2.28039,-8.56344e-07},
{3.76179,2.29215,-8.54007e-07},
{3.74585,2.30421,-8.51391e-07},
{3.73014,2.3166,-8.48494e-07},
{3.71451,2.32907,-8.45515e-07},
{3.69913,2.34185,-8.42262e-07},
{3.68392,2.35483,-8.38816e-07},
{3.66884,2.36796,-8.35227e-07},
{3.65398,2.38135,-8.314e-07},
{3.63922,2.39483,-8.27475e-07},
{3.62465,2.40853,-8.23342e-07},
{3.61031,2.42246,-8.18977e-07},
{3.59604,2.43647,-8.14543e-07},
{3.58199,2.45069,-8.09885e-07},
{3.56811,2.46509,-8.05046e-07},
{3.55435,2.4796,-8.001e-07},
{3.5408,2.4943,-7.9494e-07},
{3.52736,2.50911,-7.89669e-07},
{3.5141,2.52407,-7.84237e-07},
{3.50104,2.53922,-7.78599e-07},
{3.48804,2.55442,-7.72901e-07},
{3.47525,2.5698,-7.67007e-07},
{3.46265,2.58532,-7.60937e-07},
{3.45013,2.60092,-7.54787e-07},
{3.43783,2.61668,-7.4843e-07},
{3.42565,2.63255,-7.41948e-07},
{3.41361,2.64852,-7.35347e-07},
{3.40174,2.66462,-7.28575e-07},
{3.38994,2.68076,-7.21749e-07},
{3.37829,2.69701,-7.14768e-07},
{3.36681,2.71339,-7.07629e-07},
{3.35537,2.72979,-7.00446e-07},
{3.34408,2.7463,-6.93117e-07},
{3.3329,2.76288,-6.85682e-07},
{3.3218,2.77952,-6.78167e-07},
{3.31083,2.79624,-6.70516e-07},
{3.29992,2.813,-6.62808e-07},
{3.28912,2.82983,-6.54988e-07},
{3.27844,2.84674,-6.47042e-07},
{3.26779,2.86367,-6.39059e-07},
{3.25724,2.88066,-6.3096e-07},
{3.24678,2.8977,-6.22765e-07},
{3.23636,2.91478,-6.14519e-07},
{3.22603,2.9319,-6.06165e-07},
{3.21574,2.94905,-5.97756e-07},
{3.2055,2.96623,-5.8927e-07},
{3.19534,2.98346,-5.80686e-07},
{3.1852,3.00069,-5.72073e-07},
{3.17511,3.01796,-5.63365e-07},
{3.16507,3.03526,-5.54571e-07},
{3.15504,3.05256,-5.45746e-07},
{3.14504,3.06989,-5.3684e-07},
{3.13506,3.08722,-5.27884e-07},
{3.12509,3.10455,-5.18882e-07},
{3.11512,3.12189,-5.09812e-07},
{3.10516,3.13924,-5.0072e-07},
{3.09519,3.15657,-4.91571e-07},
{3.0852,3.1739,-4.82362e-07},
{3.07521,3.19123,-4.73139e-07},
{3.06519,3.20853,-4.63866e-07},
{3.05514,3.22583,-4.54557e-07},
{3.04506,3.2431,-4.45226e-07},
{3.03493,3.26035,-4.35855e-07},
{3.02478,3.27758,-4.26467e-07},
{3.01456,3.29477,-4.17054e-07},
{3.00426,3.31191,-4.07612e-07},
{2.99394,3.32904,-3.98163e-07},
{2.98352,3.34611,-3.88697e-07},
{2.97301,3.36313,-3.79216e-07},
{2.96245,3.38011,-3.69732e-07},
{2.95176,3.39702,-3.60244e-07},
{2.94101,3.41388,-3.50752e-07},
{2.93014,3.43067,-3.41268e-07},
{2.91912,3.44736,-3.31793e-07},
{2.90806,3.46402,-3.22321e-07},
{2.89682,3.48055,-3.12874e-07},
{2.88541,3.49698,-3.03449e-07},
{2.87393,3.51335,-2.9404e-07},
{2.86224,3.52958,-2.84673e-07},
{2.85043,3.54572,-2.75333e-07},
{2.83846,3.56174,-2.66034e-07},
{2.82627,3.57758,-2.56797e-07},
{2.81399,3.59337,-2.47581e-07},
{2.80147,3.60896,-2.38446e-07},
{2.78869,3.62434,-2.29395e-07},
{2.77583,3.63965,-2.20377e-07},
{2.76268,3.65472,-2.11467e-07},
{2.74934,3.66961,-2.02638e-07},
{2.73582,3.68433,-1.93886e-07},
{2.72199,3.69878,-1.8527e-07},
{2.70804,3.7131,-1.76714e-07},
{2.69381,3.72714,-1.68302e-07},
{2.67925,3.74085,-1.60057e-07},
{2.66461,3.75446,-1.51867e-07},
{2.64963,3.76769,-1.43874e-07},
{2.63437,3.78061,-1.36046e-07},
{2.61896,3.79334,-1.28326e-07},
{2.60321,3.80565,-1.20835e-07},
{2.58729,3.81774,-1.13466e-07},
{2.57111,3.82948,-1.06297e-07},
{2.55461,3.84075,-9.93888e-08},
{2.538,3.85189,-9.25604e-08},
{2.52108,3.86252,-8.6021e-08},
{2.50387,3.87268,-7.97521e-08},
{2.48655,3.88265,-7.35971e-08},
{2.46893,3.89209,-6.77554e-08},
{2.45113,3.90119,-6.21128e-08},
{2.43316,3.90992,-5.66898e-08},
{2.41493,3.9181,-5.15981e-08},
{2.3966,3.92609,-4.66179e-08},
{2.37806,3.93354,-4.19667e-08},
{2.35929,3.94042,-3.76574e-08},
{2.34047,3.94713,-3.34511e-08},
{2.32145,3.95328,-2.95894e-08},
{2.3023,3.95902,-2.59823e-08},
{2.28307,3.96444,-2.25655e-08},
{2.26369,3.96932,-1.94884e-08},
{2.24424,3.97395,-1.65606e-08},
{2.2247,3.97814,-1.39145e-08},
{2.20505,3.98179,-1.15961e-08},
{2.18536,3.9853,-9.37306e-09},
{2.1656,3.98831,-7.46026e-09},
{2.14578,3.9909,-5.80842e-09},
{2.12592,3.99327,-4.30086e-09},
{2.10602,3.99514,-3.10821e-09},
{2.08608,3.99674,-2.08622e-09},
{2.06612,3.99796,-1.30547e-09},
{2.04614,3.99868,-8.43295e-10},
{2.02615,3.99929,-4.66295e-10},
{2.00617,3.99995,-2.28454e-10},
{1.98618,4.00066,-1.2192e-10},
{1.96622,4.00171,-3.43264e-11},
{1.94635,4.00367,0},
{1.92652,4.00623,0},
{1.90678,4.00928,0},
{1.88716,4.01309,0},
{1.86758,4.01716,0},
{1.84818,4.02194,0},
{1.82896,4.02746,0},
{1.8098,4.03314,0},
{1.79079,4.03933,0},
{1.7719,4.0459,0},
{1.7531,4.0527,0},
{1.73446,4.05991,0},
{1.71588,4.06732,0},
{1.69744,4.07504,0},
{1.67915,4.08313,0},
{1.66091,4.09132,0},
{1.64281,4.09982,0},
{1.62483,4.10857,0},
{1.6069,4.11744,0},
{1.58909,4.12654,0},
{1.57135,4.13576,0},
{1.55369,4.14514,0},
{1.53613,4.15472,0},
{1.51861,4.16436,0},
{1.50118,4.17416,0},
{1.48384,4.18413,0},
{1.46653,4.19414,0},
{1.4493,4.2043,0},
{1.43213,4.21456,0},
{1.41501,4.22489,0},
{1.39796,4.23534,0},
{1.38093,4.24583,0},
{1.36397,4.25643,0},
{1.34706,4.26712,0},
{1.33018,4.27784,0},
{1.31335,4.28864,0},
{1.29656,4.29951,0},
{1.2798,4.31042,0},
{1.26308,4.32139,0},
{1.24638,4.3324,0},
{1.22971,4.34346,0},
{1.21309,4.35457,0},
{1.19647,4.3657,0},
{1.17988,4.37687,0},
{1.16332,4.38808,0},
{1.14676,4.3993,0},
{1.13023,4.41056,0},
{1.11371,4.42184,0},
{1.09721,4.43313,0},
{1.08072,4.44445,0},
{1.06423,4.45577,0},
{1.04775,4.4671,0},
{1.03128,4.47845,0},
{1.01481,4.48979,0},
{0.998343,4.50114,0},
{0.981876,4.51249,0},
{0.965407,4.52384,0},
{0.948932,4.53518,0},
{0.932455,4.54652,0},
{0.915968,4.55784,0},
{0.899469,4.56914,0},
{0.882964,4.58044,0},
{0.866441,4.59171,0},
{0.849904,4.60295,0},
{0.833353,4.61418,0},
{0.816776,4.62537,0},
{0.800187,4.63654,0},
{0.78357,4.64767,0},
{0.766921,4.65876,0},
{0.750261,4.66982,0},
{0.733559,4.68082,0},
{0.716822,4.69177,0},
{0.700064,4.70269,0},
{0.683257,4.71353,0},
{0.666422,4.72432,0},
{0.649544,4.73505,0},
{0.632609,4.74569,0},
{0.615656,4.7563,0},
{0.598637,4.7668,0},
{0.581552,4.7772,0},
{0.564441,4.78755,0},
{0.547253,4.79778,0},
{0.530014,4.80792,0},
{0.51272,4.81796,0},
{0.495338,4.82785,0},
{0.477923,4.83769,0},
{0.460408,4.84734,0},
{0.442786,4.8568,0},
{0.425129,4.86619,0},
{0.407356,4.87535,0},
{0.389494,4.88435,0},
{0.371565,4.8932,0},
{0.353509,4.9018,0},
{0.335393,4.91027,0},
{0.317163,4.91848,0},
{0.298794,4.92639,0},
{0.280383,4.93419,0},
{0.261823,4.94162,0},
{0.243132,4.94874,0},
{0.224364,4.95563,0},
{0.205414,4.96199,0},
{0.186357,4.96805,0},
{0.167184,4.97369,0},
{0.147848,4.97876,0},
{0.128463,4.98366,0},
{0.108927,4.98783,0},
{0.0892412,4.99131,0},
{0.0695165,4.99448,0},
{0.0496694,4.99667,0},
{0.0297399,4.9982,0},
{0.00979674,4.99908,0},
{-0.010169,4.9989,-1.38236e-18},
{-0.030143,4.99832,0},
{-0.0500417,4.99678,0},
{-0.0698591,4.99419,0},
{-0.0896403,4.99134,0},
{-0.109285,4.98772,0},
{-0.128822,4.98348,0},
{-0.148267,4.97887,0},
{-0.167533,4.97356,0},
{-0.186714,4.96791,0},
{-0.205764,4.96185,0},
{-0.224655,4.9553,0},
{-0.243499,4.94862,0},
{-0.262188,4.94151,0},
{-0.28074,4.93405,0},
{-0.299223,4.92642,0},
{-0.31754,4.9184,0},
{-0.335761,4.91016,0},
{-0.353887,4.90172,0},
{-0.371885,4.893,0},
{-0.389843,4.8842,0},
{-0.407683,4.87516,0},
{-0.425407,4.8659,0},
{-0.443097,4.85657,0},
{-0.460682,4.84705,0},
{-0.478194,4.83739,0},
{-0.495648,4.82763,0},
{-0.513008,4.8177,0},
{-0.530332,4.80771,0},
{-0.54758,4.79758,0},
{-0.564744,4.78732,0},
{-0.581886,4.77702,0},
{-0.598956,4.7666,0},
{-0.615968,4.75608,0},
{-0.632948,4.74552,0},
{-0.649865,4.73485,0},
{-0.666752,4.72413,0},
{-0.683595,4.71335,0},
{-0.700385,4.70248,0},
{-0.71716,4.69159,0},
{-0.733889,4.68063,0},
{-0.750579,4.66961,0},
{-0.767254,4.65857,0},
{-0.783891,4.64747,0},
{-0.800508,4.63634,0},
{-0.817103,4.62518,0},
{-0.833669,4.61397,0},
{-0.850227,4.60275,0},
{-0.866763,4.5915,0},
{-0.883277,4.58022,0},
{-0.899787,4.56893,0},
{-0.916282,4.55762,0},
{-0.932766,4.5463,0},
{-0.949246,4.53497,0},
{-0.965718,4.52362,0},
{-0.982187,4.51228,0},
{-0.998655,4.50093,0},
{-1.01512,4.48957,0},
{-1.03159,4.47823,0},
{-1.04806,4.46688,0},
{-1.06454,4.45555,0},
{-1.08102,4.44422,0},
{-1.09752,4.43292,0},
{-1.11402,4.42162,0},
{-1.13054,4.41034,0},
{-1.14708,4.39909,0},
{-1.16362,4.38785,0},
{-1.18019,4.37666,0},
{-1.19679,4.36549,0},
{-1.2134,4.35435,0},
{-1.23004,4.34325,0},
{-1.2467,4.33219,0},
{-1.26339,4.32117,0},
{-1.28012,4.31022,0},
{-1.29687,4.29928,0},
{-1.31366,4.28842,0},
{-1.33051,4.27764,0},
{-1.34737,4.26689,0},
{-1.36429,4.25622,0},
{-1.38125,4.24563,0},
{-1.39825,4.2351,0},
{-1.41532,4.22468,0},
{-1.43242,4.2143,0},
{-1.44959,4.20404,0},
{-1.46683,4.19391,0},
{-1.4841,4.18383,0},
{-1.50147,4.17391,0},
{-1.51891,4.16413,0},
{-1.53641,4.15444,0},
{-1.55401,4.14494,0},
{-1.57166,4.13554,0},
{-1.58941,4.12632,0},
{-1.60726,4.11732,0},
{-1.62516,4.10839,0},
{-1.64318,4.09973,0},
{-1.66132,4.0913,0},
{-1.67951,4.083,0},
{-1.69784,4.07501,0},
{-1.71625,4.06722,0},
{-1.73478,4.05969,0},
{-1.75346,4.05256,0},
{-1.77219,4.04555,0},
{-1.7911,4.03907,0},
{-1.81018,4.03311,0},
{-1.82932,4.02732,0},
{-1.84861,4.02209,0},
{-1.86801,4.01727,0},
{-1.88751,4.0129,0},
{-1.90716,4.00932,0},
{-1.92689,4.00606,0},
{-1.9467,4.00362,0},
{-1.96663,4.00211,0},
{-1.98657,4.00079,0},
{-2.00654,3.99991,0},
{-2.02654,3.99939,0},
{-2.04652,3.99871,0},
{-2.06649,3.99771,0},
{-2.08645,3.99655,0},
{-2.10638,3.99499,0},
{-2.12626,3.99294,0},
{-2.14613,3.99074,0},
{-2.16591,3.98794,0},
{-2.18562,3.9846,0},
{-2.20527,3.98098,0},
{-2.22477,3.97665,0},
{-2.24418,3.97189,0},
{-2.26342,3.96656,0},
{-2.28242,3.96041,0},
{-2.30135,3.95399,0},
{-2.31991,3.94666,0},
{-2.33811,3.9384,0},
{-2.35613,3.92981,0},
{-2.37355,3.92011,0},
{-2.39054,3.9096,0},
{-2.40707,3.89846,0},
{-2.42281,3.8862,0},
{-2.4382,3.87347,0},
{-2.45282,3.85991,0},
{-2.46654,3.8454,0},
{-2.47994,3.83062,0},
{-2.49217,3.81489,0},
{-2.50343,3.79838,0},
{-2.51421,3.78159,0},
{-2.524,3.7642,0},
{-2.53326,3.74649,0},
{-2.54177,3.72844,0},
{-2.54932,3.70995,0},
{-2.55659,3.69133,0},
{-2.56298,3.67241,0},
{-2.56856,3.65322,0},
{-2.5739,3.63396,0},
{-2.57861,3.61453,0},
{-2.58291,3.59501,0},
{-2.58685,3.57541,0},
{-2.59027,3.55571,0},
{-2.59351,3.53598,0},
{-2.59632,3.51618,0},
{-2.59867,3.49632,0},
{-2.60092,3.47645,0},
{-2.60284,3.45655,0},
{-2.60451,3.43662,0},
{-2.60606,3.41668,0},
{-2.60736,3.39673,0},
{-2.60857,3.37676,0},
{-2.60962,3.35679,0},
{-2.61051,3.33681,0},
{-2.61135,3.31683,0},
{-2.61208,3.29684,0},
{-2.61271,3.27685,0},
{-2.61331,3.25686,0},
{-2.61385,3.23687,0},
{-2.61436,3.21687,0},
{-2.61485,3.19688,0},
{-2.61531,3.17688,0},
{-2.61578,3.15689,0},
{-2.61627,3.13689,0},
{-2.61678,3.1169,0},
{-2.61731,3.09691,0},
{-2.61789,3.07692,0},
{-2.61852,3.05692,0},
{-2.61921,3.03694,0},
{-2.61999,3.01695,0},
{-2.62081,2.99697,0},
{-2.62174,2.97699,0},
{-2.62281,2.95702,0},
{-2.62392,2.93705,0},
{-2.6252,2.91709,0},
{-2.62659,2.89714,0},
{-2.62809,2.8772,0},
{-2.62978,2.85727,0},
{-2.63155,2.83735,0},
{-2.63352,2.81745,0},
{-2.63571,2.79757,0},
{-2.63797,2.7777,0},
{-2.64048,2.75786,0},
{-2.6432,2.73805,0},
{-2.64606,2.71826,0},
{-2.64919,2.69851,0},
{-2.65248,2.67878,0},
{-2.65601,2.6591,0},
{-2.65984,2.63948,0},
{-2.66377,2.61987,0},
{-2.66803,2.60033,0},
{-2.67261,2.58087,0},
{-2.67732,2.56143,0},
{-2.68239,2.54209,0},
{-2.68769,2.52281,0},
{-2.69325,2.50361,0},
{-2.69918,2.48451,0},
{-2.70525,2.46546,0},
{-2.7117,2.44653,0},
{-2.71855,2.42774,0},
{-2.72552,2.409,0},
{-2.73291,2.39042,0},
{-2.7406,2.37196,0},
{-2.74855,2.35362,0},
{-2.75694,2.33547,0},
{-2.76554,2.31741,0},
{-2.77452,2.29954,0},
{-2.78394,2.28191,0},
{-2.7935,2.26434,0},
{-2.80348,2.24702,0},
{-2.81383,2.22991,0},
{-2.82436,2.21292,0},
{-2.8353,2.19619,0},
{-2.84648,2.17961,0},
{-2.85795,2.16324,0},
{-2.86982,2.14716,0},
{-2.88181,2.13116,0},
{-2.89417,2.11546,0},
{-2.90689,2.10003,0},
{-2.91973,2.08471,0},
{-2.93292,2.06969,0},
{-2.94634,2.05486,0},
{-2.95999,2.04026,0},
{-2.974,2.026,0},
{-2.98815,2.01185,0},
{-3.0023,1.99773,0},
{-3.01648,1.98361,0},
{-3.03059,1.96944,0},
{-3.04445,1.95504,0},
{-3.05813,1.94046,0},
{-3.07164,1.92572,0},
{-3.08483,1.9107,0},
{-3.09788,1.89555,0},
{-3.11064,1.88016,0},
{-3.12305,1.86449,0},
{-3.13535,1.84873,0},
{-3.14728,1.83269,0},
{-3.15889,1.81641,0},
{-3.17031,1.79999,0},
{-3.18128,1.78327,0},
{-3.192,1.76639,0},
{-3.20239,1.74931,0},
{-3.21232,1.73195,0},
{-3.22212,1.71452,0},
{-3.23146,1.69685,0},
{-3.24037,1.67895,0},
{-3.24912,1.66097,0},
{-3.25739,1.64278,0},
{-3.26535,1.62444,0},
{-3.27299,1.60597,0},
{-3.28014,1.5873,0},
{-3.28711,1.56857,0},
{-3.2936,1.54966,0},
{-3.29958,1.53059,0},
{-3.3054,1.51146,0},
{-3.3107,1.49219,0},
{-3.3156,1.47281,0},
{-3.32021,1.45337,0},
{-3.32429,1.4338,0},
{-3.32812,1.41418,0},
{-3.33151,1.39448,0},
{-3.33435,1.37469,0},
{-3.33704,1.35488,0},
{-3.33919,1.33501,0},
{-3.34087,1.31509,0},
{-3.34232,1.29516,0},
{-3.34324,1.27519,0},
{-3.34385,1.2552,0},
{-3.34408,1.23522,0},
{-3.34377,1.21523,0},
{-3.34331,1.19524,0},
{-3.34233,1.17528,0},
{-3.34085,1.15534,0},
{-3.3392,1.13541,0},
{-3.33703,1.11554,0},
{-3.33453,1.0957,0},
{-3.33172,1.07591,0},
{-3.32841,1.05619,0},
{-3.32492,1.0365,0},
{-3.32098,1.01691,0},
{-3.31656,0.997403,0},
{-3.312,0.977936,0},
{-3.30697,0.958585,0},
{-3.30158,0.939326,0},
{-3.29596,0.92014,0},
{-3.28988,0.901091,0},
{-3.2836,0.882106,0},
{-3.27695,0.863249,0},
{-3.26987,0.844547,0},
{-3.26266,0.825894,0},
{-3.25501,0.807417,0},
{-3.247,0.789094,0},
{-3.2388,0.770853,0},
{-3.23019,0.752804,0},
{-3.22135,0.734865,0},
{-3.21223,0.717073,0},
{-3.20272,0.699483,0},
{-3.19309,0.681954,0},
{-3.18309,0.66464,0},
{-3.17273,0.647534,0},
{-3.16226,0.630503,0},
{-3.15143,0.613698,0},
{-3.14036,0.597046,0},
{-3.12907,0.580542,0},
{-3.11744,0.564276,0},
{-3.10569,0.548099,0},
{-3.09363,0.532153,0},
{-3.08125,0.516455,0},
{-3.06877,0.500833,0},
{-3.05598,0.48547,0},
{-3.04294,0.470307,0},
{-3.02975,0.455285,0},
{-3.01626,0.44053,0},
{-3.00262,0.425903,0},
{-2.98876,0.411505,0},
{-2.9746,0.397383,0},
{-2.96036,0.383346,0},
{-2.94585,0.369595,0},
{-2.9311,0.356095,0},
{-2.91624,0.342718,0},
{-2.90111,0.329636,0},
{-2.88584,0.316725,0},
{-2.87039,0.304031,0},
{-2.8547,0.29164,0},
{-2.83893,0.27934,0},
{-2.8229,0.26738,0},
{-2.80663,0.255752,0},
{-2.79028,0.244232,0},
{-2.77371,0.233049,0},
{-2.75698,0.222095,0},
{-2.74012,0.211345,0},
{-2.72305,0.200924,0},
{-2.70591,0.19063,0},
{-2.68859,0.180644,0},
{-2.67107,0.170994,0},
{-2.65351,0.161446,0},
{-2.63576,0.15224,0},
{-2.61787,0.143301,0},
{-2.5999,0.134541,0},
{-2.58177,0.126129,0},
{-2.56355,0.117884,0},
{-2.54521,0.109925,0},
{-2.52672,0.102318,0},
{-2.50818,0.0948166,0},
{-2.48951,0.0876694,0},
{-2.47071,0.0808357,0},
{-2.45186,0.0741756,0},
{-2.43286,0.0679344,0},
{-2.41378,0.0619379,0},
{-2.39462,0.0562147,0},
{-2.37535,0.0508746,0},
{-2.35605,0.0456488,0},
{-2.33665,0.0407889,0},
{-2.31717,0.0362904,0},
{-2.29766,0.0319059,0},
{-2.27808,0.0278862,0},
{-2.25844,0.0241172,0},
{-2.23877,0.02057,0},
{-2.21903,0.0173851,0},
{-2.19927,0.0143408,0},
{-2.17946,0.0116231,0},
{-2.1596,0.00926404,0},
{-2.13972,0.00703294,0},
{-2.11981,0.00525895,0},
{-2.09986,0.0038461,0},
{-2.0799,0.00262035,0},
{-2.05992,0.00176189,0},
{-2.03994,0.00108366,0},
{-2.01995,0.00060182,0},
{-1.99995,0.000362865,0},
{-1.97995,0.000193552,0},
{-1.95995,0.000239067,0},
{-1.93995,0.000476262,0},
{-1.91996,0.000755735,0},
{-1.89996,0.0011391,0},
{-1.87997,0.00158408,0},
{-1.85997,0.00211132,0},
{-1.83998,0.00275508,0},
{-1.81999,0.00343363,0},
{-1.80001,0.00421173,0},
{-1.78003,0.00508884,0},
{-1.76005,0.00600133,0},
{-1.74007,0.00702991,0},
{-1.72011,0.00813917,0},
{-1.70014,0.00930751,0},
{-1.68018,0.0105737,0},
{-1.66022,0.0118787,0},
{-1.64027,0.013288,0},
{-1.62033,0.0148128,0},
{-1.60039,0.0163703,0},
{-1.58046,0.0180338,0},
{-1.56054,0.0197817,0},
{-1.54062,0.0215811,0},
{-1.52071,0.0234832,0},
{-1.5008,0.0254365,0},
{-1.48091,0.0274865,0},
{-1.46102,0.0296575,0},
{-1.44115,0.0318635,0},
{-1.42128,0.0341798,0},
{-1.40143,0.0365956,0},
{-1.38158,0.0390549,0},
{-1.36175,0.0416232,0},
{-1.34192,0.044257,0},
{-1.32211,0.046971,0},
{-1.30231,0.0497997,0},
{-1.28251,0.0526629,0},
{-1.26274,0.0556556,0},
{-1.24298,0.058778,0},
{-1.22323,0.0619355,0},
{-1.2035,0.0652099,0},
{-1.18379,0.0685668,0},
{-1.16408,0.0719952,0},
{-1.1444,0.0755434,0},
{-1.12473,0.07914,0},
{-1.10507,0.0828469,0},
{-1.08544,0.0866768,0},
{-1.06582,0.0905465,0},
{-1.04622,0.0945543,0},
{-1.02665,0.0986735,0},
{-1.0071,0.10286,0},
{-0.987569,0.107183,0},
{-0.968058,0.111574,0},
{-0.94857,0.116072,0},
{-0.929114,0.120704,0},
{-0.909668,0.125375,0},
{-0.890255,0.130184,0},
{-0.870873,0.135118,0},
{-0.851505,0.140107,0},
{-0.832174,0.145238,0},
{-0.812866,0.150454,0},
{-0.793588,0.155773,0},
{-0.774351,0.161242,0},
{-0.755126,0.166757,0},
{-0.735949,0.172436,0},
{-0.716821,0.178279,0},
{-0.697708,0.184168,0},
{-0.678644,0.190211,0},
{-0.659614,0.196365,0},
{-0.640615,0.202612,0},
{-0.621669,0.209018,0},
{-0.602745,0.215489,0},
{-0.583871,0.222106,0},
{-0.565057,0.228886,0},
{-0.54626,0.235716,0},
{-0.527525,0.242714,0},
{-0.508841,0.249849,0},
{-0.490192,0.257071,0},
{-0.471612,0.264471,0},
{-0.453068,0.271964,0},
{-0.434587,0.279609,0},
{-0.416189,0.28745,0},
{-0.397814,0.295348,0},
{-0.379522,0.30343,0},
{-0.361304,0.311681,0},
{-0.343118,0.320004,0},
{-0.325021,0.328517,0},
{-0.306978,0.337146,0},
{-0.289001,0.345907,0},
{-0.27112,0.354861,0},
{-0.253268,0.363878,0},
{-0.235518,0.373089,0},
{-0.21787,0.3825,0},
{-0.200256,0.391973,0},
{-0.182758,0.401657,0},
{-0.165343,0.411491,0},
{-0.148006,0.421458,0},
{-0.130801,0.431653,0},
{-0.113653,0.441944,0},
{-0.0966212,0.452423,0},
{-0.0797203,0.463116,0},
{-0.0628388,0.473839,0},
{-0.045977,0.484594,0},
{-0.0291314,0.495374,0},
{-0.012146,0.505916,0},
{0.00513087,0.51596,0},
{0.0225602,0.525745,0},
{0.0402902,0.534941,0},
{0.0584059,0.543379,0},
{0.0766353,0.551579,0},
{0.0952258,0.558866,0},
{0.11415,0.56531,0},
{0.133164,0.571452,0},
{0.152411,0.576794,0},
{0.171804,0.581636,0},
{0.19131,0.585948,0},
{0.21098,0.589473,0},
{0.230701,0.592743,0},
{0.25052,0.595236,0},
{0.27044,0.596932,0},
{0.290371,0.598396,0},
{0.310339,0.599087,0},
{0.330334,0.599216,0},
{0.350319,0.598983,0},
{0.370288,0.598117,0},
{0.39025,0.596982,0},
{0.41017,0.595348,0},
{0.430041,0.59314,0},
{0.449893,0.590771,0},
{0.469674,0.587881,0},
{0.489394,0.584559,0},
{0.509068,0.581,0},
{0.528645,0.57694,0},
{0.54817,0.572615,0},
{0.567623,0.567991,0},
{0.58698,0.562979,0},
{0.60631,0.557855,0},
{0.625544,0.552389,0},
{0.644689,0.546606,0},
{0.663798,0.540714,0},
{0.682811,0.534524,0},
{0.701766,0.528145,0},
{0.720658,0.521592,0},
{0.73946,0.51478,0},
{0.758232,0.507881,0},
{0.776921,0.500764,0},
{0.795525,0.493422,0},
{0.814104,0.48602,0},
{0.832603,0.478424,0},
{0.851046,0.470685,0},
{0.869446,0.462848,0},
{0.887771,0.454837,0},
{0.906064,0.446751,0},
{0.924302,0.438542,0},
{0.942476,0.430193,0},
{0.960632,0.421806,0},
{0.978735,0.413303,0},
{0.996792,0.404705,0},
{1.01483,0.396063,0},
{1.03282,0.387327,0},
{1.05079,0.37854,0},
{1.06873,0.369697,0},
{1.08663,0.36078,0},
{1.10452,0.351842,0},
{1.12239,0.342849,0},
{1.14022,0.333805,0},
{1.15805,0.324747,0},
{1.17587,0.315652,0},
{1.19367,0.306534,0},
{1.21146,0.297404,0},
{1.22925,0.288255,0},
{1.24703,0.2791,0},
{1.26482,0.269946,0},
{1.2826,0.260792,0},
{1.30038,0.251643,0},
{1.31818,0.242514,0},
{1.33598,0.2334,0},
{1.3538,0.224308,0},
{1.37163,0.215256,0},
{1.38947,0.206221,0},
{1.40735,0.197239,0},
{1.42525,0.188319,0},
{1.44316,0.179422,0},
{1.46112,0.17062,0},
{1.47911,0.161897,0},
{1.49714,0.153229,0},
{1.51522,0.144682,0},
{1.53333,0.136202,0},
{1.5515,0.127834,0},
{1.56973,0.119618,0},
{1.58799,0.111448,0},
{1.60632,0.103465,0},
{1.62474,0.0956582,0},
{1.64318,0.0879325,0},
{1.66172,0.0804359,0},
{1.68032,0.0730873,0},
{1.699,0.0659424,0},
{1.71779,0.0591104,0},
{1.73663,0.0523872,0},
{1.75558,0.0460286,0},
{1.77467,0.0400531,0},
{1.79379,0.0342049,0},
{1.81303,0.0287914,0},
{1.83237,0.0237036,0},
{1.85179,0.0189549,0},
{1.87135,0.014823,0},
{1.89096,0.0109689,0},
{1.91068,0.00772103,0},
{1.93051,0.00518699,0},
{1.95036,0.00289673,-4.93072e-12},
{1.97028,0.00152355,-4.31708e-11},
{1.99026,0.000931615,-1.0979e-10},
{2.01024,0.000594201,-2.32747e-10},
{2.03023,0.000825821,-4.81659e-10},
{2.05023,0.00137195,-8.00188e-10},
{2.0702,0.00225928,-1.3164e-09},
{2.09015,0.00360359,-2.09736e-09},
{2.11009,0.00507727,-2.9532e-09},
{2.13001,0.00684679,-3.97884e-09},
{2.1499,0.00889858,-5.16647e-09},
{2.16978,0.0110404,-6.40554e-09},
{2.18963,0.0134431,-7.79362e-09},
{2.20946,0.0160167,-9.27935e-09},
{2.22926,0.0188113,-1.08901e-08},
{2.24901,0.0219486,-1.26948e-08},
{2.26875,0.025208,-1.45685e-08},
{2.28844,0.0287094,-1.65773e-08},
{2.30808,0.0324669,-1.8729e-08},
{2.32771,0.0362818,-2.09125e-08},
{2.3473,0.0402738,-2.31943e-08},
{2.36687,0.0444002,-2.55506e-08},
{2.38641,0.0486712,-2.79863e-08},
{2.40588,0.0532096,-3.05689e-08},
{2.42533,0.0578706,-3.32189e-08},
{2.44474,0.0627017,-3.59607e-08},
{2.46409,0.0677349,-3.88116e-08},
{2.48344,0.0728181,-4.16894e-08},
{2.50275,0.0780282,-4.46355e-08},
{2.52203,0.0833471,-4.76402e-08},
{2.54127,0.0887816,-5.07061e-08},
{2.56044,0.0944792,-5.39113e-08},
{2.57957,0.100324,-5.71945e-08},
{2.59867,0.106259,-6.05243e-08},
{2.61773,0.112315,-6.39171e-08},
{2.63677,0.118409,-6.73293e-08},
{2.65577,0.124647,-7.08163e-08},
{2.67473,0.131023,-7.4375e-08},
{2.69366,0.137468,-7.79682e-08},
{2.71253,0.144114,-8.1664e-08},
{2.73134,0.150895,-8.54277e-08},
{2.75014,0.157725,-8.92162e-08},
{2.76891,0.164635,-9.30442e-08},
{2.78766,0.171573,-9.68868e-08},
{2.80635,0.178703,-1.00825e-07},
{2.82495,0.18604,-1.04864e-07},
{2.84354,0.193413,-1.08921e-07},
{2.8621,0.200884,-1.13025e-07},
{2.88062,0.208428,-1.17165e-07},
{2.89911,0.216041,-1.21337e-07},
{2.91755,0.223784,-1.25572e-07},
{2.93597,0.231587,-1.29836e-07},
{2.95433,0.239518,-1.3416e-07},
{2.97262,0.247604,-1.38556e-07},
{2.9909,0.255725,-1.42968e-07},
{3.00914,0.263908,-1.4741e-07},
{3.02737,0.272147,-1.51878e-07},
{3.04555,0.280475,-1.56385e-07},
{3.06364,0.289008,-1.60986e-07},
{3.08167,0.297657,-1.65639e-07},
{3.09969,0.306344,-1.70309e-07},
{3.11768,0.315085,-1.75002e-07},
{3.13566,0.323844,-1.79704e-07},
{3.15356,0.332764,-1.84475e-07},
{3.17138,0.341839,-1.89315e-07},
{3.18919,0.350946,-1.94168e-07},
{3.20694,0.360148,-1.99061e-07},
{3.22467,0.369413,-2.03982e-07},
{3.24235,0.378748,-2.08931e-07},
{3.25998,0.388191,-2.13926e-07},
{3.27759,0.397674,-2.18938e-07},
{3.29513,0.40729,-2.24004e-07},
{3.31259,0.417049,-2.29129e-07},
{3.33003,0.426832,-2.34263e-07},
{3.34744,0.436673,-2.3942e-07},
{3.36483,0.44656,-2.44597e-07},
{3.38215,0.456548,-2.49813e-07},
{3.39937,0.466729,-2.55104e-07},
{3.41653,0.477,-2.60431e-07},
{3.43366,0.487313,-2.65773e-07},
{3.45077,0.497678,-2.71135e-07},
{3.46786,0.50807,-2.76507e-07},
{3.48484,0.518625,-2.8194e-07},
{3.50174,0.529321,-2.87427e-07},
{3.51862,0.540059,-2.92928e-07},
{3.53543,0.550891,-2.98463e-07},
{3.5522,0.561778,-3.04018e-07},
{3.56893,0.572745,-3.09602e-07},
{3.58558,0.583821,-3.15224e-07},
{3.60221,0.594932,-3.20859e-07},
{3.61873,0.606197,-3.26548e-07},
{3.63515,0.617614,-3.32288e-07},
{3.65156,0.629051,-3.38035e-07},
{3.66792,0.640549,-3.43802e-07},
{3.68426,0.652088,-3.49584e-07},
{3.7005,0.663757,-3.55408e-07},
{3.71659,0.675634,-3.61301e-07},
{3.73263,0.68759,-3.67219e-07},
{3.74861,0.699617,-3.73159e-07},
{3.76453,0.71172,-3.79123e-07},
{3.78042,0.723861,-3.85098e-07},
{3.7962,0.736146,-3.91118e-07},
{3.81189,0.748544,-3.97173e-07},
{3.82752,0.761023,-4.03252e-07},
{3.84302,0.773661,-4.09378e-07},
{3.85846,0.786376,-4.15526e-07},
{3.87384,0.799161,-4.21696e-07},
{3.88915,0.812032,-4.27889e-07},
{3.90442,0.824944,-4.34095e-07},
{3.9195,0.83808,-4.40362e-07},
{3.93441,0.851411,-4.46684e-07},
{3.94927,0.864792,-4.53018e-07},
{3.96402,0.87829,-4.59383e-07},
{3.97872,0.891858,-4.65766e-07},
{3.99332,0.905519,-4.72174e-07},
{4.0078,0.91931,-4.78615e-07},
{4.02225,0.933143,-4.85067e-07},
{4.03646,0.947214,-4.91577e-07},
{4.05043,0.96152,-4.98143e-07},
{4.06437,0.975867,-5.04719e-07},
{4.07816,0.990346,-5.11325e-07},
{4.09186,1.00491,-5.17951e-07},
{4.10545,1.01958,-5.24599e-07},
{4.11886,1.03441,-5.31284e-07},
{4.13221,1.04931,-5.37983e-07},
{4.14528,1.06444,-5.4473e-07},
{4.15803,1.07985,-5.51531e-07},
{4.17072,1.09531,-5.58344e-07},
{4.18319,1.11094,-5.65187e-07},
{4.19549,1.1267,-5.72055e-07},
{4.20766,1.14256,-5.7894e-07},
{4.21959,1.15861,-5.85856e-07},
{4.2314,1.17474,-5.92788e-07},
{4.24291,1.1911,-5.99753e-07},
{4.25402,1.20772,-6.0676e-07},
{4.26502,1.22442,-6.13777e-07},
{4.27562,1.24138,-6.20824e-07},
{4.28585,1.25855,-6.27898e-07},
{4.29593,1.27582,-6.34979e-07},
{4.30559,1.29331,-6.42081e-07},
{4.31502,1.31094,-6.49194e-07},
{4.32411,1.32873,-6.56317e-07},
{4.33272,1.34677,-6.63453e-07},
{4.34119,1.36488,-6.70593e-07},
{4.34909,1.38323,-6.77734e-07},
{4.35644,1.40182,-6.84878e-07},
{4.36357,1.4205,-6.92016e-07},
{4.36998,1.43943,-6.99134e-07},
{4.37589,1.45853,-7.0624e-07},
{4.38129,1.47777,-7.13314e-07},
{4.38584,1.49723,-7.20336e-07},
{4.39005,1.51678,-7.27338e-07},
{4.39346,1.53646,-7.34262e-07},
{4.39597,1.55628,-7.41102e-07},
{4.39819,1.57614,-7.47906e-07},
{4.3994,1.59605,-7.54581e-07},
{4.3998,1.61603,-7.61152e-07},
{4.39965,1.63598,-7.67632e-07},
{4.39841,1.65589,-7.7393e-07},
{4.39662,1.67578,-7.80138e-07},
{4.3939,1.69553,-7.86158e-07},
{4.39005,1.71511,-7.91945e-07},
{4.38585,1.73462,-7.97657e-07},
{4.38051,1.75382,-8.03091e-07},
{4.37416,1.77274,-8.08275e-07},
{4.36738,1.7915,-8.13335e-07},
{4.35954,1.80982,-8.18088e-07},
{4.35106,1.8279,-8.22657e-07},
{4.34192,1.84562,-8.27005e-07},
{4.33184,1.86283,-8.31037e-07},
{4.32147,1.8799,-8.34975e-07},
{4.3103,1.89643,-8.38609e-07},
{4.29834,1.91243,-8.41939e-07},
{4.28616,1.92827,-8.45183e-07},
{4.27331,1.94356,-8.48139e-07},
{4.25999,1.95848,-8.50893e-07},
{4.24625,1.97298,-8.53442e-07},
{4.2318,1.98678,-8.55648e-07},
{4.21708,2.00032,-8.57716e-07},
{4.202,2.01342,-8.59586e-07},
{4.18652,2.02606,-8.61235e-07},
{4.17094,2.03857,-8.62825e-07},
{4.15505,2.05069,-8.64233e-07},
{4.13892,2.06249,-8.65496e-07},
{4.1226,2.07402,-8.66642e-07},
{4.10592,2.08503,-8.6755e-07},
{4.08905,2.09578,-8.6834e-07},
{4.07198,2.10618,-8.68986e-07},
{4.05467,2.11617,-8.6945e-07},
{4.03729,2.12606,-8.69869e-07},
{4.01979,2.13573,-8.70198e-07},
{4.00219,2.14522,-8.70446e-07},
{3.98456,2.15467,-8.70581e-07}
};

// std::vector<std::vector<double>> getCSV(string name);

namespace gazebo
{
  class AnimatedBox : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

        // create the animation
       
        gazebo::common::PoseKeyFrame *key;

        // set starting location of the box
        // std::vector<std::vector<double>> fields;
        // fields = getCSV("track_simple_loop1.csv");
        int startValue_ = 500;
        double frequenz_ = 10;
        double vel_ = 0.6;
        double aveDist_ =  vel_ / frequenz_ / 0.02; 

        double deltaX_ =0;
        double deltaY_=0;
        double angle_=0;
        int curIdx_ = 0;
        int nextIdx_ = 0;
        bool openLoop = true;
        int i = 0;

       gazebo::common::PoseAnimationPtr anim(
              // name the animation "test",
              // make it last 10 seconds,
              // and set it on a repeat loop
              new gazebo::common::PoseAnimation("test", 0.02*1038/ vel_, true));

        while(openLoop){
          curIdx_ = startValue_ - aveDist_ * i;
          nextIdx_ = startValue_ -aveDist_ * (i+1);
          
          // Abort criteria one loop is finished 
          if (startValue_ - 1037 > nextIdx_){
            openLoop = false;
            curIdx_ = startValue_;
            nextIdx_ = startValue_ -aveDist_;
          }
          

          // avoid negative indices
          if (nextIdx_ < 0){
            nextIdx_ = 1037 + ( nextIdx_);
          }
          if (curIdx_ <0){
            curIdx_ = 1037 + (curIdx_);
          }
          // determine values
          // rotation at time instance
          deltaX_ = fields[nextIdx_][0] - fields[curIdx_][0];
          deltaY_ = fields[nextIdx_][1] - fields[curIdx_][1];        
          angle_ = atan(deltaY_/deltaX_);
          // small delta X_
          
          if (deltaX_ <0 ){
            angle_ = 3.14159 + angle_;
          }
          if (abs(deltaX_) < 0.02 ){
            if (deltaY_ >0){
            angle_ = 3.14159 /2;
            }
            else{
              angle_ = -3.14159 /2;
            }
          }
          key = anim->CreateKeyFrame(i /frequenz_);
          key->Translation(ignition::math::Vector3d(fields[curIdx_][0],fields[curIdx_][1],fields[curIdx_][2]));
          key->Rotation(ignition::math::Quaterniond(0, 0,angle_ ));
          i++;
          
          gzmsg << "CurIdex_:" << curIdx_<< "\n";
          gzmsg << "NextIdex_:"<< nextIdx_ << "\n";
          gzmsg << "Time :" << i/ frequenz_<< "\n";
          gzmsg << "Vector:"<< ignition::math::Vector3d(fields[curIdx_][0],fields[curIdx_][1],fields[curIdx_][2])<< "\n\n";
        }    
        // set final location equal to starting location
        
        

        // key = anim->CreateKeyFrame(0.0);
        // key->Translation(ignition::math::Vector3d(0, 0, 0));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 0)); 
        // // set waypoint location after 2 seconds
        // key = anim->CreateKeyFrame(2.0);
        // key->Translation(ignition::math::Vector3d(-50, -50, 0));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 1.5707));


        // key = anim->CreateKeyFrame(4.0);
        // key->Translation(ignition::math::Vector3d(10, 20, 0));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 1.5707));


        // key = anim->CreateKeyFrame(6.0);
        // key->Translation(ignition::math::Vector3d(-10, 20, 0));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 1.5707));


        // key = anim->CreateKeyFrame(8.0);
        // key->Translation(ignition::math::Vector3d(10, -20, 0));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 1.5707));

        // // set the animation
        _parent->SetAnimation(anim);
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(AnimatedBox)
}

// std::vector<std::vector<double>> getCSV(string name){
//     ifstream in(name);
//     vector<vector<double>> fields;
//     if (in) {
//         string line;
//         while (getline(in, line)) {
//             stringstream sep(line);
//             string field;
//             fields.push_back(vector<double>());
//             while (getline(sep, field, ',')) {
//                 fields.back().push_back(stod(field));
//             }
//         }
//         std::cout << "ERROR - did not find file:" << name << ".\n";
//         return fields;

//     }
//     std::vector<std::vector<double>> fields_out;
//     return fields_out;
// }
