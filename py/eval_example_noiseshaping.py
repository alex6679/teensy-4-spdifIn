# -*- coding: utf-8 -*-
"""
Created on Wed Jan 22 08:01:32 2020

@author: kanali
"""

import matplotlib.pyplot as plt
import numpy as np
from scipy import signal
from scipy.fftpack import fft, ifft

#plt.close('all')

yTeensy=np.array([
-1,
15914,
26789,
29213,
22379,
8499,
-8095,
-22106,
-29149,
-26963,
-16266,
-419,
15555,
26616,
29262,
22658,
8893,
-7684,
-21831,
-29077,
-27135,
-16609,
-847,
15207,
26420,
29324,
22912,
9304,
-7286,
-21541,
-29007,
-27296,
-16955,
-1264,
14839,
26241,
29353,
23191,
9685,
-6864,
-21261,
-28925,
-27450,
-17304,
-1675,
14468,
26050,
29391,
23443,
10088,
-6460,
-20965,
-28840,
-27604,
-17637,
-2099,
14103,
25849,
29423,
23696,
10482,
-6052,
-20663,
-28755,
-27742,
-17980,
-2511,
13727,
25649,
29446,
23943,
10875,
-5641,
-20361,
-28657,
-27885,
-18306,
-2936,
13360,
25435,
29465,
24191,
11257,
-5220,
-20063,
-28548,
-28025,
-18628,
-3358,
12988,
25215,
29484,
24422,
11652,
-4814,
-19746,
-28445,
-28150,
-18954,
-3774,
12609,
24995,
29491,
24656,
12034,
-4395,
-19435,
-28330,
-28272,
-19276,
-4187,
12225,
24771,
29493,
24882,
12419,
-3981,
-19117,
-28208,
-28394,
-19585,
-4610,
11848,
24537,
29488,
25108,
12795,
-3562,
-18794,
-28088,
-28495,
-19910,
-5010,
11447,
24315,
29468,
25331,
13172,
-3148,
-18464,
-27960,
-28599,
-20217,
-5427,
11065,
24067,
29458,
25540,
13545,
-2722,
-18146,
-27812,
-28709,
-20511,
-5847,
10680,
23816,
29442,
25740,
13927,
-2316,
-17800,
-27681,
-28793,
-20819,
-6251,
10280,
23575,
29403,
25953,
14285,
-1886,
-17472,
-27527,
-28885,
-21112,
-6661,
9884,
23320,
29371,
26146,
14658,
-1477,
-17121,
-27382,
-28960,
-21407,
-7071,
9492,
23055,
29336,
26334,
15020,
-1051,
-16787,
-27212,
-29046,
-21686,
-7483,
9095,
22790,
29289,
26523,
15377,
-629,
-16441,
-27048,
-29113,
-21973,
-7883,
8687,
22531,
29226,
26713,
15728,
-206,
-16090,
-26880,
-29177,
-22248,
-8292,
8292,
22247,
29179,
26879,
16089,
210,
-15735,
-26705,
-29234,
-22525,
-8689,
7882,
21974,
29111,
27051,
16439,
629,
-15377,
-26523,
-29290,
-22787,
-9100,
7488,
21682,
29048,
27212,
16785,
1053,
-15021,
-26333,
-29337,
-23056,
-9490,
7070,
21406,
28964,
27375,
17129,
1470,
-14654,
-26146,
-29373,
-23318,
-9886,
6662,
21113,
28884,
27527,
17471,
1889,
-14289,
-25947,
-29411,
-23567,
-10287,
6257,
20815,
28795,
27680,
17801,
2314,
-13924,
-25743,
-29439,
-23819,
-10678,
5845,
20514,
28706,
27815,
18142,
2725,
-13546,
-25539,
-29461,
-24063,
-11070,
5433,
20212,
28602,
27957,
18467,
3147,
-13174,
-25326,
-29477,
-24304,
-11457,
5018,
19905,
28496,
28090,
18790,
3566,
-12797,
-25108,
-29486,
-24542,
-11842,
4604,
19591,
28389,
28210,
19118,
3977,
-12413,
-24889,
-29488,
-24773,
-12225,
4188,
19275,
28272,
28330,
19434,
4397,
-12036,
-24655,
-29492,
-24995,
-12608,
3772,
18955,
28150,
28443,
19750,
4809,
-11649,
-24423,
-29484,
-25216,
-12985,
3353,
18635,
28016,
28558,
20053,
5229,
-11264,
-24186,
-29468,
-25434,
-13360,
2937,
18303,
27889,
28652,
20368,
5633,
-10868,
-23947,
-29447,
-25643,
-13736,
2521,
17970,
27751,
28747,
20671,
6043,
-10473,
-23703,
-29418,
-25852,
-14102,
2098,
17640,
27600,
28845,
20960,
6464,
-10091,
-23439,
-29396,
-26045,
-14472,
1678,
17304,
27446,
28932,
21252,
6874,
-9694,
-23185,
-29356,
-26240,
-14840,
1267,
16949,
27305,
28996,
21553,
7274,
-9293,
-22922,
-29315,
-26427,
-15201,
842,
16612,
27134,
29077,
21833,
7681,
-8891,
-22659,
-29261,
-26616,
-15555,
418,
16267,
26963,
29148,
22109,
8090,
-8494,
-22384,
-29209,
-26790,
-15916,
4,
15908,
26797,
29204,
22388,
8491,
-8089,
-22108,
-29151,
-26958,
-16274,
-410,
15549,
26618,
29262,
22655,
8897,
-7686,
-21831,
-29076,
-27137,
-16610,
-840,
15195,
26436,
29305,
22931,
9287,
-7272,
-21550,
-29003,
-27296,
-16959,
-1257,
14831,
26248,
29349,
23191,
9689,
-6871,
-21253,
-28933,
-27443,
-17309,
-1672,
14466,
26050,
29393,
23441,
10089,
-6461,
-20964,
-28841,
-27602,
-17641,
-2094,
14098,
25855,
29416,
23703,
10475,
-6045,
-20669,
-28750,
-27748,
-17972,
-2521,
13738,
25638,
29454,
23939,
10876,
-5640,
-20364,
-28653,
-27890,
-18301,
-2939,
13361,
25434,
29467,
24187,
11264,
-5231,
-20048,
-28565,
-28009,
-18641,
-3350,
12985,
25215,
29485,
24422,
11649,
-4809,
-19750,
-28443,
-28150,
-18956,
-3770,
12604,
25000,
29486,
24661,
12030,
-4392,
-19438,
-28328,
-28272,
-19277,
-4186,
12224,
24771,
29495,
24878,
12425,
-3988,
-19111,
-28212,
-28391,
-19587,
-4608,
11846,
24538,
29488,
25108,
12795,
-3561,
-18797,
-28083,
-28502,
-19901,
-5020,
11458,
24304,
29476,
25327,
13172,
-3145,
-18467,
-27960,
-28596,
-20221,
-5423,
11062,
24067,
29461,
25536,
13550,
-2729,
-18138,
-27819,
-28703,
-20516,
-5844,
10678,
23818,
29440,
25743,
13922,
-2310,
-17807,
-27674,
-28799,
-20814,
-6255,
10282,
23574,
29404,
25953,
14284,
-1885,
-17474,
-27523,
-28889,
-21109,
-6664,
9887,
23318,
29372,
26146,
14656,
-1472,
-17128,
-27375,
-28967,
-21400,
-7076,
9494,
23055,
29334,
26338,
15016,
-1049,
-16786,
-27216,
-29041,
-21690,
-7480,
9093,
22792,
29286,
26526,
15375,
-628,
-16439,
-27053,
-29109,
-21975,
-7883,
8691,
22523,
29235,
26704,
15736,
-212,
-16086,
-26883,
-29174,
-22252,
-8287,
8286,
22255,
29170,
26888,
16080,
219,
-15744,
-26695,
-29244,
-22515,
-8699,
7890,
21970,
29113,
27050,
16439,
629,
-15376,
-26525,
-29287,
-22790,
-9098,
7487,
21682,
29049,
27210,
16788,
1050,
-15018,
-26336,
-29336,
-23053,
-9496,
7078,
21398,
28970,
27371,
17131,
1471,
-14658,
-26140,
-29381,
-23309,
-9894,
6668,
21109,
28886,
27526,
17472,
1886,
-14284,
-25952,
-29407,
-23570,
-10287,
6261,
20808,
28805,
27667,
17814,
2303,
-13915,
-25750,
-29434,
-23823,
-10674,
5841,
20518,
28702,
27819,
18139,
2727,
-13547,
-25539,
-29459,
-24066,
-11067,
5431,
20213,
28602,
27958,
18465,
3149,
-13175,
-25325,
-29477,
-24304,
-11459,
5022,
19901,
28500,
28086,
18794,
3563,
-12795,
-25110,
-29484,
-24544,
-11838,
4599,
19595,
28386,
28213,
19114,
3983,
-12421,
-24879,
-29497,
-24767,
-12229,
4191,
19273,
28274,
28328,
19437,
4393,
-12032,
-24658,
-29489,
-24998,
-12605,
3769,
18959,
28146,
28446,
19748,
4810,
-11647,
-24427,
-29480,
-25219,
-12984,
3354,
18632,
28020,
28554,
20056,
5228,
-11266,
-24183,
-29471,
-25431,
-13362,
2937,
18304,
27888,
28653,
20366,
5636,
-10871,
-23944,
-29450,
-25641,
-13737,
2522,
17970,
27749,
28751,
20665,
6051,
-10482,
-23694,
-29427,
-25843,
-14111,
2108,
17629,
27611,
28834,
20970,
6456,
-10084,
-23446,
-29389,
-26051,
-14468,
1676,
17303,
27450,
28927,
21257,
6869,
-9691,
-23186,
-29354,
-26244,
-14833,
1255,
16964,
27289,
29012,
21539,
7285,
-9302,
-22915,
-29320,
-26423,
-15205,
846,
16608,
27137,
29077,
21829,
7688,
-8900,
-22649,
-29270,
-26609,
-15561,
424,
16261,
26969,
29142,
22113,
8089,
-8495,
-22380,
-29216,
-26784,
-15919,
5,
15910,
26792,
29209,
22383,
8496,
-8092,
-22108,
-29148,
-26965,
-16263,
-422,
15559,
26611,
29266,
22654,
8897,
-7686,
-21831,
-29077,
-27135,
-16610,
-843,
15199,
26431,
29310,
22928,
9288,
-7273,
-21549,
-29004,
-27295,
-16959,
-1259,
14836,
26241,
29357,
23183,
9695,
-6873,
-21254,
-28929,
-27449,
-17303,
-1675,
14466,
26053,
29389,
23443,
10090,
-6464,
-20960,
-28843,
-27604,
-17636,
-2101,
14104,
25851,
29419,
23700,
10478,
-6049,
-20666,
-28751,
-27748,
-17972,
-2520,
13735,
25643,
29448,
23945,
10871,
-5637,
-20364,
-28656,
-27885,
-18307,
-2934,
13359,
25433,
29471,
24181,
11269,
-5232,
-20052,
-28556,
-28020,
-18631,
-3357,
12989,
25213,
29487,
24419,
11654,
-4815,
-19746,
-28444,
-28151,
-18953,
-3775,
12609,
24996,
29490,
24657,
12034,
-4396,
-19434,
-28331,
-28271,
-19276,
-4187,
12224,
24773,
29489,
24889,
12411,
-3974,
-19123,
-28204,
-28394,
-19587,
-4608,
11846,
24538,
29488,
25109,
12792,
-3556,
-18804,
-28075,
-28510,
-19894,
-5026,
11463,
24299,
29482,
25321,
13176,
-3146,
-18469,
-27954,
-28606,
-20209,
-5435,
11072,
24061,
29463,
25537,
13547,
-2725,
-18142,
-27816,
-28705,
-20515,
-5843,
10675,
23823,
29434,
25748,
13920,
-2311,
-17803,
-27679,
-28795,
-20816,
-6254,
10282,
23573,
29406,
25950,
14289,
-1892,
-17466,
-27531,
-28883,
-21110,
-6668,
9894,
23310,
29380,
26140,
14659,
-1472,
-17131,
-27369,
-28974,
-21394,
-7080,
9496,
23055,
29333,
26339,
15015,
-1047,
-16790,
-27209,
-29049,
-21684,
-7484,
9096,
22790,
29288,
26524,
15376,
-627,
-16443,
-27045,
-29118,
-21967,
-7889,
8693,
22525,
29232,
26708,
15731,
-208,
-16089,
-26879,
-29179,
-22246,
-8295,
8295,
22247,
29176,
26884,
16081,
219,
-15742,
-26699,
-29239,
-22521,
-8694,
7888,
21971,
29111,
27054,
16433,
637,
-15384,
-26518,
-29292,
-22789,
-9093,
7478,
21693,
29037,
27221,
16781,
1054,
-15020,
-26337,
-29332,
-23058,
-9490,
7071,
21404,
28966,
27374,
17130,
1469,
-14653,
-26147,
-29372,
-23319,
-9885,
6661,
21114,
28883,
27529,
17468,
1892,
-14292,
-25945,
-29410,
-23571,
-10281,
6250,
20821,
28792,
27680,
17803,
2311,
-13920,
-25748,
-29434,
-23823,
-10675,
5845,
20510,
28713,
27806,
18153,
2714,
-13536,
-25548,
-29453,
-24068,
-11068,
5434,
20208,
28608,
27952,
18471,
3144,
-13172,
-25327,
-29475,
-24307,
-11455,
5018,
19904,
28497,
28090,
18788,
3571,
-12804,
-25103,
-29487,
-24545,
-11835,
4595,
19599,
28383,
28214,
19115,
3980,
-12417,
-24883,
-29494,
-24768,
-12230,
4192,
19273,
28274,
28328,
19437,
4392,
-12029,
-24661,
-29488,
-24996,
-12610,
3778,
18948,
28157,
28437,
19754,
4807,
-11647,
-24424,
-29484,
-25215,
-12987,
3355,
18633,
28020,
28552,
20061,
5220,
-11256,
-24192,
-29463,
-25437,
-13359,
2937,
18303,
27889,
28654,
20363,
5640,
-10876,
-23939,
-29453,
-25640,
-13736,
2519,
17973,
27748,
28749,
20671,
6042,
-10471,
-23707,
-29413,
-25856,
-14100,
2099,
17636,
27606,
28839,
20964,
6461,
-10088,
-23442,
-29394,
-26045,
-14476,
1685,
17295,
27456,
28922,
21261,
6865,
-9685,
-23193,
-29348,
-26248,
-14833,
1260,
16956,
27298,
29004,
21545,
7280,
-9297,
-22920,
-29315,
-26429,
-15199,
841,
16613,
27132,
29079,
21831,
7683,
-8891,
-22662,
-29257,
-26620,
-15552,
417,
16266,
26965,
29145,
22112,
8087,
-8490,
-22388,
-29206,
-26793,
-15913,
2,
15910,
26794,
29207,
22385,
8494,
-8090,
-22110,
-29147,
-26964,
-16266,
-417,
15553,
26616,
29263,
22656,
8895,
-7684,
-21832,
-29077,
-27134,
-16613,
-838,
15195,
26433,
29310,
22925,
9294,
-7280,
-21542,
-29011,
-27288,
-16965,
-1254,
14831,
26246,
29352,
23188,
9691,
-6871,
-21254,
-28931,
-27446,
-17306,
-1673,
14465,
26053,
29389,
23445,
10087,
-6462,
-20961,
-28844,
-27600,
-17641,
-2097,
14103,
25849,
29422,
23698,
10478,
-6046,
-20671,
-28745,
-27754,
-17967,
-2524,
13738,
25641,
29449,
23945,
10870,
-5635,
-20367,
-28653,
-27887,
-18306,
-2934,
13358,
25434,
29470,
24183,
11266,
-5228,
-20057,
-28552,
-28022,
-18631,
-3354,
12982,
25222,
29477,
24430,
11643,
-4806,
-19752,
-28442,
-28149,
-18958,
-3768,
12601,
25004,
29482,
24664,
12029,
-4394,
-19433,
-28334,
-28268,
-19279,
-4185,
12223,
24774,
29489,
24885,
12419,
-3985,
-19109,
-28219,
-28381,
-19597,
-4600,
11840,
24542,
29487,
25107,
12797,
-3563,
-18796,
-28082,
-28505,
-19897,
-5024,
11460,
24305,
29473,
25332,
13165,
-3137,
-18476,
-27950,
-28606,
-20212,
-5430,
11066,
24067,
29458,
25540,
13547,
-2728,
-18137,
-27821,
-28700,
-20520,
-5840,
10674,
23823,
29435,
25747,
13919,
-2308,
-17808,
-27673,
-28801,
-20810,
-6260,
10287,
23568,
29411,
25945,
14294,
-1897,
-17462,
-27535,
-28879,
-21115,
-6661,
9885,
23319,
29371,
26148,
14653,
-1470,
-17128,
-27377,
-28963,
-21406,
-7071,
9492,
23055,
29335,
26336,
15017,
-1048,
-16789,
-27211,
-29046,
-21687,
-7482,
9094,
22792,
29288,
26522,
15379,
-632,
-16436,
-27053,
-29110,
-21974,
-7885,
8693,
22522,
29236,
26704,
15734,
-209,
-16089,
-26880,
-29177,
-22249,
-8291,
8291,
22249,
29176,
26882,
16086,
213,
-15738,
-26702,
-29236,
-22523,
-8691,
7883,
21975,
29110,
27052,
16438,
630,
-15377,
-26525,
-29285,
-22795,
-9089,
7476,
21692,
29041,
27216,
16786,
1049,
-15016,
-26339,
-29331,
-23059,
-9489,
7070,
21406,
28963,
27377,
17127,
1472,
-14655,
-26146,
-29373,
-23318,
-9886,
6663,
21111,
28887,
27523,
17476,
1882,
-14280,
-25956,
-29403,
-23573,
-10285,
6260,
20808,
28805,
27669,
17811,
2306,
-13919,
-25744,
-29441,
-23815,
-10683,
5849,
20513,
28703,
27822,
18133,
2734,
-13553,
-25536,
-29459,
-24068,
-11064,
5428,
20214,
28603,
27955,
18470,
3143,
-13169,
-25332,
-29469,
-24313,
-11448,
5010,
19911,
28492,
28093,
18787,
3571,
-12805,
-25098,
-29497,
-24531,
-11851,
4610,
19588,
28390,
28210,
19116,
3981,
-12418,
-24883,
-29493,
-24771,
-12225,
4186,
19279,
28267,
28335,
19430,
4399,
-12036,
-24655,
-29493,
-24993,
-12612,
3778,
18951,
28151,
28444,
19748,
4811,
-11649,
-24424,
-29483,
-25216,
-12987,
3357,
18630,
28022,
28552,
20059,
5224,
-11261,
-24187,
-29468,
-25433,
-13360,
2935,
18307,
27883,
28660,
20358,
5646,
-10883,
-23932,
-29460,
-25634,
-13740,
2521,
17973,
27747,
28752,
20665,
6050,
-10480,
-23697,
-29421,
-25852,
-14100,
2095,
17643,
27598,
28845,
20962,
6460,
-10085,
-23447,
-29387,
-26053,
-14467,
1676,
17302,
27452,
28924,
21260,
6866,
-9688,
-23188,
-29354,
-26243,
-14835,
1258,
16961,
27292,
29007,
21546,
7277,
-9293,
-22924,
-29314,
-26426,
-15204,
846,
16608,
27136,
29077,
21832,
7681,
-8889,
-22664,
-29255,
-26621,
-15552,
417,
16265,
26967,
29143,
22114,
8085,
-8489,
-22389,
-29203,
-26798,
-15906,
-8,
15921,
26785,
29214,
22380,
8497,
-8092,
-22109,
-29146,
-26967,
-16262,
-421,
15556,
26616,
29259,
22661,
8891,
-7682,
-21832,
-29077,
-27136,
-16609,
-845,
15204,
26424,
29318,
22919,
9296,
-7277,
-21550,
-28999,
-27302,
-16953,
-1262,
14835,
26245,
29352,
23187,
9693,
-6873,
-21254,
-28927,
-27454,
-17295,
-1686,
14477,
26045,
29392,
23446,
10083,
-6456,
-20968,
-28837,
-27607,
-17634,
-2103,
14107,
25847,
29423,
23697,
10480,
-6049,
-20667,
-28750,
-27748,
-17973,
-2518,
13732,
25646,
29447,
23944,
10873,
-5640,
-20361,
-28657,
-27886,
-18305,
-2936,
13359,
25437,
29462,
24194,
11255,
-5219,
-20062,
-28551,
-28021,
-18632,
-3353,
12982,
25221,
29479,
24426,
11650,
-4815,
-19742,
-28451,
-28143,
-18961,
-3767,
12601,
25004,
29482,
24665,
12026,
-4389,
-19439,
-28328,
-28271,
-19279,
-4183,
12220,
24776,
29489,
24885,
12417,
-3980,
-19117,
-28210,
-28389,
-19592,
-4602,
11840,
24543,
29484,
25112,
12791,
-3558,
-18798,
-28084,
-28499,
-19905,
-5017,
11457,
24303,
29480,
25321,
13178,
-3149,
-18466,
-27957,
-28603,
-20212,
-5431,
11067,
24066,
29458,
25542,
13542,
-2720,
-18147,
-27811,
-28709,
-20513,
-5844,
10676,
23822,
29435,
25748,
13917,
-2305,
-17811,
-27670,
-28803,
-20810,
-6259,
10287,
23568,
29410,
25948,
14289,
-1890,
-17469,
-27529,
-28882,
-21114,
-6662,
9887,
23317,
29374,
26145,
14656,
-1473,
-17125,
-27379,
-28962,
-21405,
-7073,
9494,
23053,
29336,
26336,
15017,
-1048,
-16789,
-27212,
-29045,
-21687,
-7483,
9096,
22789,
29290,
26522,
15378,
-631,
-16436,
-27055,
-29107,
-21977,
-7882,
8691,
22522,
29238,
26700,
15740,
-216,
-16082,
-26886,
-29172,
-22254,
-8285,
8285,
22254,
29173,
26882,
16088,
210,
-15735,
-26703,
-29239,
-22517,
-8699,
7891,
21969,
29112,
27054,
16433,
636,
-15382,
-26522,
-29286,
-22795,
-9089,
7477,
21690,
29044,
27212,
16789,
1049,
-15019,
-26334,
-29338,
-23051,
-9497,
7078,
21399,
28968,
27374,
17129,
1471,
-14654,
-26147,
-29373,
-23315,
-9891,
6669,
21105,
28891,
27522,
17475,
1884,
-14282,
-25956,
-29401,
-23577,
-10278,
6251,
20817,
28796,
27676,
17808,
2305,
-13914,
-25753,
-29430,
-23826,
-10673,
5841,
20518,
28702,
27819,
18139,
2727,
-13547,
-25539,
-29459,
-24067,
-11065,
5428,
20216,
28600,
27958,
18466,
3147,
-13172,
-25329,
-29473,
-24309,
-11454,
5019,
19901,
28502,
28083,
18796,
3562,
-12795,
-25109,
-29485,
-24543,
-11841,
4603,
19592,
28389,
28209,
19119,
3977,
-12414,
-24886,
-29492,
-24769,
-12228,
4189,
19276,
28270,
28333,
19432,
4397,
-12035,
-24656,
-29491,
-24995,
-12609,
3774,
18953,
28151,
28444,
19747,
4812,
-11650,
-24424,
-29481,
-25220,
-12983,
3354,
18631,
28024,
28547,
20064,
5220,
-11257,
-24191,
-29465,
-25436,
-13357,
2933,
18307,
27886,
28654,
20367,
5633,
-10867,
-23948,
-29446,
-25644,
-13735,
2520,
17971,
27750,
28749,
20669,
6045,
-10476,
-23701,
-29418,
-25852,
-14102,
2098,
17637,
27606,
28838,
20967,
6457,
-10085,
-23444,
-29391,
-26050,
-14469,
1678,
17301,
27452,
28924,
21261,
6865,
-9686,
-23191,
-29351,
-26244,
-14835,
1259,
16959,
27294,
29007,
21546,
7276,
-9291,
-22927,
-29308,
-26434,
-15195,
838,
16614,
27133,
29078,
21831,
7685,
-8897,
-22652,
-29268,
-26610,
-15559,
420,
16267,
26960,
29154,
22101,
8098,
-8499,
-22383,
-29207,
-26795,
-15909])


xf=np.linspace(0, fs, len(yTeensy), False)
plt.figure()
plt.plot(xf,20 * np.log10(np.abs(fft(yTeensy))/len(yTeensy)), '-g')#