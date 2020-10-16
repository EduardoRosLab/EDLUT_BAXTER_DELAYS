#!/usr/bin/env python

##**************************************************************************
 #                           target_reaching.py                        	   *
 #                           -------------------                           *
 # copyright            : (C) 2019 by Ignacio Abadia                       *
 # email                : iabadia@ugr.es                                   *
 #**************************************************************************/

##**************************************************************************
 #                                                                         *
 #   This program is free software; you can redistribute it and/or modify  *
 #   it under the terms of the GNU General Public License as published by  *
 #   the Free Software Foundation; either version 3 of the License, or     *
 #   (at your option) any later version.                                   *
 #                                                                         *
 #**************************************************************************/


import argparse
import struct
import sys
import numpy as np
import rospy
import math
import string
import os


class TargetReaching(object):

	def __init__(self):

		file_name_pos = "positions_target_reaching_1s.txt"
		file_name_vel = "velocity_target_reaching_1s.txt"
		self.file_pos = open(file_name_pos, "w")
		self.file_vel = open(file_name_vel, "w")

		self.center = [-0.75, -0.35, 0.0, 1.35, 0.0, 0.57, 0.0]
		self.target = [[-0.76252975, -0.03976575, 0.033638525, 0.594384, -0.0174617, 1.014016, -0.026173635],
				[-0.643411477669, -0.149656566222, 0.0343816638637, 0.8402545, -0.00498985343337, 0.877410921629, 0.079112452259],
				[-0.566444480515, -0.324843169914, 0.0251291465775, 1.27737, 0.0110770075248, 0.614633893219, 0.145417354743],
				[-0.587018462781, -0.439457277036, 0.0141374397728, 1.6383475, 0.0161953109444, 0.367370202715, 0.120907753765],
				[-0.73732625, -0.47445475, 0.010095775, 1.790935, -0.0209155, 0.249872, 0.014635025],
				[-0.912215375, -0.446382, 0.0220281625, 1.6665325, -0.0625956875, 0.3487395, -0.1014182125],
				[-0.961917, -0.339755, 0.0334756, 1.32165, -0.056663, 0.588842, -0.153697],
				[-0.887188625, -0.168535, 0.0314467375, 0.8849555, -0.0342683125, 0.8532415, -0.1160662875]]

		self.position_factor = [0.0000000000, 0.0000011811, 0.0000024476, 0.0000038057, 0.0000052621, 0.0000068239, 0.0000084986, 0.0000102945, 0.0000122203, 0.0000142854, 0.0000164999, 0.0000188745, 0.0000214208, 0.0000241512, 0.0000270791, 0.0000302186, 0.0000335850, 0.0000371947, 0.0000410652, 0.0000452153, 0.0000496652, 0.0000544363, 0.0000595519, 0.0000650367, 0.0000709172, 0.0000772218, 0.0000839810, 0.0000912273, 0.0000989956, 0.0001073234, 0.0001162506, 0.0001258200, 0.0001360774, 0.0001470720, 0.0001588561, 0.0001714859, 0.0001850215, 0.0001995270, 0.0002150711, 0.0002317271, 0.0002495735, 0.0002686939, 0.0002891780, 0.0003111211, 0.0003346252, 0.0003597991, 0.0003867588, 0.0004156281, 0.0004465388, 0.0004796312, 0.0005150550, 0.0005529693, 0.0005935433, 0.0006369570, 0.0006834014, 0.0007330794, 0.0007862062, 0.0008430099, 0.0009037321, 0.0009686284, 0.0010379689, 0.0011120391, 0.0011911400, 0.0012755885, 0.0013657185, 0.0014618807, 0.0015644430, 0.0016737911, 0.0017903283, 0.0019144757, 0.0020466725, 0.0021873753, 0.0023370580, 0.0024962115, 0.0026653430, 0.0028449750, 0.0030356444, 0.0032379011, 0.0034523069, 0.0036794332, 0.0039198596, 0.0041741711, 0.0044429561, 0.0047268036, 0.0050262998, 0.0053420256, 0.0056745528, 0.0060244404, 0.0063922315, 0.0067784492, 0.0071835924, 0.0076081329, 0.0080525108, 0.0085171313, 0.0090023610, 0.0095085248, 0.0100359033, 0.0105847293, 0.0111551866, 0.0117474076, 0.0123614724, 0.0129974076, 0.0136551866, 0.0143347293, 0.0150359033, 0.0157585248, 0.0165023610, 0.0172671313, 0.0180525108, 0.0188581329, 0.0196835924, 0.0205284492, 0.0213922315, 0.0222744404, 0.0231745528, 0.0240920256, 0.0250262998, 0.0259768036, 0.0269429561, 0.0279241711, 0.0289198596, 0.0299294332, 0.0309523069, 0.0319879011, 0.0330356444, 0.0340949750, 0.0351653430, 0.0362462115, 0.0373370580, 0.0384373753, 0.0395466725, 0.0406644757, 0.0417903283, 0.0429237911, 0.0440644430, 0.0452118807, 0.0463657185, 0.0475255885, 0.0486911400, 0.0498620391, 0.0510379689, 0.0522186284, 0.0534037321, 0.0545930099, 0.0557862062, 0.0569830794, 0.0581834014, 0.0593869570, 0.0605935433, 0.0618029693, 0.0630150550, 0.0642296312, 0.0654465388, 0.0666656281, 0.0678867588, 0.0691097991, 0.0703346252, 0.0715611211, 0.0727891780, 0.0740186939, 0.0752495735, 0.0764817271, 0.0777150711, 0.0789495270, 0.0801850215, 0.0814214859, 0.0826588561, 0.0838970720, 0.0851360774, 0.0863758200, 0.0876162506, 0.0888573234, 0.0900989956, 0.0913412273, 0.0925839810, 0.0938272218, 0.0950709172, 0.0963150367, 0.0975595519, 0.0988044363, 0.1000496652, 0.1012952153, 0.1025410652, 0.1037871947, 0.1050335850, 0.1062802186, 0.1075270791, 0.1087741512, 0.1100214208, 0.1112688745, 0.1125164999, 0.1137642854, 0.1150122203, 0.1162602945, 0.1175084986, 0.1187568239, 0.1200052621, 0.1212538057, 0.1225024476, 0.1237511811, 0.1250000000, 0.1262500000, 0.1275000000, 0.1287500000, 0.1300000000, 0.1312500000, 0.1325000000, 0.1337500000, 0.1350000000, 0.1362500000, 0.1375000000, 0.1387500000, 0.1400000000, 0.1412500000, 0.1425000000, 0.1437500000, 0.1450000000, 0.1462500000, 0.1475000000, 0.1487500000, 0.1500000000, 0.1512500000, 0.1525000000, 0.1537500000, 0.1550000000, 0.1562500000, 0.1575000000, 0.1587500000, 0.1600000000, 0.1612500000, 0.1625000000, 0.1637500000, 0.1650000000, 0.1662500000, 0.1675000000, 0.1687500000, 0.1700000000, 0.1712500000, 0.1725000000, 0.1737500000, 0.1750000000, 0.1762500000, 0.1775000000, 0.1787500000, 0.1800000000, 0.1812500000, 0.1825000000, 0.1837500000, 0.1850000000, 0.1862500000, 0.1875000000, 0.1887500000, 0.1900000000, 0.1912500000, 0.1925000000, 0.1937500000, 0.1950000000, 0.1962500000, 0.1975000000, 0.1987500000, 0.2000000000, 0.2012500000, 0.2025000000, 0.2037500000, 0.2050000000, 0.2062500000, 0.2075000000, 0.2087500000, 0.2100000000, 0.2112500000, 0.2125000000, 0.2137500000, 0.2150000000, 0.2162500000, 0.2175000000, 0.2187500000, 0.2200000000, 0.2212500000, 0.2225000000, 0.2237500000, 0.2250000000, 0.2262500000, 0.2275000000, 0.2287500000, 0.2300000000, 0.2312500000, 0.2325000000, 0.2337500000, 0.2350000000, 0.2362500000, 0.2375000000, 0.2387500000, 0.2400000000, 0.2412500000, 0.2425000000, 0.2437500000, 0.2450000000, 0.2462500000, 0.2475000000, 0.2487500000, 0.2500000000, 0.2512500000, 0.2525000000, 0.2537500000, 0.2550000000, 0.2562500000, 0.2575000000, 0.2587500000, 0.2600000000, 0.2612500000, 0.2625000000, 0.2637500000, 0.2650000000, 0.2662500000, 0.2675000000, 0.2687500000, 0.2700000000, 0.2712500000, 0.2725000000, 0.2737500000, 0.2750000000, 0.2762500000, 0.2775000000, 0.2787500000, 0.2800000000, 0.2812500000, 0.2825000000, 0.2837500000, 0.2850000000, 0.2862500000, 0.2875000000, 0.2887500000, 0.2900000000, 0.2912500000, 0.2925000000, 0.2937500000, 0.2950000000, 0.2962500000, 0.2975000000, 0.2987500000, 0.3000000000, 0.3012500000, 0.3025000000, 0.3037500000, 0.3050000000, 0.3062500000, 0.3075000000, 0.3087500000, 0.3100000000, 0.3112500000, 0.3125000000, 0.3137500000, 0.3150000000, 0.3162500000, 0.3175000000, 0.3187500000, 0.3200000000, 0.3212500000, 0.3225000000, 0.3237500000, 0.3250000000, 0.3262500000, 0.3275000000, 0.3287500000, 0.3300000000, 0.3312500000, 0.3325000000, 0.3337500000, 0.3350000000, 0.3362500000, 0.3375000000, 0.3387500000, 0.3400000000, 0.3412500000, 0.3425000000, 0.3437500000, 0.3450000000, 0.3462500000, 0.3475000000, 0.3487500000, 0.3500000000, 0.3512500000, 0.3525000000, 0.3537500000, 0.3550000000, 0.3562500000, 0.3575000000, 0.3587500000, 0.3600000000, 0.3612500000, 0.3625000000, 0.3637500000, 0.3650000000, 0.3662500000, 0.3675000000, 0.3687500000, 0.3700000000, 0.3712500000, 0.3725000000, 0.3737500000, 0.3750000000, 0.3762500000, 0.3775000000, 0.3787500000, 0.3800000000, 0.3812500000, 0.3825000000, 0.3837500000, 0.3850000000, 0.3862500000, 0.3875000000, 0.3887500000, 0.3900000000, 0.3912500000, 0.3925000000, 0.3937500000, 0.3950000000, 0.3962500000, 0.3975000000, 0.3987500000, 0.4000000000, 0.4012500000, 0.4025000000, 0.4037500000, 0.4050000000, 0.4062500000, 0.4075000000, 0.4087500000, 0.4100000000, 0.4112500000, 0.4125000000, 0.4137500000, 0.4150000000, 0.4162500000, 0.4175000000, 0.4187500000, 0.4200000000, 0.4212500000, 0.4225000000, 0.4237500000, 0.4250000000, 0.4262500000, 0.4275000000, 0.4287500000, 0.4300000000, 0.4312500000, 0.4325000000, 0.4337500000, 0.4350000000, 0.4362500000, 0.4375000000, 0.4387500000, 0.4400000000, 0.4412500000, 0.4425000000, 0.4437500000, 0.4450000000, 0.4462500000, 0.4475000000, 0.4487500000, 0.4500000000, 0.4512500000, 0.4525000000, 0.4537500000, 0.4550000000, 0.4562500000, 0.4575000000, 0.4587500000, 0.4600000000, 0.4612500000, 0.4625000000, 0.4637500000, 0.4650000000, 0.4662500000, 0.4675000000, 0.4687500000, 0.4700000000, 0.4712500000, 0.4725000000, 0.4737500000, 0.4750000000, 0.4762500000, 0.4775000000, 0.4787500000, 0.4800000000, 0.4812500000, 0.4825000000, 0.4837500000, 0.4850000000, 0.4862500000, 0.4875000000, 0.4887500000, 0.4900000000, 0.4912500000, 0.4925000000, 0.4937500000, 0.4950000000, 0.4962500000, 0.4975000000, 0.4987500000, 0.5000000000, 0.5012500000, 0.5025000000, 0.5037500000, 0.5050000000, 0.5062500000, 0.5075000000, 0.5087500000, 0.5100000000, 0.5112500000, 0.5125000000, 0.5137500000, 0.5150000000, 0.5162500000, 0.5175000000, 0.5187500000, 0.5200000000, 0.5212500000, 0.5225000000, 0.5237500000, 0.5250000000, 0.5262500000, 0.5275000000, 0.5287500000, 0.5300000000, 0.5312500000, 0.5325000000, 0.5337500000, 0.5350000000, 0.5362500000, 0.5375000000, 0.5387500000, 0.5400000000, 0.5412500000, 0.5425000000, 0.5437500000, 0.5450000000, 0.5462500000, 0.5475000000, 0.5487500000, 0.5500000000, 0.5512500000, 0.5525000000, 0.5537500000, 0.5550000000, 0.5562500000, 0.5575000000, 0.5587500000, 0.5600000000, 0.5612500000, 0.5625000000, 0.5637500000, 0.5650000000, 0.5662500000, 0.5675000000, 0.5687500000, 0.5700000000, 0.5712500000, 0.5725000000, 0.5737500000, 0.5750000000, 0.5762500000, 0.5775000000, 0.5787500000, 0.5800000000, 0.5812500000, 0.5825000000, 0.5837500000, 0.5850000000, 0.5862500000, 0.5875000000, 0.5887500000, 0.5900000000, 0.5912500000, 0.5925000000, 0.5937500000, 0.5950000000, 0.5962500000, 0.5975000000, 0.5987500000, 0.6000000000, 0.6012500000, 0.6025000000, 0.6037500000, 0.6050000000, 0.6062500000, 0.6075000000, 0.6087500000, 0.6100000000, 0.6112500000, 0.6125000000, 0.6137500000, 0.6150000000, 0.6162500000, 0.6175000000, 0.6187500000, 0.6200000000, 0.6212500000, 0.6225000000, 0.6237500000, 0.6250000000, 0.6262500000, 0.6275000000, 0.6287500000, 0.6300000000, 0.6312500000, 0.6325000000, 0.6337500000, 0.6350000000, 0.6362500000, 0.6375000000, 0.6387500000, 0.6400000000, 0.6412500000, 0.6425000000, 0.6437500000, 0.6450000000, 0.6462500000, 0.6475000000, 0.6487500000, 0.6500000000, 0.6512500000, 0.6525000000, 0.6537500000, 0.6550000000, 0.6562500000, 0.6575000000, 0.6587500000, 0.6600000000, 0.6612500000, 0.6625000000, 0.6637500000, 0.6650000000, 0.6662500000, 0.6675000000, 0.6687500000, 0.6700000000, 0.6712500000, 0.6725000000, 0.6737500000, 0.6750000000, 0.6762500000, 0.6775000000, 0.6787500000, 0.6800000000, 0.6812500000, 0.6825000000, 0.6837500000, 0.6850000000, 0.6862500000, 0.6875000000, 0.6887500000, 0.6900000000, 0.6912500000, 0.6925000000, 0.6937500000, 0.6950000000, 0.6962500000, 0.6975000000, 0.6987500000, 0.7000000000, 0.7012500000, 0.7025000000, 0.7037500000, 0.7050000000, 0.7062500000, 0.7075000000, 0.7087500000, 0.7100000000, 0.7112500000, 0.7125000000, 0.7137500000, 0.7150000000, 0.7162500000, 0.7175000000, 0.7187500000, 0.7200000000, 0.7212500000, 0.7225000000, 0.7237500000, 0.7250000000, 0.7262500000, 0.7275000000, 0.7287500000, 0.7300000000, 0.7312500000, 0.7325000000, 0.7337500000, 0.7350000000, 0.7362500000, 0.7375000000, 0.7387500000, 0.7400000000, 0.7412500000, 0.7425000000, 0.7437500000, 0.7450000000, 0.7462500000, 0.7475000000, 0.7487500000, 0.7500000000, 0.7512500000, 0.7525000000, 0.7537500000, 0.7550000000, 0.7562500000, 0.7575000000, 0.7587500000, 0.7600000000, 0.7612500000, 0.7625000000, 0.7637500000, 0.7650000000, 0.7662500000, 0.7675000000, 0.7687500000, 0.7700000000, 0.7712500000, 0.7725000000, 0.7737500000, 0.7750000000, 0.7762500000, 0.7775000000, 0.7787500000, 0.7800000000, 0.7812500000, 0.7825000000, 0.7837500000, 0.7850000000, 0.7862500000, 0.7875000000, 0.7887500000, 0.7900000000, 0.7912500000, 0.7925000000, 0.7937500000, 0.7950000000, 0.7962500000, 0.7975000000, 0.7987500000, 0.8000000000, 0.8012500000, 0.8025000000, 0.8037500000, 0.8050000000, 0.8062500000, 0.8075000000, 0.8087500000, 0.8100000000, 0.8112500000, 0.8125000000, 0.8137500000, 0.8150000000, 0.8162500000, 0.8175000000, 0.8187500000, 0.8200000000, 0.8212500000, 0.8225000000, 0.8237500000, 0.8250000000, 0.8262500000, 0.8275000000, 0.8287500000, 0.8300000000, 0.8312500000, 0.8325000000, 0.8337500000, 0.8350000000, 0.8362500000, 0.8375000000, 0.8387500000, 0.8400000000, 0.8412500000, 0.8425000000, 0.8437500000, 0.8450000000, 0.8462500000, 0.8475000000, 0.8487500000, 0.8500000000, 0.8512500000, 0.8525000000, 0.8537500000, 0.8550000000, 0.8562500000, 0.8575000000, 0.8587500000, 0.8600000000, 0.8612500000, 0.8625000000, 0.8637500000, 0.8650000000, 0.8662500000, 0.8675000000, 0.8687500000, 0.8700000000, 0.8712500000, 0.8725000000, 0.8737500000, 0.8750000000, 0.8762488189, 0.8774975524, 0.8787461943, 0.8799947379, 0.8812431761, 0.8824915014, 0.8837397055, 0.8849877797, 0.8862357146, 0.8874835001, 0.8887311255, 0.8899785792, 0.8912258488, 0.8924729209, 0.8937197814, 0.8949664150, 0.8962128053, 0.8974589348, 0.8987047847, 0.8999503348, 0.9011955637, 0.9024404481, 0.9036849633, 0.9049290828, 0.9061727782, 0.9074160190, 0.9086587727, 0.9099010044, 0.9111426766, 0.9123837494, 0.9136241800, 0.9148639226, 0.9161029280, 0.9173411439, 0.9185785141, 0.9198149785, 0.9210504730, 0.9222849289, 0.9235182729, 0.9247504265, 0.9259813061, 0.9272108220, 0.9284388789, 0.9296653748, 0.9308902009, 0.9321132412, 0.9333343719, 0.9345534612, 0.9357703688, 0.9369849450, 0.9381970307, 0.9394064567, 0.9406130430, 0.9418165986, 0.9430169206, 0.9442137938, 0.9454069901, 0.9465962679, 0.9477813716, 0.9489620311, 0.9501379609, 0.9513088600, 0.9524744115, 0.9536342815, 0.9547881193, 0.9559355570, 0.9570762089, 0.9582096717, 0.9593355243, 0.9604533275, 0.9615626247, 0.9626629420, 0.9637537885, 0.9648346570, 0.9659050250, 0.9669643556, 0.9680120989, 0.9690476931, 0.9700705668, 0.9710801404, 0.9720758289, 0.9730570439, 0.9740231964, 0.9749737002, 0.9759079744, 0.9768254472, 0.9777255596, 0.9786077685, 0.9794715508, 0.9803164076, 0.9811418671, 0.9819474892, 0.9827328687, 0.9834976390, 0.9842414752, 0.9849640967, 0.9856652707, 0.9863448134, 0.9870025924, 0.9876385276, 0.9882525924, 0.9888448134, 0.9894152707, 0.9899640967, 0.9904914752, 0.9909976390, 0.9914828687, 0.9919474892, 0.9923918671, 0.9928164076, 0.9932215508, 0.9936077685, 0.9939755596, 0.9943254472, 0.9946579744, 0.9949737002, 0.9952731964, 0.9955570439, 0.9958258289, 0.9960801404, 0.9963205668, 0.9965476931, 0.9967620989, 0.9969643556, 0.9971550250, 0.9973346570, 0.9975037885, 0.9976629420, 0.9978126247, 0.9979533275, 0.9980855243, 0.9982096717, 0.9983262089, 0.9984355570, 0.9985381193, 0.9986342815, 0.9987244115, 0.9988088600, 0.9988879609, 0.9989620311, 0.9990313716, 0.9990962679, 0.9991569901, 0.9992137938, 0.9992669206, 0.9993165986, 0.9993630430, 0.9994064567, 0.9994470307, 0.9994849450, 0.9995203688, 0.9995534612, 0.9995843719, 0.9996132412, 0.9996402009, 0.9996653748, 0.9996888789, 0.9997108220, 0.9997313061, 0.9997504265, 0.9997682729, 0.9997849289, 0.9998004730, 0.9998149785, 0.9998285141, 0.9998411439, 0.9998529280, 0.9998639226, 0.9998741800, 0.9998837494, 0.9998926766, 0.9999010044, 0.9999087727, 0.9999160190, 0.9999227782, 0.9999290828, 0.9999349633, 0.9999404481, 0.9999455637, 0.9999503348, 0.9999547847, 0.9999589348, 0.9999628053, 0.9999664150, 0.9999697814, 0.9999729209, 0.9999758488, 0.9999785792, 0.9999811255, 0.9999835001, 0.9999857146, 0.9999877797, 0.9999897055, 0.9999915014, 0.9999931761, 0.9999947379, 0.9999961943, 0.9999975524, 0.9999988189]



		self.trajectory_period = 1.0
		self.dt = self.trajectory_period / 1000

	def write_positions_and_velocities(self):
		final_positions = []
		source = []
		target = []
		new_position = []
		for x in range(0,8):
			for i in range(0,2):
				if i==0:
					source = self.center
					target = self.target[x]
				else:
					source = self.target[x]
					target = self.center
				
				difference = []
				for j in range(0, len(target)):
					difference.append(target[j] - source[j])
				
				#Moving arm for 1.0s
				count = 0
				index = 0
				while (count < 500):
					for joint in range(0,7):
						new_position.append(source[joint] + (difference[joint] * self.position_factor[index]))
					final_positions.append(new_position)
					new_position = []
					index+=2
					count+=1
				#Target position maintained for 0.0s
				count = 0
				while (count < 0):
					final_positions.append(target)
					count+=1

		#Write positions file
		for z in range(0,len(final_positions)):
			self.file_pos.write(str(final_positions[z][0])+" "+str(final_positions[z][1])+" "+str(final_positions[z][2])+" "+str(final_positions[z][3])+" "+str(final_positions[z][4])+" "+str(final_positions[z][5])+" "+str(final_positions[z][6])+"\n")

		self.file_pos.close()

		
		#Compute velocities
		final_positions.append(final_positions[0])
		final_velocities = []
		
		

		for z in range(0,len(final_positions) - 1):
			new_velocity = []
			for joint in range(0,7):
				#We compute the velocity as the variation in the position divided by the time (0.002s)
				new_velocity.append((final_positions[z+1][joint]-final_positions[z][joint])*500)
			final_velocities.append(new_velocity)


		#Write velocity file
		for z in range(0,len(final_velocities)):
			self.file_vel.write(str(final_velocities[z][0])+" "+str(final_velocities[z][1])+" "+str(final_velocities[z][2])+" "+str(final_velocities[z][3])+" "+str(final_velocities[z][4])+" "+str(final_velocities[z][5])+" "+str(final_velocities[z][6])+"\n")

		self.file_vel.close()




def main():
	create_positions = TargetReaching()
	create_positions.write_positions_and_velocities()

	return 0


if __name__ == '__main__':
	sys.exit(main())
