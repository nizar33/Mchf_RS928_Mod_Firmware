/*  -*-  mode: c; tab-width: 4; indent-tabs-mode: t; c-basic-offset: 4; coding: utf-8  -*-  */
/************************************************************************************
**                                                                                 **
**                               mcHF QRP Transceiver                              **
**                             K Atanassov - M0NKA 2014                            **
**                                                                                 **
**---------------------------------------------------------------------------------**
**                                                                                 **
**  File name:                                                                     **
**  Description:                                                                   **
**  Last Modified:                                                                 **
**  Licence:		GNU GPLv3                                                      **
************************************************************************************/

#include "filters.h"
//
// alternative filter designed with MATLAB fdatools by DD4WH 2016-02-15
// 12k sampling rate, Lattice ARMA structure
// 10th order IIR Elliptic lowpass
// Fpass 2500Hz
// Astop 60dB
// coefficients in reverse order than that spit out by MATLAB (original)
//
// bandpass filter 150 - 2500Hz
//
//
/*
#define IIR_2k5_numStages 10
const arm_iir_lattice_instance_f32 IIR_2k5_LPF =
{
    .numStages = IIR_2k5_numStages,
    .pkCoeffs  = (float*) (const float[])
    {
         0.304008884053488,
        -0.586479754574523,
         0.728707358729566,
        -0.717591127840479,
         0.886460005029009,
        -0.509239560057348,
         0.988906686305070,
        -0.320749263130520,
         0.999445512653576,
        -0.267299520678722
    },

    .pvCoeffs = (float*) (const float[])
    {
        0.0121126909328103,
        0.0563705089486983,
        0.135077706371490,
        0.177369645199308,
        0.122940273900304,
        0.0249807561603719,
       -0.0272356075029574,
       -0.00442102257133439,
       -0.000487963028261214,
        9.54073216847401e-06,
        8.24834253008722e-05
    }
};
*/



// Rx Audio LPF 2.5K IIR filter ;  Elleptic stop band = -80 db  (3.1 ms max)
// lattice ARMA coefficients designed with MATLAb fdatools
// NIZZ 2021-04-28
// LPF 2k5 (0.4175), FS 12Khz ,  Elleptic 10em ordre, stop band = -80 db  (3.1 ms max)  (  0.0db Ripples band )


// format long
// a = [ 1.00000000000000000000, -2.95221225052378822 ,  6.18483438057519574 ,  -8.55100464948116823, 9.1876029153560026, -7.44329302682091765, 4.7058793978734581, -2.22969665383041793 , 0.768700450084464304,  -0.172355537069678189, 0.0194471601307744968]
// b = [ 0.00465360196839371554,  0.0152520479721136804, 0.0373350111446439081,   0.0641985290658250829,  0.0885740119779471335,   0.0978757820360794462,   0.0885740119779471335, 0.0641985290658250829, 0.0373350111446439081,  0.0152520479721136804, 0.00465360196839371554]
// [k,v] = tf2latc(b,a)



#define IIR_2k5_numStages 10
const arm_iir_lattice_instance_f32 IIR_2k5_LPF =
{
    .numStages = IIR_2k5_numStages,
    .pkCoeffs  = (float*) (const float[])
    {
     	 0.019447160130774,
    	-0.114986879816031,
    	 0.313606257924069,
    	-0.508829583425954,
    	 0.670521166575699,
    	-0.631021989435516,
   		 0.882028064183546,
     	-0.452919108271483,
     	 0.988249085160155,
        -0.294925505616180
    },

    .pvCoeffs = (float*) (const float[])
    {
    	  0.004653601968394,
		  0.028990468712267,
		  0.094074443011678,
		  0.199180936733267,
		  0.275364968162159,
		  0.215581781742923,
		  0.052643557150837,
		 -0.035735765216471,
		 -0.038137178892855,
		 -0.001406561024234,
    	  0.005797946017323
   }
};


// Rx Audio LPF 2.6K IIR filter ;  Elleptic stop band = -80 db  (3.06 ms max)
// lattice ARMA coefficients designed with MATLAb fdatools
// NIZZ 2021-04-28
// LPF 2k6 (0.4341), FS 12Khz ,  Elleptic 10em ordre, stop band = -80 db  (3.06 ms max)  (  0.0db Ripples band )
// format long
// a = [ 1.00000000000000000000,  -2.58186009457355192 ,  5.31974631788729457 ,  -7.00682092283482749,   7.45482254941459299, -5.92675248502574004, 3.75152006147319961, -1.77779009879051797, 0.621834170768726491 ,  -0.142037852146212762,  0.0165806738498161454 ]
// b = [ 0.00550212300226820794,  0.0202664663754732732, 0.0510101149557591249,  0.0907017499421507445, 0.126693857871920335,   0.140893695727636592,   0.126693857871920335,  0.0907017499421507267, 0.0510101149557591249,  0.0202664663754732732, 0.00550212300226820794]
// [k,v] = tf2latc(b,a)

#define IIR_2k6_numStages 10
const arm_iir_lattice_instance_f32 IIR_2k6_LPF =
{
    .numStages = IIR_2k6_numStages,
    .pkCoeffs  = (float*) (const float[])
    {
    	  0.016580673849816,
		 -0.099256159371032,
		  0.280436570179164,
		 -0.467183409573944,
		  0.646531066213921,
		 -0.586191950512651,
		  0.882467198786431,
		 -0.395490959784023,
		  0.988588573295094,
      	 -0.242907603319910
     },

    .pvCoeffs = (float*) (const float[])
    {
    	 0.005502123002268,
		 0.034472178190465,
		 0.110685825580957,
		 0.228688199287757,
		 0.304756048222296,
		 0.220835155658220,
		 0.030094823383936,
		-0.050351028417621,
		-0.036606387223931,
		 0.000411018996907,
    	 0.006866412045647
    }
};










const arm_iir_lattice_instance_f32 IIR_2k5_BPF =
{
    .numStages = IIR_2k5_numStages,
    .pkCoeffs  = (float*) (const float[])
    {
         0.319949354758169,
        -0.584143220123557,
         0.699676230227154,
        -0.809140966154901,
         0.704873387106954,
        -0.653974039723370,
         0.976867907943965,
        -0.994591635529659,
         0.999649711777868,
        -0.996834233858384
    },

    .pvCoeffs = (float*) (const float[])
    {
        -0.0194772962002087,
        -0.0868514981143777,
        -0.170190605680555,
        -0.131302479532905,
         0.0336504666821513,
         0.0956282907461719,
         0.0311274943500153,
         0.00344191566243132,
        -0.000363680054390594,
        -9.63538256713858e-06,
         3.36233973739775e-07
    }
};
