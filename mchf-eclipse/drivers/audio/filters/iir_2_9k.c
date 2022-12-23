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

// alternative filter designed with MATLAB fdatools by DD4WH 2016-02-03 (original)
// 12k sampling rate, Lattice ARMA structure
// 10th order IIR Elliptic lowpass
// Fpass 2900Hz
// Astop 60dB
// coefficients in reverse order than that spit out by MATLAB
/*
#define IIR_2k9_numStages 10

const arm_iir_lattice_instance_f32 IIR_2k9_LPF =
{
    .numStages = IIR_2k9_numStages,
    .pkCoeffs  = (float*) (const float[])
    {

        0.251607461238023,
        -0.482252659454225,
        0.649010601492657,
        -0.587098281132217,
        0.884933496818835,
        -0.297598183921305,
        0.990030999070584,
        -0.113255360948473,
        0.999450459548208,
        -0.0609467228813790
    },

    .pvCoeffs = (float*) (const float[])
    {

        0.0200465239002972,
        0.0961515965852007,
        0.222315717811456,
        0.266325692682369,
        0.137175400914474,
        -0.0104425480557769,
        -0.0563674296455557,
        -0.00456438174446905,
        0.00418951198974554,
        0.000124467441153527,
        5.62783747425286e-05
    }
};
*/


// Rx Audio LPF 2.9K IIR filter ;  Elleptic stop band = -60 db  ( 4.16 ms max) 26/03/2021 by Nizar 3V8MN
// lattice ARMA coefficients designed with MATLAb fdatools
// LPF 2k9 ,  Elleptic 10em ordre, stop band = -60 db  ( 4.16 ms max)  (  0.00db Ripples band )
// Lattice (treuillis) coeff calculés par Mathlab-2018 avec la fonction suivante :
// format long
// a = [ 1.0000000000000000000, -1.089291497568376290,  3.18880662662342296,  -2.62320968698032031, 3.5861386745406536,   -2.16722448328987971,   1.700216889871764,   -0.699094834888498351, 0.303862577748830542, -0.06784122639445334,   0.010443110346194253]
// b = [ 0.0227111434659762512,  0.0851848513765560966, 0.217862647208957849,  0.390677614166482945, 0.549209859958141067, 0.611513917657108763,  0.549209859958141067, 0.390677614166482945, 0.217862647208957849,  0.0851848513765560966, 0.0227111434659762512 ]
// [k,v] = tf2latc(b,a)
// NB : Les coeffs doivent etre copié avec l'ordre inverse de ceux affichés sur mathlab ( copier/coller, ajouter les virgules, puis inverser l'ordre des coeffs)
// NB : pas d'inversion de signe des coeffs lors du copiage depuis l'editeur de Mathlab.
// NB : les correspondances entre k (10 coeff)  et v (11 coeff) sont correctes, à ne pas inverser

/*

#define IIR_2k9_numStages 10
const arm_iir_lattice_instance_f32 IIR_2k9_LPF =
{
    .numStages = IIR_2k9_numStages,
    .pkCoeffs  = (float*) (const float[])
    {
    		0.010443110346194,
    	   -0.056471793818334,
     	 	0.209779086250206,
    	   -0.279842570254606,
    		0.664108898610623,
    	   -0.278403172015099,
     	 	0.943760113352032,
    	   -0.127880732675711,
     	 	0.995438288766020,
           -0.068497307322295
    },

    .pvCoeffs = (float*) (const float[])
    {
			0.022711143465976,
			0.109923906854100,
			0.265115552888215,
			0.385529297221562,
			0.310607848978553,
			0.052798459096084,
		   -0.141813256784106,
		   -0.047449288495689,
			0.015285874324094,
			0.003191894561606,
			0.002147538827344
     }
};
*/

// Rx Audio LPF 2.9K IIR filter ;  Elleptic stop band = -60 db  ( 12.1 ms max) 17/05/2021 by Nizar 3V8MN
// lattice ARMA coefficients designed with MATLAb fdatools
// LPF 2k9 (0.4827) ,  Elleptic 10em ordre, stop band = -60 db  ( 12.1 ms max)  (  0.25db Ripples band )
// Lattice (treuillis) coeff calculés par Mathlab-2018 avec la fonction suivante :
// format long
// a = [ 1.00000000000000000000,  -2.0239572199821918, 5.03917102307387754, -6.66488828018858026,   8.5987950367699284,     -7.7773846310129553,  6.25141891845310216,   -3.74867873671110274,  1.82540584884270229, -0.6083493034633638,   0.130052620745269221 ]
// b = [ 0.0212937463260357696,   0.0596616183982141202, 0.150401258901012991, 0.247078691598676636, 0.344942775080490538,  0.374829095917828825, 0.344942775080490494,   0.247078691598676636, 0.150401258901012969, 0.0596616183982141202,  0.0212937463260357696]
// [k,v] = tf2latc(b,a)
// NB : Les coeffs doivent etre copié avec l'ordre inverse de ceux affichés sur mathlab ( copier/coller, ajouter les virgules, puis inverser l'ordre des coeffs)
// NB : pas d'inversion de signe des coeffs lors du copiage depuis l'editeur de Mathlab.
// NB : les correspondances entre k (10 coeff)  et v (11 coeff) sont correctes, à ne pas inverser

/*

#define IIR_2k9_numStages 10
const arm_iir_lattice_instance_f32 IIR_2k9_LPF =
{
    .numStages = IIR_2k9_numStages,
    .pkCoeffs  = (float*) (const float[])
    {
    	 0.130052620745269,
		-0.351066185306859,
		 0.565341365025704,
		-0.560798331311501,
		 0.847539771521561,
		-0.326853479253422,
		 0.983750226493247,
		-0.128137261208710,
		 0.999010062536800,
    	-0.067462162637181
    },

    .pvCoeffs = (float*) (const float[])
    {
    	 0.021293746326036,
		 0.102759250015263,
		 0.246387068393433,
		 0.325609956505720,
		 0.200432429461339,
		 0.001436365234322,
		-0.081205537526334,
		-0.010077925952286,
		 0.006949282392577,
		 0.000339439528760,
       	 0.000178585309385
      }
};

*/

// Rx Audio LPF 2.9K IIR filter ;  Elleptic stop band = -60 db  ( 9.33 ms max) 19/05/2021 by Nizar 3V8MN
// lattice ARMA coefficients designed with MATLAb fdatools
// LPF 2k9 (0.4827) ,  Elleptic 10em ordre, stop band = -60 db  ( 9.33 ms max)  (  0.10db Ripples band )
// Lattice (treuillis) coeff calculés par Mathlab-2018 avec la fonction suivante :
// format long
// a = [ 1.00000000000000000000,   -1.85781284599709284, 4.64541738850331054, -5.78995454247741304,   7.39040319639857923,     -6.36562124250726047,  5.00596832889701115,   -2.86528058721712853,   1.34657398376517223, -0.426293366530627793,    0.0841417290302548437 ]
// b = [ 0.0212788594308721191,   6.333222735739395760E-2, 1.588812712944448480E-1, 2.663033531622172050E-1, 3.710025598920459360E-1,  4.059454995908597040E-1, 3.710025598920459360E-1,  2.663033531622172050E-1, 1.588812712944448480E-1, 6.333222735739395760E-2,  2.127885943087211910E-2]
// [k,v] = tf2latc(b,a)
// NB : Les coeffs doivent etre copié avec l'ordre inverse de ceux affichés sur mathlab ( copier/coller, ajouter les virgules, puis inverser l'ordre des coeffs)
// NB : pas d'inversion de signe des coeffs lors du copiage depuis l'editeur de Mathlab.
// NB : les correspondances entre k (10 coeff)  et v (11 coeff) sont correctes, à ne pas inverser

/*


const arm_iir_lattice_instance_f32 IIR_2k9_LPF =
{
    .numStages = 10,
    .pkCoeffs  = (float*) (const float[])
    {
     	  0.084141729030255,
		 -0.271898778737891,
		  0.500607851730601,
		 -0.528000644434506,
		  0.821699653864467,
		 -0.333803672702611,
		  0.978775795579834,
		 -0.132764025272357,
		  0.998629469664592,
    	 -0.068600676545090
    },

    .pvCoeffs = (float*) (const float[])
    {
    	 0.021278859430872,
		 0.102864365756235,
		 0.248781493392269,
		 0.342624828091534,
		 0.228834722533478,
		 0.010918912704070,
		-0.093347782824959,
		-0.014498144815969,
		 0.008612932276460,
		 0.000560406380664,
    	 0.000311671370818
      }
};

*/

// Rx Audio LPF 2.9K IIR filter ;  Elleptic stop band = -68 db  ( 14.5 ms max) 20/07/2021 by Nizar 3V8MN
// lattice ARMA coefficients designed with MATLAb fdatools
// LPF 2k9 (0.4827) ,  Elleptic 10em ordre, stop band = -68 db  ( 14.5 ms max)  (  1.00db Ripples band )
// Lattice (treuillis) coeff calculés par Mathlab-2018 avec la fonction suivante :
// format long
// a = [ 1.00000000000000000000,   -2.488385432861901060E0,   6.035187664503271600E0, -8.971820591867990300E0,   1.165203172960717740E1, -1.143074105373583340E1, 9.416875964557364750E0,   -6.032338054600828950E0,   3.059993602433302320E0, -1.084640287962705510E0,  2.525563924887936910E-1 ]
// b = [ 1.423875286601797850E-2,   4.131335520664955570E-2, 1.038926052541168990E-1,  1.726918040623030230E-1, 2.408450235113829940E-1,  2.627568507597077390E-1,  2.408450235113829940E-1,  1.726918040623030230E-1,  1.038926052541168990E-1,  4.131335520664955570E-2,  1.423875286601797850E-2]
// [k,v] = tf2latc(b,a)
// NB : Les coeffs doivent etre copié avec l'ordre inverse de ceux affichés sur mathlab ( copier/coller, ajouter les virgules, puis inverser l'ordre des coeffs)
// NB : pas d'inversion de signe des coeffs lors du copiage depuis l'editeur de Mathlab.
// NB : les correspondances entre k (10 coeff)  et v (11 coeff) sont correctes, à ne pas inverser




const arm_iir_lattice_instance_f32 IIR_2k9_LPF =
{
    .numStages = 10,
    .pkCoeffs  = (float*) (const float[])
    {
    	 0.252556392488794,
		-0.487262550837532,
		 0.639762505918972,
		-0.626756115444366,
		 0.841353616985258,
		-0.379807087742391,
		 0.981242404934972,
		-0.146427587369126,
		 0.998855565303115,
       	-0.071599673974024
     },

    .pvCoeffs = (float*) (const float[])
    {
    	 0.014238752866018,
		 0.076744860420569,
		 0.199485532171320,
		 0.278726483349383,
		 0.188531880602529,
		 0.015029626685001,
		-0.066361265639676,
		-0.009977960138819,
		 0.005689941153609,
		 0.000341106858260,
	   	 0.000190537372098
       }
};









const arm_iir_lattice_instance_f32 IIR_2k9_BPF =
{
    .numStages = 10,
    .pkCoeffs  = (float*) (const float[])
    {
        0.263419613809752,
        -0.487748680969863,
        0.579151989014181,
        -0.766691138931688,
        0.644526948603595,
        -0.455099813849530,
        0.966822380971438,
        -0.994647491908158,
        0.999641053036454,
        -0.996851003050428
    },

    .pvCoeffs = (float*) (const float[])
    {
        -0.0324231770435912,
        -0.140665197185480,
        -0.244988773730778,
        -0.127152356090031,
        0.127397853986530,
        0.136542957067850,
        0.00600155113876917,
        0.00834595632603165,
        -0.000738848558080582,
        -2.00888540803704e-05,
        6.96894635783540e-07
    }
};



