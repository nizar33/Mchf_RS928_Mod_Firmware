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



// Rx Audio LPF 2.8K IIR filter ;  Elleptic stop band = -80 db  (3.00 ms max)  0.00dB ripples // Nizar 07 Mai 2021
// simulé par lowa Hills IIR filter designer  vers 6.5:
// LPF 2k8 ( 0.4660) ,  Elleptic 10em ordre, stop band = -80 db  (3.00 ms max)  (  0.00dB Ripples band ) fs = 12Khz
// lattice ARMA coefficients designed with MATLAb avec la fonction suivante :
// Lattice (treuillis) coeff calculés par Mathlab online 2020 avec la fonction suivante :
// format long
// a = [ 1.00000000000000000000, -1.85975319109059312,  3.95567888047550031,   -4.5510038987523922,    4.91165092316276652, -3.68859959554152095, 2.38502162168795584, -1.12071775023083031, 0.408237524392387385,  -0.0966297305925191985,  0.0122102504901905617 ]
// b = [ 0.00756757130637454889,  0.0334823182819604659, 0.0899441325754735566, 0.168955864040339465, 0.24211551308907584,   0.271964235414496702,   0.242115513089075884,  0.168955864040339465, 0.0899441325754735566,  0.0334823182819604659,  0.00756757130637454889]
// [k,v] = tf2latc(b,a)
// NB : Les coeffs doivent etre copié avec l'ordre inverse de ceux affichés sur mathlab ( copier/coller, ajouter les virgules, puis inverser l'ordre des coeffs)
// NB : pas d'inversion de signe des coeffs lors du copiage depuis l'editeur de Mathlab.
// NB : les correspondances entre k (10 coeff)  et v (11 coeff) sont correctes, à ne pas inverser


const   arm_iir_lattice_instance_f32  IIR_2k8_LPF_S =
{
    .numStages = 10,
    .pkCoeffs  = (float*) (const float[])
    {
    		 0.012210250490191,
			-0.073932700921798,
			 0.223784748238098,
			-0.384398771897832,
			 0.602981184368225,
			-0.484937542558251,
			 0.886041411955346,
			-0.278396670915914,
			 0.989296215201702,
    	    -0.140917637719722
    },

    .pvCoeffs = (float*) (const float[])
    {
			0.007567571306375,
			0.047556133167796,
			0.148408990325728,
			0.288902846695565,
			0.350275107774587,
			0.205107958281735,
		   -0.033304378423410,
		   -0.073789835417632,
		   -0.020653117364821,
			0.004808828889240,
			0.007164517177795
     }
};



// Rx Audio LPF 2.8K IIR filter ;  Elleptic stop band = -76 db  (11.6 ms max)  1.00dB ripples // Nizar 07 Mai 2021
// simulé par lowa Hills IIR filter designer  vers 6.5:
// LPF 2k8 ( 0.4660) ,  Elleptic 10em ordre, stop band = -76 db  (11.6 ms max)  (  1.00dB Ripples band ) fs = 12Khz
// lattice ARMA coefficients designed with MATLAb avec la fonction suivante :
// Lattice (treuillis) coeff calculés par Mathlab online 2020 avec la fonction suivante :
// format long
// a = [ 1.00000000000000000000,   -3.083245507293616240E0,   7.382421962573943120E0, -1.181790810476004650E1,   1.525746029717032660E1, -1.529467959503291840E1, 1.238709316015142780E1,   -7.845618456205539100E0,   3.815836377100379820E0, -1.295603640716817660E0,  2.650875018022312180E-1 ]
// b = [ 7.964907258921714470E-3,   2.292536438860729220E-2, 5.704266616763879580E-2,  9.459902398840455360E-2, 1.312190079865324320E-1,  1.433420552091664300E-1,  1.312190079865324320E-1,  9.459902398840457140E-2, 5.704266616763879580E-2,  2.292536438860729220E-2,  7.964907258921714470E-3]
// [k,v] = tf2latc(b,a)
// NB : Les coeffs doivent etre copié avec l'ordre inverse de ceux affichés sur mathlab ( copier/coller, ajouter les virgules, puis inverser l'ordre des coeffs)
// NB : pas d'inversion de signe des coeffs lors du copiage depuis l'editeur de Mathlab.
// NB : les correspondances entre k (10 coeff)  et v (11 coeff) sont correctes, à ne pas inverser


const   arm_iir_lattice_instance_f32  IIR_2k8_LPF =
{
    .numStages = 10,
    .pkCoeffs  = (float*) (const float[])
    {
    	  0.265087501802231,
		 -0.514423008300386,
		  0.657361564150447,
		 -0.677547913910428,
		  0.807797153173305,
		 -0.503077347742172,
		  0.967153859226828,
		 -0.234100081991921,
		  0.997877925834044,
    	 -0.131664451676716
    },

    .pvCoeffs = (float*) (const float[])
    {
    	 0.007964907258922,
		 0.047483128910688,
		 0.138169366705680,
		 0.223797904579387,
		 0.193630083808021,
		 0.055335534234796,
		-0.044668138005248,
		-0.015495022528740,
		 0.001621043250397,
		 0.000485880638911,
       	 0.000530665552511
      }
};

