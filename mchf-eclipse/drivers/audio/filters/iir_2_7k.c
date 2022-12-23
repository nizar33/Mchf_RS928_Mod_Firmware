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
// Rx Audio LPF 2.7K IIR filter ;  Elleptic stop band = -60 db  (22 ms max)  original
// lattice ARMA coefficients designed with MATLAb fdatools
// DD4WH 2016-03-05
// inspired by Clint KA7OEI
//
// LPF 2k7 ,  Elleptic 10em ordre, stop band = -60 db  (22 ms max)  ( 1 dB Ripples band )

/*

const arm_iir_lattice_instance_f32 IIR_2k7_LPF_S =
{
    .numStages = 10,
    .pkCoeffs  = (float*) (const float[])  // a
    {
         0.276517739456119,
        -0.534458119364593,
         0.688627879533550,
        -0.657860665637030,
         0.883760854885895,
        -0.408163764378125,
         0.989378253422330,
        -0.218531341587163,
         0.999442039749756,
        -0.165077404674072
    },

    .pvCoeffs = (float*) (const float[])  // b
    {
        0.0155856864050234,
        0.0740622771621721,
        0.175501156733159,
        0.222640088831044,
        0.137215023898017,
        0.0116117418712191,
       -0.0432897242829734,
       -0.00519209144189112,
        0.00156095841010052,
        6.72317701914359e-05,
        9.08341518316962e-05
    }
};

*/


// Rx Audio LPF 2.7K IIR filter ;  Elleptic stop band = -70 db  (3.44 ms max) (  0.0db Ripples band ) fs = 12Khz  // Nizar 23 Fev 2021
// simul� par lowa Hills IIR filter designer  vers 6.5:
// LPF 2k7 ( 0.4500) ,  Elleptic 10em ordre, stop band = -70 db  (3.44 ms max @2689 Hz)  (  0.0db Ripples band ) fs = 12Khz
// lattice ARMA coefficients designed with MATLAb avec la fonction suivante :
// Lattice (treuillis) coeff calcul�s par Mathlab online 2020 avec la fonction suivante :
// format long
// a = [ 1.0000000000000000000, -2.06295231936133394 , 4.3833238092510225   , -5.26958312590995614, 5.71737738844338583, -4.36751007893832632, 2.83327946277509257,  -1.33126615722365038, 0.483636030035623588, -0.112738988913762017, 0.014051076870502941]
// b = [ 0.0104762969593732569, 0.0365470442759389158, 0.0911948222911868811, 0.159859159765020387, 0.222417500192440976, 0.246627450060678122, 0.222417500192440976, 0.159859159765020387, 0.0911948222911868811, 0.0365470442759389158, 0.0104762969593732569]
// [k,v] = tf2latc(b,a)
// NB : Les coeffs doivent etre copi� avec l'ordre inverse de ceux affich�s sur mathlab ( copier/coller, ajouter les virgules, puis inverser l'ordre des coeffs)
// NB : pas d'inversion de signe des coeffs lors du copiage depuis l'editeur de Mathlab.
// NB : les correspondances entre k (10 coeff)  et v (11 coeff) sont correctes, � ne pas inverser


const   arm_iir_lattice_instance_f32  IIR_2k7_LPF_S =
{
    .numStages = 10,
    .pkCoeffs  = (float*) (const float[])
    {
     	 0.014051076870503,
		-0.083768826004856,
		 0.251179035475081,
		-0.410378544167893,
		 0.647204489795616,
		-0.485227875528627,
		 0.913525629585947,
		-0.289664841999105,
		 0.992553245531578,
	    -0.181989206831686
    },

    .pvCoeffs = (float*) (const float[])
    {
    	0.010476296959373,
		0.058159145386596,
		0.165184908494510,
		0.297577983249454,
		0.333905110149336,
		0.175131521293748,
	   -0.043082470139573,
	   -0.058548161365789,
	   -0.017666253213322,
		0.002138133739242,
		0.005024871122291
    }
};



// Rx Audio LPF 2.7K IIR filter ;  Elleptic stop band = -76 db  (11.9 ms max)  1.00dB ripples // Nizar 19 juillet 2021
// simul� par lowa Hills IIR filter designer  vers 6.5:
// LPF 2k7  ( 0.4500) ,  Elleptic 10em ordre, stop band = -76 db  (11.9 ms max)  (  1.00dB Ripples band ) fs = 12Khz
// lattice ARMA coefficients designed with MATLAb avec la fonction suivante :
// Lattice (treuillis) coeff calcul�s par Mathlab online 2020 avec la fonction suivante :
// format long
// a = [ 1.00000000000000000000,   -3.465672018828946490,   8.454368360177067390E0, -1.404654088382965020E1,   1.829520901773937110E1, -1.851121990999741840E1, 1.491563138939011160E1,   -9.349693232307355830E0,   4.439988946009403750E0, -1.459000653895814900E0,  2.774587698788879560E-1 ]
// b = [ 6.849667527605306280E-3,    1.690847674305642420E-2, 4.235428377386796230E-2,  6.672759182293241940E-2, 9.259766908657208970E-2,  9.965440642758983000E-2,  9.259766908657208970E-2,  6.672759182293241050E-2, 4.235428377386795340E-2,  1.690847674305642420E-2,  6.849667527605306280E-3]
// [k,v] = tf2latc(b,a)
// NB : Les coeffs doivent etre copi� avec l'ordre inverse de ceux affich�s sur mathlab ( copier/coller, ajouter les virgules, puis inverser l'ordre des coeffs)
// NB : pas d'inversion de signe des coeffs lors du copiage depuis l'editeur de Mathlab.
// NB : les correspondances entre k (10 coeff)  et v (11 coeff) sont correctes, � ne pas inverser


const   arm_iir_lattice_instance_f32  IIR_2k7_LPF =
{
    .numStages = 10,
    .pkCoeffs  = (float*) (const float[])
    {
    	 0.277458769878888,
		-0.538906388067598,
		 0.679030178046975,
		-0.702817929470082,
		 0.812700613327298,
		-0.549931345230059,
		 0.965793412160316,
		-0.285722334504026,
		 0.997833379343734,
      	-0.181545282715784
     },

    .pvCoeffs = (float*) (const float[])
    {
    	 0.006849667527605,
		 0.040647177831759,
		 0.119236717251575,
		 0.196658028723970,
		 0.178001500876698,
		 0.061793852648391,
		-0.030972548736252,
		-0.014228493880011,
		-0.001235331797216,
		 0.000301826329245,
    	 0.000559233273866
     }
};


// Rx Audio LPF 2.7K IIR filter ;  Elleptic stop band = -70 db  (10.1 ms max)  0.40dB ripples // Nizar 18 juillet 2021
// simul� par lowa Hills IIR filter designer  vers 6.5:
// LPF 2k7  ( 0.4500) ,  Elleptic 10em ordre, stop band = -70 db  (10.1 ms max)  (  0.40dB Ripples band ) fs = 12Khz
// lattice ARMA coefficients designed with MATLAb avec la fonction suivante :
// Lattice (treuillis) coeff calcul�s par Mathlab online 2020 avec la fonction suivante :
// format long
// a = [ 1.00000000000000000000,   -3.15959193485279943,   7.51090699436101961, -11.9136444921550111,   15.0871770371812985, -14.7287391344651253, 11.5272023154201619,   -6.9877585297413658,   3.21254187800816915, -1.01785340528774593,   0.184288991833392801 ]
// b = [ 0.00934127965525383175,    0.0220053301588088868, 5.562983317264720280E-2,  8.615110578478441640E-2, 1.199787191713442840E-1,  1.283171844163161880E-1,  1.199787191713442610E-1,  8.615110578478441640E-2, 5.562983317264720280E-2,  2.200533015880888680E-2,  9.341279655253831750E-3]
// [k,v] = tf2latc(b,a)
// NB : Les coeffs doivent etre copi� avec l'ordre inverse de ceux affich�s sur mathlab ( copier/coller, ajouter les virgules, puis inverser l'ordre des coeffs)
// NB : pas d'inversion de signe des coeffs lors du copiage depuis l'editeur de Mathlab.
// NB : les correspondances entre k (10 coeff)  et v (11 coeff) sont correctes, � ne pas inverser

/*
const   arm_iir_lattice_instance_f32  IIR_2k7_LPF =
{
    .numStages = 10,
    .pkCoeffs  = (float*) (const float[])
    {
    	 0.184288991833393,
		-0.450888668999666,
		 0.634473955792387,
		-0.674079399294392,
		 0.815013653779851,
		-0.518589429399963,
		 0.969788402522280,
		-0.270122085319427,
		 0.998102388845639,
      	-0.178269652146017
    },

    .pvCoeffs = (float*) (const float[])
    {
    	0.009341279655254,
		0.051519962018753,
		0.143969416592426,
		0.230183248539924,
		0.199542680813530,
		0.060859607898900,
	   -0.041842525406622,
	   -0.015206885254694,
	   -0.000425709004716,
		0.000311481925537,
    	0.000548741614992
     }
};

*/



// Rx Audio LPF 2.7K IIR filter ;  Elleptic stop band = -80 db  (3.01 ms max)  0.00dB ripples // Nizar 07 Mai 2021
// simul� par lowa Hills IIR filter designer  vers 6.5:
// LPF 2k7  ( 0.4500) ,  Elleptic 10em ordre, stop band = -80 db  (3.01 ms max)  (  0.00dB Ripples band ) fs = 12Khz
// lattice ARMA coefficients designed with MATLAb avec la fonction suivante :
// Lattice (treuillis) coeff calcul�s par Mathlab online 2020 avec la fonction suivante :
// format long
// a = [ 1.00000000000000000000, -2.22354908959085762,  4.58959444495248636,   -5.70502851674837697,    6.06623937897229926, -4.71142140449206792, 3.00085871879283728,  -1.41936430326514063, 0.505223373264397324,  -0.117514625513794857,  0.0142331935549430133 ]
// b = [ 0.00645280940841098882,  0.0261930974237936498, 0.0680305195849324473, 0.124467928157334207, 0.176072841959229143,  0.196836776859324214, 0.176072841959229187, 0.124467928157334184, 0.0680305195849324473, 0.0261930974237936498,  0.00645280940841098882]
// [k,v] = tf2latc(b,a)
// NB : Les coeffs doivent etre copi� avec l'ordre inverse de ceux affich�s sur mathlab ( copier/coller, ajouter les virgules, puis inverser l'ordre des coeffs)
// NB : pas d'inversion de signe des coeffs lors du copiage depuis l'editeur de Mathlab.
// NB : les correspondances entre k (10 coeff)  et v (11 coeff) sont correctes, � ne pas inverser

/*
const   arm_iir_lattice_instance_f32  IIR_2k7_LPF_S =
{
    .numStages = 10,
    .pkCoeffs  = (float*) (const float[])
    {
    	 0.014233193554943,
		-0.085883819613161,
		 0.250977232890623,
		-0.426332137063513,
		 0.624228248079949,
		-0.538341584180660,
		 0.883833248364750,
		-0.338114369594603,
		 0.988934632604968,
        -0.192355057483474
    },

    .pvCoeffs = (float*) (const float[])
    {
    	 0.006452809408411,
		 0.040541235909169,
		 0.128510611918030,
		 0.258289976882883,
		 0.329743935752644,
		 0.217629076489486,
		 0.001431006430704,
		-0.063329204706015,
		-0.030862141108781,
		 0.002531052517024,
    	 0.007360399779446
    }
};

*/



// Rx Audio LPF 2.75K IIR filter ;  Elleptic stop band = -80 db  (3.01 ms max)  0.00dB ripples // Nizar 26 Fev 2021
// simul� par lowa Hills IIR filter designer  vers 6.5:
// LPF 2k75 ( 0.4580) ,  Elleptic 10em ordre, stop band = -80 db  (3.01 ms max)  (  0.00dB Ripples band ) fs = 12Khz
// lattice ARMA coefficients designed with MATLAb avec la fonction suivante :
// Lattice (treuillis) coeff calcul�s par Mathlab online 2020 avec la fonction suivante :
// format long
// a = [ 1.00000000000000000000, -2.0420377365549256   , 4.25978308361425562  , -5.10971345615380734, 5.45996931034561594 , -4.17694879499285143 , 2.67668442489661862, -1.2629229474253858  ,  0.454368887485009676 , -0.106634524332365577 ,  0.013182184444152365  ]
// b = [ 0.00698889277917462159,  0.0296568598908042613, 0.0783243509626228018, 0.145238557812113633, 0.206787540154632277,  0.231738028127620943, 0.206787540154632232, 0.145238557812113633,  0.0783243509626228107,  0.0296568598908042658,  0.00698889277917462159]
// [k,v] = tf2latc(b,a)
// NB : Les coeffs doivent etre copi� avec l'ordre inverse de ceux affich�s sur mathlab ( copier/coller, ajouter les virgules, puis inverser l'ordre des coeffs)
// NB : pas d'inversion de signe des coeffs lors du copiage depuis l'editeur de Mathlab.
// NB : les correspondances entre k (10 coeff)  et v (11 coeff) sont correctes, � ne pas inverser

#define IIR_2k75_numStages 10
const   arm_iir_lattice_instance_f32  IIR_2k75_LPF =
{
    .numStages = IIR_2k75_numStages,
    .pkCoeffs  = (float*) (const float[])
    {
    	  0.013182184444152,
		 -0.079729860904050,
		  0.237064245640883,
		 -0.405462681498745,
		  0.613419066990369,
		 -0.512319661956419,
		  0.884840237075161,
		 -0.308479969475244,
		  0.989114138642434,
       	 -0.166697970589127
    },

    .pvCoeffs = (float*) (const float[])
    {
    	 0.006988892779175,
		 0.043928442682615,
		 0.138210551998200,
		 0.273528366408483,
		 0.340681681195045,
		 0.212586993730763,
		-0.015300838210974,
		-0.068983324055214,
		-0.026300515330250,
		 0.003668724105880,
         0.007356398977336
     }
};




// Rx Audio LPF 2.77K IIR filter ;  Elleptic stop band = -80 db  (3.01 ms max)  0.00dB ripples // Nizar 24 Avril 2021
// simul� par lowa Hills IIR filter designer  vers 6.5:
// LPF 2k77 ( 0.4609) ,  Elleptic 10em ordre, stop band = -80 db  (3.01 ms max)  (  0.00dB Ripples band ) fs = 12Khz
// lattice ARMA coefficients designed with MATLAb avec la fonction suivante :
// Lattice (treuillis) coeff calcul�s par Mathlab online 2020 avec la fonction suivante :
// format long
// a = [ 1.00000000000000000000, -1.97604696882006126   ,  4.1465535043987316 , -4.90308834775923597, 5.25465470816549018, -3.9948021984756128, 2.56732324935610379, -1.20979505468013904 ,  0.437119388423779309,  -0.102910427031825935, 0.0128210174120670439]
// b = [ 0.00719357741782404503,  0.031000148640749261, 0.082376203514689692,  0.1534778697674686,  0.219031536162009521,   0.245670199983815696,  0.219031536162009521, 0.1534778697674686, 0.082376203514689692,  0.0310001486407492655, 0.00719357741782404503]
// [k,v] = tf2latc(b,a)
// NB : Les coeffs doivent etre copi� avec l'ordre inverse de ceux affich�s sur mathlab ( copier/coller, ajouter les virgules, puis inverser l'ordre des coeffs)
// NB : pas d'inversion de signe des coeffs lors du copiage depuis l'editeur de Mathlab.
// NB : les correspondances entre k (10 coeff)  et v (11 coeff) sont correctes, � ne pas inverser
/*
#define IIR_2k7_numStages 10
const   arm_iir_lattice_instance_f32  IIR_2k7_LPF =
{
    .numStages = IIR_2k7_numStages,
    .pkCoeffs  = (float*) (const float[])
    {
    	  0.012821017412067,
		 -0.077588248276420,
		  0.232176323842964,
		 -0.397848948438073,
		  0.609588480392350,
		 -0.502552748435650,
		  0.885254081326869,
		 -0.297624041496490,
		  0.989179881066047,
   	     -0.157365839955361
     },

    .pvCoeffs = (float*) (const float[])
    {
    	 0.007193577417824,
		 0.045214995492213,
		 0.141849626569576,
		 0.279089951509413,
		 0.344324497950504,
		 0.210161450064274,
		-0.021693574191595,
		-0.070833557053435,
		-0.024375571039064,
		 0.004083639765816,
       	 0.007308991933332
     }
};

*/




 // Rx Audio LPF IIR 2k7 definit par NIZAR pour essayer de reduire les distorsions de phase audio, � l'�coute ( 2.02 ms max)
 // simul� par lowa Hills IIR filter designer  vers 6.5:  0 - 2.7Khz, Papouli, 10 poles,  fs = 12Khz
 // Lattice (treuillis) coeff calcul�s par Mathlab online 2020 avec la fonction  :
 //
 //format long
 // a = [1.00000000000000000000, -3.10871732065926842    , 6.352826026095395480 ,-8.939790703570036360, 9.577830662990146850 , -7.899865717559023000 , 5.049428981698820710 , -2.450195405460327440, 0.8626762542265542070 , -0.1992332068446158730 ,  0.02303745029467041850 ]
 // b = [0.00026171584102765193,  0.00261715841027652063 , 0.0117772128462443448, 0.031405900923318244, 0.0549603266158069204,  0.0659523919389683133, 0.0549603266158069204,  0.031405900923318244, 0.0117772128462443448 ,  0.00261715841027652063,  0.00026171584102765193 ]
 // [k,v] = tf2latc(b,a)
 // NB : Les coeffs doivent etre copi� avec l'ordre inverse de ceux affich�s sur mathlab ( copier/coller, ajouter les virgules, puis inverser l'ordre des coeffs)
 // NB : pas d'inversion de signe des coeffs lors du copiage depuis l'editeur de Mathlab.
 // NB : les correspondances entre k (10 coeff)  et v (11 coeff) sont correctes, � ne pas inverser
/*
#define IIR_2k7_numStages 10
const arm_iir_lattice_instance_f32 IIR_2k7_LPF =
{
    .numStages = IIR_2k7_numStages,
    .pkCoeffs  = (float*) (const float[])  // a
    {
     	 0.023037450294670,
     	-0.127684051094902,
     	 0.325451583459221,
     	-0.517784188231734,
     	 0.635285461305358,
    	-0.683076123371484,
    	 0.752784248031444,
    	-0.619107840799261,
    	 0.932917180134125,
        -0.297466511603443
    },

    .pvCoeffs = (float*) (const float[])  // b
    {

    	0.000261715841028,
     	0.003430758978370,
    	0.020769745870059,
    	0.075650304099542,
    	0.174276318279840,
    	0.240629353078431,
    	0.166946965809181,
    	0.006671414504518,
       -0.056920126584286,
       -0.003194207125115,
        0.017364552059658,
    }
};
*/



 // Rx Audio LPF IIR 2k7    definit par NIZAR pour essayer de reduire les distorsions de phase audio, a l'�coute ( 1.42 ms max )
 // simul� par lowa Hills IIR filter designer  ver 6.5 : 0 - 2.7Khz , 10 poles , Inverse Chebyshev, -60db stop band , fs = 12Khz
 // les coeffs treillis sont calcul�s par mathlab online via la fonction :
 // format long
 // a = [ 1.000000000000000000 , 0.0550514081685438672, 1.7074722328677634  , 0.354449606209278612, 1.01916430007098313, 0.300102842310930651, 0.26625316028085404, 0.0760696412624159013, 0.0276461784309724212, 0.00494081751770871946, 0.000540128996405415052]
 // b = [ 0.0230255825391693847, 0.112257905097994359 , 0.311559110788379989, 0.599664329784376005, 0.86914550280725269, 0.980385454081512719, 0.86914550280725269, 0.599664329784375916 , 0.311559110788379989 , 0.112257905097994359  , 0.02302558253916938470 ]
 // [k,v] = tf2latc(b,a)
 // NB : Les coeffs doivent etre copi� avec l'ordre inverse de ceux affich�s sur mathlab ( copier/coller, ajouter les virgules, puis inverser l'ordre des coeffs)
 // NB : pas d'inversion de signe des coeffs lors du copiage depuis l'editeur de Mathlab.
 // NB : les correspondances entre k (10 coeff)  et v (11 coeff) sont correctes, � ne pas inverser
 // theoriquement ce filtre semble etre le meilleur compromis pour ecoute Audio USB entre selectivit�-distorsion de phase

/*
#define IIR_2k7_T_numStages 10
const arm_iir_lattice_instance_f32 IIR_2k7_LPF_T =
{
    .numStages = IIR_2k7_T_numStages,
    .pkCoeffs  = (float*) (const float[])
    {

    	0.000540128996405,
		0.004911084088620,
		0.026454219939023,
		0.066087788895568,
		0.216392324021418,
		0.173855050106791,
		0.656544828251348,
		0.060325811197986,
		0.953738668575539,
	   -0.107504933188777
    },

    .pvCoeffs = (float*) (const float[])
    {
    	  0.023025582539169,
		  0.110990314355312,
		  0.266133689273822,
		  0.387375890708375,
		  0.331421915349204,
		  0.094460927736170,
		 -0.135107713716374,
		 -0.149169590049521,
		 -0.034824909833453,
		  0.016209685053528,
    	  0.034346751400878
     }
};

*/

// Rx Audio LPF IIR 2k7    definit par NIZAR pour essayer de reduire les distorsions de phase audio, a l'�coute ( 6.4 ms max )
 // simul� par lowa Hills IIR filter designer  ver 6.5 : 0 - 2.7Khz (0.450)  , 10 poles , Elliptic, -70db stop band , fs = 12Khz  0.06dB riples
 // les coeffs treillis sont calcul�s par mathlab online via la fonction :
 // format long
 // a = [ 1.00000000000000      , -2.78331402268307171 , 6.30598020665396408  , -9.22718269184116835, 11.0367543852693428   , -10.0182242818494927, 7.34428977724031018, -4.1208611164655311, 1.74803494551938887, -0.503378696048444851, 0.0795881856301711643]
 // b = [ 0.00949910777916111826, 0.0261673918567789654, 0.0645381715665078559,  0.105555456694610261, 0.1456913894896941560, 0.1587836566519647, 0.145691389489694156, 0.105555456694610283,  0.0645381715665078559, 0.0261673918567789654, 0.00949910777916111826]
 // [k,v] = tf2latc(b,a)
 // NB : Les coeffs doivent etre copi� avec l'ordre inverse de ceux affich�s sur mathlab ( copier/coller, ajouter les virgules, puis inverser l'ordre des coeffs)
 // NB : pas d'inversion de signe des coeffs lors du copiage depuis l'editeur de Mathlab.
 // NB : les correspondances entre k (10 coeff)  et v (11 coeff) sont correctes, � ne pas inverser
 // theoriquement ce filtre semble etre le meilleur compromis pour ecoute Audio USB entre selectivit�-distorsion de phase

/*
#define IIR_2k7_T_numStages 10
const arm_iir_lattice_instance_f32 IIR_2k7_LPF_T =
{
    .numStages = IIR_2k7_T_numStages,
    .pkCoeffs  = (float*) (const float[])
    {
    	 0.079588185630171,
		-0.283656542708578,
		 0.512208570424576,
		-0.612995502593183,
		 0.770138392448469,
		-0.529498832100956,
		 0.953887274393428,
		-0.284615458344081,
		 0.996762032516743,
    	-0.180952737645039
    },

    .pvCoeffs = (float*) (const float[])
    {
    	0.009499107779161,
		0.052606391741496,
		0.149869467125297,
		0.258696956374143,
		0.253431778549934,
		0.099656750629059,
	   -0.043922821171319,
	   -0.026529992550069,
	   -0.003808803993887,
	    0.000681504719932,
    	0.001337839594763,
     }
};

*/





// Rx Audio LPF IIR 2k7    definit par NIZAR pour essayer de reduire les distorsions de phase audio, a l'�coute ( 3.1 ms max )
 // simul� par lowa Hills IIR filter designer  ver 6.5 : 0 - 2.7Khz , 10 poles ,  Chebyshev, 0.2db ripples , fs = 12Khz
 // les coeffs treillis sont calcul�s par Mathlab online via la fonction :
 // format long
 // a = [ 1.000000000000000000, -3.97940851303799992  , 9.44208382612047004   , -15.5303584348744339  , 19.2311003002750458  , -18.4078810955718986  , 13.7084485329831751  , -7.826646617147818400 , 3.289684888425917110  , -0.92822060730602427  ,  0.13691694506219485    ]
 // b = [ 0.000132538305594362,  0.0013253830559436226, 0.00596422375174630126,  0.0159045966713234677, 0.0278330441748160684,  0.033399653009779291,  0.0278330441748160728,  0.0159045966713234699, 0.00596422375174630304,  0.0013253830559436226,  0.000132538305594362305]
 // [k,v] = tf2latc(b,a)
 // NB : Les coeffs doivent etre copi� avec l'ordre inverse de ceux affich�s sur mathlab ( copier/coller, ajouter les virgules, puis inverser l'ordre des coeffs)
 // NB : pas d'inversion de signe des coeffs lors du copiage depuis l'editeur de Mathlab.
 // NB : les correspondances entre k (10 coeff)  et v (11 coeff) sont correctes, � ne pas inverser
/*

#define IIR_2k7_numStages 10
const arm_iir_lattice_instance_f32 IIR_2k7_LPF =
{
    .numStages = IIR_2k7_numStages,
    .pkCoeffs  = (float*) (const float[])  // a
    {
      		0.136916945062195,
    	   -0.390696239872588,
    		0.591501698263190,
    	   -0.684643344659028,
    		0.722849817024267,
    	   -0.732283328640686,
     	 	0.775330700424932,
    	   -0.644354159084327,
    		0.935761588221647,
       	   -0.307534783822037
      },

    .pvCoeffs = (float*) (const float[])  // b
    {

    		0.000132538305594,
    		0.001852807117529,
    		0.011986750284830,
    		0.045273719424192,
    		0.104008586357509,
    		0.142100084559449,
    		0.099419434488887,
    		0.007480723018222,
    	   -0.030364106913355,
    	   -0.002018056194240,
       	    0.009149616831521
    }
};

*/






 // Rx Audio LPF IIR 2k7    definit par NIZAR pour essayer de reduire les distorsions de phase audio, a l'�coute ( 1.03 ms max )
 // simul� par lowa Hills IIR filter designer vers 6.5  : 0 - 2.7Khz , 10 poles , Butterworth,  fs = 12Khz
 // Les coeff treillis sont Calcul�s  avec mathlab par la fonction :
 // format long
 // a = [ 1.00000000000000000000, -0.99600985716836874   , 1.75955882059809698  , -1.11209789308878682, 0.874743027209397006, -0.347447044820450746, 0.144163370056258766, -0.0330711613940974969, 0.00669189040512852884, -0.000679889655126410197, 0.0000391470990890749926 ]
 // b = [ 0.00126551797777455111,  0.01265517977774551770, 0.0569483089998548131,  0.15186215733294619, 0.265758775332655794,  0.318910530399186998, 0.265758775332655839,  0.15186215733294619  , 0.0569483089998548131 ,  0.0126551797777455177  , 0.00126551797777455111   ]
 // [k,v] = tf2latc(b,a)
 // NB : Les coeffs doivent etre copi� avec l'ordre inverse de ceux affich�s sur mathlab ( copier/coller, ajouter les virgules, puis inverser l'ordre des coeffs)
 // NB : pas d'inversion de signe des coeffs lors du copiage depuis l'editeur de Mathlab.
 // NB : les correspondances entre k (10 coeff)  et v (11 coeff) sont correctes, � ne pas inverser

/*
#define IIR_2k7_numStages 10
const arm_iir_lattice_instance_f32 IIR_2k7_LPF =
{
    .numStages = IIR_2k7_numStages,
    .pkCoeffs  = (float*) (const float[])  // a
    {
    		 0.000039147099089,
    		-0.000640898759536,
    		 0.005984669784120,
    		-0.025940102327595,
    		 0.107130565778596,
    		-0.190605810497498,
    		 0.493617356072020,
    		-0.335682960633936,
    		 0.899260915365407,
		    -0.216400087999179
      },

    .pvCoeffs = (float*) (const float[])  // b
    {
    	0.001265517977775,
    	0.013915648158033,
    	0.068581678064589,
    	0.197091901959980,
    	0.355729359738162,
    	0.390071662312935,
    	0.185689120217429,
       -0.102846977657765,
       -0.162183512430388,
    	0.003255580060449,
        0.062583421272260
     }
};

*/
// Rx Audio HPF 48 Hz IIR filter  (0.008);  PAPOULI  // Nizar 22 Mars 2021
// simul� par lowa Hills IIR filter designer  vers 6.5:
// HPF 48 Hz ( 0.0080) ,  PAPOULI 3em ordre,  fs = 12Khz
// lattice ARMA coefficients designed with MATLAb avec la fonction suivante :
// Lattice (treuillis) coeff calcul�s par Mathlab online 2020 avec la fonction suivante :
// format long
// a = [ 1.000000000000000000, -2.94644738613308466,  2.89426904730124868, -0.947800757524850468 ]
// b = [ 0.973564648869897908, -2.92069394660969373,  2.92069394660969373, -0.973564648869897908 ]
// [k,v] = tf2latc(b,a)
// NB : Les coeffs doivent etre copi� avec l'ordre inverse de ceux affich�s sur mathlab ( copier/coller, ajouter les virgules, puis inverser l'ordre des coeffs)
// NB : pas d'inversion de signe des coeffs lors du copiage depuis l'editeur de Mathlab.
// NB : les correspondances entre k (3 coeff)  et v (4 coeff) sont correctes, � ne pas inverser

#define IIR_HPF_50HZ_numStages 3
const   arm_iir_lattice_instance_f32  IIR_50HZ_HPF =
{
    .numStages = IIR_HPF_50HZ_numStages,
    .pkCoeffs  = (float*) (const float[])
    {
    	-0.947800757524850,
		 0.999510775093601,
		-0.999799721635099
    },

    .pvCoeffs = (float*) (const float[])
    {
    	-0.973564648869898,
		 0.052136931715409,
		 0.001291560216015,
       	-0.000000786313830
     }
};



// Rx Audio HPF 70 Hz IIR filter  (0.0117);  PAPOULI  // Nizar 23 Mars 2021
// simul� par lowa Hills IIR filter designer  vers 6.5:
// HPF 70 Hz ( 0.0117) ,  PAPOULI 3em ordre,  fs = 12Khz
// lattice ARMA coefficients designed with MATLAb avec la fonction suivante :
// Lattice (treuillis) coeff calcul�s par Mathlab online 2020 avec la fonction suivante :
// format long
// a = [ 1.000000000000000000, -2.92171533930535698,  2.84636494295643727, -0.924585007033105555 ]
// b = [ 0.961583161161862421, -2.88474948348558735,  2.88474948348558735, -0.961583161161862421 ]
// [k,v] = tf2latc(b,a)
// NB : Les coeffs doivent etre copi� avec l'ordre inverse de ceux affich�s sur mathlab ( copier/coller, ajouter les virgules, puis inverser l'ordre des coeffs)
// NB : pas d'inversion de signe des coeffs lors du copiage depuis l'editeur de Mathlab.
// NB : les correspondances entre k (3 coeff)  et v (4 coeff) sont correctes, � ne pas inverser
/*
#define IIR_HPF_70HZ_numStages 3
const   arm_iir_lattice_instance_f32  IIR_70HZ_HPF =
{
    .numStages = IIR_HPF_70HZ_numStages,
    .pkCoeffs  = (float*) (const float[])
    {
    	-0.924585007033106,
		 0.998953998409964,
       	-0.999571501519796
     },

    .pvCoeffs = (float*) (const float[])
    {
    	 -0.961583161161862,
		  0.075277211501239,
		  0.002678320500889,
    	 -0.000003511237584
      }
};
*/


// Rx Audio HPF 65 Hz IIR filter  (0.01084) 3em ordre;  PAPOULI  3 em ordre// Nizar 11 Mars 2022
// simul� par lowa Hills IIR filter designer  vers 6.5:
// HPF 65 Hz ( 0.01084) ,  PAPOULI 3em ordre,  fs = 12Khz
// lattice ARMA coefficients designed with MATLAb avec la fonction suivante :
// Lattice (treuillis) coeff calcul�s par Mathlab online 2020 avec la fonction suivante :
// format long
// a = [  1.000000000000000000,     -2.927461918840513990E0,   2.857443639824655430E0,  -9.299302017835707090E-1 ]
// b = [  9.643544700560923390E-1,  -2.893063410168277280E0,  2.893063410168277280E0,   -9.643544700560923390E-1 ]
// [k,v] = tf2latc(b,a)
// NB : Les coeffs doivent etre copi� avec l'ordre inverse de ceux affich�s sur mathlab ( copier/coller, ajouter les virgules, puis inverser l'ordre des coeffs)
// NB : pas d'inversion de signe des coeffs lors du copiage depuis l'editeur de Mathlab.
// NB : les correspondances entre k (3 coeff)  et v (4 coeff) sont correctes, � ne pas inverser

#define IIR_HPF_65HZ_numStages 3
const   arm_iir_lattice_instance_f32  IIR_65HZ_HPF =
{
    .numStages = IIR_HPF_65HZ_numStages,
    .pkCoeffs  = (float*) (const float[])
    {
    	-0.929930201783571,
		 0.999102025815312,
      	-0.999632207143919
    },

    .pvCoeffs = (float*) (const float[])
    {
    	-0.964354470056092,
		 0.069952422815442,
		 0.002315734091352,
       	-0.000002601838759
     }
};






// Rx Audio HPF 75 Hz IIR filter  (0.0125) 3em ordre;  PAPOULI  3 em ordre// Nizar 23 Mars 2021
// simul� par lowa Hills IIR filter designer  vers 6.5:
// HPF 75 Hz ( 0.0125) ,  PAPOULI 3em ordre,  fs = 12Khz
// lattice ARMA coefficients designed with MATLAb avec la fonction suivante :
// Lattice (treuillis) coeff calcul�s par Mathlab online 2020 avec la fonction suivante :
// format long
// a = [ 1.000000000000000000, -2.9163707272893804,  2.83608940129911824, -0.919640106965414006 ]
// b = [ 0.959012529444239092, -2.87703758833271728, 2.87703758833271728, -0.959012529444239092 ]
// [k,v] = tf2latc(b,a)
// NB : Les coeffs doivent etre copi� avec l'ordre inverse de ceux affich�s sur mathlab ( copier/coller, ajouter les virgules, puis inverser l'ordre des coeffs)
// NB : pas d'inversion de signe des coeffs lors du copiage depuis l'editeur de Mathlab.
// NB : les correspondances entre k (3 coeff)  et v (4 coeff) sont correctes, � ne pas inverser

#define IIR_HPF_75HZ_numStages 3
const   arm_iir_lattice_instance_f32  IIR_75HZ_HPF =
{
    .numStages = IIR_HPF_75HZ_numStages,
    .pkCoeffs  = (float*) (const float[])
    {
    	-0.919640106965414,
		 0.998806187727408,
     	-0.999510863152492
    },

    .pvCoeffs = (float*) (const float[])
    {
    	-0.959012529444239,
		 0.080201520357793,
		 0.003036565088471,
     	-0.000004550720946
     }
};


// Rx Audio HPF 75 Hz IIR filter  (0.0125);  PAPOULI  // Nizar 23 Mars 2021
// simul� par lowa Hills IIR filter designer  vers 6.5:
// HPF 75 Hz ( 0.0125) ,  PAPOULI 4em ordre,  fs = 12Khz  (9.5 ms max )
// lattice ARMA coefficients designed with MATLAb avec la fonction suivante :
// Lattice (treuillis) coeff calcul�s par Mathlab online 2020 avec la fonction suivante :
// format long
// a = [ 1.000000000000000000, -3.88815669309070522, 5.67068300710456352,  -3.6766943817136295, 0.894171987277317903 ]
// b = [ 0.945606629324138481, -3.7824265172965541,  5.67363977594482982,  -3.7824265172965541, 0.945606629324138481 ]
// [k,v] = tf2latc(b,a)
// NB : Les coeffs doivent etre copi� avec l'ordre inverse de ceux affich�s sur mathlab ( copier/coller, ajouter les virgules, puis inverser l'ordre des coeffs)
// NB : pas d'inversion de signe des coeffs lors du copiage depuis l'editeur de Mathlab.
// NB : les correspondances entre k (4 coeff)  et v (5 coeff) sont correctes, � ne pas inverser
/*
#define IIR_HPF_75HZ_numStages 4
const   arm_iir_lattice_instance_f32  IIR_75HZ_HPF =
{
    .numStages = IIR_HPF_75HZ_numStages,
    .pkCoeffs  = (float*) (const float[])
    {
    	 0.894171987277318,
		-0.997790679493797,
		 0.999552582690642,
    	-0.999531587690949
    },

    .pvCoeffs = (float*) (const float[])
    {
    	 0.945606629324139,
		-0.105759772458963,
		-0.005447739354716,
		 0.000010757787332,
       	 0.000001609863913
     }
};
*/


// Rx Audio HPF 75 Hz IIR filter  (0.0125) 2em ordre;  PAPOULI  // Nizar 23 Mars 2021
// simul� par lowa Hills IIR filter designer  vers 6.5:
// HPF 75 Hz ( 0.0125) ,  PAPOULI 2em ordre,  fs = 12Khz  (3.6 ms max )
// lattice ARMA coefficients designed with MATLAb avec la fonction suivante :
// Lattice (treuillis) coeff calcul�s par Mathlab online 2020 avec la fonction suivante :
// format long
// a = [ 1.000000000000000000, -1.94444591348792217, 0.94594788539333301  ]
// b = [ 0.972598449720313774, -1.9451968994406276,  0.972598449720313774 ]
// [k,v] = tf2latc(b,a)
// NB : Les coeffs doivent etre copi� avec l'ordre inverse de ceux affich�s sur mathlab ( copier/coller, ajouter les virgules, puis inverser l'ordre des coeffs)
// NB : pas d'inversion de signe des coeffs lors du copiage depuis l'editeur de Mathlab.
// NB : les correspondances entre k (2 coeff)  et v (3 coeff) sont correctes, � ne pas inverser
/*
#define IIR_HPF_75HZ_numStages 2
const   arm_iir_lattice_instance_f32  IIR_75HZ_HPF =
{
    .numStages = IIR_HPF_75HZ_numStages,
    .pkCoeffs  = (float*) (const float[])
    {
    	 0.945947885393333,
    	-0.999228154095653
    },

    .pvCoeffs = (float*) (const float[])
    {
    	  0.972598449720314,
		 -0.054031818417275,
     	 -0.001419111308976
    }
};
*/

// Rx Audio HPF 100 Hz IIR filter  (0.0167) 3em ordre;  PAPOULI  // Nizar 24 Mars 2021
// simul� par lowa Hills IIR filter designer  vers 6.5:
// HPF 100 Hz ( 0.0167) ,  PAPOULI 3em ordre,  fs = 12Khz
// lattice ARMA coefficients designed with MATLAb avec la fonction suivante :
// Lattice (treuillis) coeff calcul�s par Mathlab online 2020 avec la fonction suivante :
// format long
// a = [ 1.000000000000000000, -2.88832768456559474,  2.78261865014997412, -0.89410617113617441 ]
// b = [ 0.945631563231467886, -2.83689468969440384,  2.83689468969440384, -0.945631563231467886]
// [k,v] = tf2latc(b,a)
// NB : Les coeffs doivent etre copi� avec l'ordre inverse de ceux affich�s sur mathlab ( copier/coller, ajouter les virgules, puis inverser l'ordre des coeffs)
// NB : pas d'inversion de signe des coeffs lors du copiage depuis l'editeur de Mathlab.
// NB : les correspondances entre k (3 coeff)  et v (4 coeff) sont correctes, � ne pas inverser

#define IIR_HPF_100HZ_numStages 3
const   arm_iir_lattice_instance_f32  IIR_100HZ_HPF =
{
    .numStages = IIR_HPF_100HZ_numStages,
    .pkCoeffs  = (float*) (const float[])
    {
    	-0.894106171136174,
		 0.997870555055543,
       	-0.999126524110493
    },

    .pvCoeffs = (float*) (const float[])
    {
    	-0.945631563231468,
		 0.105600866213914,
		 0.005229912319594,
       	-0.000014103940882
    }
};


// Rx Audio HPF 150 Hz IIR filter  (0.025) 3 em ordre;  PAPOULI  (3.2 ms max) // Nizar 25 Mars 2021
// simul� par lowa Hills IIR filter designer  vers 6.5:
// HPF 150 Hz ( 0.025) ,  PAPOULI 3em ordre,  fs = 12Khz
// lattice ARMA coefficients designed with MATLAb avec la fonction suivante :
// Lattice (treuillis) coeff calcul�s par Mathlab online 2020 avec la fonction suivante :
// format long
// a = [ 1.000000000000000000, -2.83298587828370252,  2.67927790188010251, -0.845688525879980091 ]
// b = [ 0.919744038255473129, -2.75923211476641939,  2.75923211476641939, -0.919744038255473129 ]
// [k,v] = tf2latc(b,a)
// NB : Les coeffs doivent etre copi� avec l'ordre inverse de ceux affich�s sur mathlab ( copier/coller, ajouter les virgules, puis inverser l'ordre des coeffs)
// NB : pas d'inversion de signe des coeffs lors du copiage depuis l'editeur de Mathlab.
// NB : les correspondances entre k (3 coeff)  et v (4 coeff) sont correctes, � ne pas inverser

#define IIR_HPF_150HZ_numStages 3
const   arm_iir_lattice_instance_f32  IIR_150HZ_HPF =
{
    .numStages = IIR_HPF_150HZ_numStages,
    .pkCoeffs  = (float*) (const float[])
    {
    	-0.845688525879980,
		 0.995236606192100,
      	-0.998039878434139
     },

    .pvCoeffs = (float*) (const float[])
    {
    	 -0.919744038255473,
		  0.153610242753039,
		  0.010905786478106,
     	 -0.000067068506739
    }
};


// Rx Audio HPF 200 Hz IIR filter  (0.033) 3 em ordre;  Inverse Thchebychef  -60db  (2.32 ms max) // Nizar 30 Nov 2021
// simule par lowa Hills IIR filter designer  vers 6.5:
// HPF 200 Hz ( 0.033) , Inverse Thchebychef 3em ordre,  fs = 12Khz
// lattice ARMA coefficients designed with MATLAb avec la fonction suivante :
// Lattice (treuillis) coeff calcule par Mathlab online 2020 avec la fonction suivante :
// format long
// a = [  1.000000000000000000,    -2.792689674614181610E0,  2.606490047230018090E0,  -8.127758551611110600E-1 ]
// b = [  9.015408318882142780E-1,  -2.704436956614441280E0,  2.704436956614441280E0,  -9.015408318882142780E-1 ]
// [k,v] = tf2latc(b,a)
// NB : Les coeffs doivent etre copie avec l'ordre inverse de ceux affiches sur mathlab ( copier/coller, ajouter les virgules, puis inverser l'ordre des coeffs)
// NB : pas d'inversion de signe des coeffs lors du copiage depuis l'editeur de Mathlab.
// NB : les correspondances entre k (3 coeff)  et v (4 coeff) sont correctes,  a ne pas inverser

#define IIR_HPF_200HZ_numStages 3
const   arm_iir_lattice_instance_f32  IIR_200HZ_HPF =
{
    .numStages = IIR_HPF_200HZ_numStages,
    .pkCoeffs  = (float*) (const float[])
    {
    	-0.812775855161111,
		 0.991938310164061,
		-0.997252854753280
     },

    .pvCoeffs = (float*) (const float[])
    {
    	 -0.901540831888214,
		  0.186713184157145,
		  0.016319672004627,
		 -0.000142909595327
    }
};



//
// 120 - 2700Hz bandpass, 1410Hz center frequency
//
const arm_iir_lattice_instance_f32 IIR_2k7_BPF =
{
    .numStages = 10,
    .pkCoeffs  = (float*) (const float[])
    {
         0.286204893123822,
        -0.535401193365074,
         0.636552242537530,
        -0.801759238351553,
         0.632340313225578,
        -0.598674284342736,
         0.984168394137399,
        -0.996691132601798,
         0.999768009572177,
        -0.997990472561290
    },

    .pvCoeffs = (float*) (const float[])
    {
        -0.0262124858208275,
        -0.117385216786601,
        -0.223626900059669,
        -0.159085812308114,
         0.0583824351916630,
         0.119904708047547,
         0.0261315687651823,
         0.00496617536907647,
        -0.000240288448030184,
        -6.41272493413780e-06,
         1.37062857848147e-07
    }
};


