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
**  Licence:        GNU GPLv3                                                      **
************************************************************************************/
#include "uhsdr_board.h"
#include "audio_filter.h"
#include "audio_driver.h"
#include "filters.h"

#include "arm_math.h"
#include "math.h"
#include "audio_driver.h"



enum
{
    FILTER_MASK_CW = 1 << FILTER_MODE_CW,
    FILTER_MASK_SSB = 1 << FILTER_MODE_SSB,
    FILTER_MASK_AM = 1 << FILTER_MODE_AM,
    FILTER_MASK_FM = 1 << FILTER_MODE_FM,
    FILTER_MASK_SAM = 1 << FILTER_MODE_SAM,
};

#define FILTER_MASK_ALL (FILTER_MASK_CW|FILTER_MASK_SSB|FILTER_MASK_AM|FILTER_MASK_FM)
#define FILTER_MASK_NOFM (FILTER_MASK_CW|FILTER_MASK_SSB|FILTER_MASK_AM)
#define FILTER_MASK_SSBAM (FILTER_MASK_SSB|FILTER_MASK_AM)
#define FILTER_MASK_SSBAMCW (FILTER_MASK_SSB|FILTER_MASK_AM|FILTER_MASK_CW)
#define FILTER_MASK_SSBSAM (FILTER_MASK_SSB|FILTER_MASK_AM|FILTER_MASK_SAM)
#define FILTER_MASK_SSBSAMCW (FILTER_MASK_SSB|FILTER_MASK_AM|FILTER_MASK_SAM|FILTER_MASK_CW)
#define FILTER_MASK_AMSAM (FILTER_MASK_AM|FILTER_MASK_SAM)
#define FILTER_MASK_SSBCW (FILTER_MASK_SSB|FILTER_MASK_CW)
#define FILTER_MASK_AMFM (FILTER_MASK_AM|FILTER_MASK_FM)
#define FILTER_MASK_NONE (0)
#define FILTER_MASK_SSBAMFM (FILTER_MASK_SSB|FILTER_MASK_AM|FILTER_MASK_FM)

const FilterDescriptor FilterInfo[AUDIO_FILTER_NUM ] =
{
    // 	id ,	name	 ,  width
    {  AUDIO_300HZ,  "300Hz",   300},
    {  AUDIO_500HZ,  "500Hz",   500},
    {  AUDIO_1P4KHZ, "1.4k",  1400},
    {  AUDIO_1P6KHZ, "1.6k",  1600},
    {  AUDIO_1P8KHZ, "1.8k",  1800},
    {  AUDIO_2P1KHZ, "2.1k",  2100},
    {  AUDIO_2P3KHZ, "2.3k",  2300},
    {  AUDIO_2P5KHZ, "2.5k",  2500},
	{  AUDIO_2P6KHZ, "2.6k",  2600},
    {  AUDIO_2P7KHZ, "2.7k",  2700},
 	{  AUDIO_2P75KHZ,"2k75",  2745},
	{  AUDIO_2P8KHZ, "2.8k",  2800},
    {  AUDIO_2P9KHZ, "2.9k",  2900},
    {  AUDIO_3P2KHZ, "3.2k",  3200},
    {  AUDIO_3P4KHZ, "3.4k",  3400},
    {  AUDIO_3P6KHZ, "3.6k",  3600},
    {  AUDIO_3P8KHZ, "3.8k",  3800},
    {  AUDIO_4P0KHZ, "4.0k",  4000},
    {  AUDIO_4P2KHZ, "4.2k",  4200},
    {  AUDIO_4P4KHZ, "4.4k",  4400},
    {  AUDIO_4P6KHZ, "4.6k",  4600},
    {  AUDIO_4P8KHZ, "4.8k",  4800},
    {  AUDIO_5P0KHZ, "5.0k",  5000},
    {  AUDIO_5P5KHZ, "5.5k",  5500},
    {  AUDIO_6P0KHZ, "6.0k",  6000},
    {  AUDIO_6P5KHZ, "6.5k",  6500},
    {  AUDIO_7P0KHZ, "7.0k",  7000},
    {  AUDIO_7P5KHZ, "7.5k",  7500},
    {  AUDIO_8P0KHZ, "8.0k",  8000},
    {  AUDIO_8P5KHZ, "8.5k",  8500},
    {  AUDIO_9P0KHZ, "9.0k",  9000},
    {  AUDIO_9P5KHZ, "9.5k",  9500},
    {  AUDIO_10P0KHZ,"10.0k", 10000}
};

uint16_t filterpath_mode_map[FILTER_MODE_MAX];

/*################################################################
 * FILTER PLAYGROUND for the brave mcHF owner
 *
 * FilterPathInfo
 *
 * put together your custom filters here
 * only add NEW filters at the END of the list in FilterPathInfo!!! (otherwise your filter settings will get mashed up completely in EEPROM)
 * All components of the whole filterpath are defined in this file
 * But: it could be that you have to change "FilterInfo" too
 * Have fun and learn about DSP ;-) but do not forget:
 * finally, the filter has to please your own ears
 * DD4WH
 *
 * Most of the combinations below had been previously defined and put
 * together in an old version of the firmware by Clint KA7OEI,
 * so the good sound of the mcHF is his merit !
 * ###############################################################
 */

/*
id --> ID for bandwidth

mode name --> for display

filter_select_ID

FIR coeff_I_numTaps (= FIR coeff_Q_numTaps)
FIR coeff_I_coeffs: points to the array of FIR filter coeffs used in the I path
FIR coeff_Q_coeffs: points to the array of FIR filter coeffs used in the Q path

&decimation filter instance
[dec- filter_numTaps: points to the array of FIR coeffs used in the decimation fliter
[dec- filter_coeffs

sample rate: gives the sample rate used after decimation (12ksps, 24ksps . . .)

&IIR_Pre_Filter instance
[IIR audio coeff_numStages: points to the array of IIR coeff used for the audio IIR filter
[IIR audio coeff_pk
[IIR audio coeff_pv

&FIR_interpolation filter instance
[int. filter coeffs_numTaps
[int. filter coeffs: points to the array of IIR coeffs for the antialias interpolation filter used by the ARM interpolation routine

&IIR interpolation filter instance
[IIR_antialias_numStages
[IIR_antialias_coeff_pk
[IIR_antialias_coeff_pv: points to the array of IIR coeffs for the antialias IIR filter (works at 48ksps)

centre frequency of the filterpath in Hz (for the graphical display of the bandwidth under spectrum display)

*/

// in the STM32F4, this can be switched on to lower the processor load
// if USE_SMALL_HILBERT_DECIMATION_FILTERS is defined in uhsdr_board.h,
// we use lowpass filters with 89 taps (FirRxDecimate) instead of the high sideband suppression filters with 199 taps (FirRxDecimate_sideband_supp)
#ifdef USE_SMALL_HILBERT_DECIMATION_FILTERS
    #define  FIR_RX_DECIMATE_PTR (&FirRxDecimate)
#else
    #define  FIR_RX_DECIMATE_PTR   (&FirRxDecimate_sideband_supp)
#endif

const FilterPathDescriptor FilterPathInfo [ AUDIO_FILTER_PATH_NUM ] =
    //
{
// id, mode name (for display), filter_select_ID, FIR_numTaps, FIR_I_coeff_file, FIR_Q_coeff_file, &decimation filter,
//		sample_rate_dec, &IIR_PreFilter,
//		&FIR_interpolaton filter, &IIR_interpolation filter, centre frequency of the filterpath (for graphical bandwidth display)
//
    // SPECIAL AUDIO_OFF Entry
    {
        AUDIO_OFF, "", FILTER_MASK_NONE, 0, 0, NULL, NULL, NULL,
        0, NULL,
        NULL, NULL
    },

//###################################################################################################################################
// FM filters
//	very special case, FM demodulation mainly in separate void, filterpath not defined in FilterPathInfo
//###################################################################################################################################
// 1
    {
        AUDIO_3P6KHZ, "FM", FILTER_MASK_FM, 1, IQ_RX_NUM_TAPS, iq_rx_am_3k6_coeffs, iq_rx_am_3k6_coeffs, NULL,
        RX_DECIMATION_RATE_48KHZ, NULL,
        NULL, NULL
    },

    {
        AUDIO_5P0KHZ, "FM", FILTER_MASK_FM, 1, IQ_RX_NUM_TAPS, iq_rx_am_5k_coeffs, iq_rx_am_5k_coeffs, NULL,
        RX_DECIMATION_RATE_48KHZ, NULL,
        NULL, NULL
    },

    {
//        AUDIO_6P0KHZ, "FM", FILTER_MASK_FM, 1, IQ_NUM_TAPS, iq_rx_am_5k_coeffs, iq_rx_am_5k_coeffs, NULL,
	    AUDIO_6P0KHZ, "FM", FILTER_MASK_FM, 1, IQ_RX_NUM_TAPS, iq_rx_am_6k_coeffs, iq_rx_am_6k_coeffs, NULL,
	    RX_DECIMATION_RATE_48KHZ, NULL,
        NULL, NULL
    },

//###################################################################################################################################
// CW & SSB filters:
//###################################################################################################################################


    // 10 filters � 300Hz
// 4
    {
        AUDIO_300HZ, "500Hz", FILTER_MASK_SSBCW, 1, IQ_RX_NUM_TAPS_HI, i_rx_new_coeffs, q_rx_new_coeffs, FIR_RX_DECIMATE_PTR,
        RX_DECIMATION_RATE_12KHZ, &IIR_300hz_500,
        &FirRxInterpolate, NULL, 500
    },

    {
            AUDIO_300HZ, "550Hz", FILTER_MASK_SSBCW, 2, IQ_RX_NUM_TAPS_HI, i_rx_new_coeffs, q_rx_new_coeffs, FIR_RX_DECIMATE_PTR,
            RX_DECIMATION_RATE_12KHZ, &IIR_300hz_550,
            &FirRxInterpolate, NULL, 550
/*            AUDIO_300HZ, "wowHz", FILTER_MASK_SSBCW, 2, IQ_NUM_TAPS_HI, i_rx_wow_coeffs, q_rx_wow_coeffs, &FirRxDecimate,
            RX_DECIMATION_RATE_12KHZ, &IIR_300hz_550,
            &FirRxInterpolate, NULL, 550*/
    },

    {
        AUDIO_300HZ, "600Hz", FILTER_MASK_SSBCW, 3, IQ_RX_NUM_TAPS_HI, i_rx_new_coeffs, q_rx_new_coeffs, FIR_RX_DECIMATE_PTR,
        RX_DECIMATION_RATE_12KHZ, &IIR_300hz_600,
        &FirRxInterpolate, NULL, 600
    },

    {
        AUDIO_300HZ, "650Hz", FILTER_MASK_SSBCW, 4, IQ_RX_NUM_TAPS_HI, i_rx_new_coeffs, q_rx_new_coeffs, FIR_RX_DECIMATE_PTR,
        RX_DECIMATION_RATE_12KHZ, &IIR_300hz_650,
        &FirRxInterpolate, NULL, 650
    },

    {
        AUDIO_300HZ, "700Hz", FILTER_MASK_SSBCW, 5, IQ_RX_NUM_TAPS_HI, i_rx_new_coeffs, q_rx_new_coeffs, FIR_RX_DECIMATE_PTR,
        RX_DECIMATION_RATE_12KHZ, &IIR_300hz_700,
        &FirRxInterpolate, NULL, 700
    },

    {
        AUDIO_300HZ, "750Hz", FILTER_MASK_SSBCW, 6, IQ_RX_NUM_TAPS_HI, i_rx_new_coeffs, q_rx_new_coeffs, FIR_RX_DECIMATE_PTR,
        RX_DECIMATION_RATE_12KHZ, &IIR_300hz_750,
        &FirRxInterpolate, NULL, 750
    },
//10
    {
        AUDIO_300HZ, "800Hz", FILTER_MASK_SSBCW, 7, IQ_RX_NUM_TAPS_HI, i_rx_new_coeffs, q_rx_new_coeffs, FIR_RX_DECIMATE_PTR,
        RX_DECIMATION_RATE_12KHZ, &IIR_300hz_800,
        &FirRxInterpolate, NULL, 800
    },

    {
        AUDIO_300HZ, "850Hz", FILTER_MASK_SSBCW, 8, IQ_RX_NUM_TAPS_HI, i_rx_new_coeffs, q_rx_new_coeffs, FIR_RX_DECIMATE_PTR,
        RX_DECIMATION_RATE_12KHZ, &IIR_300hz_850,
        &FirRxInterpolate, NULL, 850
    },

    {
        AUDIO_300HZ, "900Hz", FILTER_MASK_SSBCW, 9, IQ_RX_NUM_TAPS_HI, i_rx_new_coeffs, q_rx_new_coeffs, FIR_RX_DECIMATE_PTR,
        RX_DECIMATION_RATE_12KHZ, &IIR_300hz_900,
        &FirRxInterpolate, NULL, 900
    },

    {
        AUDIO_300HZ, "950Hz", FILTER_MASK_SSBCW, 10, IQ_RX_NUM_TAPS_HI, i_rx_new_coeffs, q_rx_new_coeffs, FIR_RX_DECIMATE_PTR,
        RX_DECIMATION_RATE_12KHZ, &IIR_300hz_950,
        &FirRxInterpolate, NULL, 950
    },

    // 5 filters � 500Hz
    {
        AUDIO_500HZ, "550Hz", FILTER_MASK_SSBCW, 1, IQ_RX_NUM_TAPS_HI, i_rx_new_coeffs, q_rx_new_coeffs, FIR_RX_DECIMATE_PTR,
        RX_DECIMATION_RATE_12KHZ, &IIR_500hz_550,
        &FirRxInterpolate, NULL, 550
    },
//15
    {
        AUDIO_500HZ, "650Hz", FILTER_MASK_SSBCW, 2, IQ_RX_NUM_TAPS_HI, i_rx_new_coeffs, q_rx_new_coeffs, FIR_RX_DECIMATE_PTR,
        RX_DECIMATION_RATE_12KHZ, &IIR_500hz_650,
        &FirRxInterpolate, NULL, 650
    },

    {
        AUDIO_500HZ, "750Hz", FILTER_MASK_SSBCW, 3, IQ_RX_NUM_TAPS_HI, i_rx_new_coeffs, q_rx_new_coeffs, FIR_RX_DECIMATE_PTR,
        RX_DECIMATION_RATE_12KHZ, &IIR_500hz_750,
        &FirRxInterpolate, NULL, 750
    },

    {
        AUDIO_500HZ, "850Hz", FILTER_MASK_SSBCW, 4, IQ_RX_NUM_TAPS_HI, i_rx_new_coeffs, q_rx_new_coeffs, FIR_RX_DECIMATE_PTR,
        RX_DECIMATION_RATE_12KHZ, &IIR_500hz_850,
        &FirRxInterpolate, NULL, 850
    },

    {
        AUDIO_500HZ, "950Hz", FILTER_MASK_SSBCW, 5, IQ_RX_NUM_TAPS_HI, i_rx_new_coeffs, q_rx_new_coeffs, FIR_RX_DECIMATE_PTR,
        RX_DECIMATION_RATE_12KHZ, &IIR_500hz_950,
        &FirRxInterpolate, NULL, 950
    },
// 19
    {
        AUDIO_1P4KHZ, "LPF", FILTER_MASK_SSBCW, 1, IQ_RX_NUM_TAPS_HI, i_rx_new_coeffs, q_rx_new_coeffs, FIR_RX_DECIMATE_PTR,
        RX_DECIMATION_RATE_12KHZ, &IIR_1k4_LPF,
        &FirRxInterpolate, NULL, 700
    },
//20
    {
        AUDIO_1P4KHZ, "BPF", FILTER_MASK_SSBCW, 2, IQ_RX_NUM_TAPS_HI, i_rx_new_coeffs, q_rx_new_coeffs, FIR_RX_DECIMATE_PTR,
        RX_DECIMATION_RATE_12KHZ, &IIR_1k4_BPF,
        &FirRxInterpolate, NULL, 775
    },

    {
        AUDIO_1P6KHZ, "LPF", FILTER_MASK_SSBCW, 1, IQ_RX_NUM_TAPS_HI, i_rx_new_coeffs, q_rx_new_coeffs, FIR_RX_DECIMATE_PTR,
        RX_DECIMATION_RATE_12KHZ, &IIR_1k6_LPF,
        &FirRxInterpolate, NULL, 800
    },

    {
        AUDIO_1P6KHZ, "BPF", FILTER_MASK_SSBCW, 2, IQ_RX_NUM_TAPS_HI, i_rx_new_coeffs, q_rx_new_coeffs, FIR_RX_DECIMATE_PTR,
        RX_DECIMATION_RATE_12KHZ, &IIR_1k6_BPF,
        &FirRxInterpolate, NULL, 875
    },

    {
        AUDIO_1P8KHZ, "1.1k", FILTER_MASK_SSBCW, 1, IQ_RX_NUM_TAPS_HI, i_rx_new_coeffs, q_rx_new_coeffs, FIR_RX_DECIMATE_PTR,
        RX_DECIMATION_RATE_12KHZ, &IIR_1k8_1k125,
        &FirRxInterpolate, NULL, 1125
    },

    {
        AUDIO_1P8KHZ, "1.3k", FILTER_MASK_SSBCW, 2, IQ_RX_NUM_TAPS_HI, i_rx_new_coeffs, q_rx_new_coeffs, FIR_RX_DECIMATE_PTR,
        RX_DECIMATION_RATE_12KHZ, &IIR_1k8_1k275,
        &FirRxInterpolate, NULL, 1275
    },
//25
    {
        AUDIO_1P8KHZ, "1.4k", FILTER_MASK_SSBCW, 3, IQ_RX_NUM_TAPS_HI, i_rx_new_coeffs, q_rx_new_coeffs, FIR_RX_DECIMATE_PTR,
        RX_DECIMATION_RATE_12KHZ, &IIR_1k8_1k425,
        &FirRxInterpolate, NULL, 1425
    },

    {
        AUDIO_1P8KHZ, "1.6k", FILTER_MASK_SSBCW, 4, IQ_RX_NUM_TAPS_HI, i_rx_new_coeffs, q_rx_new_coeffs, FIR_RX_DECIMATE_PTR,
        RX_DECIMATION_RATE_12KHZ, &IIR_1k8_1k575,
        &FirRxInterpolate, NULL, 1575
    },

    {
        AUDIO_1P8KHZ, "1.7k", FILTER_MASK_SSBCW, 5, IQ_RX_NUM_TAPS_HI, i_rx_new_coeffs, q_rx_new_coeffs, FIR_RX_DECIMATE_PTR,
        RX_DECIMATION_RATE_12KHZ, &IIR_1k8_1k725,
        &FirRxInterpolate, NULL, 1725
    },

    {
        AUDIO_1P8KHZ, "LPF", FILTER_MASK_SSBCW, 6, IQ_RX_NUM_TAPS_HI, i_rx_new_coeffs, q_rx_new_coeffs, FIR_RX_DECIMATE_PTR,
        RX_DECIMATION_RATE_12KHZ, &IIR_1k8_LPF,
        &FirRxInterpolate, NULL, 900
    },

    {
        AUDIO_2P1KHZ, "LPF", FILTER_MASK_SSBCW, 1, IQ_RX_NUM_TAPS_HI, i_rx_new_coeffs, q_rx_new_coeffs, FIR_RX_DECIMATE_PTR,
        RX_DECIMATION_RATE_12KHZ, &IIR_2k1_LPF,
        &FirRxInterpolate, NULL, 1050
    },
//30
    {
        AUDIO_2P1KHZ, "BPF", FILTER_MASK_SSBCW, 2, IQ_RX_NUM_TAPS_HI, i_rx_new_coeffs, q_rx_new_coeffs, FIR_RX_DECIMATE_PTR,
        RX_DECIMATION_RATE_12KHZ, &IIR_2k1_BPF,
        &FirRxInterpolate, NULL, 1125
    },

    {
        AUDIO_2P3KHZ, "1.3k", FILTER_MASK_SSBCW, 1, IQ_RX_NUM_TAPS_HI, i_rx_new_coeffs, q_rx_new_coeffs, FIR_RX_DECIMATE_PTR,
        RX_DECIMATION_RATE_12KHZ, &IIR_2k3_1k275,
        &FirRxInterpolate, NULL, 1275
    },

    {
        AUDIO_2P3KHZ, "1.4k", FILTER_MASK_SSBCW, 2, IQ_RX_NUM_TAPS_HI, i_rx_new_coeffs, q_rx_new_coeffs, FIR_RX_DECIMATE_PTR,
        RX_DECIMATION_RATE_12KHZ, &IIR_2k3_1k412,
        &FirRxInterpolate, NULL, 1412
    },

    {
        AUDIO_2P3KHZ, "1.6k", FILTER_MASK_SSBCW, 3, IQ_RX_NUM_TAPS_HI, i_rx_new_coeffs, q_rx_new_coeffs, FIR_RX_DECIMATE_PTR,
        RX_DECIMATION_RATE_12KHZ, &IIR_2k3_1k562,
        &FirRxInterpolate, NULL, 1562
    },

    {
        AUDIO_2P3KHZ, "1.7k", FILTER_MASK_SSBCW, 4, IQ_RX_NUM_TAPS_HI, i_rx_new_coeffs, q_rx_new_coeffs, FIR_RX_DECIMATE_PTR,
        RX_DECIMATION_RATE_12KHZ, &IIR_2k3_1k712,
        &FirRxInterpolate, NULL, 1712
    },
//35
    {
        AUDIO_2P3KHZ, "LPF", FILTER_MASK_SSBCW, 5, IQ_RX_NUM_TAPS_HI, i_rx_narrow_coeffs, q_rx_narrow_coeffs, FIR_RX_DECIMATE_PTR,
        RX_DECIMATION_RATE_12KHZ, &IIR_2k3_LPF,
        &FirRxInterpolate, NULL, 1150
    },

//###################################################################################################################################
// SSB only filters:
//###################################################################################################################################

    {
        AUDIO_2P5KHZ, "LPF", FILTER_MASK_SSB, 1, IQ_RX_NUM_TAPS_HI, i_rx_narrow_coeffs, q_rx_narrow_coeffs, FIR_RX_DECIMATE_PTR,
        RX_DECIMATION_RATE_12KHZ, &IIR_2k5_LPF,
        &FirRxInterpolate, NULL, 1250
    },

    {
        AUDIO_2P5KHZ, "BPF", FILTER_MASK_SSB, 2, IQ_RX_NUM_TAPS_HI, i_rx_narrow_coeffs, q_rx_narrow_coeffs, FIR_RX_DECIMATE_PTR,
        RX_DECIMATION_RATE_12KHZ, &IIR_2k5_BPF,
        &FirRxInterpolate, NULL, 1325
    },

	{
	        AUDIO_2P6KHZ, "LPF", FILTER_MASK_SSB, 1, IQ_RX_NUM_TAPS_HI, i_rx_narrow_coeffs, q_rx_narrow_coeffs, FIR_RX_DECIMATE_PTR,
	        RX_DECIMATION_RATE_12KHZ, &IIR_2k6_LPF,
	        &FirRxInterpolate, NULL, 1300
	},

    {
        AUDIO_2P7KHZ, "LPF", FILTER_MASK_SSB, 1, IQ_RX_NUM_TAPS_HI, i_rx_narrow_coeffs, q_rx_narrow_coeffs, FIR_RX_DECIMATE_PTR,
        RX_DECIMATION_RATE_12KHZ, &IIR_2k7_LPF,
        &FirRxInterpolate, NULL, 1350
    },

	{
	        AUDIO_2P7KHZ, "LPF*", FILTER_MASK_SSB, 1, IQ_RX_NUM_TAPS_HI, i_rx_narrow_coeffs, q_rx_narrow_coeffs, FIR_RX_DECIMATE_PTR,
	        RX_DECIMATION_RATE_12KHZ, &IIR_2k7_LPF_S,
	        &FirRxInterpolate, NULL, 1350
	},

	{
			AUDIO_2P75KHZ, "LPF", FILTER_MASK_SSB, 1, IQ_RX_NUM_TAPS_HI, i_rx_narrow_coeffs, q_rx_narrow_coeffs, FIR_RX_DECIMATE_PTR,
			RX_DECIMATION_RATE_12KHZ, &IIR_2k75_LPF,
			&FirRxInterpolate, NULL, 1375
	},


    {
        AUDIO_2P7KHZ, "BPF", FILTER_MASK_SSB, 2, IQ_RX_NUM_TAPS_HI, i_rx_narrow_coeffs, q_rx_narrow_coeffs, FIR_RX_DECIMATE_PTR,
        RX_DECIMATION_RATE_12KHZ, &IIR_2k7_BPF,
        &FirRxInterpolate, NULL, 1350
    },

	 {
		AUDIO_2P8KHZ, "LPF", FILTER_MASK_SSB, 1, IQ_RX_NUM_TAPS_HI, i_rx_narrow_coeffs, q_rx_narrow_coeffs, FIR_RX_DECIMATE_PTR,
		RX_DECIMATION_RATE_12KHZ, &IIR_2k8_LPF,
		&FirRxInterpolate, NULL, 1400
	 },

	 {
			AUDIO_2P8KHZ, "LPF*", FILTER_MASK_SSB, 1, IQ_RX_NUM_TAPS_HI, i_rx_narrow_coeffs, q_rx_narrow_coeffs, FIR_RX_DECIMATE_PTR,
			RX_DECIMATION_RATE_12KHZ, &IIR_2k8_LPF_S,
			&FirRxInterpolate, NULL, 1400
		 },
//40
    {
        AUDIO_2P9KHZ, "LPF", FILTER_MASK_SSB, 1, IQ_RX_NUM_TAPS_HI, i_rx_narrow_coeffs, q_rx_narrow_coeffs, FIR_RX_DECIMATE_PTR,
        RX_DECIMATION_RATE_12KHZ, &IIR_2k9_LPF,
        &FirRxInterpolate, NULL, 1450
    },

    {
        AUDIO_2P9KHZ, "BPF", FILTER_MASK_SSB, 2, IQ_RX_NUM_TAPS_HI, i_rx_new_coeffs, q_rx_new_coeffs, FIR_RX_DECIMATE_PTR,
        RX_DECIMATION_RATE_12KHZ, &IIR_2k9_BPF,
        &FirRxInterpolate, NULL, 1450  // 1525
    },

    {
        AUDIO_3P2KHZ, "LPF", FILTER_MASK_SSB, 1, IQ_RX_NUM_TAPS_HI, i_rx_new_coeffs, q_rx_new_coeffs, FIR_RX_DECIMATE_PTR,
        RX_DECIMATION_RATE_12KHZ, &IIR_3k2_LPF,
        &FirRxInterpolate, NULL, 1600
    },

    {
        AUDIO_3P2KHZ, "BPF", FILTER_MASK_SSB, 2, IQ_RX_NUM_TAPS_HI, i_rx_new_coeffs, q_rx_new_coeffs, FIR_RX_DECIMATE_PTR,
        RX_DECIMATION_RATE_12KHZ, &IIR_3k2_BPF,
        &FirRxInterpolate, NULL, 1675
    },

    // in filters from 3k4 on, the FIR interpolate is 4 taps and an additional IIR interpolation filter
//44	// is switched in to accurately prevent alias frequencies
    {
        AUDIO_3P4KHZ, "LPF", FILTER_MASK_SSB, 1, IQ_RX_NUM_TAPS_HI, i_rx_new_coeffs, q_rx_new_coeffs, FIR_RX_DECIMATE_PTR,
        RX_DECIMATION_RATE_12KHZ, &IIR_3k4_LPF,
        &FirRxInterpolate_4_5k, &IIR_aa_5k, 1700
    },
//45
    {
        AUDIO_3P4KHZ, "BPF", FILTER_MASK_SSB, 2, IQ_RX_NUM_TAPS_HI, i_rx_new_coeffs, q_rx_new_coeffs, FIR_RX_DECIMATE_PTR,
        RX_DECIMATION_RATE_12KHZ, &IIR_3k4_BPF,
        &FirRxInterpolate_4_5k, &IIR_aa_5k, 1775
    },

    {
        AUDIO_3P6KHZ, "LPF", FILTER_MASK_SSB, 1, IQ_RX_NUM_TAPS_HI, i_rx_new_coeffs, q_rx_new_coeffs, FIR_RX_DECIMATE_PTR,
        RX_DECIMATION_RATE_12KHZ, &IIR_3k6_LPF,
        &FirRxInterpolate_4_5k, &IIR_aa_5k, 1800
    },

    {
        AUDIO_3P6KHZ, "BPF", FILTER_MASK_SSB, 2, IQ_RX_NUM_TAPS_HI, i_rx_new_coeffs, q_rx_new_coeffs, FIR_RX_DECIMATE_PTR,
        RX_DECIMATION_RATE_12KHZ, &IIR_3k6_BPF,
        &FirRxInterpolate_4_5k, &IIR_aa_5k, 1875
    },

    {
        AUDIO_3P8KHZ, "LPF", FILTER_MASK_SSB, 1, IQ_RX_NUM_TAPS, i_rx_4k5_coeffs, q_rx_4k5_coeffs, &FirRxDecimate,
        RX_DECIMATION_RATE_12KHZ, &IIR_3k8_LPF,
        &FirRxInterpolate_4_5k, &IIR_aa_5k, 1900
    },

    {
        AUDIO_3P8KHZ, "BPF", FILTER_MASK_SSB, 2, IQ_RX_NUM_TAPS, i_rx_4k5_coeffs, q_rx_4k5_coeffs, &FirRxDecimate,
        RX_DECIMATION_RATE_12KHZ, &IIR_3k8_BPF,
        &FirRxInterpolate_4_5k, &IIR_aa_5k, 1975
    },
//50
    {
        AUDIO_4P0KHZ, "LPF", FILTER_MASK_SSB, 1, IQ_RX_NUM_TAPS, i_rx_4k5_coeffs, q_rx_4k5_coeffs, &FirRxDecimate,
        RX_DECIMATION_RATE_12KHZ, &IIR_4k_LPF,
        &FirRxInterpolate_4_5k, &IIR_aa_5k, 2000
    },

    {
        AUDIO_4P2KHZ, "LPF", FILTER_MASK_SSB, 1, IQ_RX_NUM_TAPS, i_rx_4k5_coeffs, q_rx_4k5_coeffs, &FirRxDecimate,
        RX_DECIMATION_RATE_12KHZ, &IIR_4k2_LPF,
        &FirRxInterpolate_4_5k, &IIR_aa_5k, 2100
    },

    {
        AUDIO_4P4KHZ, "LPF", FILTER_MASK_SSB, 1, IQ_RX_NUM_TAPS, i_rx_4k5_coeffs, q_rx_4k5_coeffs, &FirRxDecimate,
        RX_DECIMATION_RATE_12KHZ, &IIR_4k4_LPF,
        &FirRxInterpolate_4_5k, &IIR_aa_5k, 2200
    },

    {
        AUDIO_4P6KHZ, "LPF", FILTER_MASK_SSB, 1, IQ_RX_NUM_TAPS, i_rx_4k5_coeffs, q_rx_4k5_coeffs, &FirRxDecimate,
        RX_DECIMATION_RATE_12KHZ, &IIR_4k6_LPF,
        &FirRxInterpolate_4_5k, &IIR_aa_5k, 2300
    },

    {
        AUDIO_4P8KHZ, "LPF", FILTER_MASK_SSB, 1, IQ_RX_NUM_TAPS, i_rx_4k5_coeffs, q_rx_4k5_coeffs, &FirRxDecimate,
        RX_DECIMATION_RATE_12KHZ, &IIR_4k8_LPF,
        &FirRxInterpolate_4_5k, &IIR_aa_5k, 2400
    },

//55		// new decimation rate, new decimation filter, new interpolation filter, no IIR Prefilter, no IIR interpolation filter
    {
        AUDIO_5P0KHZ, "LPF", FILTER_MASK_SSB, 1, IQ_RX_NUM_TAPS, i_rx_5k_coeffs, q_rx_5k_coeffs, &FirRxDecimateMinLPF,
        RX_DECIMATION_RATE_24KHZ, NULL,
        &FirRxInterpolate10KHZ, NULL
    },

    {
        AUDIO_5P5KHZ, "LPF", FILTER_MASK_SSB, 1, IQ_RX_NUM_TAPS, i_rx_5k_coeffs, q_rx_5k_coeffs, &FirRxDecimateMinLPF,
        RX_DECIMATION_RATE_24KHZ, NULL,
        &FirRxInterpolate10KHZ, NULL
    },

    {
        AUDIO_6P0KHZ, "LPF", FILTER_MASK_SSB, 1, IQ_RX_NUM_TAPS, i_rx_6k_coeffs, q_rx_6k_coeffs, &FirRxDecimateMinLPF,
        RX_DECIMATION_RATE_24KHZ, NULL,
        &FirRxInterpolate10KHZ, NULL
    },

    {
        AUDIO_6P5KHZ, "LPF", FILTER_MASK_SSB, 1, IQ_RX_NUM_TAPS, i_rx_6k_coeffs, q_rx_6k_coeffs, &FirRxDecimateMinLPF,
        RX_DECIMATION_RATE_24KHZ, NULL,
        &FirRxInterpolate10KHZ, NULL
    },

    {
        AUDIO_7P0KHZ, "LPF", FILTER_MASK_SSB, 1, IQ_RX_NUM_TAPS, i_rx_6k_coeffs, q_rx_6k_coeffs, &FirRxDecimateMinLPF,
        RX_DECIMATION_RATE_24KHZ, NULL,
        &FirRxInterpolate10KHZ, NULL
    },
//60
    {
        AUDIO_7P5KHZ, "LPF", FILTER_MASK_SSB, 1, IQ_RX_NUM_TAPS, i_rx_7k5_coeffs, q_rx_7k5_coeffs, &FirRxDecimateMinLPF,
        RX_DECIMATION_RATE_24KHZ, NULL,
        &FirRxInterpolate10KHZ, NULL
    },
    // additional IIR interpolation filter
    {
        AUDIO_8P0KHZ, "LPF", FILTER_MASK_SSB, 1, IQ_RX_NUM_TAPS, i_rx_10k_coeffs, q_rx_10k_coeffs, &FirRxDecimateMinLPF,
        RX_DECIMATION_RATE_24KHZ, NULL,
        &FirRxInterpolate_4_10k, &IIR_aa_8k
    },

    {
        AUDIO_8P5KHZ, "LPF", FILTER_MASK_SSB, 1, IQ_RX_NUM_TAPS, i_rx_10k_coeffs, q_rx_10k_coeffs, &FirRxDecimateMinLPF,
        RX_DECIMATION_RATE_24KHZ, NULL,
        &FirRxInterpolate_4_10k, &IIR_aa_8k5
    },

    {
        AUDIO_9P0KHZ, "LPF", FILTER_MASK_SSB, 1, IQ_RX_NUM_TAPS, i_rx_10k_coeffs, q_rx_10k_coeffs, &FirRxDecimateMinLPF,
        RX_DECIMATION_RATE_24KHZ, NULL,
        &FirRxInterpolate_4_10k, &IIR_aa_9k
    },

    {
        AUDIO_9P5KHZ, "LPF", FILTER_MASK_SSB, 1, IQ_RX_NUM_TAPS, i_rx_10k_coeffs, q_rx_10k_coeffs, &FirRxDecimateMinLPF,
        RX_DECIMATION_RATE_24KHZ, NULL,
        &FirRxInterpolate_4_10k, &IIR_aa_9k5
    },

    {
        AUDIO_10P0KHZ, "LPF", FILTER_MASK_SSB, 1, IQ_RX_NUM_TAPS, i_rx_10k_coeffs, q_rx_10k_coeffs, &FirRxDecimateMinLPF,
        RX_DECIMATION_RATE_24KHZ, NULL,
        &FirRxInterpolate_4_10k, &IIR_aa_10k
    },

    //###################################################################################################################################
    // These are the "new" AM/SAM filters, January 2017
    // designed for an IIR lowpass stopband frequency that is exactly the same as the filter bandwidth
    // In sideband-selected SAM, there is no FIR filter, so the IIR has to do all the work
    // Let�s try them
    //###################################################################################################################################

    {
        AUDIO_1P4KHZ, "AM/SAM", FILTER_MASK_AMSAM, 1, IQ_RX_NUM_TAPS, iq_rx_am_2k3_coeffs, iq_rx_am_2k3_coeffs, &FirRxDecimate,
        RX_DECIMATION_RATE_12KHZ, &IIR_1k4_LPF,
        &FirRxInterpolate, NULL, 700
    },

    {
        AUDIO_1P6KHZ, "AM/SAM", FILTER_MASK_AMSAM, 1, IQ_RX_NUM_TAPS, iq_rx_am_2k3_coeffs, iq_rx_am_2k3_coeffs, &FirRxDecimate,
        RX_DECIMATION_RATE_12KHZ, &IIR_1k6_LPF,
        &FirRxInterpolate, NULL, 800
    },

    {
        AUDIO_1P8KHZ, "AM/SAM", FILTER_MASK_AMSAM, 1, IQ_RX_NUM_TAPS, iq_rx_am_2k3_coeffs, iq_rx_am_2k3_coeffs, &FirRxDecimate,
        RX_DECIMATION_RATE_12KHZ, &IIR_1k8_LPF,
        &FirRxInterpolate, NULL, 900
    },

    {
        AUDIO_2P1KHZ, "AM/SAM", FILTER_MASK_AMSAM, 1, IQ_RX_NUM_TAPS, iq_rx_am_2k3_coeffs, iq_rx_am_2k3_coeffs, &FirRxDecimate,
        RX_DECIMATION_RATE_12KHZ, &IIR_2k1_LPF,
        &FirRxInterpolate, NULL, 1050
    },

    {
        AUDIO_2P3KHZ, "AM/SAM", FILTER_MASK_AMSAM, 1, IQ_RX_NUM_TAPS, iq_rx_am_2k3_coeffs, iq_rx_am_2k3_coeffs, &FirRxDecimate,
        RX_DECIMATION_RATE_12KHZ, &IIR_2k3_LPF,
        &FirRxInterpolate, NULL, 1150
    },

    {
        AUDIO_2P5KHZ, "AM/SAM", FILTER_MASK_AMSAM, 1, IQ_RX_NUM_TAPS, iq_rx_am_3k6_coeffs, iq_rx_am_3k6_coeffs, &FirRxDecimate,
        RX_DECIMATION_RATE_12KHZ, &IIR_2k5_LPF,
        &FirRxInterpolate, NULL, 1250
    },

    {
        AUDIO_2P7KHZ, "AM/SAM", FILTER_MASK_AMSAM, 1, IQ_RX_NUM_TAPS, iq_rx_am_3k6_coeffs, iq_rx_am_3k6_coeffs, &FirRxDecimate,
        RX_DECIMATION_RATE_12KHZ, &IIR_2k7_LPF,
        &FirRxInterpolate, NULL, 1350
    },

    {
        AUDIO_2P9KHZ, "AM/SAM", FILTER_MASK_AMSAM, 1, IQ_RX_NUM_TAPS, iq_rx_am_3k6_coeffs, iq_rx_am_3k6_coeffs, &FirRxDecimateMinLPF,
        RX_DECIMATION_RATE_12KHZ, &IIR_2k9_LPF,
        &FirRxInterpolate, NULL, 1450
    },

    {
        AUDIO_3P2KHZ, "AM/SAM", FILTER_MASK_AMSAM, 1, IQ_RX_NUM_TAPS, iq_rx_am_3k6_coeffs, iq_rx_am_3k6_coeffs, &FirRxDecimateMinLPF,
        RX_DECIMATION_RATE_12KHZ, &IIR_3k2_LPF,
        &FirRxInterpolate, NULL, 1600
    },

    {
        AUDIO_3P4KHZ, "AM/SAM", FILTER_MASK_AMSAM, 1, IQ_RX_NUM_TAPS, iq_rx_am_3k6_coeffs, iq_rx_am_3k6_coeffs, &FirRxDecimateMinLPF,
        RX_DECIMATION_RATE_12KHZ, &IIR_3k4_LPF,
        &FirRxInterpolate_4_5k, &IIR_aa_5k, 1700
    },

    {
        AUDIO_3P6KHZ, "AM/SAM", FILTER_MASK_AMSAM, 1, IQ_RX_NUM_TAPS, iq_rx_am_4k5_coeffs, iq_rx_am_4k5_coeffs, &FirRxDecimateMinLPF,
        RX_DECIMATION_RATE_12KHZ, &IIR_3k6_LPF,
        &FirRxInterpolate_4_5k, &IIR_aa_5k, 1800
    },

    {
        AUDIO_3P8KHZ, "AM/SAM", FILTER_MASK_AMSAM, 1, IQ_RX_NUM_TAPS, iq_rx_am_4k5_coeffs, iq_rx_am_4k5_coeffs, &FirRxDecimateMinLPF,
        RX_DECIMATION_RATE_12KHZ, &IIR_3k8_LPF,
        &FirRxInterpolate_4_5k, &IIR_aa_5k, 1900
    },

    {
        AUDIO_4P0KHZ, "AM/SAM", FILTER_MASK_AMSAM, 1, IQ_RX_NUM_TAPS, iq_rx_am_4k5_coeffs, iq_rx_am_4k5_coeffs, &FirRxDecimateMinLPF,
        RX_DECIMATION_RATE_12KHZ, &IIR_4k_LPF,
        &FirRxInterpolate_4_5k, &IIR_aa_5k, 2000
    },

    {
        AUDIO_4P2KHZ, "AM/SAM", FILTER_MASK_AMSAM, 1, IQ_RX_NUM_TAPS, iq_rx_am_4k5_coeffs, iq_rx_am_4k5_coeffs, &FirRxDecimateMinLPF,
        RX_DECIMATION_RATE_12KHZ, &IIR_4k2_LPF,
        &FirRxInterpolate_4_5k, &IIR_aa_5k, 2100
    },

    {
        AUDIO_4P4KHZ, "AM/SAM", FILTER_MASK_AMSAM, 1, IQ_RX_NUM_TAPS, iq_rx_am_4k5_coeffs, iq_rx_am_4k5_coeffs, &FirRxDecimateMinLPF,
        RX_DECIMATION_RATE_12KHZ, &IIR_4k4_LPF,
        &FirRxInterpolate_4_5k, &IIR_aa_5k, 2200
    },

    {
        AUDIO_4P6KHZ, "AM/SAM", FILTER_MASK_AMSAM, 1, IQ_RX_NUM_TAPS, iq_rx_am_4k5_coeffs, iq_rx_am_4k5_coeffs, &FirRxDecimateMinLPF,
        RX_DECIMATION_RATE_12KHZ, &IIR_4k6_LPF,
        &FirRxInterpolate_4_5k, &IIR_aa_5k, 2300
    },

    {
        AUDIO_4P8KHZ, "AM/SAM", FILTER_MASK_AMSAM, 1, IQ_RX_NUM_TAPS, iq_rx_am_5k_coeffs, iq_rx_am_5k_coeffs, &FirRxDecimateMinLPF,
        RX_DECIMATION_RATE_12KHZ, &IIR_4k8_LPF,
        &FirRxInterpolate_4_5k, &IIR_aa_5k, 2400
    },

    {
        AUDIO_5P0KHZ, "AM/SAM", FILTER_MASK_AMSAM, 1, IQ_RX_NUM_TAPS, iq_rx_am_5k_coeffs, iq_rx_am_5k_coeffs, &FirRxDecimateMinLPF,
        RX_DECIMATION_RATE_24KHZ, &IIR_5k_LPF,
        &FirRxInterpolate10KHZ, NULL
    },

    {
        AUDIO_6P0KHZ, "AM/SAM", FILTER_MASK_AMSAM, 1, IQ_RX_NUM_TAPS, iq_rx_am_6k_coeffs, iq_rx_am_6k_coeffs, &FirRxDecimateMinLPF,
        RX_DECIMATION_RATE_24KHZ, &IIR_6k_LPF,
        &FirRxInterpolate10KHZ, NULL
    },

    {
        AUDIO_7P5KHZ, "AM/SAM", FILTER_MASK_AMSAM, 1, IQ_RX_NUM_TAPS, iq_rx_am_7k5_coeffs, iq_rx_am_7k5_coeffs, &FirRxDecimateMinLPF,
        RX_DECIMATION_RATE_24KHZ, &IIR_7k5_LPF,
        &FirRxInterpolate10KHZ, NULL
    },

    {
        AUDIO_10P0KHZ, "AM/SAM", FILTER_MASK_AMSAM, 1, IQ_RX_NUM_TAPS, iq_rx_am_10k_coeffs, iq_rx_am_10k_coeffs, &FirRxDecimateMinLPF,
        RX_DECIMATION_RATE_24KHZ, &IIR_10k_LPF,
        &FirRxInterpolate_4_10k, &IIR_aa_10k
    },


/*
//###################################################################################################################################
// AM filters: designed for an IIR lowpass stopband frequency that is approx. 1.8 times higher than the FIR bandwidth
//  --> an AM 2.5kHz filter has 2.5kHz of audio when centre tuned, but can have up to 1.8 * 2.5kHz = 4.6kHz of bandwidth (IIR filter!)
//      when tuned away from the carrier, this has been called "sideband-selected AM demodulation" . . . wow . . .
//###################################################################################################################################

     // in AM, we ALWAYS use the lowpass filter version of the IIR audio PreFilter, regardless of the selected filter_select_ID.
    // this is because we assume AM mode to be used to demodulate DSB signals, so BPF (sideband suppression) is not necessary

    {
        AUDIO_1P4KHZ, "AM/SAM", FILTER_MASK_AMSAM, 1, IQ_NUM_TAPS, iq_rx_am_2k3_coeffs, iq_rx_am_2k3_coeffs, &FirRxDecimate,
        RX_DECIMATION_RATE_12KHZ, &IIR_2k3_LPF,
        &FirRxInterpolate, NULL
    },

    {
        AUDIO_1P6KHZ, "AM/SAM", FILTER_MASK_AMSAM, 1, IQ_NUM_TAPS, iq_rx_am_2k3_coeffs, iq_rx_am_2k3_coeffs, &FirRxDecimate,
        RX_DECIMATION_RATE_12KHZ, &IIR_2k9_LPF,
        &FirRxInterpolate, NULL
    },

    {
        AUDIO_1P8KHZ, "AM/SAM", FILTER_MASK_AMSAM, 1, IQ_NUM_TAPS, iq_rx_am_2k3_coeffs, iq_rx_am_2k3_coeffs, &FirRxDecimate,
        RX_DECIMATION_RATE_12KHZ, &IIR_3k2_LPF,
        &FirRxInterpolate, NULL
    },

    {
        AUDIO_2P1KHZ, "AM/SAM", FILTER_MASK_AMSAM, 1, IQ_NUM_TAPS, iq_rx_am_3k6_coeffs, iq_rx_am_3k6_coeffs, &FirRxDecimate,
        RX_DECIMATION_RATE_12KHZ, &IIR_3k6_LPF,
        &FirRxInterpolate_4_5k, &IIR_aa_5k
    },

    {
        AUDIO_2P3KHZ, "AM/SAM", FILTER_MASK_AMSAM, 1, IQ_NUM_TAPS, iq_rx_am_3k6_coeffs, iq_rx_am_3k6_coeffs, &FirRxDecimate,
        RX_DECIMATION_RATE_12KHZ, &IIR_4k2_LPF,
        &FirRxInterpolate_4_5k, &IIR_aa_5k
    },

    {
        AUDIO_2P5KHZ, "AM/SAM", FILTER_MASK_AMSAM, 1, IQ_NUM_TAPS, iq_rx_am_3k6_coeffs, iq_rx_am_3k6_coeffs, &FirRxDecimate,
        RX_DECIMATION_RATE_12KHZ, &IIR_4k6_LPF,
        &FirRxInterpolate_4_5k, &IIR_aa_5k
    },

    {
        AUDIO_2P7KHZ, "AM/SAM", FILTER_MASK_AMSAM, 1, IQ_NUM_TAPS, iq_rx_am_3k6_coeffs, iq_rx_am_3k6_coeffs, &FirRxDecimate,
        RX_DECIMATION_RATE_12KHZ, &IIR_4k8_LPF,
        &FirRxInterpolate_4_5k, &IIR_aa_5k
    },

    {
        AUDIO_2P9KHZ, "AM/SAM", FILTER_MASK_AMSAM, 1, IQ_NUM_TAPS, iq_rx_am_3k6_coeffs, iq_rx_am_3k6_coeffs, &FirRxDecimateMinLPF,
        RX_DECIMATION_RATE_24KHZ, &IIR_5k5_LPF,
        &FirRxInterpolate10KHZ, NULL
    },

    {
        AUDIO_3P2KHZ, "AM/SAM", FILTER_MASK_AMSAM, 1, IQ_NUM_TAPS, iq_rx_am_3k6_coeffs, iq_rx_am_3k6_coeffs, &FirRxDecimateMinLPF,
        RX_DECIMATION_RATE_24KHZ, &IIR_6k_LPF,
        &FirRxInterpolate10KHZ, NULL
    },

    {
        AUDIO_3P4KHZ, "AM/SAM", FILTER_MASK_AMSAM, 1, IQ_NUM_TAPS, iq_rx_am_3k6_coeffs, iq_rx_am_3k6_coeffs, &FirRxDecimateMinLPF,
        RX_DECIMATION_RATE_24KHZ, &IIR_6k5_LPF,
        &FirRxInterpolate10KHZ, NULL
    },

    {
        AUDIO_3P6KHZ, "AM/SAM", FILTER_MASK_AMSAM, 1, IQ_NUM_TAPS, iq_rx_am_4k5_coeffs, iq_rx_am_4k5_coeffs, &FirRxDecimateMinLPF,
        RX_DECIMATION_RATE_24KHZ, &IIR_7k_LPF,
        &FirRxInterpolate10KHZ, NULL
    },

    {
        AUDIO_3P8KHZ, "AM/SAM", FILTER_MASK_AMSAM, 1, IQ_NUM_TAPS, iq_rx_am_4k5_coeffs, iq_rx_am_4k5_coeffs, &FirRxDecimateMinLPF,
        RX_DECIMATION_RATE_24KHZ, &IIR_7k_LPF,
        &FirRxInterpolate10KHZ, NULL
    },

    {
        AUDIO_4P0KHZ, "AM/SAM", FILTER_MASK_AMSAM, 1, IQ_NUM_TAPS, iq_rx_am_4k5_coeffs, iq_rx_am_4k5_coeffs, &FirRxDecimateMinLPF,
        RX_DECIMATION_RATE_24KHZ, &IIR_7k5_LPF,
        &FirRxInterpolate10KHZ, NULL
    },

    {
        AUDIO_4P2KHZ, "AM/SAM", FILTER_MASK_AMSAM, 1, IQ_NUM_TAPS, iq_rx_am_4k5_coeffs, iq_rx_am_4k5_coeffs, &FirRxDecimateMinLPF,
        RX_DECIMATION_RATE_24KHZ, &IIR_8k_LPF,
        &FirRxInterpolate10KHZ, NULL
    },

    // from 4.4kHz on, the AM filter has no more IIR PreFilter (at 24ksps sample rate), BUT we add IIR filtering after interpolation (at 48 ksps)!
//80
    {
        AUDIO_4P4KHZ, "AM/SAM", FILTER_MASK_AMSAM, 1, IQ_NUM_TAPS, iq_rx_am_4k5_coeffs, iq_rx_am_4k5_coeffs, &FirRxDecimateMinLPF,
        RX_DECIMATION_RATE_24KHZ, NULL,
        &FirRxInterpolate_4_10k, &IIR_aa_8k
    },

    {
        AUDIO_4P6KHZ, "AM/SAM", FILTER_MASK_AMSAM, 1, IQ_NUM_TAPS, iq_rx_am_4k5_coeffs, iq_rx_am_4k5_coeffs, &FirRxDecimateMinLPF,
        RX_DECIMATION_RATE_24KHZ, NULL,
        &FirRxInterpolate_4_10k, &IIR_aa_8k5
    },

    {
        AUDIO_4P8KHZ, "AM/SAM", FILTER_MASK_AMSAM, 1, IQ_NUM_TAPS, iq_rx_am_5k_coeffs, iq_rx_am_5k_coeffs, &FirRxDecimateMinLPF,
        RX_DECIMATION_RATE_24KHZ, NULL,
        &FirRxInterpolate_4_10k, &IIR_aa_9k
    },

    {
        AUDIO_5P0KHZ, "AM/SAM", FILTER_MASK_AMSAM, 1, IQ_NUM_TAPS, iq_rx_am_5k_coeffs, iq_rx_am_5k_coeffs, &FirRxDecimateMinLPF,
        RX_DECIMATION_RATE_24KHZ, NULL,
        &FirRxInterpolate_4_10k, &IIR_aa_9k5
    },

    // from 6kHz on, we have no PreFilter, an IIR interpolation filter of 10k and only change the FIR filters bandwidths
    // remember that the AM 5k filter is capable of up to 10kHz bandwidth, if you offtune the AM carrier
    // . . . same for 6k = 12kHz bw, 7k5 = 15kHz bw, 10kHz = max of 20kHz bandwidth

    {
        AUDIO_6P0KHZ, "AM/SAM", FILTER_MASK_AMSAM, 1, IQ_NUM_TAPS, iq_rx_am_6k_coeffs, iq_rx_am_6k_coeffs, &FirRxDecimateMinLPF,
        RX_DECIMATION_RATE_24KHZ, NULL,
        &FirRxInterpolate_4_10k, &IIR_aa_10k
    },

    {
        AUDIO_7P5KHZ, "AM/SAM", FILTER_MASK_AMSAM, 1, IQ_NUM_TAPS, iq_rx_am_7k5_coeffs, iq_rx_am_7k5_coeffs, &FirRxDecimateMinLPF,
        RX_DECIMATION_RATE_24KHZ, NULL,
        &FirRxInterpolate_4_10k, &IIR_aa_10k
    },

    {
        AUDIO_10P0KHZ, "AM/SAM", FILTER_MASK_AMSAM, 1, IQ_NUM_TAPS, iq_rx_am_10k_coeffs, iq_rx_am_10k_coeffs, &FirRxDecimateMinLPF,
        RX_DECIMATION_RATE_24KHZ, NULL,
        &FirRxInterpolate_4_10k, &IIR_aa_10k
    },


*/




//###################################################################################################################################
// SAM filters:
// they now use the same filter paths as the AM filters
// so these definitions are obsolete from 2017 on
//###################################################################################################################################
/*

    {
        AUDIO_1P8KHZ, "SAM", FILTER_MASK_SAM, 1, IQ_NUM_TAPS, iq_rx_am_2k3_coeffs, iq_rx_am_2k3_coeffs, &FirRxDecimate,
        RX_DECIMATION_RATE_12KHZ, &IIR_3k2_LPF,
        &FirRxInterpolate, NULL
    },

    {
        AUDIO_2P3KHZ, "SAM", FILTER_MASK_SAM, 1, IQ_NUM_TAPS, iq_rx_am_3k6_coeffs, iq_rx_am_3k6_coeffs, &FirRxDecimate,
        RX_DECIMATION_RATE_12KHZ, &IIR_4k2_LPF,
        &FirRxInterpolate_4_5k, &IIR_aa_5k
    },

    {
        AUDIO_2P9KHZ, "SAM", FILTER_MASK_SAM, 1, IQ_NUM_TAPS, iq_rx_am_3k6_coeffs, iq_rx_am_3k6_coeffs, &FirRxDecimate,
        RX_DECIMATION_RATE_24KHZ, &IIR_5k5_LPF,
        &FirRxInterpolate10KHZ, NULL
    },

    {
        AUDIO_3P4KHZ, "SAM", FILTER_MASK_SAM, 1, IQ_NUM_TAPS, iq_rx_am_3k6_coeffs, iq_rx_am_3k6_coeffs, &FirRxDecimate,
        RX_DECIMATION_RATE_24KHZ, &IIR_6k5_LPF,
        &FirRxInterpolate10KHZ, NULL
    },
    // old remark, must be analysed again
    // measurements with Spectrum Lab have shown that there was considerable, but only barely
	// audible alias noise in the SAM 4k2 and 4k8 filters (25-30dB below signal, you could hear it clearly when treble gain was 20dB)
	// now I have implemented the IIR_aa_5k antialiasing filter in SAM 4k2 and 4k8 filters and all the aliases
	// are down by at least 60dB below signal level
    {
        AUDIO_4P2KHZ, "SAM", FILTER_MASK_SAM, 1, IQ_NUM_TAPS, iq_rx_am_4k5_coeffs, iq_rx_am_4k5_coeffs, &FirRxDecimate,
        RX_DECIMATION_RATE_24KHZ, &IIR_8k_LPF,
        &FirRxInterpolate10KHZ, NULL
    },

    {
        AUDIO_4P8KHZ, "SAM", FILTER_MASK_SAM, 1, IQ_NUM_TAPS, iq_rx_am_5k_coeffs, iq_rx_am_5k_coeffs, &FirRxDecimate,
        RX_DECIMATION_RATE_24KHZ, NULL,
        &FirRxInterpolate_4_10k, &IIR_aa_9k
    },

    {
        AUDIO_6P0KHZ, "SAM", FILTER_MASK_SAM, 1, IQ_NUM_TAPS, iq_rx_am_6k_coeffs, iq_rx_am_6k_coeffs, &FirRxDecimateMinLPF,
        RX_DECIMATION_RATE_24KHZ, NULL,
        &FirRxInterpolate_4_10k, &IIR_aa_10k
    },

    {
        AUDIO_7P5KHZ, "SAM", FILTER_MASK_SAM, 1, IQ_NUM_TAPS, iq_rx_am_7k5_coeffs, iq_rx_am_7k5_coeffs, &FirRxDecimateMinLPF,
        RX_DECIMATION_RATE_24KHZ, NULL,
        &FirRxInterpolate_4_10k, &IIR_aa_10k
    },

    {
        AUDIO_10P0KHZ, "SAM", FILTER_MASK_SAM, 1, IQ_NUM_TAPS, iq_rx_am_10k_coeffs, iq_rx_am_10k_coeffs, &FirRxDecimateMinLPF,
        RX_DECIMATION_RATE_24KHZ, NULL,
        &FirRxInterpolate_4_10k, &IIR_aa_10k
    }
*/
}; // end FilterPath

/*
 * @brief Converts demodulation mode (DEMOD_CW ... ) to FILTER_MODE_CW ...
 * @param demodulation mode (DEMOD_CW ... DEMOD_SAM)
 * @returns filter mode for given demodulation (FILTER_MODE_CW / _SSB, ... )
 */
uint16_t AudioFilter_GetFilterModeFromDemodMode(const uint8_t dmod_mode)
{
    uint16_t filter_mode;
    switch(dmod_mode)
    {
		case DEMOD_AM:	filter_mode = FILTER_MODE_AM; 	break;
		case DEMOD_FM:	filter_mode = FILTER_MODE_FM;	break;
		case DEMOD_CW:	filter_mode = FILTER_MODE_CW;	break;
		case DEMOD_SAM: filter_mode = FILTER_MODE_AM;	break;	// filter_mode = FILTER_MODE_SAM; // from 2017 on, the new SAM mode uses the AM filters!
		default:		filter_mode = FILTER_MODE_SSB;
    }
    return filter_mode;
}

static bool AudioFilter_IsInPathMemory(uint16_t filter_mode, uint16_t filter_path)
{
    uint16_t idx;
    bool retval = false;
    for (idx = 1; idx < FILTER_PATH_MEM_MAX; idx++)
    {
        if (ts.filter_path_mem[filter_mode][idx] == filter_path)
        {
            retval = true;
            break;
        }
        // leave loop if match found
    }
    return retval;
}

bool AudioFilter_IsApplicableFilterPath(const uint16_t query, const uint16_t filter_mode, const uint8_t filter_path)
{
    bool retval = false;
    uint16_t filter_mode_mask = 1<<filter_mode;

    // these rules handle special cases
    // if((FilterPathInfo[idx].id == AUDIO_1P8KHZ) && ((ts.filter_cw_wide_disable) && (current_mode == DEMOD_CW))) { idx = AUDIO_300HZ; break; }
    // in this case, next applicable mode is 300 Hz, so selected and leave loop
    if ((FilterPathInfo[filter_path].mode & filter_mode_mask) != 0)
    {
        if ((query & PATH_USE_RULES) != 0)
        {
            // we have to check if this mode is IN the list of modes always offered in this mode, regardless of enablement
            /*if ((FilterInfo[FilterPathInfo[filter_path].id].always_on_modes & filter_mode_mask) != 0) {
              // okay, applicable, leave loop
              retval = true;
            } else*/ if (AudioFilter_IsInPathMemory(filter_mode,filter_path))
            {
                retval = true;
//      } else if (!ts.filter_select[FilterPathInfo[filter_path].id]) {
//        retval = false;
//      } else if((FilterPathInfo[filter_path].id == AUDIO_300HZ || FilterPathInfo[filter_path].id == AUDIO_500HZ) && ((ts.filter_ssb_narrow_disable) && (filter_mode_mask != FILTER_MASK_CW))) {
//        // jump over 300 Hz / 500 Hz if ssb_narrow_disable and voice mode
//        retval = false;
            }
            else     // no match in rules, not applicable
            {
                retval = false;
            }
        }
        else
        {
            retval = true;
        }

    }
    return retval;
}

/*
 * @brief Find Next Applicable Filter Path based on the information in the filter data structure
 *
 * Takes into account the current mode (SSB, CW, ..., some special rules, etc,). It will wrap around and always return a
 * valid filter id. In case not applicable filter was found, it returns the currently selected filter_id
 *
 * @param query specifies which selection approach is used: ALL_APPLICABLE_PATHS, NEXT_BANDWIDTH, SAME_BANDWIDTH
 * @param current_mode -> all values allowed for FILTER_MODE_xxx
 * @param current_path -> a valid filter path id, which is used as starting point
 *        use ts.filter_path unless in special cases (e.g. use for filter selection menus)
 * @returns next applicable filter id
 */


uint8_t AudioFilter_NextApplicableFilterPath(const uint16_t query, const uint16_t filter_mode, const uint8_t current_path)
{

    uint8_t last_bandwidth_id = FilterPathInfo[current_path].id;
    int idx = 0;

    if ((query & PATH_LAST_USED_IN_MODE) != 0 &&  ts.filter_path_mem[filter_mode][0] != 0)
    {
        idx = ts.filter_path_mem[filter_mode][0];
    }
    else
    {
        // we run through all audio filters, starting with the next following, making sure to wrap around
        // we leave this loop once we found a filter that is applicable using "break"
        // or skip to next filter to check using "continue"
        for (idx = current_path+((query&PATH_DOWN)?-1:1); idx != current_path;
                idx+=(query&PATH_DOWN)?-1:1)
        {
            idx %= AUDIO_FILTER_PATH_NUM;
            if (idx<0)
            {
                idx+=AUDIO_FILTER_PATH_NUM;
            }

            // skip over all filters of current bandwidth
            if (((query & PATH_NEXT_BANDWIDTH) != 0) && (last_bandwidth_id == FilterPathInfo[idx].id))
            {
                continue;
            }
            // skip over all filters of different bandwidth
            if (((query & PATH_SAME_BANDWITH) != 0) && (last_bandwidth_id != FilterPathInfo[idx].id))
            {
                continue;
            }

            if (AudioFilter_IsApplicableFilterPath(query, filter_mode, idx))
            {
                break;
            }
        }
    }
    if ((query&PATH_DONT_STORE) == 0)
    {
        ts.filter_path_mem[filter_mode][0] = idx;
    }
    return  idx;
}



/*
 * HILBERT FIR FILTER HANDLING START
 */

// SSB Hilbert TX Filter
#include "iq_tx_filter.h"

// RX Hilbert transform (90 degree) FIR filter state tables and instances
arm_fir_instance_f32    Fir_Rx_Hilbert_I;
arm_fir_instance_f32    Fir_Rx_Hilbert_Q;

// FIXME: Needs comment and check!
#define FIR_RX_HILBERT_STATE_SIZE (IQ_RX_NUM_TAPS_MAX + IQ_RX_BLOCK_SIZE)
static float32_t    __MCHF_SPECIALMEM Fir_Rx_Hilbert_State_I[FIR_RX_HILBERT_STATE_SIZE];
static float32_t    __MCHF_SPECIALMEM Fir_Rx_Hilbert_State_Q[FIR_RX_HILBERT_STATE_SIZE];

//
// TX Hilbert transform (90 degree) FIR filter state tables and instances
arm_fir_instance_f32    Fir_Tx_Hilbert_I;
arm_fir_instance_f32    Fir_Tx_Hilbert_Q;

// FIXME: Needs comment and check!
#define FIR_TX_HILBERT_STATE_SIZE (IQ_TX_NUM_TAPS_MAX + IQ_TX_BLOCK_SIZE)
static float   __MCHF_SPECIALMEM         Fir_Tx_Hilbert_State_I[FIR_TX_HILBERT_STATE_SIZE];
static float   __MCHF_SPECIALMEM         Fir_Tx_Hilbert_State_Q[FIR_TX_HILBERT_STATE_SIZE];

// TX FIR filter for use with FreeDV
arm_fir_instance_f32    Fir_TxFreeDV_Interpolate_I;
arm_fir_instance_f32    Fir_TxFreeDV_Interpolate_Q;

// FIXME: Needs comment and check!
#define FIR_FREEDV_INTERPOLATE_STATE_SIZE (FIR_TX_FREEDV_INTERPOLATE_NUM_TAPS + IQ_TX_BLOCK_SIZE)
static float   __MCHF_SPECIALMEM Fir_TxFreeDV_Interpolate_State_I[FIR_FREEDV_INTERPOLATE_STATE_SIZE];
static float   __MCHF_SPECIALMEM Fir_TxFreeDV_Interpolate_State_Q[FIR_FREEDV_INTERPOLATE_STATE_SIZE];


// Audio RX - Decimator (numTaps+blockSize-1)
arm_fir_decimate_instance_f32   FirDecim_RxSam_I;
arm_fir_decimate_instance_f32   FirDecim_RxSam_Q;

// FIXME: Needs comment and check!!!
#define FIR_DECIM_SAM_STATE_SIZE (IQ_RX_NUM_TAPS + IQ_RX_BLOCK_SIZE)
static float32_t __MCHF_SPECIALMEM FirDecim_RxSam_State_I[FIR_DECIM_SAM_STATE_SIZE];
static float32_t __MCHF_SPECIALMEM FirDecim_RxSam_State_Q[FIR_DECIM_SAM_STATE_SIZE];

typedef struct
{
    float32_t   fir_rx_hilbert_taps_q[IQ_RX_NUM_TAPS_MAX];
    float32_t   fir_rx_hilbert_taps_i[IQ_RX_NUM_TAPS_MAX];

    float32_t   fir_tx_hilbert_taps_q[IQ_TX_NUM_TAPS_MAX];
    float32_t   fir_tx_hilbert_taps_i[IQ_TX_NUM_TAPS_MAX];

} IQFilterCoeffs_t;

static IQFilterCoeffs_t   __MCHF_SPECIALMEM     fc;


/*
 * @brief Initialize RX Hilbert filters
 */
void 	AudioFilter_Init_Rx_Hilbert_FIR( uint8_t  dmod_mode )
{
    // always make a fresh copy of the original Q and I coefficients into fast RAM
    // this speeds up processing on the STM32F4
    // NOTE:  We are assuming that the I and Q filters are of the same length!

    // new filter_path method
    // take all info from FilterPathInfo
    // phase adjustment is now done in audio_driver.c audio_rx_processor

    const  uint32_t   rx_iq_num_taps = FilterPathInfo[ts.filter_path].FIR_numTaps;

    // in FilterPathInfo, we have stored the coefficients already, so no if . . . necessary
    // also applicable for FM case !
    for(int i = 0; i < rx_iq_num_taps; i++)
    {
        fc.fir_rx_hilbert_taps_i[i] = FilterPathInfo[ts.filter_path].FIR_I_coeff_file[i];
        fc.fir_rx_hilbert_taps_q[i] = FilterPathInfo[ts.filter_path].FIR_Q_coeff_file[i];
    }

    // Initialization of the FIR/Hilbert filters
    arm_fir_init_f32 ( &Fir_Rx_Hilbert_I, rx_iq_num_taps, fc.fir_rx_hilbert_taps_i, Fir_Rx_Hilbert_State_I, IQ_RX_BLOCK_SIZE ); // load "I" with "I" coefficients
    arm_fir_init_f32 ( &Fir_Rx_Hilbert_Q, rx_iq_num_taps, fc.fir_rx_hilbert_taps_q, Fir_Rx_Hilbert_State_Q, IQ_RX_BLOCK_SIZE ); // load "Q" with "Q" coefficients
    //
    // Set up RX SAM decimation/filter
    if (dmod_mode == DEMOD_SAM || dmod_mode == DEMOD_AM)
    {
        if (FilterPathInfo[ts.filter_path].FIR_numTaps != 0)
        {
            FirDecim_RxSam_I.numTaps = FilterPathInfo[ts.filter_path].FIR_numTaps;      // Number of taps in FIR filter
            FirDecim_RxSam_Q.numTaps = FilterPathInfo[ts.filter_path].FIR_numTaps;      // Number of taps in FIR filter
            FirDecim_RxSam_I.pCoeffs = fc.fir_rx_hilbert_taps_i; //FilterPathInfo[ts.filter_path].FIR_I_coeff_file;       // Filter coefficients
            FirDecim_RxSam_Q.pCoeffs = fc.fir_rx_hilbert_taps_q; //FilterPathInfo[ts.filter_path].FIR_Q_coeff_file;       // Filter coefficients
        }
        else
        {
            FirDecim_RxSam_I.numTaps = 0;
            FirDecim_RxSam_Q.numTaps = 0;
            FirDecim_RxSam_I.pCoeffs = NULL;
            FirDecim_RxSam_Q.pCoeffs = NULL;
        }

        FirDecim_RxSam_I.M           = ads.decimation_rate;
        FirDecim_RxSam_Q.M           = ads.decimation_rate;
        FirDecim_RxSam_I.pState      = FirDecim_RxSam_State_I;            // Filter state variables
        FirDecim_RxSam_Q.pState      = FirDecim_RxSam_State_Q;

        arm_fill_f32 ( 0.0, FirDecim_RxSam_State_I, FIR_DECIM_SAM_STATE_SIZE );
        arm_fill_f32 ( 0.0, FirDecim_RxSam_State_Q, FIR_DECIM_SAM_STATE_SIZE );
    }
}


/*
 * @brief Initialize TX Hilbert filters
 */
void AudioFilter_Init_Tx_Hilbert_FIR(void)
{

     ads.tx_filter_adjusting++;        // disable TX I/Q filter during adjustment
    // always make a fresh copy of the original Q and I coefficients
    // NOTE:  We are assuming that the I and Q filters are of the same length!
    //
    // phase adjustment is now done in audio_driver.c audio_tx_processor

  /*  IQ_FilterDescriptor iq_tx_filter =
            (ts.tx_filter == TX_FILTER_BASS ||
            		//             ts.tx_filter == TX_FILTER_WIDE_TREBLE) ? iq_tx_wide:iq_tx_narrow;
            		// FIXME: dirty trial: always use the "wide" 201 tap Hilbert filter
             ts.tx_filter == TX_FILTER_SOPRANO) ? iq_tx_wide:iq_tx_wide;
  */

    IQ_FilterDescriptor  iq_tx_filter = ts.tx_decimatedIQ ?  iq_tx_wide_decim : iq_tx_wide;

    const uint32_t tx_iq_num_taps = iq_tx_filter.num_taps;

    for ( int i = 0; i < iq_tx_filter.num_taps; i++ )
    {
        fc.fir_tx_hilbert_taps_q[i] = iq_tx_filter.q[i];
        fc.fir_tx_hilbert_taps_i[i] = iq_tx_filter.i[i];
    }

    uint32_t   IQ_block_size = ts.tx_decimatedIQ ?  8 : 32;

    arm_fir_init_f32 ( &Fir_Tx_Hilbert_I, tx_iq_num_taps, fc.fir_tx_hilbert_taps_i, Fir_Tx_Hilbert_State_I, IQ_block_size); // IQ_TX_BLOCK_SIZE);
    arm_fir_init_f32 ( &Fir_Tx_Hilbert_Q, tx_iq_num_taps, fc.fir_tx_hilbert_taps_q, Fir_Tx_Hilbert_State_Q, IQ_block_size); // IQ_TX_BLOCK_SIZE);

    arm_fir_init_f32 ( &Fir_TxFreeDV_Interpolate_I, Fir_TxFreeDV_Interpolate.numTaps, Fir_TxFreeDV_Interpolate.pCoeffs, Fir_TxFreeDV_Interpolate_State_I, IQ_TX_BLOCK_SIZE);
    arm_fir_init_f32 ( &Fir_TxFreeDV_Interpolate_Q, Fir_TxFreeDV_Interpolate.numTaps, Fir_TxFreeDV_Interpolate.pCoeffs, Fir_TxFreeDV_Interpolate_State_Q, IQ_TX_BLOCK_SIZE);

  if ( ads.tx_filter_adjusting )    ads.tx_filter_adjusting--;        // re-enable TX I/Q filter now that we are done
}

void AudioFilter_GetNamesOfFilterPath(uint16_t filter_path,const char** filter_names)
{

    const FilterPathDescriptor *path   = &FilterPathInfo[filter_path];
    const FilterDescriptor     *filter = &FilterInfo[path->id];

    filter_names[0] = filter->name;
    filter_names[1] = path->name;
}

void AudioFilter_SetDefaultMemories()
{
    // filter selection for FM is hardcoded
    ts.filter_path_mem[FILTER_MODE_FM][1] = 1;
    ts.filter_path_mem[FILTER_MODE_FM][2] = 2;
    ts.filter_path_mem[FILTER_MODE_FM][3] = 3;
}

void AudioFilter_CalcGoertzel(Goertzel* g, float32_t freq, const uint32_t size, const float goertzel_coeff, float32_t samplerate)
{
    g->a   = (0.5 + (freq * goertzel_coeff) * size/samplerate);
    g->b   = (2*PI*g->a)/size;
    g->sin = sinf(g->b);
    g->cos = cosf(g->b);
    g->r   = 2 * g->cos;
}

void AudioFilter_GoertzelInput(Goertzel* goertzel, float32_t in)
{
	goertzel->buf[0] = goertzel->r * goertzel->buf[1] - goertzel->buf[2]	+ in;
	goertzel->buf[2] = goertzel->buf[1];
	goertzel->buf[1] = goertzel->buf[0];
}

float32_t AudioFilter_GoertzelEnergy(Goertzel* goertzel)
{
	float32_t a      = (goertzel->buf[1] - (goertzel->buf[2] * goertzel->cos));// calculate energy at frequency
	float32_t b      = (goertzel->buf[2] * goertzel->sin);
	goertzel->buf[0] = 0;
	goertzel->buf[1] = 0;
	goertzel->buf[2] = 0;
	return sqrtf(a * a + b * b);
}




