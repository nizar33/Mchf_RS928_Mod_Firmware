/*  -*-  mode: c; tab-width: 4; indent-tabs-mode: t; c-basic-offset: 4; coding: utf-8  -*-  */

#include "uhsdr_board.h"

#include "audio_management.h"
#include "math.h"
#include "audio_driver.h"
#include "softdds.h"
#include "fm_subaudible_tone_table.h"
#include "radio_management.h"
#if 0
//
//
//*----------------------------------------------------------------------------
//* Function Name       : UiCalcAGCDecay
//* Object              : Calculate Decay timing for AGC (RECEIVE!)
//* Input Parameters    :
//* Output Parameters   :
//* Functions called    :
//*----------------------------------------------------------------------------
//

void AudioManagement_CalcAGCDecay()
{
    switch (ts.agc_mode)
    {
    case AGC_SLOW:
        ads.agc_decay = AGC_SLOW_DECAY;
        break;
    case AGC_FAST:
        ads.agc_decay = AGC_FAST_DECAY;
        break;
    case AGC_CUSTOM:      // calculate custom AGC setting
    {
        ads.agc_decay = powf(10,-((((float32_t)ts.agc_custom_decay)+30.0)/10.0));
    }
    break;
    default:
        ads.agc_decay = AGC_MED_DECAY;
    }
}
#endif
//
//
//*----------------------------------------------------------------------------
//* Function Name       : UiCalcALCDecay
//* Object              : Calculate Decay timing for ALC (TRANSMIT!)
//* Input Parameters    :
//* Output Parameters   :
//* Functions called    :
//*----------------------------------------------------------------------------
//
void AudioManagement_CalcALCDecay( void )
{
    // calculate ALC decay (release) time constant - this needs to be moved to its own function (and the one in "ui_menu.c")
    ads.alc_decay = powf(10,-((((float32_t)ts.alc_decay_var) + 20.0)/10.0));   //20.0 // 35.0
}

#if 0//
//
//*----------------------------------------------------------------------------
//* Function Name       : UiCalcRFGain
//* Object              : Calculate RF Gain internal value from user setting
//* Input Parameters    :
//* Output Parameters   :
//* Functions called    :
//*----------------------------------------------------------------------------
//
void AudioManagement_CalcRFGain(void)
{
    float tcalc;    // temporary value as "ads.agc_rf_gain" may be used during the calculation!

    // calculate working RF gain value
    tcalc = (float)ts.rf_gain;
    tcalc *= 1.4;
    tcalc -= 20;
    tcalc /= 10;
    ads.agc_rf_gain = powf(10, tcalc);

}
//
//
//*----------------------------------------------------------------------------
//* Function Name       : AudioManagement_CalcAGCVals
//* Object              : Calculate internal AGC values from user settings
//* Input Parameters    :
//* Output Parameters   :
//* Functions called    :
//*----------------------------------------------------------------------------
//
void AudioManagement_CalcAGCVals(void)
{
    float max_rf_gain = 1 + (ts.max_rf_gain <= MAX_RF_GAIN_MAX ? ts.max_rf_gain : MAX_RF_GAIN_DEFAULT);

    ads.agc_knee = AGC_KNEE_REF * max_rf_gain;
    ads.agc_val_max = AGC_VAL_MAX_REF / max_rf_gain;
    ads.post_agc_gain = POST_AGC_GAIN_SCALING_REF / (float)(ts.max_rf_gain + 1);
    // TODO: Why is here always ts.max_rf_gain used? Shouldn't it be max_rf_gain too?
}
#endif


//
//
//*----------------------------------------------------------------------------
//* Function Name       : AudioManagement_CalcNB_AGC
//* Object              : Calculate Noise Blanker AGC settings
//* Input Parameters    :
//* Output Parameters   :
//* Functions called    :
//*----------------------------------------------------------------------------
/*
void AudioManagement_CalcNB_AGC(void)
{
    float temp_float;

    temp_float       =   0;                    //  (float)ts.nb_agc_time_const;       // get user setting
    temp_float       =   35 - temp_float;      //  NB_MAX_AGC_SETTING - temp_float;  //  invert (0 = minimum))
    temp_float      /=  1.1;                   //  scale calculation
    temp_float      *=  temp_float;            //  square value
    temp_float      +=  1;                     //  offset by one
    temp_float      /=  44000;                 //  rescale
    temp_float      +=  1;                     //  prevent negative log result
    ads.nb_sig_filt  =  log10f(temp_float);    //  de-linearize and save in "new signal" contribution parameter
    ads.nb_agc_filt  =  1 - ads.nb_sig_filt;   //  calculate parameter for recyling "old" AGC value
}
*/
//
//*******************************************************************************************************************
static float AudioManagement_CalcAdjustInFreqRangeHelper( float32_t adj_low, float32_t adj_high, float32_t freq, float32_t scaling)
{
     return ((adj_high - adj_low) / (28100000.0 - 3600000.0) * (freq - 3600000.0) + adj_low)/scaling;        // get current gain adjustment setting  USB and other modes
}
//**************************************************************************************************************
static  void  AudioManagement_CalcIqGainAdjustVarHelper ( volatile iq_float_t* var,  float32_t  adj )
{
        var->i = 1 + adj;
        var->q = 1 - adj;
}

void AudioManagement_CalcIqPhaseGainAdjust ( float freq )
{

	uint8_t    tx_band = RadioManagement_GetBand ( freq );
	float32_t  adj_i_rx;
	float32_t  adj_i_tx;
	uint8_t    IQ_Band1 = 30;
	uint8_t    IQ_Band2 = 30;

	switch ( tx_band )  ///  NIZZZ  pour corriger les IQ par bande
	{
		case  BAND_MODE_20  : IQ_Band1 = IQ_20M;  IQ_Band2 = IQ_20M;  break;
		case  BAND_MODE_40  : IQ_Band1 = IQ_40M;  IQ_Band2 = IQ_40M;  break;
		case  BAND_MODE_80  : IQ_Band1 = IQ_80M;  IQ_Band2 = IQ_80M;  break;
	    case  BAND_MODE_160 : IQ_Band1 = IQ_160M; IQ_Band2 = IQ_160M; break;
		case  BAND_MODE_17  : IQ_Band1 = IQ_17M;  IQ_Band2 = IQ_17M;  break;
		case  BAND_MODE_15  : IQ_Band1 = IQ_15M;  IQ_Band2 = IQ_15M;  break;
		case  BAND_MODE_12  : IQ_Band1 = IQ_12M;  IQ_Band2 = IQ_12M;  break;
		case  BAND_MODE_10  : IQ_Band1 = IQ_10M;  IQ_Band2 = IQ_10M;  break;
#ifndef TUNISIANN
		case  BAND_MODE_60  : IQ_Band1 = IQ_60M;  IQ_Band2 = IQ_60M;  break;
		case  BAND_MODE_30  : IQ_Band1 = IQ_30M;  IQ_Band2 = IQ_30M;  break;
#endif
		default             : IQ_Band1 = IQ_80M;  IQ_Band2 = IQ_10M;  break;
	}

	                          //*************  Rx  Xlat_On ( SSB + CW ) *************************

	ads.iq_phase_balance_rx    = AudioManagement_CalcAdjustInFreqRangeHelper ( ts.rx_iq_phase_balance[IQ_Band1].value[IQ_TRANS_ON],  ts.rx_iq_phase_balance[IQ_Band2].value[IQ_TRANS_ON], freq, SCALING_FACTOR_IQ_PHASE_ADJUST     );
	adj_i_rx                   = AudioManagement_CalcAdjustInFreqRangeHelper (-ts.rx_iq_gain_balance [IQ_Band1].value[IQ_TRANS_ON],- ts.rx_iq_gain_balance [IQ_Band2].value[IQ_TRANS_ON], freq, SCALING_FACTOR_IQ_AMPLITUDE_ADJUST );
	                             AudioManagement_CalcIqGainAdjustVarHelper   (&ts.rx_adj_gain_var, adj_i_rx * 2);  /// NIZZ pour calculer la chaine I sans le Q au correction IQ


	                          //*************  Tx  Xlat On  ( SSB ) *******************************

	ads.iq_phase_balance_tx[1] = AudioManagement_CalcAdjustInFreqRangeHelper ( ts.tx_iq_phase_balance[IQ_Band1].value[1], ts.tx_iq_phase_balance[IQ_Band2].value[1], freq, SCALING_FACTOR_IQ_PHASE_ADJUST    );
	adj_i_tx                   = AudioManagement_CalcAdjustInFreqRangeHelper ( ts.tx_iq_gain_balance [IQ_Band1].value[1], ts.tx_iq_gain_balance [IQ_Band2].value[1], freq, SCALING_FACTOR_IQ_AMPLITUDE_ADJUST);
								 AudioManagement_CalcIqGainAdjustVarHelper   ( &ts.tx_adj_gain_var[1], adj_i_tx );


		                     //************** Tx  Xlat off ( CW )  *********************************

	ads.iq_phase_balance_tx[0] = AudioManagement_CalcAdjustInFreqRangeHelper ( ts.tx_iq_phase_balance[IQ_80M].value[0], ts.tx_iq_phase_balance[IQ_10M].value[0], freq, 1000.0    );
	adj_i_tx                   = AudioManagement_CalcAdjustInFreqRangeHelper ( ts.tx_iq_gain_balance [IQ_80M].value[0], ts.tx_iq_gain_balance [IQ_10M].value[0], freq, 2500.0    );
								 AudioManagement_CalcIqGainAdjustVarHelper   ( &ts.tx_adj_gain_var[0], adj_i_tx );
}

//*----------------------------------------------------------------------------
typedef struct AlcParams_s
{
    uint32_t   tx_postfilt_gain;
    uint32_t   alc_decay;
} AlcParams;

/*
static const AlcParams alc_params[] =
{
    { 3,  0},       // (1,15)  ancienne valeur
    { 2, 12},
    { 4, 10},
    { 6,  9},
    { 7,  8},
    { 8,  7},
	{ 10, 6},
    { 12, 5},
    { 15, 4},
    { 17, 3},
    { 20, 2},
    { 25, 1},
    { 25, 0},
};
*/



/**
 * @brief Set TX audio compression settings (gain and ALC decay rate) based on user setting
 */
void AudioManagement_CalcTxCompLevel()
{

	/*
    int16_t tx_comp_level = ts.tx_comp_level;

    if (TX_AUDIO_COMPRESSION_MIN < tx_comp_level && tx_comp_level < TX_AUDIO_COMPRESSION_CUS)
    {
        ts.alc_tx_postfilt_gain_var = alc_params[ ts.tx_comp_level ].tx_postfilt_gain;      // restore "pristine" EEPROM values
        ts.alc_decay_var            = alc_params[ ts.tx_comp_level ].alc_decay;

    }*/


	// read saved values from EEPROM
	ts.alc_tx_postfilt_gain_var = ts.alc_tx_custom_comp_gain;      // restore "pristine" EEPROM values
	ts.alc_decay_var            = ts.alc_tx_custom_comp_decay;


    AudioManagement_CalcALCDecay();
}

#include "fm_subaudible_tone_table.h"
//
//*----------------------------------------------------------------------------
//* Function Name       : UiCalcSubaudibleGenFreq
//* Object              : Calculate frequency word for subaudible tone generation  [KA7OEI October, 2015]
//* Input Parameters    :
//* Output Parameters   :
//* Functions called    :
//*----------------------------------------------------------------------------
void AudioManagement_CalcSubaudibleGenFreq(void)
{
    ads.fm_subaudible_tone_gen_freq = fm_subaudible_tone_table[ts.fm_subaudible_tone_gen_select];       // look up tone frequency (in Hz)
    ads.fm_subaudible_tone_word     = (ulong)(ads.fm_subaudible_tone_gen_freq * FM_SUBAUDIBLE_TONE_WORD_CALC_FACTOR);   // calculate tone word
}

/**
 * @brief Calculate frequency word for subaudible tone, call after change of detection frequency  [KA7OEI October, 2015]
 */
void AudioManagement_CalcSubaudibleDetFreq(void)
{
    const uint32_t size = BUFF_LEN/2;

    ads.fm_subaudible_tone_det_freq = fm_subaudible_tone_table[ts.fm_subaudible_tone_det_select];       // look up tone frequency (in Hz)

    // Calculate Goertzel terms for tone detector(s)
    AudioFilter_CalcGoertzel(&ads.fm_goertzel[FM_HIGH], ads.fm_subaudible_tone_det_freq, FM_SUBAUDIBLE_GOERTZEL_WINDOW*size,FM_GOERTZEL_HIGH, IQ_SAMPLE_RATE);
    AudioFilter_CalcGoertzel(&ads.fm_goertzel[FM_LOW] , ads.fm_subaudible_tone_det_freq, FM_SUBAUDIBLE_GOERTZEL_WINDOW*size,FM_GOERTZEL_LOW, IQ_SAMPLE_RATE);
    AudioFilter_CalcGoertzel(&ads.fm_goertzel[FM_CTR] , ads.fm_subaudible_tone_det_freq, FM_SUBAUDIBLE_GOERTZEL_WINDOW*size,1.0, IQ_SAMPLE_RATE);
}

//
//
//*----------------------------------------------------------------------------
//* Function Name       : UiLoadToneBurstMode
//* Object              : Load tone burst mode  [KA7OEI October, 2015]
//* Input Parameters    :
//* Output Parameters   :
//* Functions called    :
//*----------------------------------------------------------------------------
void AudioManagement_LoadToneBurstMode(void)
{
    switch(ts.fm_tone_burst_mode)
    {
		case FM_TONE_BURST_1750_MODE: 	ads.fm_tone_burst_word = FM_TONE_BURST_1750;	break;
		case FM_TONE_BURST_2135_MODE:	ads.fm_tone_burst_word = FM_TONE_BURST_2135;	break;
		default:			            ads.fm_tone_burst_word = 0;			            break;
    }
}

/**
 * @brief Generates the sound for a given beep frequency, call after change of beep freq or before beep  [KA7OEI October, 2015]
 */
void AudioManagement_LoadBeepFreq()
{
    float calc;

    if(ts.flags2 & FLAGS2_KEY_BEEP_ENABLE)      // is beep enabled?
    {
        softdds_setFreqDDS(&ads.beep, ts.beep_frequency,ts.samp_rate,false);
    }
    else
    {
        softdds_setFreqDDS(&ads.beep, 0,ts.samp_rate,true); // not enabled - zero out frequency word
    }

    calc = (float)(ts.beep_loudness-1);  // range 0-20
    calc /= 2;                          // range 0-10
    calc *= calc;                       // range 0-100
    calc += 3;                          // range 3-103
    ads.beep_loudness_factor = calc / 400;      // range from 0.0075 to 0.2575 - multiplied by DDS output
}

/**
 * @brief Tell audio driver to make beeping sound  [KA7OEI October, 2015]
 */
void AudioManagement_KeyBeep()
{
    // FIXME: Do we really need to call this here, or shall we rely on the calls
    // made when changing the freq/settings?
    // right now every beep runs the generator code
    AudioManagement_LoadBeepFreq();       // load and calculate beep frequency
    ts.beep_timing = ts.sysclock + BEEP_DURATION;       // set duration of beep
    ts.beep_active = 1;                                 // activate tone
}

void AudioManagement_SetSidetoneForDemodMode(uint8_t dmod_mode, bool tune_mode)
{
    float tonefreq[2] = {0.0, 0.0};
    switch(dmod_mode)
    {
		case DEMOD_CW:
						tonefreq[0] = tune_mode? CW_SIDETONE_FREQ_DEFAULT : ts.cw_sidetone_freq;
						break;
		case DEMOD_DIGI:
						if (ts.digital_mode == DigitalMode_RTTY || ts.digital_mode == DigitalMode_BPSK)
						{
							tonefreq[0] = tune_mode?  CW_SIDETONE_FREQ_DEFAULT : ts.cw_sidetone_freq;
						}
						break;
		default:
						tonefreq[0] = tune_mode?  SSB_TUNE_FREQ : 0.0;

						if ( tune_mode )
						{
							if ( ts.tune_tone_mode == TUNE_TONE_TWO)
							{  			// ARRL Standard is 700Hz and 1900Hz
										// so I temporarily changed this to SSB_TUNE_FREQ + 1200, DD4WH 2016_07_14
										// --> TWO_TONE = 750Hz and 1950Hz
										//            tonefreq[1] = tune_mode?(SSB_TUNE_FREQ+600):0.0;
								tonefreq[1] = SSB_TUNE_FREQ + 1200;
							}
							else {  tonefreq[0] = 0; }     //pour un test tune en battement null d'un QSO, et sans perturber l'audio du QSO en cours
						}
    }

    softdds_configRunIQ ( tonefreq, ts.samp_rate,  0 );
}
