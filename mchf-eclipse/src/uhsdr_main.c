/*  -*-  mode: c; tab-width: 4; indent-tabs-mode: t; c-basic-offset: 4; coding: utf-8  -*-  */
/************************************************************************************
 **                                                                                 **
 **                                        UHSDR                                    **
 **               a powerful firmware for STM32 based SDR transceivers              **
 **                                                                                 **
 **---------------------------------------------------------------------------------**
 **                                                                                 **
 **  File name:                                                                     **
 **  Description:                                                                   **
 **  Last Modified:                                                                 **
 **  Licence:		GNU GPLv3                                                      **
 ************************************************************************************/

// Common
#include "uhsdr_board.h"
#include <stdio.h>
#include <malloc.h>
#include "uhsdr_rtc.h"
#include "ui_spectrum.h"

#include "ui_configuration.h"
#include "config_storage.h"

// serial EEPROM driver
#include "uhsdr_hw_i2c.h"

// Audio Driver
#include "drivers/audio/audio_driver.h"
#include "drivers/audio/audio_management.h"
#include "drivers/audio/cw/cw_gen.h"
#include "drivers/audio/freedv_uhsdr.h"
#include "drivers/audio/audio_nr.h"

//cat
#include "drivers/cat/cat_driver.h"

// UI Driver
#include "drivers/ui/ui_driver.h"
#include "drivers/ui/lcd/ui_lcd_hy28.h"
#include "drivers/ui/menu/ui_menu.h"
#include "drivers/ui/oscillator/osc_interface.h"
#include "drivers/ui/oscillator/osc_si5351a.h"
#include "drivers/ui/oscillator/osc_si570.h"
#include "drivers/audio/codec/codec.h"
#include "misc/profiling.h"
// Keyboard Driver
// #include "keyb_driver.h"

// Misc
#include "drivers/audio/softdds/softdds.h"

// Eeprom
#include "misc/v_eprom/eeprom.h"
//
#include "drivers/ui/radio_management.h"
//

#include "misc/TestCPlusPlusInterface.h"

void HAL_GPIO_EXTI_Callback ( uint16_t GPIO_Pin )   // Interruption Hardware des PINs du microcontroleur
{
	if ( ts.paddles_active )
	{
		switch( GPIO_Pin )
		{
			case PADDLE_DAH:	// Call handler   // Straight Key  est aussi pris en charge ici
								if (Board_Ptt_Dah_Line_Pressed() && ts.dmod_mode != DEMOD_SAM)
								{  // was PTT line low? Not in a RX Only Mode?
									ts.ptt_req = true;     // yes - ONLY then do we activate PTT!  (e.g. prevent hardware bug from keying PTT!)
									if(ts.dmod_mode == DEMOD_CW || is_demod_rtty() || ts.cw_text_entry)
									{
										CwGen_DahIRQ();     // Yes - go to CW state machine
									}
								}
								break;

			case PADDLE_DIT:	if( Board_Dit_Line_Pressed() && ( ts.dmod_mode == DEMOD_CW || is_demod_rtty() || ts.cw_text_entry) )
								{
									ts.ptt_req = true;
									CwGen_DitIRQ();   /// Le Straight key n'est pas pris en charge ici par le Dit
								}
								break;

			case BUTTON_PWR:   	break;


		}
    }
}

void TransceiverStateInit(void)
{
    // Defaults always
    ts.txrx_mode 		              = TRX_MODE_RX;				// start in RX
    ts.samp_rate		              = I2S_AUDIOFREQ_48K;			// set sampling rate
    ts.enc_one_mode 	              = ENC_ONE_MODE_AUDIO_GAIN;
    ts.enc_two_mode 	              = ENC_TWO_MODE_RF_GAIN;
 // ts.enc_thr_mode		              = ENC_THREE_MODE_RIT;
    ts.enc_thr_mode                   = ENC_THREE_MODE_INPUT_CTRL;  // pour initialiser le bouton "DIG Ext" au lieu du RIT au d�marrage de la radio
    ts.band		  		              = BAND_MODE_20;			// band from eeprom
    ts.rx_temp_mute		              = false;				// used in muting audio during band change
    ts.filter_band		              = 0;					// used to indicate the bpf filter selection for power detector coefficient selection
    ts.dmod_mode 		              = DEMOD_USB;			// demodulator mode
    ts.rx_gain[RX_AUDIO_SPKR].value   = AUDIO_GAIN_DEFAULT;
    ts.rx_gain[RX_AUDIO_DIG].value	  = DIG_GAIN_DEFAULT;

    ts.rx_gain[RX_AUDIO_SPKR].max	  = MAX_VOLUME_DEFAULT;		// Set max volume default
    ts.rx_gain[RX_AUDIO_DIG].max      =  DIG_GAIN_MAX;			// Set max volume default

    ts.rx_gain[RX_AUDIO_SPKR].active_value = 1;			// this variable is used in the active RX audio processing function
    ts.rx_gain[RX_AUDIO_DIG ].active_value = 1;			// this variable is used in the active RX audio processing function
    ts.rf_gain			    = DEFAULT_RF_GAIN;		        //
    ts.lineout_gain         = LINEOUT_GAIN_DEFAULT;
    ts.rf_codec_gain	    = DEFAULT_RF_CODEC_GAIN_VAL;   // Set default RF gain (0 = lowest, 8 = highest, 9 = "Auto")
    ts.rit_value		    = 0;					       // RIT value

    ts.cw_sidetone_gain	    = DEFAULT_SIDETONE_GAIN;	    // Sidetone gain

    ts.cw_keyer_mode		= CW_KEYER_MODE_IAM_B;			// CW keyer mode
    ts.cw_keyer_speed		= CW_KEYER_SPEED_DEFAULT;		// CW keyer speed
    ts.cw_sidetone_freq	    = CW_SIDETONE_FREQ_DEFAULT;		// CW sidetone and TX offset frequency
    ts.cw_paddle_reverse	= 0;					        // Paddle defaults to NOT reversed
    ts.cw_rx_delay		    = CW_TX2RX_DELAY_DEFAULT;		// Delay of TX->RX turnaround
    ts.cw_keyer_weight      = CW_KEYER_WEIGHT_DEFAULT;

    ts.audio_spkr_unmute_delay_count		= VOICE_TX2RX_DELAY_DEFAULT;			// TX->RX delay turnaround

    ts.nb_setting		= 0;					// Noise Blanker setting

    for (int i = 0; i < IQ_ADJUST_POINTS_NUM; i++)
    {
        for (int j = 0; j < IQ_TRANS_NUM; j++)
        {
            ts.tx_iq_gain_balance[i].value[j]    = IQ_BALANCE_OFF;                // Default settings for RX and TX gain and phase balance
            ts.tx_iq_phase_balance[i].value[j]   = IQ_BALANCE_OFF;                // Default settings for RX and TX gain and phase balance
            ts.rx_iq_gain_balance[i].value[j]    = IQ_BALANCE_OFF;                // Default settings for RX and TX gain and phase balance
            ts.rx_iq_phase_balance[i].value[j]   = IQ_BALANCE_OFF;                // Default settings for RX and TX gain and phase balance
        }
    }

        ts.tune_freq		= 0;
    //  ts.tune_freq_old	= 0;
    //	ts.calib_mode		= 0;				// calibrate mode

    ts.menu_mode		= 0;					// menu mode
    ts.menu_item		= 0;					// menu item selection
    ts.menu_var			= 0;					// menu item change variable
    ts.menu_var_changed	= 0;					// TRUE if a menu variable was changed and that an EEPROM save should be done

    ts.tx_audio_source	            = TX_AUDIO_MIC;			// default source is microphone
    ts.tx_mic_gain_mult	            = MIC_GAIN_DEFAULT;		// actual operating value for microphone gain
    ts.tx_gain[TX_AUDIO_MIC]		= MIC_GAIN_DEFAULT;	    // default line gain
    ts.tx_gain[TX_AUDIO_LINEIN_L]	= LINE_GAIN_DEFAULT;	// default line gain
    ts.tx_gain[TX_AUDIO_LINEIN_R]	= LINE_GAIN_DEFAULT;	// default line gain
    ts.tx_gain[TX_AUDIO_DIG_EX]	    = LINE_GAIN_DEFAULT;	// default line gain
    ts.tx_gain[TX_AUDIO_DIG_IQ]		= LINE_GAIN_DEFAULT;	// default line gain

    ts.tune				= 0;				// reset tuning flag

    ts.tx_power_factor	= 0.50;					// TX power factor

    ts.pa_bias			= PA_BIAS_DEFAULT;		// Use lowest possible voltage as default
    ts.pa_cw_bias		= PA_BIAS_DEFAULT;			// Use lowest possible voltage as default (nonzero sets separate bias for CW mode)
    ts.freq_cal			= 0;				// Initial setting for frequency calibration
    ts.power_level		= PA_LEVEL_DEFAULT;			// See uhsdr_board.h for setting
    //
    //	ts.codec_vol		= 0;					// Holder for codec volume
    //	ts.codec_mute_state	= 0;					// Holder for codec mute state
    //	ts.codec_was_muted  = 0;						// Indicator that codec *was* muted
    //
    ts.powering_down	    = 0;						// TRUE if powering down
    //

    //
    ts.menu_item		        = 0;		// start out with a reasonable menu item
    ts.radio_config_menu_enable = 0;		// TRUE if radio configuration menu is to be enabled
    ts.xverter_mode		        = 0;		// TRUE if transverter mode is active (e.g. offset of display)
    ts.xverter_offset	        = 0;		// Frequency offset in transverter mode (added to frequency display)
    ts.refresh_freq_disp	    = 1;		// TRUE if frequency/color display is to be refreshed when next called - NORMALLY LEFT AT 0 (FALSE)!!!
    // This is NOT reset by the LCD function, but must be enabled/disabled externally

    for (int idx=0; idx<MAX_BANDS; idx++)
    {
        ts.pwr_adj [ ADJ_5W         ] [ idx ] = 1;
        ts.pwr_adj [ ADJ_FULL_POWER ] [ idx ] = 1;
    }

    ts.filter_cw_wide_disable		= 0;			// TRUE if wide filters are to be disabled in CW mode
    ts.filter_ssb_narrow_disable	= 0;				// TRUE if narrow (CW) filters are to be disabled in SSB mode
    ts.demod_mode_disable			= 0;		// TRUE if AM mode is to be disabled
    //
    ts.tx_meter_mode	            = METER_SWR;
    //
    ts.alc_tx_custom_comp_decay  	= ALC_DECAY_DEFAULT;			// ALC Decay (release) default value
    ts.alc_decay_var	            = ALC_DECAY_DEFAULT;			// ALC Decay (release) default value
    ts.alc_tx_custom_comp_gain	    = ALC_POSTFILT_GAIN_DEFAULT;	// Post-filter added gain default (used for speech processor/ALC)
    ts.alc_tx_postfilt_gain_var  	= ALC_POSTFILT_GAIN_DEFAULT;	// Post-filter added gain default (used for speech processor/ALC)
    ts.tx_comp_level	            = 0;					// 0=Release Time/Pre-ALC gain manually adjusted, >=1:  Value calculated by this parameter
    ts.tx_comp_out_gain             = 25;
    ts.tx_comp_out_gain_float       = 2.5;
    //
    ts.freq_step_config		        = 0;				// disabled both marker line under frequency and swapping of STEP buttons
    //
    ts.dsp_active		            = 0;					// TRUE if DSP noise reduction is to be enabled
    //    ts.dsp_active		        = 0;					// if this line is enabled win peaks issue is present when starting mcHF with activated NB
    ts.digital_mode		            = DigitalMode_None;					// digital modes OFF by default
    ts.dsp_active_toggle	        = 0xff;					// used to hold the button G2 "toggle" setting.
    ts.dsp_nr_strength	            = 5;					// "Strength" of DSP noise reduction (50 = medium)

#ifdef USE_LMS_AUTONOTCH
    ts.dsp_notch_numtaps            = DSP_NOTCH_NUMTAPS_DEFAULT;		// default for number of FFT taps for notch filter
    ts.dsp_notch_delaybuf_len       = DSP_NOTCH_DELAYBUF_DEFAULT;
    ts.dsp_notch_mu                 = DSP_NOTCH_MU_DEFAULT;
#endif
    ts.dsp_inhibit		            = 1;					// TRUE if DSP is to be inhibited - power up with DSP disabled

    ts.lcd_backlight_brightness     = 0;			// = 0 full brightness
    ts.lcd_backlight_blanking       = 0;				// MSB = 1 for auto-off of backlight, lower nybble holds time for auto-off in seconds
    ts.low_power_config             = LOW_POWER_THRESHOLD_DEFAULT; // add LOW_POWER_THRESHOLD_OFFSET for voltage value
    //
    ts.tune_step		            = 0;					// Used for press-and-hold step size changing mode
    ts.frequency_lock	            = 0;					// TRUE if frequency knob is locked
    //
    ts.tx_disable		            = TX_DISABLE_OFF;	    // > 0 if transmitter is to be disabled
    ts.flags1			            = 0;					// Used to hold individual status flags, stored in EEPROM location "EEPROM_FLAGS1"
    ts.flags2			            = 0;					// Used to hold individual status flags, stored in EEPROM location "EEPROM_FLAGS2"
    ts.sysclock			            = 0;					// This counts up from zero when the unit is powered up at precisely 100 Hz over the long term.  This
    // is NEVER reset and is used for timing certain events.
    ts.version_number_release	    = 0;			// version release - used to detect firmware change
    ts.version_number_major         = 0;			// version build - used to detect firmware change
    ts.nb_agc_time_const	        = 0;			// used to calculate the AGC time constant
    ts.cw_offset_mode	            = CW_OFFSET_USB_RX;	// CW offset mode (USB, LSB, etc.)
    ts.cw_lsb			            = false;		// Flag that indicates CW operates in LSB mode when TRUE
    ts.iq_freq_mode		            = FREQ_IQ_CONV_MODE_DEFAULT;	// used to set/configure the I/Q frequency/conversion mode
    ts.conv_sine_flag	            = 0;			// FALSE until the sine tables for the frequency conversion have been built (normally zero, force 0 to rebuild)
    ts.lsb_usb_auto_select	        = 0;			// holds setting of LSB/USB auto-select above/below 10 MHz
    ts.last_tuning		            = 0;			// this is a timer used to hold off updates of the spectrum scope when an SPI LCD display interface is used
    ts.lcd_blanking_time            = 0;			// this holds the system time after which the LCD is blanked - if blanking is enabled
    ts.lcd_blanking_flag            = 0;			// if TRUE, the LCD is blanked completely (e.g. backlight is off)
    ts.xvtr_adjust_flag             = 0;			// set TRUE if transverter offset adjustment is in process
    ts.vfo_mem_mode                 = 0;		// this is used to record the VFO/memory mode (0 = VFO "A" = backwards compatibility)
    ts.voltmeter_calibrate	        = POWER_VOLTMETER_CALIBRATE_DEFAULT;	// Voltmeter calibration constant

    // spectrum general settings
    ts.spectrum_filter              = SPECTRUM_FILTER_DEFAULT;  // default filter strength for spectrum scope
    ts.spectrum_centre_line_colour  = SPEC_COLOUR_GRID_DEFAULT;      // color of center line of scope grid
    ts.spectrum_freqscale_colour    = SPEC_COLOUR_SCALE_DEFAULT;        // default colour for the spectrum scope frequency scale at the bottom
    ts.spectrum_db_scale            = DB_DIV_10;               // default to 10dB/division
    ts.spectrum_size                = SPECTRUM_SIZE_DEFAULT;        // adjustment for waterfall size
    ts.spectrum_agc_rate            = SPECTRUM_SCOPE_AGC_DEFAULT;       // load default spectrum scope AGC rate

    // scope ui settings
    ts.scope_trace_colour           = SPEC_COLOUR_TRACE_DEFAULT;        // default colour for the spectrum scope trace
    ts.scope_grid_colour            = SPEC_COLOUR_GRID_DEFAULT;     // default colour for the spectrum scope grid
    ts.scope_speed                  = SPECTRUM_SCOPE_SPEED_DEFAULT;     // default rate of spectrum scope update
    ts.scope_scheduler              = 0;         // timer for scheduling the next update of the spectrum update

    // ts.spectrum_scope_nosig_adjust = SPECTRUM_SCOPE_NOSIG_ADJUST_DEFAULT;   // Adjustment for no signal adjustment conditions for spectrum scope

    ts.waterfall.speed                 = WATERFALL_SPEED_DEFAULT;      // default speed of update of the waterfall
    ts.waterfall.color_scheme          = WATERFALL_COLOR_DEFAULT;		// color scheme for waterfall display
    ts.waterfall.vert_step_size        = WATERFALL_STEP_SIZE_DEFAULT;	// step size in waterfall display
#if 0
    ts.waterfall.offset                = WATERFALL_OFFSET_DEFAULT;		// Offset for waterfall display (brightness)
#endif
    ts.waterfall.contrast              = WATERFALL_CONTRAST_DEFAULT;	// contrast setting for waterfall display
    ts.waterfall.scheduler             = 0;

#if 0
    ts.waterfall.nosig_adjust          = WATERFALL_NOSIG_ADJUST_DEFAULT;	// Adjustment for no signal adjustment conditions for waterfall
#endif
//  ts.fft_window_type                 = FFT_WINDOW_DEFAULT; // FFT Windowing type
    ts.dvmode                          = false;		    // disable "DV" mode RX/TX functions by default

    ts.txrx_switch_audio_muting_timing = 0;				// timing value used for muting TX audio when keying PTT to suppress "click" or "thump"
    ts.audio_dac_muting_timer          = 0;				// timer used for muting TX audio when keying PTT to suppress "click" or "thump"
    ts.audio_dac_muting_flag           = 0;				// when TRUE, audio is to be muted after PTT/keyup

    ts.filter_disp_colour              = FILTER_DISP_COLOUR_DEFAULT;
    ts.vfo_mem_flag                    = 0;				// when TRUE, memory mode is enabled
    ts.mem_disp                        = 0;				// when TRUE, memory display is enabled
    ts.load_eeprom_defaults            = 0;				// when TRUE, defaults are loaded when "UiDriverLoadEepromValues()" is called - must be saved by user w/power-down to be permanent!
    ts.fm_subaudible_tone_gen_select   = 0;				// lookup ("tone number") used to index the table generation (0 corresponds to "tone disabled")
    ts.fm_tone_burst_mode              = 0;				// this is the setting for the tone burst generator
    ts.fm_tone_burst_timing            = 0;			    // used to time the duration of the tone burst
    ts.fm_sql_threshold                = FM_SQUELCH_DEFAULT;  // squelch threshold
    ts.fm_subaudible_tone_det_select   = 0;				// lookup ("tone number") used to index the table for tone detection (0 corresponds to "tone disabled")
    ts.beep_active 					   = 1;				// TRUE if beep is active
    ts.beep_frequency 				   = DEFAULT_BEEP_FREQUENCY;	// beep frequency, in Hz
    ts.beep_loudness 				   = DEFAULT_BEEP_LOUDNESS;		// loudness of keyboard/CW sidetone test beep
    ts.load_freq_mode_defaults         = 0;				// when TRUE, load frequency defaults into RAM when "UiDriverLoadEepromValues()" is called - MUST be saved by user IF these are to take effect!
    ts.ser_eeprom_type                 = 0;				// serial eeprom not present
    ts.configstore_in_use              = CONFIGSTORE_IN_USE_FLASH;	// serial eeprom not in use

    ts.tp     						   = &mchf_touchscreen;
    ts.display 						   = &mchf_display;

    ts.show_debug_info				   = false;	// dont show coordinates on LCD
    ts.multi 						   = 0;		// non-translate
    ts.tune_power_level                = 0;		// Tune with FULL POWER
    ts.xlat                            = 0;		// 0 = report base frequency, 1 = report xlat-frequency;
    ts.audio_int_counter               = 0;		// test DL2FW
    ts.cat_band_index                  = 255;	// no CAT command arrived
    ts.notch_frequency                 = 800;	// notch start frequency for manual notch filter
    ts.peak_frequency                  = 750;   // peak start frequency
    ts.bass_gain                       = 6;		// gain of the RX low  shelf EQ filter
    ts.treble_gain					   = 0;		// gain of the RX high shelf EQ filter
    ts.tx_bass_gain 				   = 3;		// gain of the TX low  shelf EQ filter
    ts.tx_treble_gain                  = 13;	// gain of the TX high shelf EQ filter
    ts.s_meter                         = 1;		// S-Meter configuration, 0 = old school, 1 = dBm-based, 2=dBm/Hz-based
    ts.display_dbm                     = 0;						// style of dBm display, 0=OFF, 1= dbm, 2= dbm/Hz
//  ts.dBm_count                       = 0;						// timer start
    ts.tx_filter                       = 0;						// which TX filter has been chosen by the user
    ts.iq_auto_correction              = 1;              // disable/enable automatic IQ correction
    ts.twinpeaks_tested                = 2;                // twinpeak_tested = 2 --> wait for system to warm up
 // twinpeak_tested                    = 0 --> go and test the IQ phase
 // twinpeak_tested                    = 1 --> tested, verified, go and have a nice day!
//  ts.agc_wdsp                        = 0;
    ts.agc_wdsp_mode                   = 2;
    ts.agc_wdsp_slope                  = 70;
    ts.agc_wdsp_hang_enable            = 0;
    ts.agc_wdsp_hang_time              = 100;
    ts.agc_wdsp_hang_thresh            = 40;
    ts.agc_wdsp_thresh                 = 60;
    ts.agc_wdsp_action                 = 0;
    ts.agc_wdsp_switch_mode            = 1;
    ts.agc_wdsp_hang_action            = 0;
    ts.agc_wdsp_tau_decay[0]           = 4000;
    ts.agc_wdsp_tau_decay[1]           = 2000;
    ts.agc_wdsp_tau_decay[2]           = 500;
    ts.agc_wdsp_tau_decay[3]           = 250;
    ts.agc_wdsp_tau_decay[4]           = 50;
    ts.agc_wdsp_tau_decay[5]           = 500;

    ts.agc_wdsp_tau_hang_decay         = 200;
    ts.dbm_constant                    = 0;

    ts.FDV_TX_encode_ready             = false;		// FREEDV handshaking test DL2FW
    ts.FDV_TX_samples_ready            = 0;	// FREEDV handshaking test DL2FW
    ts.FDV_TX_out_start_pt             = 0;
    ts.FDV_TX_in_start_pt              = 0;
	ts.new_nb                          = false;	// new nb OFF at poweron
	ts.nr_alpha                        = 0.94; // spectral noise reduction
	ts.nr_alpha_int                    = 940;
	ts.nr_beta                         = 0.96;
	ts.nr_beta_int                     = 960;
//	ts.nr_vad_thresh                   = 4.0;
//	ts.nr_vad_thresh_int               = 4000;
	ts.nr_enable                       = false;
	ts.NR_FFT_L                        = 256;
	ts.NR_FFT_LOOP_NO                  = 1;
//	ts.nr_gain_smooth_enable           = false;
//	ts.nr_gain_smooth_alpha            = 0.25;
//	ts.nr_gain_smooth_alpha_int        = 250;
//	ts.nr_long_tone_enable             = false;
//	ts.nr_long_tone_alpha_int          = 99900;
//	ts.nr_long_tone_alpha              = 0.999;
//	ts.nr_long_tone_thresh             = 10000;
//	ts.nr_long_tone_reset              = true;
	ts.nr_first_time                   = 1;
//	ts.nr_vad_delay                    = 7;
	ts.NR_decimation_enable            = true;
	ts.nr_fft_256_enable               = true;
	NR2.width                          = 4;
	NR2.power_threshold                = 0.40;
	NR2.power_threshold_int            = 20;
	NR2.asnr                           = 18;


    ts.i2c_speed[I2C_BUS_1]            = I2C1_SPEED_DEFAULT; // Si570, MCP9801
    ts.i2c_speed[I2C_BUS_2]            = I2C2_SPEED_DEFAULT; // Codec, EEPROM

    ts.rtty_atc_enable                 = true;
    ts.keyer_mode.active               = false;
    ts.keyer_mode.button_recording     = KEYER_BUTTON_NONE;
    for (int idx = 0; idx<KEYER_BUTTONS; idx++)
    {

    	ts.keyer_mode.macro[idx][0] = '\0';
    	strcpy((char *) ts.keyer_mode.cap[idx], "BTN");

    }
    ts.buffered_tx                = false;
    ts.cw_text_entry              = false;
    ts.debug_si5351a_pllreset     = 3;   // "Never"  //  2  start with "reset on IQ Divider"
    ts.switch_pause               = 1050;// pour ne pas corriger IQ coeff lors de switch Tx-->Rx
    ts.RFG_wait                   = 0;   // pour ne pas reafficher "RFG" au lancement
    ts.AFG_wait                   = 0;   // pour ne pas reafficher "AFG" au lancement
    ts.current_pwr_adj            = 0;   // Power "PWR" setting � afficher en haut _milieu de l'ecran during Tx
    ts.dynamic_tune_activ_counter = 0;   // tempo pour changer en jaune couleur freq durant Dynamic Tune actif
    ts.old_rfg_calc               = 100; // pour forcer l'ecriture du gain codec  NIZZZ
    ts.rx_iq_auto_Ampl_correc     = 0;   // pour affichage des corrections d'amplitude NIZZZ
    ts.rx_iq_auto_Phas_correc     = 0;   // pour affichage des corrections de phase
    ts.agc_wdsp_Attackk           = 45;  // coeff attack du Rx AGC WDSP
    ts.tx_bass_filter_freq        = 81;  // 80*5 = 400 Hz frequence de la basse de l'equaliseur Tx
    ts.tx_bass_filter_bwidth      = 99;  // coeff de la bande de frequence Bass de l 'qualiseur Tx
    ts.tx_treb_filter_freq        = 49;  // 900Hz Passe haut Tx filter
    ts.tx_treb_filter_bwidth      = 90;
    ts.Sm_peak_moyen              =  0;  // pour calculer le smeter moyen � partir du signal
    ts.tx_alc_agc_min             = 1;   // pour afficher sur ecran le taux du compresseur audio en cours de Tx en temps reel .
    ts.codec_mic_boost_enable     = 0;   // pour activer le gain 20db hardware du Codec ou NOn
    ts.pwr_out_of_band_factor     = 3;   // 40; // 3; // nizou
    ts.nb_setting_float           = 10.5;
    ts.rx_hp_filter               = 2;   // pour choisir le filtre audio HP 48 Hz
    ts.tx_decimatedIQ             = false; // reperer si processus Tx de d�cimation 48Khz-->12Khz
    ts.last_volume                = 0;
    ts.spec_decal                 = 0;   // le decallage d'abaissement progressif du graphic du spectre en cas de saturation haute <= 7
    ts.rx_bass_filter_freq        = 143; // 143 +150 = 299 Hz frequence de la basse de l'equaliseur Rx
    ts.rx_bass_filter_bwidth      = 98;  // coeff de la bande de frequence Bass de l 'qualiseur Rx
    ts.rx_trebl_filter_freq       = 254; // 254*5 + 400 Hz frequence de la treble de l'equaliseur Rx
    ts.rx_trebl_filter_bwidth     = 98;  // coeff de la bande de frequence treble de l 'qualiseur Rx
    ts.paddles_active             = false; // pour ne pas prendre en consid�ration les GPIO PTT/Paddle avant lancement du Rig
    ts.audio_processor_input_mute_counter  = 0;  // pour passer � la saisie Rx directement
    ts.audio_dac_muting_buffer_count = 00;  // pour suspendre les taches audio lourdes au d�marrage y compris le DSP
    ts.tx_bass_sheeft             =  0;
    ts.Noise_Gate_Thresh          = 500;
 // ts.cmpt_volume                = 0;
    ts.reverberation_rate         = 10;  // 16 %
    ts.reverberation_delay        = 48;  // 64 ms = 96 x 0.666666 ms
    ts.reverberation_enable       = 1;   // Rever ON
    ts.enable_display_on_screen   = true;
    ts.FFT_sampling_enable        = true; // pour echantillonnage bloc 512 pt pour FFT et affichage spectre
    ts.FFT_stand_by               = 0;
    ts.tempo_avg_FFT              = 0;
    ts.tx_medium_filter_freq      = 46;  // 730Hz Passe haut Tx filter
    ts.tx_medium_filter_gain      = +14;
    ts.tx_medium_filter_bwidth    = 98;
    ts.tx_hpf_filter              = 2;
    ts.antenna_attenuator_enable  = 0;
}

void MiscInit(void)
{
    // Init Soft DDS
    float freq[2] = { 0.0, 0.0 };
    softdds_configRunIQ ( freq, ts.samp_rate, 0 );
}


const static uint8_t canary_word[16] = { 'D', 'O',' ' ,'N', 'O', 'T', ' ', 'O', 'V', 'E', 'R' , 'W', 'R' , 'I', 'T', 'E' };
uint8_t*  canary_word_ptr;

// in hex 44 4f 20 4e 4f 54 20 4f 56 45 52 57 52 49 54 45
// this has to be called after all dynamic memory allocation has happened
void Canary_Create()
{
    canary_word_ptr = (uint8_t*)malloc(sizeof(canary_word));
    memcpy( canary_word_ptr, canary_word, 16 );
}
// in hex 44 4f 20 4e 4f 54 20 4f 56 45 52 57 52 49 54 45
// this has to be called after all dynamic memory allocation has happened
bool Canary_IsIntact()
{
    return memcmp( canary_word_ptr, canary_word, 16 ) == 0;
}

uint8_t* Canary_GetAddr()
{
    return canary_word_ptr;
}


// #include "Trace.h"
#if 0
void timeTest1()
{
    static uint32_t time = 0;
    if (time != RTC->TR)
    {
        Board_RedLed(LED_STATE_TOGGLE);
        time = RTC->TR;
    }
}
void timeTest()
{
    MchfRtc_Start();
    while (1)
    {
        timeTest1();
    }
}
#endif
// Power on
int mchfMain(void)      /// NIZZZ  tache principale: initiatialisation du systeme et lanchement de la boucle infini des traitements
{

    ///trace_puts("Hello mcHF World!");
    // trace_printf(" %u\n", 1u);


    *(__IO uint32_t*)(SRAM2_BASE) = 0x0;	// clearing delay prevent for bootloader

    // Set default transceiver state
    TransceiverStateInit();  // initialisation des variables globaux

     Board_RamSizeDetection();

#ifdef TESTCPLUSPLUS
    test_call_cpp();
#endif






   // HW init
	  Board_InitMinimal();



    // Show logo & HW Info
    UiDriver_StartUpScreenInit();

    if (ts.display != DISPLAY_NONE)
    {
        // TODO: Better indication of non-detected display
        Board_GreenLed(LED_STATE_ON);
    }

	if(Si570_IsPresent()) 	{  ts.si570_is_present = true;	}
	else   					{  ts.si570_is_present = false;	}


    Board_InitFull();       // KeypadInit  // 3 Encoders init  // Init DACs // Enable all ADCs 1,2,3
    ConfigStorage_Init();

									// init mchf_touchscreen to see if it is present
									// we don't care about the screen being reverse or not
									// here, so we simply set reverse to false
    UiLcdHy28_TouchscreenInit(0);


    MiscInit();   // Init Soft DDS

    // Usb Host driver init
    //keyb_driver_init();

    // UI HW init
    UiDriver_Init();




       /*	GPIO_SetBits   (BAND0_PIO, BAND0); GPIO_SetBits  (BAND1_PIO, BAND1);   GPIO_ResetBits(BAND2_PIO, BAND2); HAL_Delay(100);  GPIO_SetBits(BAND2_PIO, BAND2);     /// NIZZZ
       	GPIO_ResetBits (BAND0_PIO, BAND0); GPIO_ResetBits(BAND1_PIO, BAND1);   GPIO_ResetBits(BAND2_PIO, BAND2); HAL_Delay(100);  GPIO_SetBits(BAND2_PIO, BAND2);
       	GPIO_SetBits   (BAND0_PIO, BAND0); GPIO_SetBits  (BAND1_PIO, BAND1);   GPIO_ResetBits(BAND2_PIO, BAND2); HAL_Delay(100);  GPIO_SetBits(BAND2_PIO, BAND2);
       	GPIO_SetBits   (BAND0_PIO, BAND0); GPIO_SetBits  (BAND1_PIO, BAND1);   GPIO_ResetBits(BAND2_PIO, BAND2); HAL_Delay(100);  GPIO_SetBits(BAND2_PIO, BAND2);     /// NIZZZ
       	GPIO_ResetBits (BAND0_PIO, BAND0); GPIO_ResetBits(BAND1_PIO, BAND1);   GPIO_ResetBits(BAND2_PIO, BAND2); HAL_Delay(100);  GPIO_SetBits(BAND2_PIO, BAND2);
       	GPIO_SetBits   (BAND0_PIO, BAND0); GPIO_SetBits  (BAND1_PIO, BAND1);   GPIO_ResetBits(BAND2_PIO, BAND2); HAL_Delay(100);  GPIO_SetBits(BAND2_PIO, BAND2);
       */

    if(mchf_touchscreen.present)
    {
    	//preventing DSP functions mask to have not proper value
    	ts.dsp_mode_mask |= 1;
    	ts.dsp_mode_mask &= (1<<DSP_SWITCH_MAX)-1;
    }
    else
    {
    	ts.dsp_mode_mask = ( 1 << DSP_SWITCH_MAX ) - 1;		//disable masking when no touchscreen controller detected
    }

    // we now reinit the I2C buses with the configured speed settings. Loading the EEPROM always uses the default speed!
    mchf_hw_i2c1_init();
    mchf_hw_i2c2_init();

	// disable rx iq settings in menu when autocorr is enabled
	if(ts.iq_auto_correction == 1) 	{  ts.display_rx_iq = false; }
	else 							{  ts.display_rx_iq = true;  }
    profileTimedEventInit();

    // non_os_delay();
    AudioDriver_Rx_HP_Filter_Init ();               // Rx Hight pass filter   75Hz

    AudioDriver_Init();		// Audio HW init    Lancement du Codec
    // non_os_delay();

    UiDriver_StartupScreen_LogIfProblem ( ts.codec_present == false, "Audiocodec WM8731 NOT detected!" );

    AudioManagement_CalcSubaudibleGenFreq();    // load/set current FM subaudible tone settings for generation
    AudioManagement_CalcSubaudibleDetFreq();    // load/set current FM subaudible tone settings	for detection
    AudioManagement_LoadToneBurstMode();	    // load/set tone burst frequency
    AudioManagement_LoadBeepFreq();		        // load/set beep frequency

    AudioFilter_SetDefaultMemories();

    ts.rx_gain[RX_AUDIO_SPKR].value_old = 0;	// Force update of volume control

#ifdef USE_FREEDV
    FreeDV_mcHF_init();
    // we now try to place a marker after last dynamically
    // allocated memory
    Canary_Create();
#endif

    UiDriver_StartUpScreenFinish();      //   (2000);

    UiDriver_SetDemodMode( ts.dmod_mode ); // NIZZ Nouvelle position du 07/06/2021

    ts.tx_comp_out_gain_float = ts.tx_comp_out_gain/10;  /// NIZZZ  initialisation du variable image float

    UiDriver_Display_spectrum_ZOOM();                   /// NIZZZ  affichage du magnify du spectrum

    Board_RedLed ( LED_STATE_OFF );




    // non_os_delay();  // rajout� pour �viter le blocage du son au 1er d�marrage  .




     UhsdrHwI2s_Codec_StartDMA();         // Start DMA transfers  // Nouvelle position NIZZZ au 07/06/2021

     non_os_delay();

     ts.dsp_inhibit   =  0;
     ads.af_disabled  =  0;

     //  non_os_delay();

                      	 //  group    0: 80m,    1: 40m,    2: 20m ,     3:10m
           				 //  80m , 20m , 80
           	  Board_SelectLpfBpf(0);  //ts.nr_first_time =  1;// 80m band     /// NIZZZ  pour tictacter les relais au demarrage de la radio pour un meilleur contact
           	  non_os_delay();  non_os_delay();  non_os_delay(); // non_os_delay();// non_os_delay();  non_os_delay(); //non_os_delay();  non_os_delay();
           	  Board_SelectLpfBpf(2);       //ts.nr_first_time =  1;// 20m band
           	  non_os_delay();  non_os_delay();  non_os_delay(); // non_os_delay();  // non_os_delay();  non_os_delay();
           	  //  RadioManagement_SetHWFiltersForFrequency( df.tune_new );

           	  if      ((df.tune_new) <  4500000) { Board_SelectLpfBpf(0); } // aiguillage au bon filtre de bande initial
           	  else if ((df.tune_new) <  8500000) { Board_SelectLpfBpf(1); }
           	  else if ((df.tune_new) < 15500000) { Board_SelectLpfBpf(2); }
           	  else                               { Board_SelectLpfBpf(3); }

           	  non_os_delay(); non_os_delay();

           	  //  osc->prepareNextFrequency(ts.tune_freq, df.temp_factor);
    // TODO: We need to set the digital mode once to make it active
    // if we just loaded the mode from EEPROM since we do not active ts.dvmode

   // if (ts.dmod_mode == DEMOD_DIGI)  { 	UiDriver_SetDemodMode(ts.dmod_mode); }  // ancienne position avant 07/06/2021
      ts.paddles_active = true;



	// Transceiver main loop
    for(;;)
    {
        // UI events processing
        UiDriver_TaskHandler_MainTasks();
    }
    return 0;
}
