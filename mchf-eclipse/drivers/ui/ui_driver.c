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
#include "profiling.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <src/uhsdr_version.h>
#include "hardware/uhsdr_board_config.h"
#include "ui_lcd_layouts.h"
#include "ui_menu.h"
#include "uhsdr_rtc.h"
#include "adc.h"
#include "drivers/ui/oscillator/osc_si5351a.h"
#include "audio_nr.h"
#include "uhsdr_keypad.h"
#include "serial_eeprom.h"
#include "ui.h"
// LCD
#include "ui_lcd_hy28.h"
#include "ui_spectrum.h"

#include "freedv_uhsdr.h"

// Encoders
#include "ui_rotary.h"

#include "cat_driver.h"

// Codec control
#include "codec.h"

#include "audio_management.h"
#include "ui_driver.h"

#include "ui_configuration.h"
#include "config_storage.h"

#include "cw_gen.h"

#include "radio_management.h"
#include "soft_tcxo.h"

#include "rtty.h"
#include "cw_decoder.h"
#include "psk.h"

#include "audio_convolution.h"

#define SPLIT_ACTIVE_COLOUR         		Yellow      // colour of "SPLIT" indicator when active
#define SPLIT_INACTIVE_COLOUR           	Grey        // colour of "SPLIT" indicator when NOT active
#define COL_PWR_IND                 		White



static void     UiDriver_CreateMainFreqDisplay();

static void     UiDriver_CreateMeters();
static void     UiDriver_DeleteMeters();
static void 	UiDriver_DrawSMeter(ushort color);
//
static void 	UiDriver_UpdateTopMeterA(uchar val);
static void 	UiDriver_UpdateBtmMeter(float val, uchar warn);

static void 	UiDriver_InitFrequency();
//

static void     UiDriver_UpdateLcdFreq(ulong dial_freq,ushort color,ushort mode);
static bool 	UiDriver_IsButtonPressed(ulong button_num);
static void		UiDriver_TimeScheduler();				// Also handles audio gain and switching of audio on return from TX back to RX
static void 	UiDriver_ChangeToNextDemodMode(bool select_alternative_mode);
static void 	UiDriver_ChangeBand(uchar is_up);
static bool 	UiDriver_CheckFrequencyEncoder();

static void     UiDriver_DisplayBand(uchar band);
static uchar    UiDriver_DisplayBandForFreq(ulong freq);

static void     UiDriver_DisplayEncoderOneMode();
static void     UiDriver_DisplayEncoderTwoMode();
static void     UiDriver_DisplayEncoderThreeMode();

static void 	UiDriver_DisplayNoiseBlanker(bool encoder_active);
static void 	UiDriver_DisplayDSPMode(bool encoder_active);
static void 	UiDriver_DisplayTone(bool encoder_active);
static void 	UiDriver_DisplayRit(bool encoder_active);
static void     UiDriver_DisplayPowerAdjust(bool encoder_active);
static void     UiDriver_DisplayAfGain(bool encoder_active);
static void     UiDriver_DisplayRfGain(bool encoder_active);
static void     UiDriver_DisplaySidetoneGain(bool encoder_active);
static void     UiDriver_DisplayCmpLevel(bool encoder_active);
static void     UiDriver_DisplayKeyerSpeed(bool encoder_active);
static void     UiDriver_DisplayLineInModeAndGain(bool encoder_active);

// static void     UiDriver_DisplayMemoryLabel();


// static void 	UiDriver_DisplayModulationType();
static void 	UiDriver_DisplayPowerLevel();
static void     UiDriver_DisplayTemperature(int temp);
static void     UiDriver_DisplayVoltage();

static void 	UiDriver_HandleSMeter () ;  // (bool  IQCodecGain_Execute);
static void 	UiDriver_HandleTXMeters();
static bool     UiDriver_HandleVoltage();

#if 0
static void 	UiDriverUpdateLoMeter(uchar val,uchar active);
#endif
static void     UiDriver_CreateVoltageDisplay();

static void 	UiDriver_HandleLoTemperature();


static bool	    UiDriver_LoadSavedConfigurationAtStartup();
static bool	    UiDriver_TouchscreenCalibration();

static void     UiDriver_PowerDownCleanup(bool saveConfiguration);

static void UiDriver_HandlePowerLevelChange(uint8_t power_level);
static void UiDriver_HandleBandButtons(uint16_t button);

static void UiDriver_KeyTestScreen();

static bool UiDriver_SaveConfiguration();

static void UiDriver_DisplayRttySpeed(bool encoder_active);
static void UiDriver_DisplayRttyShift(bool encoder_active);
static void UiDriver_DisplayPskSpeed(bool encoder_active);



// Tuning steps
const ulong tune_steps[T_STEP_MAX_STEPS] =
{
		T_STEP_1HZ,
		T_STEP_5HZ,
		T_STEP_10HZ,
		T_STEP_100HZ,
		T_STEP_500HZ,
		T_STEP_1KHZ,
		T_STEP_5KHZ,
		T_STEP_9KHZ,
		T_STEP_10KHZ,
		T_STEP_100KHZ,
		T_STEP_1MHZ,
		T_STEP_10MHZ
};


// The following are calibrations for the S-meter based on 6 dB per S-unit, 10 dB per 10 dB mark above S-9
// The numbers within are linear gain values, not logarithmic, starting with a zero signal level of 1
// There are 33 entries, one corresponding with each point on the S-meter
#define	S_Meter_Cal_Size	33	// number of entries in table below
const float S_Meter_Cal[S_Meter_Cal_Size] =
{
		14.1,	    //1.41,	        //1, S0.5, 3dB
		20,		    //2,		    //2, S1, 6dB
		28.1,	    //2.81,	        //3, S1.5, 9dB
		30,		    //3,		    //4, S2, 12dB
		56.2,	    //5.62,	        //5, S2.5, 15dB
		79.4,	    //7.94,	        //6, S3, 18dB
		112.2,	    //11.22,	    //7, S3.5, 21dB
		158.5,	    //15.85,	    //8, S4, 24dB
		223.9,	    //22.39,	    //9, S4.5, 27dB
		316.3,	    //31.63,	    //10, S5, 30dB
		446.7,	    //44.67,	    //11, S5.5, 33dB
		631,	    //63.10,	    //12, S6, 36dB
		891.3,	    //89.13,	    //13, S6.5, 39dB
		1258.9,	    //125.89,	    //14, S7, 42dB
		1778.3,	    //177.83,	    //15, S7.5, 45dB
		2511.9,	    //251.19,	    //16, S8, 48dB
		3548.1,	    //354.81,	    //17, S8.5, 51dB
		5011.9,	    //501.19,	    //18, S9, 54dB
		8912.5,	    //891.25,	    //19, +5, 59dB
		15848.9,	//1584.89,	    //20, +10, 64dB
		28183.8,	//2818.38,	    //21, +15, 69dB
		50118.7,	//5011.87,	    //22, +20, 74dB
		89125.1,	//8912.51,	    //23, +25, 79dB
		158489.3,	//15848.93,	    //24, +30, 84dB
		281838.2,	//28183.82,	    //25, +35, 89dB
		501187.2,	//50118.72,	    //26, +40, 94dB
		891250.9,	//89125.09,	    //27, +45, 99dB
		1585893.2,	//158489.32,	//28, +50, 104dB
		2818382.9,	//281838.29,	//29, +55, 109dB
		5011872.3,	//501187.23,	//30, +60, 114dB
		8912509.4,	//891250.94,	//31, +65, 119dB
		15848931.9,	//1584893.19,	//32, +70, 124dB
		28183829.3,	//2818382.93	//33, +75, 129dB
};

// if S-meter is based on FFT / dBm calculation
const float S_Meter_Cal_dbm[S_Meter_Cal_Size] =
{
		// dBm vs. S-value
		-124.0,	// S0.5
		-121.0,	// S1
		-118.0,	// S1.5
		-115.0,	// S2
		-112.0,	// S2.5
		-109.0,	// S3
		-106.0,	// S3.5
		-103.0,	// S4
		-100.0,	// S4.5
		-97.0,	// S5
		-94.0,	// S5.5
		-91.0,	// S6
		-88.0,	// S6.5
		-85.0,	// S7
		-82.0,	// S7.5
		-79.0,	// S8
		-76.0,	// S8.5
		-73.0,	// S9
		-68.0,	// S9+5
		-63.0,	// +10
		-58.0,	// +15
		-53.0,	// +20
		-48.0,	// +25
		-43.0,	// +30
		-38.0,	// +35
		-33.0,	// +40
		-28.0,	// +45
		-23.0,	// +50
		-18.0,	// +55
		-13.0,	// +60
		-8.0,	// +65
		-3.0,	// +70
		2.0,    // +75
};

BandRegs vfo[VFO_MAX];

// ------------------------------------------------
// Keypad state
__IO KeypadState				ks;

// ------------------------------------------------
// Power supply meter
PowerMeter					pwmt;


// ------------------------------------------------

uchar drv_state = 0;
ui_driver_mode_t    ui_driver_state;
bool filter_path_change = false;

// check if touched point is within rectangle of valid action
bool UiDriver_CheckTouchRegion(const UiArea_t* tr_p)
{
	return ((    ts.tp->hr_x <= (tr_p->x+tr_p->w)) &&
				(ts.tp->hr_x >= (tr_p->x)) &&
				(ts.tp->hr_y <= (tr_p->y+tr_p->h))) &&
				(ts.tp->hr_y >= (tr_p->y));

}

///*********************************************************************************
///
inline int32_t change_and_limit_int(volatile int32_t val, int32_t change, int32_t min, int32_t max)
{
	val +=change;
	if (val< min)
	{
		val = min;
	}
	else if (val>  max)
	{
		val = max;
	}
	return val;
}


inline uint32_t change_and_limit_uint(volatile uint32_t val, int32_t change, uint32_t min, uint32_t max)
{
	if (change < 0 && ( -change  > (val - min)))
	{
		val = min;
	}
	else if (change > 0 && change >  max - val)
	{
		val = max;
	}
	else
	{
		val +=change;
	}
	return val;
}

inline uint32_t change_and_wrap_uint(volatile uint32_t val, int32_t change, uint32_t min, uint32_t max)
{
	if (change  > ((int32_t)max - (int32_t)val))
	{
		val = min;
	}
	else if ((change + (int32_t)val) <  (int32_t)min)
	{
		val = max;
	}
	else
	{
		val +=change;
	}
	return val;
}

inline void incr_wrap_uint8(volatile uint8_t* ptr, uint8_t min, uint8_t max )
{
	*ptr = (change_and_wrap_uint(*ptr,+1,min,max))&0xff;
}
inline void incr_wrap_uint16(volatile uint16_t* ptr, uint16_t min, uint16_t max )
{
	*ptr = (change_and_wrap_uint(*ptr,+1,min,max))&0xff;
}
inline void decr_wrap_uint8(volatile uint8_t* ptr, uint8_t min, uint8_t max )
{
	*ptr = (change_and_wrap_uint(*ptr,-1,min,max))&0xff;
}
inline void decr_wrap_uint16(volatile uint16_t* ptr, uint16_t min, uint16_t max )
{
	*ptr = (change_and_wrap_uint(*ptr,-1,min,max))&0xff;
}

inline bool is_touchscreen_pressed()
{
	return (ts.tp->state == TP_DATASETS_VALID);	// touchscreen data available
}

bool is_vfo_b()
{
	return (ts.vfo_mem_mode & VFO_MEM_MODE_VFO_B) != 0;
}

inline bool is_dsp_nb()
{
	return (ts.dsp_active & DSP_NB_ENABLE) != 0;
//	return (ts.nb_setting > 0); // noise blanker ON
}

inline bool is_dsp_nr()
{
	return (ts.dsp_active & DSP_NR_ENABLE) != 0;
}

inline bool is_dsp_nr_postagc()
{
	return (ts.dsp_active & DSP_NR_POSTAGC_ENABLE) != 0;
}

inline bool is_dsp_notch()
{
	return (ts.dsp_active & DSP_NOTCH_ENABLE) != 0;
}

inline bool is_dsp_mnotch()
{
	return (ts.dsp_active & DSP_MNOTCH_ENABLE) != 0;
}

inline bool is_dsp_mpeak()
{
	return (ts.dsp_active & DSP_MPEAK_ENABLE) != 0;
}

#define KEYACTION_NOP    (NULL)				// This action for the pressed key is treated as being executed, but it is a no-operation
#define KEYACTION_PASS ((void(*)())-1)		// This action for the pressed key is treated as not present, i.e. we do not report the key event has been processed

typedef struct
{
	uint32_t key_code;
	// use KEYACTION_NONE and KEYACTION_PASS to handled nop and pass for further processing,
	// see comments for these constants
	void (*press_func)(); // executed if short press of key is detected
	void (*hold_func)();  // executed if press and hold of key is detected
} keyaction_descr_t;

typedef struct
{
	const keyaction_descr_t* actions;
	int32_t size;
} keyaction_list_descr_t;


/*
 * @brief find the matching region in a list of region and associated function
 * @returns: true, if a match for the touch coordinates region was found.
 */
bool UiDriver_ProcessTouchActions(const touchaction_list_descr_t* tld, bool is_long_press)
{
	bool retval = false;
	if (tld != NULL)
	{
		for (uint32_t idx = 0; idx < tld->size; idx++)
		{
			if (UiDriver_CheckTouchRegion(&tld->actions[idx].region))
			{
			    if (is_long_press)
			    {
			        if (tld->actions[idx].function_long_press != NULL)
			        {
			            (*tld->actions[idx].function_long_press)();
			        }
			    }
			    else
			    {
			        if (tld->actions[idx].function_short_press != NULL)
			        {
			            (*tld->actions[idx].function_short_press)();
			        }
			    }
			    retval = true;
			    break;
			}
		}
	}
	return retval;
}

/*
 * @brief find the matching keycode in a list of keycodes and associated functions
 * @returns: true, if a match for the keycode was found and a function (which can be a "no operation") has been executed.
 */

bool UiDriver_ProcessKeyActions(const keyaction_list_descr_t* kld)
{
	bool retval = false;
	if (kld != NULL)
	{
		for (uint32_t idx = 0; idx < kld->size; idx++)
		{
			if (kld->actions[idx].key_code == ks.button_id)
			{
				void (*func_ptr)() =  ks.press_holded == true ? kld->actions[idx].hold_func:kld->actions[idx].press_func;
				if (func_ptr != KEYACTION_NOP && func_ptr != KEYACTION_PASS)
				{
					(*func_ptr)();
				}
				retval = func_ptr != KEYACTION_PASS;
				break;
			}
		}
	}

	return retval;
}
/**
 * @brief restarts lcd blanking timer, called in all functions which detect user interaction with the device
 */
void UiDriver_LcdBlankingStartTimer()
{
	if(ts.lcd_backlight_blanking & LCD_BLANKING_ENABLE)     // is LCD blanking enabled?
	{
		uint32_t  ltemp      = (ulong)(ts.lcd_backlight_blanking & LCD_BLANKING_TIMEMASK);      // get setting of LCD blanking timing
		          ltemp     *= 200;   // 100    // multiply to convert to deciseconds
		ts.lcd_blanking_time = ltemp + ts.sysclock;     // calculate future time at which LCD is to be turned off
		ts.lcd_blanking_flag = false;       // clear flag to make LCD turn on
	}
}

static void   UiDriver_LcdBlankingProcessTimer()
{
	// Process LCD auto-blanking
	if(ts.lcd_backlight_blanking & LCD_BLANKING_ENABLE)      // is LCD auto-blanking enabled?
	{
		if(ts.sysclock > ts.lcd_blanking_time)      // has the time expired and the LCD should be blanked?
		{
			ts.lcd_blanking_flag = true;             // yes - blank the LCD
		}
		else                                        // time not expired
		{
			ts.lcd_blanking_flag = false;             // un-blank the LCD
		}
	}
	else                                  // auto-blanking NOT enabled
	{
		ts.lcd_blanking_flag = false;               // always un-blank the LCD in this case
	}
}

static char ui_txt_msg_buffer[ui_txt_msg_buffer_size];

static int ui_txt_msg_idx= 0;
static bool ui_txt_msg_update = false;


void UiDriver_TextMsgClear()
{
	uint32_t fillcnt;
	for(fillcnt=0; fillcnt<ts.Layout->TextMsg_buffer_max;fillcnt++)
	{
		ui_txt_msg_buffer[fillcnt]=' ';
	}
	ui_txt_msg_buffer[fillcnt]='\0';

    UiLcdHy28_PrintText(ts.Layout->TextMsgLine.x,ts.Layout->TextMsgLine.y, ui_txt_msg_buffer,Yellow,Black,ts.Layout->TextMsg_font);
    ui_txt_msg_idx = 0;
    ui_txt_msg_update = true;
}
//*******************************************************************************************************
//************************************ Affichage texte decode en CW ou digital **************************
//*******************************************************************************************************

void UiDriver_TextMsgDisplay()
{
    if (ui_txt_msg_update == true)
    {
        ui_txt_msg_update = false;
        if(ui_txt_msg_idx==0)
        {
        	uint32_t fillcnt;
        	for(fillcnt=0; fillcnt<ts.Layout->TextMsg_buffer_max;fillcnt++)
        	{
        		ui_txt_msg_buffer[fillcnt]=' ';
        	}
        	ui_txt_msg_buffer[fillcnt]='\0';
        }

        UiLcdHy28_PrintText(ts.Layout->TextMsgLine.x,ts.Layout->TextMsgLine.y, ui_txt_msg_buffer,Yellow,Black,ts.Layout->TextMsg_font);
    }
}

//**********************************************************************************************************

void UiDriver_TextMsgPutChar(char ch)
{
    if (ch=='\n' || ch == '\r')
    {
        ui_txt_msg_idx=0;
    	ui_txt_msg_buffer[ui_txt_msg_idx] = '\0';
    }
    else if (ui_txt_msg_idx < (ts.Layout->TextMsg_buffer_max))
    {
        ui_txt_msg_idx++;
    	ui_txt_msg_buffer[ui_txt_msg_idx] = '\0'; // set the line end before we add the character prevents unterminated strings
        ui_txt_msg_buffer[ui_txt_msg_idx-1]=ch; //fill from left to right
    }
    else
    {
        for (int shift_count = 0;shift_count < (ts.Layout->TextMsg_buffer_max-1);shift_count++)
        {
            ui_txt_msg_buffer[shift_count]=ui_txt_msg_buffer[shift_count+1];
        }
        ui_txt_msg_buffer[ts.Layout->TextMsg_buffer_max-1]=ch;
    }
    ui_txt_msg_update = true;
}

void UiDriver_TextMsgPutSign(const char *s)
{
	UiDriver_TextMsgPutChar('<');
	UiDriver_TextMsgPutChar(s[0]);
	UiDriver_TextMsgPutChar(s[1]);
	UiDriver_TextMsgPutChar('>');
}

static void UiDriver_LeftBoxDisplay ( const uint8_t row,  const char *label, bool encoder_active, const char* text, uint32_t color, uint32_t clr_val, bool text_is_value)
{

	uint32_t label_color = encoder_active?Black:color;

	// max visibility of active element
	uint32_t bg_color   = encoder_active?Orange:Blue;
	uint32_t brdr_color = encoder_active?Orange:Blue;

	uint16_t posX, posY;

	if(ts.Layout->LEFTBOXES_MODE==MODE_HORIZONTAL)
	{
		posX = ts.Layout->LEFTBOXES_IND.x + (row * (ts.Layout->LEFTBOXES_IND.w + 1) );
		posY = ts.Layout->LEFTBOXES_IND.y;
	}
	else
	{
		posX=ts.Layout->LEFTBOXES_IND.x;
		posY=ts.Layout->LEFTBOXES_IND.y + (row * ts.Layout->LEFTBOXES_IND.h);
	}


	UiLcdHy28_DrawEmptyRect     ( posX, posY    , ts.Layout->LEFTBOXES_IND.h - 2, ts.Layout->LEFTBOXES_IND.w - 4 + row*2, brdr_color);
	UiLcdHy28_PrintTextCentered ( posX, posY + 1, ts.Layout->LEFTBOXES_IND.w - 3 + row*2, label                         , label_color, bg_color, 0);

	// this causes flicker, but I am too lazy to fix that now
	UiLcdHy28_DrawFullRect ( posX + 1, posY + 1 + 12, ts.Layout->LEFTBOXES_IND.h - 4 - 11, ts.Layout->LEFTBOXES_IND.w - 5 +row*2, text_is_value?Black:bg_color);
	if (text_is_value)
	{
		UiLcdHy28_PrintTextRight ( (posX + ts.Layout->LEFTBOXES_IND.w - 5+row*2), (posY   + ts.Layout->LEFTBOXES_ROW_2ND_OFF), text,
				clr_val, text_is_value?Black:bg_color, 0);
	}
	else
	{
		UiLcdHy28_PrintTextCentered ( (posX + 1), (posY + 1 + ts.Layout->LEFTBOXES_ROW_2ND_OFF),ts.Layout->LEFTBOXES_IND.w - 5+row*2, text, color, bg_color, 0);
	}

}

static void UiDriver_LcdBlankingStealthSwitch()
{
	if(ts.lcd_backlight_blanking & LCD_BLANKING_ENABLE)
	{         // Yes - is MSB set, indicating "stealth" (backlight timed-off) mode?
		ts.lcd_backlight_blanking &= ~LCD_BLANKING_ENABLE;
	} // yes - clear that bit, turning off "stealth" mode
	else
	{
		if(ts.lcd_backlight_blanking & LCD_BLANKING_TIMEMASK)    // bit NOT set AND the timing set to NON-zero?
		{
			ts.lcd_backlight_blanking |= LCD_BLANKING_ENABLE;       // no - turn on MSB to activate "stealth" mode
		}
	}
}

void UiDriver_DisplayFilter()
{
	const char* filter_ptr;
	uint32_t font_clr= filter_path_change?Black:White;

	const char *filter_names[2];

	AudioFilter_GetNamesOfFilterPath(ts.filter_path,filter_names);
	if (filter_names[1] != NULL)
	{
		filter_ptr = filter_names[1];

	/*	if (ts.dmod_mode  == DEMOD_CW )
		{
			int32_t  gh = 0;
			if      (filter_ptr=="500Hz") { gh = 500; }
			else if (filter_ptr=="600Hz") { gh = 600; }
			else if (filter_ptr=="650Hz") { gh = 650; }
			else if (filter_ptr=="700Hz") { gh = 700; }
			else if (filter_ptr=="750Hz") { gh = 750; }
			else if (filter_ptr=="800Hz") { gh = 800; }
			else if (filter_ptr=="850Hz") { gh = 850; }
			else if (filter_ptr=="900Hz") { gh = 900; }
			else if (filter_ptr=="950Hz") { gh = 950; }
			if (gh !=0) ts.cw_sidetone_freq = gh;
		} */


		if (ts.dmod_mode  == DEMOD_CW )
		{
			int32_t  gh = 0;
			if      (!strcmp  (filter_ptr,"500Hz")) { gh = 500; }
			else if (!strcmp  (filter_ptr,"600Hz")) { gh = 600; }
			else if (!strcmp  (filter_ptr,"650Hz")) { gh = 650; }
			else if (!strcmp  (filter_ptr,"700Hz")) { gh = 700; }
			else if (!strcmp  (filter_ptr,"750Hz")) { gh = 750; }
			else if (!strcmp  (filter_ptr,"800Hz")) { gh = 800; }
			else if (!strcmp  (filter_ptr,"850Hz")) { gh = 850; }
			else if (!strcmp  (filter_ptr,"900Hz")) { gh = 900; }
			else if (!strcmp  (filter_ptr,"950Hz")) { gh = 950; }
			if (gh !=0) ts.cw_sidetone_freq = gh;
		}

	}
	else
	{
		filter_ptr = " ";
	}

	UiDriver_LeftBoxDisplay ( 0, filter_names[0], filter_path_change, filter_ptr, font_clr, font_clr, false);
}

// TODO: most of this belongs to radio management, not UI
static void UiDriver_ToggleDigitalMode()
{
	if (ts.digital_mode != DigitalMode_None)
	{
		// a valid digital mode is set but may not be active yet
		if (ts.dmod_mode != DEMOD_DIGI)
		{
			if (RadioManagement_IsApplicableDemodMode(DEMOD_DIGI))
			{
				// this will switch to the corresponding sideband if we come from
				// SSB, otherwise the automatically selected default (AUTO LSB/USB ON) or the previously used
				// will be the selected one.
				if (is_ssb(ts.dmod_mode))
				{
					ts.digi_lsb = RadioManagement_LSBActive(ts.dmod_mode);
				}
				 RadioManagement_SetDemodMode(DEMOD_DIGI);
			}
		}
	}
	else
	{
		if (ts.dmod_mode == DEMOD_DIGI)
		{
			// we are in digital mode but the current digital mode is in fact
			// None, i.e. we are going analog now
			RadioManagement_SetDemodMode(ts.digi_lsb?DEMOD_LSB:DEMOD_USB);
		}
	}
	UiDriver_UpdateDisplayAfterParamChange();
}

//************************************************************************************************
/*
 * @brief Function will update LO and Display Digits, it will never change LO if not necessary
 *
 * @param full_update set to true in order to have the full display digits being updated
 *
 */
void UiDriver_FrequencyUpdateLOandDisplay(bool full_update)
{
	if (full_update)
	{
		ts.refresh_freq_disp = 1;           // update ALL digits
	}
	if(is_splitmode())
	{
		// SPLIT mode
		UiDriver_UpdateFrequency(false,UFM_SMALL_TX);
		UiDriver_UpdateFrequency(false,UFM_SMALL_RX);
	}
	else
	{
		UiDriver_UpdateFrequency(false,UFM_AUTOMATIC);
	}
	ts.refresh_freq_disp = 0;           // update ALL digits
}

//*****************************************************************

void UiDriver_DebugInfo_DisplayEnable(bool enable)
{

	//  UiLcdHy28_PrintText(ts.Layout->DEBUG_X,ts.Layout->LOADANDDEBUG_Y,enable?"enabled":"       ",Green,Black,0);

	if (enable == false)
	{
		UiLcdHy28_PrintText(ts.Layout->LOAD_X,ts.Layout->LOADANDDEBUG_Y,"     ",White,Black,0);
		UiLcdHy28_PrintText(10, 95,"      ",White,Black,0); // NIZZZ   pour effacer l affichage des corrections IQ
		UiLcdHy28_PrintText(10,108,"      ",White,Black,0);
	}

	ts.show_debug_info = enable;

}

void UiDriver_SpectrumChangeLayoutParameters()
{
	if (ts.switch_pause < 350) ts.switch_pause += 400; // pour suspendre momentan�ment le calcul non prioritaire de reajustement des correction IQ, besoin du processeur
	UiSpectrum_WaterfallClearData();
	AudioDriver_SetRxAudioProcessing(ts.dmod_mode, false);


	if (ts.menu_mode == false)
	{
		UiSpectrum_Init();      // init spectrum scope
	}
}

void UiDriver_Display_spectrum_ZOOM()   /// NIZZZ  affichage de la valeur du ZOOM
{
	char outxt[12];
	snprintf( outxt ,12," x%u", (1 << sd.magnify)  );
	// UiLcdHy28_PrintTextRight( ts.Layout->BAND_MODE.x + 44, ts.Layout->BAND_MODE.y  + 10, outxt, Blue, Black, 0); /// NIZZ  -8 pour d�caler plus haut l'affichage de la bande
	UiLcdHy28_PrintTextRight( 320, 115, outxt, Blue2, Black, 0); // NIZ
}


void UiDriver_HandlePowerLevelChange(uint8_t power_level)
{

	if (RadioManagement_PowerLevelChange(ts.band,power_level))
	{
		UiDriver_DisplayPowerLevel();
		if (ts.menu_mode)
		{
			UiMenu_RenderMenu(MENU_RENDER_ONLY);
		}
	}
}

void UiDriver_HandleBandButtons(uint16_t button)
{

	static const int BAND_DOWN = 0;
	static const int BAND_UP = 1;

	bool buttondirSwap = (ts.flags1 & FLAGS1_SWAP_BAND_BTN)?true:false;
	uint8_t dir;


	if (button == BUTTON_BNDM)
	{
		dir = buttondirSwap ? BAND_UP : BAND_DOWN;
	}
	else
	{
		dir = buttondirSwap ? BAND_DOWN : BAND_UP;
	}

	UiDriver_ChangeBand(dir);
}


static void UiDriver_PublicsInit()
{
	// Button state structure init state
	ks.button_id			       = BUTTON_NONE;
	ks.button_pressed              = 0;
//	ks.button_released		       = 0;
	ks.button_execution_Wait       = 0;
	ks.debounce_time		       = 0;


	// Auto button blink state
	//abst.blink_flag 		= 0;
	//abst.blink_skip 		= 0;

	// SWR meter init
	swrm.p_curr				= 0;
	swrm.fwd_calc			= 0;
	swrm.rev_calc			= 0;
	swrm.fwd_pwr			= 0;
	swrm.rev_pwr			= 0;
	swrm.fwd_dbm			= 0;
	swrm.rev_dbm			= 0;
	swrm.vswr			 	= 0;
	swrm.sensor_null		= SENSOR_NULL_DEFAULT;
	{
		int idx;
		for (idx = 0; idx < COUPLING_MAX; idx++)
		{
			swrm.coupling_calc[idx]    = SWR_COUPLING_DEFAULT;
		}
	}
	swrm.pwr_meter_disp		= 0;	// Display of numerical FWD/REV power metering off by default
	swrm.pwr_meter_was_disp = 0;	// Used to indicate if FWD/REV numerical power metering WAS displayed

	// Power supply meter
	pwmt.p_curr				= 0;
	pwmt.pwr_aver 			= 0;
	pwmt.undervoltage_detected = false;

}



void UiDriver_Init()
{
	// Driver publics init
	UiDriver_PublicsInit();
	// Init frequency publics
	UiDriver_InitFrequency();

	Keypad_Scan();

	// Load stored data from eeprom or calibrate touchscreen
	bool run_keytest = (UiDriver_LoadSavedConfigurationAtStartup() == false && UiDriver_TouchscreenCalibration() == false);


	if(mchf_touchscreen.present)
	{
		//Touchscreen calibration test.
		//We cannot distinguish when touchscreen is uncalibrated with other method than comparing calibration values to empty state of EEPROM (0xff).
	    //It would be nice if someone has better idea how to do it without digging into calibration matrix computation to describe the allowed range of coefficients. Feb 2018, SP9BSL.
		bool  IS_TSCalibrated = 0;
		for( int16_t m=0;  m<6; m++ )
		{
			IS_TSCalibrated |= ts.tp->cal[m]!=0xffffffff;
		}

		UiDriver_StartupScreen_LogIfProblem( IS_TSCalibrated == 0, 	"WARNING:  TOUCHSCREEN NOT CALIBRATED!!!\nRun calibration first!");
	}

// 	UiDriver_StartupScreen_LogIfProblem( AudioDriver_GetTranslateFreq() == 0, "WARNING:  Freq. Translation is OFF!!!\nTranslation is STRONGLY recommended!!");

	// now run all inits which need to be done BEFORE going into test screen mode
	uint8_t mirtemp;
	if(ts.flags1 & FLAGS1_REVERSE_X_TOUCHSCREEN)
	{
		mirtemp = 1;
	}
	else
	{
		mirtemp = 0;
	}
	if(ts.flags1 & FLAGS1_REVERSE_Y_TOUCHSCREEN)
	{
		mirtemp += 2;
	}

	UiLcdHy28_TouchscreenInit(mirtemp);

	if (run_keytest)
	{
		UiDriver_KeyTestScreen();
	}


	osc->setPPM((float)ts.freq_cal/10.0);

	df.tune_new = vfo[is_vfo_b()?VFO_B:VFO_A].band[ts.band].dial_value;		// init "tuning dial" frequency based on restored settings
	df.tune_old = 0;

	ts.cw_lsb = RadioManagement_CalculateCWSidebandMode();			// determine CW sideband mode from the restored frequency

	AudioManagement_CalcTxCompLevel();      // calculate current settings for TX speech compressor

	AudioFilter_Init_Rx_Hilbert_FIR(ts.dmod_mode);
	AudioFilter_Init_Tx_Hilbert_FIR();

	AudioManagement_SetSidetoneForDemodMode(ts.dmod_mode,false);

	sd.display_offset = INIT_SPEC_AGC_LEVEL;		// initialize setting for display offset/AGC

	// Reset inter driver requests flag
	ts.LcdRefreshReq	= 0;
	ts.new_band 		= ts.band;
	df.step_new 		= df.tuning_step;

	// Extra HW init
	Board_PostInit();


	UiDriver_LcdBlankingStartTimer();			// init timing for LCD blanking
	ts.lcd_blanking_time       = ts.sysclock + LCD_STARTUP_BLANKING_TIME;
	ts.low_power_shutdown_time = ts.sysclock + LOW_POWER_SHUTDOWN_DELAY_TIME;
}

#define  BOTTOM_BAR_LABEL_W       (56)
#define  POS_BOTTOM_BAR_F1_offset  2
void UiDriver_DrawFButtonLabel(uint8_t button_num, const char* label, uint32_t label_color)
{

	//UiLcdHy28_PrintTextCentered(BOTTOM_BAR_F1_X + (button_num - 1)*(POS_BOTTOM_BAR_BUTTON_W+2), POS_BOTTOM_BAR_F1_Y, BOTTOM_BAR_LABEL_W, label,
	UiLcdHy28_PrintTextCentered ( ts.Layout->BOTTOM_BAR.x+POS_BOTTOM_BAR_F1_offset + (button_num - 1)*(ts.Layout->BOTTOM_BAR.w+2), ts.Layout->BOTTOM_BAR.y  , BOTTOM_BAR_LABEL_W, label,
			label_color, Black, 0);
}

void UiDriver_EncoderDisplay(const uint8_t row, const uint8_t column, const char *label, bool encoder_active,
		const char temp[5], uint32_t color)
{

	uint32_t label_color = encoder_active?Black:Grey1;

	// max visibility of active element
	uint32_t bg_color   = encoder_active?Orange:Grey;
	uint32_t brdr_color = encoder_active?Orange:Grey;

	if(ts.Layout->ENCODER_MODE==MODE_HORIZONTAL)
	{
		UiLcdHy28_DrawEmptyRect(ts.Layout->ENCODER_IND.x + ENC_COL_W *2 * column + row *ENC_COL_W+column*Xspacing, ts.Layout->ENCODER_IND.y , ENC_ROW_H - 2, ENC_COL_W - 2, brdr_color);
		UiLcdHy28_PrintTextCentered((ts.Layout->ENCODER_IND.x + 1 + ENC_COL_W * 2 * column + row *ENC_COL_W+column*Xspacing), (ts.Layout->ENCODER_IND.y + 1),ENC_COL_W - 3, label,
				label_color, bg_color, 0);
		UiLcdHy28_PrintTextRight((ts.Layout->ENCODER_IND.x + ENC_COL_W - 4 + ENC_COL_W * 2 * column+ row *ENC_COL_W+column*Xspacing), (ts.Layout->ENCODER_IND.y + 1 + ENC_ROW_2ND_OFF), temp,
				color, Black, 0);
	}
	else
	{
		UiLcdHy28_DrawEmptyRect(ts.Layout->ENCODER_IND.x + ENC_COL_W * column, ts.Layout->ENCODER_IND.y + row * ENC_ROW_H, ENC_ROW_H - 2, ENC_COL_W - 2, brdr_color);
		UiLcdHy28_PrintTextCentered((ts.Layout->ENCODER_IND.x + 1 + ENC_COL_W * column), (ts.Layout->ENCODER_IND.y + 1 + row * ENC_ROW_H),ENC_COL_W - 3, label,
				label_color, bg_color, 0);
		UiLcdHy28_PrintTextRight((ts.Layout->ENCODER_IND.x + ENC_COL_W - 4 + ENC_COL_W * column), (ts.Layout->ENCODER_IND.y + 1 + row * ENC_ROW_H + ENC_ROW_2ND_OFF), temp,
				color, Black, 0);
	}

}


void UiDriver_DisplayFButton_F1MenuExit()
{
	char*     cap;
	uint32_t  color;
	if (ts.keyer_mode.active)
	{
		if (ts.keyer_mode.button_recording == KEYER_BUTTON_1)
		{
			cap   = "REC";
			color = Red;
		}
		else
		{
			cap   = (char*) ts.keyer_mode.cap[0];
			color = White;
		}
	}
	else
	{
		if (!ts.menu_var_changed)
		{

			if (ts.menu_mode)
			{
				cap   = "EXIT";
				color = Yellow;
			}
			else
			{
				cap   = "MENU";
				color = White;
			}
		}
		else
		{
			cap   = ts.menu_mode?"EXIT *":"MENU *";
			color = Orange;
		}
	}
	UiDriver_DrawFButtonLabel ( 1, cap, color );
}


static void UiDriver_DisplayFButton_F2SnapMeter()
{
	const  char* cap;
	uint32_t   color;
	if (ts.keyer_mode.active)
	{
		if (ts.keyer_mode.button_recording == KEYER_BUTTON_2)
		{
			cap   = "REC";
			color = Red;
		}
		else
		{
			cap   = (char *)ts.keyer_mode.cap[1];
			color = White;
		}
	}
	else
	{

		cap   = "SNAP";
		color = White;    // yes - indicate with color
//		color = White;
//		cap   = "METER";
	}
	UiDriver_DrawFButtonLabel(2,cap,color);
}



static void UiDriver_FButton_F3MemSplit()
{
	const char* cap;
	uint32_t color;

	if (ts.keyer_mode.active)
	{
		if (ts.keyer_mode.button_recording == KEYER_BUTTON_3)
		{
			cap = "REC  <";
			color = Red;
		}
		else
		{
			cap = (char *)ts.keyer_mode.cap[2];
			color = White;
		}

	}
	else
	{
		if (ts.vfo_mem_flag)            // is it in VFO mode now?
		{
			cap = "MEM  <";
			color = White;    // yes - indicate with color
		}
		else
		{
			color = is_splitmode() ?
					SPLIT_ACTIVE_COLOUR : SPLIT_INACTIVE_COLOUR;
			cap = "SPLIT <";
		}
	}
	UiDriver_DrawFButtonLabel(3, cap, color);
}


static inline void UiDriver_FButton_F4ActiveVFO()
{
	const char* cap;
	if (ts.keyer_mode.active)
	{
		cap = " "; //FIXME This will be DEL
	}
	else
	{
		cap = is_vfo_b() ? "> VFO B" : "> VFO A";
	}
	UiDriver_DrawFButtonLabel ( 4, cap, White);
}

static inline void UiDriver_FButton_F5Tune()
{
	const char* cap;
	uint32_t color;
	color = RadioManagement_IsTxDisabled() ? Grey1 : (ts.tune ? Red : White);
	if (ts.keyer_mode.active)
	{
		if (ts.buffered_tx)
		{
			cap = "TRX B";
		}
		else
		{
			cap = "TRX U";
		}
	}
	else
	{
		cap = "TUNE";
	}
	UiDriver_DrawFButtonLabel(5, cap, color);
}


void UiDriver_EncoderDisplaySimple(const uint8_t column, const uint8_t row, const char *label, bool encoder_active,
		uint32_t value)
{

	char temp[5];
	uint32_t color = encoder_active?White:Grey;

	snprintf(temp,5," %2lu",value);
	UiDriver_EncoderDisplay(column, row, label, encoder_active,
			temp, color);
}

void UiDriver_DisplaySplitFreqLabels()
{
	// in SPLIT mode?
	const char *split_rx, *split_tx;
	if (!(is_vfo_b()))
	{
		split_rx = "(A) RX->";  // Place identifying marker for RX frequency
		split_tx = "(B) TX->";  // Place identifying marker for TX frequency
	}
	else
	{
		split_rx = "(B) RX->";  // Place identifying marker for RX frequency
		split_tx = "(A) TX->";  // Place identifying marker for TX frequency
	}
	UiLcdHy28_PrintText(ts.Layout->TUNE_SPLIT_MARKER_X - (SMALL_FONT_WIDTH * 5),
			ts.Layout->TUNE_FREQ.y, split_rx, RX_Grey, Black,
			0);  // Place identifying marker for RX frequency
	UiLcdHy28_PrintText(ts.Layout->TUNE_SPLIT_MARKER_X - (SMALL_FONT_WIDTH * 5),
			ts.Layout->TUNE_SPLIT_FREQ_Y_TX, split_tx, TX_Grey, Black,
			0);  // Place identifying marker for TX frequency
}

void UiAction_CopyVfoAB()
{
	// not in menu mode:  Make VFO A = VFO B or VFO B = VFO A, as appropriate
	VfoReg* vfo_store;
	if(is_vfo_b())      // are we in VFO B mode?
	{
		vfo_store = &vfo[VFO_A].band[ts.band];
	}
	else        // we were in VFO A mode
	{
		vfo_store   =    &vfo[VFO_B].band[ts.band];
	}
	vfo_store->dial_value   = df.tune_new;
	vfo_store->decod_mode   = ts.dmod_mode;                   // copy active VFO settings into other VFO
	vfo_store->digital_mode = ts.digital_mode;

	UiDriver_FrequencyUpdateLOandDisplay(true);

	if (ts.menu_mode == false)
	{
		UiSpectrum_Clear();          // clear display under spectrum scope
		UiLcdHy28_PrintText(80,160,is_vfo_b()?"VFO B -> VFO A":"VFO A -> VFO B",Cyan,Black,1);
		HAL_Delay(3000);
		UiSpectrum_Init();           // init spectrum scope
	}
}

void UiAction_ToggleVfoAB()
{

	uint32_t old_dmod_mode = ts.dmod_mode;

	RadioManagement_ToggleVfoAB();
	UiDriver_SetDemodMode( ts.dmod_mode ); // ligne ajout�e suite � la version UHSDR 2.11.49
	UiDriver_FButton_F4ActiveVFO();

	// do frequency/display update
	if(is_splitmode())      // in SPLIT mode?
	{
		UiDriver_DisplaySplitFreqLabels();
	}

	// Change decode mode if need to
	if(ts.dmod_mode != old_dmod_mode)
	{
		UiDriver_UpdateDisplayAfterParamChange();
	}
	else
	{
		UiDriver_FrequencyUpdateLOandDisplay(true);
	}
}

void UiDriver_SetSplitMode(bool mode_active)
{
	if(mode_active)      // are we in SPLIT mode?
	{
		ts.vfo_mem_mode |= VFO_MEM_MODE_SPLIT;      // yes - turn on MSB to activate SPLIT
	}
	else // are we NOT in SPLIT mode?
	{
		ts.vfo_mem_mode &= ~VFO_MEM_MODE_SPLIT; // yes - turn off MSB to turn off SPLIT
	}
	UiDriver_CreateMainFreqDisplay();      //
	UiDriver_FrequencyUpdateLOandDisplay(true);
}

/**
 * @brief: process hardcoded buttons click and hold
 */
void UiDriver_RefreshEncoderDisplay()
{
	UiDriver_DisplayEncoderOneMode();
	UiDriver_DisplayEncoderTwoMode();
	UiDriver_DisplayEncoderThreeMode();
}

/**
 * @brief This is THE function to call after changing operational parameters such as frequency or demod mode
 * It will make sure to update the display AND also tunes to a newly selected frequency if not already tuned to it.
 */
bool UiDriver_IsDemodModeChange()
{
	bool retval = (ts.dmod_mode != ui_driver_state.dmod_mode);
	retval     |= ts.dmod_mode == DEMOD_DIGI && ts.digital_mode != ui_driver_state.digital_mode;

	return retval;
}

/**
 * @brief cleans out mode specific ui elements before switching to next mode
 */
void UiDriver_ModeSpecificDisplayClear(uint8_t dmod_mode, uint8_t digital_mode)
{
	switch(dmod_mode)
	{
		case DEMOD_CW:
							UiDriver_TextMsgClear();
							CwDecoder_WpmDisplayClearOrPrepare(false);
							UiSpectrum_InitCwSnapDisplay(false);
							break;
		case DEMOD_DIGI:
						{
							switch(digital_mode)
							{
#ifdef USE_FREEDV
								case DigitalMode_FreeDV:  FreeDv_DisplayClear();	break;
	#endif
								case DigitalMode_RTTY:
								case DigitalMode_BPSK:
														UiDriver_TextMsgClear();
														UiSpectrum_InitCwSnapDisplay(false);
														break;
								default:     		break;
							}
						}
						break;
		case DEMOD_AM:
		case DEMOD_SAM:  UiSpectrum_InitCwSnapDisplay(false);  	break;
		default:     		break;
	}
}

/**
 * @brief prepares mode specific ui elements run the mode
 */
void UiDriver_ModeSpecificDisplayPrepare(uint8_t dmod_mode, uint8_t digital_mode)
{
	switch(dmod_mode)
	{
	case DEMOD_CW:
	    if (ts.cw_decoder_enable == true)
	    {
	        CwDecoder_WpmDisplayClearOrPrepare(true);
	    }
	    if(cw_decoder_config.snap_enable == true && ts.dmod_mode == DEMOD_CW)
	    {
	    	UiSpectrum_InitCwSnapDisplay(true);
	    }
		break;
	case DEMOD_DIGI:
	{
		switch(digital_mode)
		{
#ifdef USE_FREEDV
		case DigitalMode_FreeDV:
			FreeDv_DisplayPrepare();
			break;
#endif
		case DigitalMode_RTTY:
		case DigitalMode_BPSK:
			UiDriver_TextMsgClear();
		    if(cw_decoder_config.snap_enable == true)
		    {
		    	UiSpectrum_InitCwSnapDisplay(true);
		    }
			break;
		default:
			break;
		}
	}
	break;
	case DEMOD_AM:
	case DEMOD_SAM:
					if(cw_decoder_config.snap_enable == true)
					{
						UiSpectrum_InitCwSnapDisplay(true);
					}
					break;

	default:
		break;
	}
}

void UiDriver_UpdateDemodSpecificDisplayAfterParamChange()
{
    // clear display content specific for the old mode
    UiDriver_ModeSpecificDisplayClear(ui_driver_state.dmod_mode,ui_driver_state.digital_mode);
    // prepare mode specific UI elements used in the new mode
    UiDriver_ModeSpecificDisplayPrepare(ts.dmod_mode,ts.digital_mode);
    ui_driver_state.dmod_mode = ts.dmod_mode;
    ui_driver_state.digital_mode = ts.digital_mode;
}

void UiDriver_UpdateDisplayAfterParamChange()
{
    // TODO Maybe we should split this, so that we clear BEFORE doing the general stuff
    // and prepare after, but for now it should work this way
	if (UiDriver_IsDemodModeChange())
	{
	    UiDriver_UpdateDemodSpecificDisplayAfterParamChange();
	}

	// below are the always present UI elements
	UiDriver_FrequencyUpdateLOandDisplay(false);   // update frequency display without checking encoder

	UiDriver_DisplayDemodMode();

	// UiDriver_DisplayMemoryLabel();

	UiDriver_DisplayFilter();    // make certain that numerical on-screen bandwidth indicator is updated

	UiSpectrum_DisplayFilterBW();  // update on-screen filter bandwidth indicator (graphical)

	UiDriver_RefreshEncoderDisplay();

	if(ts.menu_mode)    // are we in menu mode?
	{
		UiMenu_RenderMenu(MENU_RENDER_ONLY);    // yes, update display when we change modes
	}

	UiVk_Redraw();			//virtual keypads call (refresh purpose)
}

static  uint16_t  step_ordre = 3;

void  Gett_Tune_ordre_integer(ulong stepp)
{
	switch (df.tuning_step)
		{
			case 1   : step_ordre = 0;  break;
			case 5   : step_ordre = 1;  break;
			case 10  : step_ordre = 2;  break;
			case 100 : step_ordre = 3;  break;
			case 500 : step_ordre = 4;  break;
			case 1000: step_ordre = 5;  break;
			case 5000:
			default  : step_ordre = 6;  break;
		}
}
//
//
//*----------------------------------------------------------------------------
//* Function Name       : UiDriverPressHoldStep
//* Object              : Select the step size for the press-and-hold of the step size button
//* Input Parameters    : 0=Decrease step size, 1=Increase step size
//* Output Parameters   :
//* Functions called    :
//*----------------------------------------------------------------------------
static void UiDriver_PressHoldStep(uchar is_up)
{
	ulong	minus_idx, plus_idx;

	switch(df.selected_idx)	 		// select appropriate "alternate" step size based on current step size
	{
		case T_STEP_1HZ_IDX:	// 1Hz step size
		case T_STEP_5HZ_IDX:
		case T_STEP_10HZ_IDX:	// 10Hz step size
			minus_idx = T_STEP_1HZ_IDX;		// use 1 Hz as small step size
			plus_idx = T_STEP_1KHZ_IDX;		// use 1 kHz as large step size

			break;
		case T_STEP_100HZ_IDX:	// 100Hz step size
			minus_idx = T_STEP_10HZ_IDX;		// use 10 Hz as small step size
			plus_idx  = T_STEP_1KHZ_IDX;		// use 1 kHz as large step size

			break;
		case T_STEP_10KHZ_IDX:	// 10 kHz step size
		case T_STEP_100KHZ_IDX:	// 100 kHz step size
			minus_idx = T_STEP_100HZ_IDX;	// use 100 Hz as small step size
			plus_idx  = T_STEP_100KHZ_IDX;	// use 100 kHz as large step size
			break;
		case T_STEP_1KHZ_IDX:	// 1 kHz step size
		default:
			minus_idx = T_STEP_10HZ_IDX;	// use 10 Hz as small step size
			plus_idx  = T_STEP_10KHZ_IDX;	// use 10 kHz as large step size
			break;
	}

	if(!is_up)	 		// temporary decrease of step size
	{
		ts.tune_step                        = STEP_PRESS_MINUS;
		ts.tune_step_idx_holder             = df.selected_idx;
		if(df.selected_idx)  df.tuning_step	= tune_steps[minus_idx];
		df.selected_idx                     = minus_idx;
	}
	else	 			// temporary increase of step size
	{
		ts.tune_step            = STEP_PRESS_PLUS;
		ts.tune_step_idx_holder = df.selected_idx;
		df.tuning_step	        = tune_steps[plus_idx];
		df.selected_idx         = plus_idx;
	}
	Gett_Tune_ordre_integer(df.tuning_step);
	//
	UiDriver_DisplayFreqStepSize();		// update display
}




void UiDriver_DisplayDemodMode()
{
	// Clear control
	char* txt = "???";
	uint16_t clr_fg = White,clr_bg = Blue;

	// Create Decode Mode (USB/LSB/AM/FM/CW)
	switch(ts.dmod_mode)
	{
		case DEMOD_USB:   txt = "USB";	break;
		case DEMOD_LSB:   txt = "LSB";	break;
		case DEMOD_SAM:
						if(ads.sam_sideband == SAM_SIDEBAND_LSB)
						{
							txt = "SAM-L";
						}
						else if (ads.sam_sideband == SAM_SIDEBAND_USB)
						{
							txt = "SAM-U";
						}
						else
						{
							txt = "SAM";
						}
						break;
		case DEMOD_AM: 	txt = "AM";  	break;
		case DEMOD_FM:
						txt = RadioManagement_FmDevIs5khz() ? "FM-W" : "FM-N";
						{
							if(ts.txrx_mode == TRX_MODE_RX)
							{
								if(ads.fm_squelched == false)
								{
									// is audio not squelched?
									if((ads.fm_subaudible_tone_detected) && (ts.fm_subaudible_tone_det_select))
									{
										// is tone decoding enabled AND a tone being detected?
										clr_fg =  Black;
										clr_bg = Red2;	// Not squelched, passing audio - change color!
									}
									else  	// tone decoder disabled - squelch only
									{
										clr_fg = Black;
										clr_bg = White;	// Not squelched, passing audio - change color, but different from tone
									}
								}
							}
							else if(ts.txrx_mode == TRX_MODE_TX)	 	// in transmit mode?
							{
								if(ads.fm_tone_burst_active)	 		// yes - is tone burst active?
								{
									clr_fg = Black;
									clr_bg = Yellow;	// Yes, make "FM" yellow
								}
							}
							break;
						}
		case DEMOD_CW:	txt = ts.cw_lsb?"CW-L":"CW-U"; 	break;
		case DEMOD_DIGI:
						switch(ts.digital_mode)
						{
							case DigitalMode_RTTY: 		txt = ts.digi_lsb?"RT-L":"RT-U";	break;
							case DigitalMode_BPSK:		txt = ts.digi_lsb?"PSK-L":"PSK-U";	break;
							default:					txt = ts.digi_lsb?"DI-L":"DI-U";
						}
						break;

			default:   	break;
	}
	UiLcdHy28_PrintTextCentered(ts.Layout->DEMOD_MODE_MASK.x,ts.Layout->DEMOD_MODE_MASK.y,ts.Layout->DEMOD_MODE_MASK.w,txt,clr_fg,clr_bg,0);

	//  UiDriver_DisplayModulationType();  // NIZZZ pour ne pas afficher le Pad en bas ou il est marqu� SSB
}


void UiDriver_DisplayFreqStepSize()
{

	int	line_loc;
	static	bool	step_line = 0;	// used to indicate the presence of a step line
	uint32_t	color;
	uint32_t 	stepsize_background;

	color = ts.tune_step?Cyan:White;		// is this a "Temporary" step size from press-and-hold?
	stepsize_background = (ts.flags1 & FLAGS1_DYN_TUNE_ENABLE)?Blue:Black;
	// dynamic_tuning active -> yes, display on Grey3

	if(step_line)	 	// Remove underline indicating step size if one had been drawn
	{
		UiLcdHy28_DrawStraightLineDouble( ts.Layout->TUNE_FREQ.x      , ( ts.Layout->TUNE_FREQ.y + 25 ), ( LARGE_FONT_WIDTH*10 ), LCD_DIR_HORIZONTAL, Black );
		UiLcdHy28_DrawStraightLineDouble( ts.Layout->TUNE_SPLIT_FREQ_X, ( ts.Layout->TUNE_FREQ.y + 25 ), ( SMALL_FONT_WIDTH*10 ), LCD_DIR_HORIZONTAL, Black );
	}

	// Blank old step size
	// UiLcdHy28_DrawFullRect(POS_TUNE_STEP_X,POS_TUNE_STEP_Y-1,POS_TUNE_STEP_MASK_H,POS_TUNE_STEP_MASK_W,stepsize_background);

	{
		char step_name[10];

		// I know the code below will not win the price for the most readable code
		// ever. But it does the job of display any freq step somewhat reasonable.
		// khz/Mhz only whole  khz/Mhz is shown, no fraction
		// showing fractions would require some more coding, which is not yet necessary
		const uint32_t pow10 = log10f(df.tuning_step);
		line_loc = 9 - pow10 - pow10/3;
		if (line_loc < 0)
		{
			line_loc = -1;
		}
		const char* stepUnitPrefix[] = { "","k","M","G","T"};

		if (df.tuning_step < 50000)
		{
			snprintf(step_name,10,"%d%sHz",(int)(df.tuning_step/exp10((pow10/3)*3)), stepUnitPrefix[pow10/3]);
		}
		else
		{
			snprintf(step_name,10,"%d%sH",(int)(df.tuning_step/exp10((pow10/3)*3)), stepUnitPrefix[pow10/3]);
		}

		UiLcdHy28_PrintTextCentered(ts.Layout->TUNE_STEP.x,ts.Layout->TUNE_STEP.y,ts.Layout->TUNE_STEP.w  ,step_name,color,stepsize_background,0);
	}
	//
	if((ts.freq_step_config & FREQ_STEP_SHOW_MARKER) && line_loc >= 0)	 		// is frequency step marker line enabled?
	{
		if(is_splitmode())
		{ UiLcdHy28_DrawStraightLineDouble( (ts.Layout->TUNE_SPLIT_FREQ_X + (SMALL_FONT_WIDTH * line_loc)), (ts.Layout->TUNE_FREQ.y + 25), (SMALL_FONT_WIDTH), LCD_DIR_HORIZONTAL, White); }
		else
		{ UiLcdHy28_DrawStraightLineDouble( (ts.Layout->TUNE_FREQ.x       + (LARGE_FONT_WIDTH * line_loc)), (ts.Layout->TUNE_FREQ.y + 25), (LARGE_FONT_WIDTH), LCD_DIR_HORIZONTAL, White); }

		step_line = 1;	// indicate that a line under the step size had been drawn
	}
	else	// marker line not enabled
	{
		step_line = 0;	// we don't need to erase "step size" marker line in the future
	}
}


typedef struct
{
	uint32_t     start;
	uint32_t     end;
	const char*  name;
} BandGenInfo;



#ifdef TUNISIANN

const  BandGenInfo  bandGenInfo[] =
{
		{  150000,   285000, "Gen" },
		{  525000,  1605000, "Gen" },
		{ 2300000,  2495000, "Gen" },
		{ 3200000,  3400000, "Gen" },
		{ 3900000,  4000000, "Gen" },
		{ 4750000,  5060000, "Gen" },
		{ 5950000,  6200000, "Gen" },
		{ 7300000,  7350000, "Gen" },
		{ 9400000,  9900000, "Gen" },
		{11600000, 12100000, "Gen" },
		{13570000, 13870000, "Gen" },
		{15100000, 15800000, "Gen" },
		{17480000, 17900000, "Gen" },
		{18900000, 19020000, "Gen" },
		{21450000, 21750000, "Gen" },
		{25670000, 26100000, "Gen" },
		{26965000, 27405000, "Gen" },
		{       0,        0, "Gen" }
};



#else
const BandGenInfo bandGenInfo[] =
{
		{  150000,   285000, "LW" },
		{  525000,  1605000, "MW" },
		{ 2300000,  2495000, "120m" },
		{ 3200000,  3400000, "90m" },
		{ 3900000,  4000000, "75m" },
		{ 4750000,  5060000, "60m" },
		{ 5950000,  6200000, "49m" },
		{ 7300000,  7350000, "41m" },
		{ 9400000,  9900000, "31m" },
		{11600000, 12100000, "25m" },
		{13570000, 13870000, "22m" },
		{15100000, 15800000, "19m" },
		{17480000, 17900000, "16m" },
		{18900000, 19020000, "15m" },
		{21450000, 21750000, "13m" },
		{25670000, 26100000, "11m" },
		{26965000, 27405000, "11m" },
		{       0,        0, "Gen" }
};
#endif
//*----------------------------------------------------------------------------
/*
static void UiDriver_DisplayMemoryLabel()
{
	char txt[12];
	uint32_t col = White;
	if (ts.band < MAX_BAND_NUM && ts.cat_band_index == 255)
	{
		snprintf(txt,12,"Bnd%s ", bandInfo[ts.band].name);
	}
	if (ts.cat_band_index != 255)		// no band storage place active because of "CAT running in sandbox"
	{
		snprintf(txt,12,"  CAT  ");
	}
	UiLcdHy28_PrintText(ts.Layout->MEMORYLABEL.x,  ts.Layout->MEMORYLABEL.y,txt,col,Black,0);
}
*/

static void UiDriver_DisplayBand(uchar band)   /// NIZZZ  affichage bande � droite de la bande en minuscule
{
	const char* bandName;
	bool print_bc_name = true;
	int idx;

	if (band < MAX_BAND_NUM)
	{
		ulong col;
		// Clear control
		if (band == BAND_MODE_GEN)
		{
			for (idx = 0; bandGenInfo[idx].start !=0; idx++)
			{
				if (df.tune_old >= bandGenInfo[idx].start && df.tune_old < bandGenInfo[idx].end)
				{
					break; // found match
				}
			}

			if (bandGenInfo[idx].start !=0)  col = Yellow;  // Print name of BC band in yellow, if frequency is within a broadcast band
			else  				             col = Orange;

			if  (bandGenInfo[idx].start == 26965000)
				col = Blue;		// CB radio == blue


			if (idx == ts.bc_band) 	print_bc_name = false;
			ts.bc_band =idx;

			bandName = bandGenInfo[idx].name;



		}
		else
		{
			print_bc_name =  true;
			col           =  Orange;
			bandName      =  bandInfo[band].name;
			ts.bc_band    =  0xff;
		}
		if (print_bc_name)
		{
			UiLcdHy28_DrawFullRect  ( ts.Layout->BAND_MODE_MASK.x , ts.Layout->BAND_MODE_MASK.y -7, ts.Layout->BAND_MODE_MASK.h, ts.Layout->BAND_MODE_MASK.w, Black);
			UiLcdHy28_PrintTextRight( ts.Layout->BAND_MODE.x + 5*8, ts.Layout->BAND_MODE.y      -7, bandName, col, Black, 0 ); /// NIZZ  -8 pour d�caler plus haut l'affichage de la bande
		}

	}

	// add indicator for broadcast bands here
	// if Band = "Gen" AND frequency inside one of the broadcast bands, print name of the band
#ifdef TUNISIANN
	else bandName == "Gen" ;
	if ( bandName == "Gen")  { 	ts.tx_disable |=  TX_DISABLE_ALWAYS; }         //TX_DISABLE_OUTOFRANGE;}
	else                     {  ts.tx_disable &= ~TX_DISABLE_ALWAYS; }         //TX_DISABLE_OUTOFRANGE;}
	UiDriver_DrawFButtonLabel ( 5, "TUNE", ts.tx_disable ?Grey1 : White );
#endif
}



//*----------------------------------------------------------------------------
//* Function Name       : UiDriverInitMainFreqDisplay
//* Object              :
//* Input Parameters    :
//* Output Parameters   :
//* Functions called    :
//*----------------------------------------------------------------------------
static void UiDriver_CreateMainFreqDisplay()
{
	UiDriver_FButton_F3MemSplit();
	if((is_splitmode()))	 	// are we in SPLIT mode?
	{
		UiLcdHy28_PrintText( ts.Layout->TUNE_FREQ.x-16, ts.Layout->TUNE_FREQ.y, "          ",White, Black, 1 );	// clear large frequency digits
		UiDriver_DisplaySplitFreqLabels();
	}
	UiDriver_DisplayFreqStepSize();
}

//*----------------------------------------------------------------------------
//* Function Name       : UiDriverCreateFunctionButtons
//* Object              : function keys based on decoder mode
//* Input Parameters    :
//* Output Parameters   :
//* Functions called    :
//*----------------------------------------------------------------------------
static void UiDriver_CreateFunctionButtons ( bool  full_repaint )
{
	// Create bottom bar
	if(full_repaint)
	{
		for (int i = 0; i < 5; i++)
		{
			UiLcdHy28_DrawBottomButton ( (ts.Layout->BOTTOM_BAR.x + (ts.Layout->BOTTOM_BAR.w+1)*i), (ts.Layout->BOTTOM_BAR.y - 4),ts.Layout->BOTTOM_BAR.h+2,ts.Layout->BOTTOM_BAR.w,Grey);
		}
	}

	// Button F1
	UiDriver_DisplayFButton_F1MenuExit();

	// Button F2
	UiDriver_DisplayFButton_F2SnapMeter();

	// Button F3
	UiDriver_FButton_F3MemSplit();

	// Button F4
	UiDriver_FButton_F4ActiveVFO();

	// Button F5
	UiDriver_FButton_F5Tune();
}

void UiDriver_SetSpectrumMode(SpectrumMode_t mode)
{
    ts.flags1 = (ts.flags1 & ~(FLAGS1_SCOPE_ENABLED | FLAGS1_WFALL_ENABLED)) |(mode << 7);
}
SpectrumMode_t UiDriver_GetSpectrumMode()
{
    return (ts.flags1 & (FLAGS1_SCOPE_ENABLED | FLAGS1_WFALL_ENABLED))  >> 7;
}

//
//*----------------------------------------------------------------------------
//* Function Name       : UiDriverDrawSMeter
//* Object              : draw the part of the S meter
//* Input Parameters    : uchar color
//* Output Parameters   :
//* Functions called    :
//*----------------------------------------------------------------------------
static void UiDriver_CreateDesktop()
{
	// Backlight off - hide startup logo
	UiLcdHy28_BacklightEnable(false);

	// Clear display
	UiLcdHy28_LcdClear(Black);

	// Create Band value
	UiDriver_DisplayBand(ts.band);

	// Frequency
	UiDriver_CreateMainFreqDisplay();

	// Function buttons
	UiDriver_CreateFunctionButtons(true);

	// S-meter
	UiDriver_CreateMeters();

	if (UiDriver_GetSpectrumMode() == SPECTRUM_BLANK)
	{
	    UiDriver_SetSpectrumMode(SPECTRUM_DUAL);
	}

	//UiSpectrum_GetSpectrumGraticule()->y=ts.graticulePowerupYpos;
	// Spectrum scope
	UiSpectrum_Init();

	UiDriver_RefreshEncoderDisplay();

	// Power level
	UiDriver_DisplayPowerLevel();


	UiDriver_CreateVoltageDisplay();

	// Create temperature if SI570 is in use
	// if(ts.si570_is_present)
	{
	  UiDriver_CreateTemperatureDisplay();
	}

	// Set correct frequency
	UiDriver_FrequencyUpdateLOandDisplay(true);

	UiDriver_UpdateDisplayAfterParamChange();

	// Backlight on - only when all is drawn
	UiLcdHy28_BacklightEnable(true);
}


static void UiDriver_DrawSMeter ( uint16_t color )
{
	// Draw top line
	UiLcdHy28_DrawStraightLineDouble ( (ts.Layout->SM_IND.x +  18), (ts.Layout->SM_IND.y + 20), 92 + 22, LCD_DIR_HORIZONTAL, color );
	// Draw s markers on top white line

	for ( uint16_t i = 0; i < 8; i++ )  // tracer les marquages du Smeter
	{
		uint8_t 	   v_s = 5;         // hauteur de la marque en position normale
		if ( i==0 )  { v_s = 7; }       // hauteur de la marque au debut du trait
		UiLcdHy28_DrawStraightLineDouble(((ts.Layout->SM_IND.x + 18) + i*16), ((ts.Layout->SM_IND.y + 20) - v_s), v_s, LCD_DIR_VERTICAL, color);
	}
}




#define BTM_MINUS 12    //   14   NIZZZ pour faire baisser le SWR meter en graphic

//*----------------------------------------------------------------------------
//* Function Name       : UiDriverDeleteSMeter
//* Object              : delete the S meter
//* Input Parameters    :
//* Output Parameters   :
//* Functions called    :
//*----------------------------------------------------------------------------
static void UiDriver_DeleteMeters()
{
	UiLcdHy28_DrawFullRect(ts.Layout->SM_IND.x+1,ts.Layout->SM_IND.y+1,ts.Layout->SM_IND.h ,ts.Layout->SM_IND.w,Black);
}

static void UiDriver_DeleteSMeterLabels()
{
	UiLcdHy28_DrawFullRect(ts.Layout->SM_IND.x+1,ts.Layout->SM_IND.y+1,21,ts.Layout->SM_IND.w,Black);
}


static void UiDriver_DrawPowerMeterLabels()
{
	uchar   i;
	char    num[20];

	// Leading text
	UiLcdHy28_PrintText(((ts.Layout->SM_IND.x + 18) - 12),(ts.Layout->SM_IND.y + 5),"P",  White,Black,4);

	UiLcdHy28_PrintText((ts.Layout->SM_IND.x + 185),(ts.Layout->SM_IND.y + 5)," W",White,Black,4);

	// Draw middle line
	UiLcdHy28_DrawStraightLineDouble((ts.Layout->SM_IND.x + 18),(ts.Layout->SM_IND.y + 20),170,LCD_DIR_HORIZONTAL,White);
	// S Meter -> Y + 20

	// Draw s markers on middle white line
	for(i = 0; i < 12; i++)
	{
		uint8_t v_s;
		if(i < 10)
		{
			num[0] = i + 0x30;
			num[1] = 0;
		}
		else
		{
			num[0] = i/10 + 0x30;
			num[1] = i%10 + 0x30;
			num[2] = 0;
		}

		// Draw s text, only odd numbers
		if(!(i%2))
		{
			UiLcdHy28_PrintText(((ts.Layout->SM_IND.x + 18) - 3 + i*15),(ts.Layout->SM_IND.y + 5),num,White,Black,4);
		}
		// Lines
		v_s=(i%2)?3:5;
		UiLcdHy28_DrawStraightLineDouble(((ts.Layout->SM_IND.x + 18) + i*15),((ts.Layout->SM_IND.y + 20) - v_s),v_s,LCD_DIR_VERTICAL,White);
	}


}

///*********************************************************

static void UiDriver_DrawSMeterLabels()
{
	uchar   i, v_s;
	char    num[20];

	// Draw top line
	UiLcdHy28_DrawFullRect ( (ts.Layout->SM_IND.x +       18), (ts.Layout->SM_IND.y + 20), 2, 92 + 22, White);  /// NIZZZ  pour �largir la barre du Smeter
	UiLcdHy28_DrawFullRect ( (ts.Layout->SM_IND.x +  113 +22), (ts.Layout->SM_IND.y + 20), 2, 75 - 22, Green);  // Blue2

	// Leading text
	UiLcdHy28_PrintText (((ts.Layout->SM_IND.x + 18) - 12),(ts.Layout->SM_IND.y +  5),"S",  White, Black, 4);

	// Trailing text
	UiLcdHy28_PrintText ((ts.Layout->SM_IND.x + 185),(ts.Layout->SM_IND.y + 5), "dB", Green, Black, 4); // Blue2


	num[1] = 0;
	// Draw s markers on top white line
	// int debutg = 1;
	for(i = 0; i < 8; i++)
	{
		num[0] = i + 0x32;
							// Draw s text, only odd numbers
		if(i != 0)
		{
			UiLcdHy28_PrintText ( ((ts.Layout->SM_IND.x + 18) - 4  + i*16 ), ( ts.Layout->SM_IND.y + 5 ), num, White, Black, 4 ); /// NIZZZ  -4 +i*10
			 v_s = 5;
		}
		else  { v_s = 7; }


		// Lines
		UiLcdHy28_DrawStraightLineDouble ( ((ts.Layout->SM_IND.x + 18)   + i*16), ((ts.Layout->SM_IND.y + 20) - v_s), v_s, LCD_DIR_VERTICAL, White );
		//debutg = 0;
	}

	num[0] = 0x2B;  // '+'  du +10dB ou +20dB
	num[2] = 0x30;
	num[3] = 0x00;
	// Draw s markers on top green line
	for(i = 0; i < 3; i++)  // NIZZZ 4
	{
		// Prepare text
		num[1] = i  + 0x30;   /// NIZZZ  i*2

		if(i)
		{
			// Draw text
			UiLcdHy28_PrintText(((ts.Layout->SM_IND.x + 108)    - 15       + i*25 + 20), (ts.Layout->SM_IND.y + 5),num,Green,Black,4); // Blue2     NIZZ *20

			// Draw vert lines
			UiLcdHy28_DrawStraightLineDouble(((ts.Layout->SM_IND.x + 108) + i*25 + 20), (ts.Layout->SM_IND.y + 15),5,LCD_DIR_VERTICAL,Green);  // Blue2
		}
	}
}


static bool  SWR_Zoom1 = false;  ///  NIZZZ pour indiquer si on est en mode Zoom de la barre du SWR
static bool  SWR_Zoom2 = false;  ///  NIZZZ pour indiquer si on est en mode Zoom de la barre du SWR

static void UiDriver_CreateMeters()
{
	uchar 	i;
	char	num[20];
	int		col;

	UiLcdHy28_DrawEmptyRect ( ts.Layout->SM_IND.x, ts.Layout->SM_IND.y, ts.Layout->SM_IND.h, ts.Layout->SM_IND.w + 2, Grey );

	if (ts.txrx_mode == TRX_MODE_RX )  { UiDriver_DrawSMeterLabels();     }
	else							   { UiDriver_DrawPowerMeterLabels(); }

	if ( ts.tx_meter_mode == METER_SWR)
	{
		SWR_Zoom1 = false;
		SWR_Zoom2 = false;
		UiLcdHy28_PrintText(((ts.Layout->SM_IND.x + 18) - 12), (ts.Layout->SM_IND.y + 59 - BTM_MINUS), "SWR", White, Black, 4);

		// Draw bottom line for SWR indicator
		UiLcdHy28_DrawStraightLineDouble((ts.Layout->SM_IND.x + 18   ),(ts.Layout->SM_IND.y + 55 - BTM_MINUS), 62 + 31,LCD_DIR_HORIZONTAL, White);  /// NIZZZ
		UiLcdHy28_DrawStraightLineDouble((ts.Layout->SM_IND.x + 83+31),(ts.Layout->SM_IND.y + 55 - BTM_MINUS),105 - 31,LCD_DIR_HORIZONTAL, Blue2); // Red
		col = White;

		// Draw S markers on middle white line
		for(i = 0; i < 12; i++)
		{
			if(i > 6) col = Blue2;  // Red;

			if(!(i%2))
			{
				if(i)
				{
					num[0] = i/2 + 0x30;
					num[1] = 0;

					// Text
					UiLcdHy28_PrintText(((ts.Layout->SM_IND.x + 18) - 3          -14 + i*17 + i/2 ),(ts.Layout->SM_IND.y + 59 - BTM_MINUS)      ,num,White,Black,4);  // i*10

					UiLcdHy28_DrawStraightLineDouble(((ts.Layout->SM_IND.x + 18) -14 + i*17 + i/2 ),((ts.Layout->SM_IND.y + 55 - BTM_MINUS) - 2),2,LCD_DIR_VERTICAL,col);
				}
				else
				{
					UiLcdHy28_DrawStraightLineDouble(((ts.Layout->SM_IND.x + 18) + i*10),((ts.Layout->SM_IND.y + 55 - BTM_MINUS) - 7),7,LCD_DIR_VERTICAL,col);
				}
			}
		}
	}
	else if(ts.tx_meter_mode == METER_ALC)
	{
		UiLcdHy28_PrintText(((ts.Layout->SM_IND.x + 18) - 12),(ts.Layout->SM_IND.y + 59 - BTM_MINUS),"ALC",Yellow,Black,4);


		UiLcdHy28_DrawStraightLineDouble((ts.Layout->SM_IND.x + 18),(ts.Layout->SM_IND.y + 55 - BTM_MINUS), 62,LCD_DIR_HORIZONTAL,White);
		UiLcdHy28_DrawStraightLineDouble((ts.Layout->SM_IND.x + 83),(ts.Layout->SM_IND.y + 55 - BTM_MINUS),105,LCD_DIR_HORIZONTAL,Red);

		col = White;

		// Draw markers on middle line
		for(i = 0; i < 17; i++)
		{
			if(i > 6) col = Red;
			if(!(i%2))
			{
				if(i)
				{
					snprintf(num,20,"%d",(i*2));
					// Text
					UiLcdHy28_PrintText(((ts.Layout->SM_IND.x + 18) - 3 + i*10),(ts.Layout->SM_IND.y + 59 - BTM_MINUS),num,White,Black,4);

					UiLcdHy28_DrawStraightLineDouble(((ts.Layout->SM_IND.x + 18) + i*10),((ts.Layout->SM_IND.y + 55 - BTM_MINUS) - 2),2,LCD_DIR_VERTICAL,col);
				}
				else
				{
					UiLcdHy28_DrawStraightLineDouble(((ts.Layout->SM_IND.x + 18) + i*10),((ts.Layout->SM_IND.y + 55 - BTM_MINUS) - 7),7,LCD_DIR_VERTICAL,col);
				}
			}
		}
	}
	else if(ts.tx_meter_mode == METER_AUDIO)
	{
		UiLcdHy28_PrintText(((ts.Layout->SM_IND.x + 18) - 12),(ts.Layout->SM_IND.y + 59 - BTM_MINUS),"AUD",Cyan,Black,4);

		// Draw bottom line for SWR indicator
		UiLcdHy28_DrawStraightLineDouble((ts.Layout->SM_IND.x + 18),(ts.Layout->SM_IND.y + 55 - BTM_MINUS), 108,LCD_DIR_HORIZONTAL,White);
		UiLcdHy28_DrawStraightLineDouble((ts.Layout->SM_IND.x + 129),(ts.Layout->SM_IND.y + 55 - BTM_MINUS),59,LCD_DIR_HORIZONTAL,Red);
		col = White;

		// Draw markers on middle line
		for(i = 0; i < 17; i++)
		{
			if(i > 10) col = Red;
			if(!(i%2))
			{
				if(i)
				{
					snprintf(num,20,"%d",(i*2)-20);
					// Text
					UiLcdHy28_PrintText(((ts.Layout->SM_IND.x + 18) - 3 + i*10),(ts.Layout->SM_IND.y + 59 - BTM_MINUS),num,White,Black,4);

					UiLcdHy28_DrawStraightLineDouble(((ts.Layout->SM_IND.x + 18) + i*10),((ts.Layout->SM_IND.y + 55 - BTM_MINUS) - 2),2,LCD_DIR_VERTICAL,col);
				}
				else
				{
					UiLcdHy28_DrawStraightLineDouble(((ts.Layout->SM_IND.x + 18) + i*10),((ts.Layout->SM_IND.y + 55 - BTM_MINUS) - 7),7,LCD_DIR_VERTICAL,col);
				}
			}
		}
	}
	// Draw meters
	UiDriver_UpdateTopMeterA(34);
	UiDriver_UpdateTopMeterA(0);
	UiDriver_UpdateBtmMeter(34, 34);
	UiDriver_UpdateBtmMeter(0, 34);

}

//*----------------------------------------------------------------------------
//* Function Name       : UiDriverUpdateTopMeterA
//* Object              : redraw indicator, same like upper implementation
//* Input Parameters    : but no hold
//* Output Parameters   :
//* Functions called    :
//*----------------------------------------------------------------------------
#define SMETER_MAX_LEVEL 33

enum
{
	METER_TOP = 0,
	METER_BTM,
	METER_NUM
};
typedef struct MeterState_s
{
	uint8_t last;
	uint8_t last_warn;
} MeterState;

static MeterState meters[METER_NUM];

//*----------------------------------------------------------------------------
//* Function Name       : UiDriverUpdateBtmMeter
//* Object              : redraw indicator
//* Input Parameters    : val=indicated value, warn=red warning threshold
//* Output Parameters   :
//* Functions called    :
//*----------------------------------------------------------------------------

static void UiDriver_UpdateMeter(uchar val, uchar warn, uint32_t color_norm, uint8_t meterId)
{
	uchar               i;
	const    uint8_t   v_s       = 7;  // 3; moins d'espace entre rectangles
	uint32_t           col       = color_norm;
	uint8_t            from, to;
	uint8_t            from_warn = 255;

	uint16_t ypos = meterId==METER_TOP?(ts.Layout->SM_IND.y + 28 +2):(ts.Layout->SM_IND.y + 51+1 - BTM_MINUS );  /// NIZZZ

	// limit meter
	if(val > SMETER_MAX_LEVEL) 	{  val  = SMETER_MAX_LEVEL   ;	}
	if (warn == 0)          	{  warn = SMETER_MAX_LEVEL +1;  }  // never warn if warn == 0


	if(warn != meters[meterId].last_warn)
	{
		if (warn < meters[meterId].last_warn)	{	from_warn = warn;						}
		else									{	from_warn = meters[meterId].last_warn;	}
	}


	if ( val !=  meters[meterId].last || from_warn != 255 )
	{

		// decide if we need to draw more boxes or delete some
		if (val > meters[meterId].last)
		{
			// we will draw more active boxes
			from = meters[meterId].last;
			to   = val + 1;

		}
		else
		{
			from = val;
			to   = meters[meterId].last + 1;
		}
		if ( from_warn < from) 	{ from = from_warn; }

		// Draw indicator
		// we never draw a zero, so we start from 1 min
		if ( from == 0 )	{ from = 1; }

		for ( i = from; i < to; i++ )
		{
			if (i>val)
			{
				col = Grid;    // switch to delete color
			}
			if((i >= warn) && warn && col != Grid)    // is level above "warning" color? (is "warn" is zero, disable warning)
			{
				col =  Red2;                 // yes - display values above that color in red
			}
			// Lines
		    UiLcdHy28_DrawStraightLineTriple    ( ((ts.Layout->SM_IND.x + 18) + i*5), (ypos - v_s), v_s, LCD_DIR_VERTICAL, col );
		 // UiLcdHy28_DrawStraightLineQuadriple ( ((ts.Layout->SM_IND.x + 18) + i*5), (ypos - v_s), v_s, LCD_DIR_VERTICAL, col);  // NIZZ
		}

		meters[meterId].last      = val;
		meters[meterId].last_warn = warn;
	}
}


static void UiDriver_UpdateTopMeterA(uchar val)
{
	ulong clr;
	UiMenu_MapColors     (ts.meter_colour_up, NULL, &clr);
	UiDriver_UpdateMeter (val, SMETER_MAX_LEVEL+1, clr, METER_TOP);
}

/**
 * @brief updates the lower meter
 * @param val the value to display, max value is S_METER_MAX, min is 0, values will be limited to range
 * @param warn the level value from which the meter is going to show warning indication
 */
static void UiDriver_UpdateBtmMeter(float val, uchar warn)
{
	ulong clr;
	UiMenu_MapColors( ts.meter_colour_down, NULL, &clr );
	if (val < 0)
	{
		val = 0;
	}
	if (val > S_METER_MAX)
	{
		val = S_METER_MAX;
	}
	UiDriver_UpdateMeter( val, warn, clr, METER_BTM );
}




// FIXME: Move to RadioManagement()
void UiDriver_InitBandSet()
{
    // TODO: Do this setting based on the detected RF board capabilities
    // set the enabled bands
    for( int i = 0; i < MAX_BANDS; i++ )
    {
        vfo[VFO_A].enabled[i] = true; // we enable all bands but right below we turn off a few
        vfo[VFO_B].enabled[i] = true;
    }

    switch (ts.rf_board)
    {
		case FOUND_RF_BOARD_MCHF:
			vfo[VFO_A].enabled[BAND_MODE_2   ] = false;          vfo[VFO_B].enabled[BAND_MODE_2   ] = false;
			vfo[VFO_A].enabled[BAND_MODE_70  ] = false;          vfo[VFO_B].enabled[BAND_MODE_70  ] = false;
			vfo[VFO_A].enabled[BAND_MODE_23  ] = false;          vfo[VFO_B].enabled[BAND_MODE_23  ] = false;
#ifdef TUNISIANN
			vfo[VFO_A].enabled[BAND_MODE_60  ] = false;		    vfo[VFO_B].enabled[BAND_MODE_60  ] = false;
			vfo[VFO_A].enabled[BAND_MODE_30  ] = false;		    vfo[VFO_B].enabled[BAND_MODE_30  ] = false;
#endif
			vfo[VFO_A].enabled[BAND_MODE_4   ] = false;		    vfo[VFO_B].enabled[BAND_MODE_4   ] = false;
			vfo[VFO_A].enabled[BAND_MODE_6   ] = false;	 	    vfo[VFO_B].enabled[BAND_MODE_6   ] = false;
			vfo[VFO_A].enabled[BAND_MODE_630 ] = false;          vfo[VFO_B].enabled[BAND_MODE_630 ] = false;
			vfo[VFO_A].enabled[BAND_MODE_2200] = false;          vfo[VFO_B].enabled[BAND_MODE_2200] = false;
			break;


    case FOUND_RF_BOARD_OVI40:

			vfo[VFO_A].enabled[BAND_MODE_2   ] = false;      	vfo[VFO_B].enabled[BAND_MODE_2   ] = false;
			vfo[VFO_A].enabled[BAND_MODE_70  ] = false;		    vfo[VFO_B].enabled[BAND_MODE_70  ] = false;
			vfo[VFO_A].enabled[BAND_MODE_23  ] = false;		    vfo[VFO_B].enabled[BAND_MODE_23  ] = false;
#ifdef TUNISIANN
			vfo[VFO_A].enabled[BAND_MODE_60  ] = false;		    vfo[VFO_B].enabled[BAND_MODE_60  ] = false;
			vfo[VFO_A].enabled[BAND_MODE_30  ] = false;		    vfo[VFO_B].enabled[BAND_MODE_30  ] = false;
#endif
			vfo[VFO_A].enabled[BAND_MODE_4   ] = false;		    vfo[VFO_B].enabled[BAND_MODE_4   ] = false;
			vfo[VFO_A].enabled[BAND_MODE_6   ] = false;	 	    vfo[VFO_B].enabled[BAND_MODE_6   ] = false;
			vfo[VFO_A].enabled[BAND_MODE_630 ] = false;		    vfo[VFO_B].enabled[BAND_MODE_630 ] = false;
			vfo[VFO_A].enabled[BAND_MODE_2200] = false;		    vfo[VFO_B].enabled[BAND_MODE_2200] = false;

        break;
    }
}

//*----------------------------------------------------------------------------
//* Function Name       : UiDriverInitFrequency
//* Object              : set default values, some could be overwritten later
static void UiDriver_InitFrequency()
{
	ulong i;

	// Clear band values array
	for(i = 0; i < MAX_BANDS; i++)
	{
		vfo[VFO_A].band[i].dial_value = 0xFFFFFFFF;	// clear dial values
		vfo[VFO_A].band[i].decod_mode = DEMOD_USB; 	// clear decode mode
        vfo[VFO_A].band[i].decod_mode = DigitalMode_None;   // clear decode mode

		vfo[VFO_B].band[i].dial_value = 0xFFFFFFFF;  // clear dial values
		vfo[VFO_B].band[i].decod_mode = DEMOD_USB;   // clear decode mode
        vfo[VFO_B].band[i].decod_mode = DigitalMode_None;   // clear decode mode
	}



	// Lower bands default to LSB mode
	// TODO: This needs to be checked, some even lower bands have higher numbers now
	for(i = 0; i < 4; i++)
	{
		vfo[VFO_A].band[i].decod_mode = DEMOD_LSB;
		vfo[VFO_B].band[i].decod_mode = DEMOD_LSB;
	}
	// Init frequency publics(set diff values so update on LCD will be done)
	df.tune_old 	= bandInfo[ts.band].tune;
	df.tune_new 	= bandInfo[ts.band].tune;
	df.selected_idx = 3; 		// 1 Khz startup step
	df.tuning_step	= tune_steps[df.selected_idx];  Gett_Tune_ordre_integer(df.tuning_step);
	df.temp_factor	= 0;
	df.temp_factor_changed = false;
	df.temp_enabled = 0;		// startup state of TCXO

	UiDriver_InitBandSet();

	// Set virtual segments initial value (diff than zero!)
	df.dial_digits[8]	= 0;
	df.dial_digits[7]	= 0;
	df.dial_digits[6]	= 0;
	df.dial_digits[5]	= 0;
	df.dial_digits[4]	= 0;
	df.dial_digits[3]	= 0;
	df.dial_digits[2]	= 0;
	df.dial_digits[1]	= 0;
	df.dial_digits[0]	= 0;
}

/**
 * @brief Checks in which band the current frequency lies and updates display only if changed
 *
 * @param freq frequency in Hz
 * @returns band index (0 - (MAX_BANDS-1))
 */

uchar UiDriver_DisplayBandForFreq(ulong freq)
{
	// here we maintain our local state of the last band shown
	static uint8_t ui_band_scan_old = 99;
	uint8_t band_scan = RadioManagement_GetBand(freq);
	if(band_scan != ui_band_scan_old || band_scan == BAND_MODE_GEN)        // yes, did the band actually change?
	{
		UiDriver_DisplayBand(band_scan);    // yes, update the display with the current band
	}
	ui_band_scan_old = band_scan;
	return band_scan;
}



//***********************************************************************************************************************
/*
 * @brief Changes the tune frequency according to mode and other settings
 * @param dial_freq The desired dial frequency in Hz (not the tune frequency of the LO)
 * @returns true if the change was executed  (even if it is not tunable freq), false if the change is pending
 */
/*
 * @brief Check if a frequency is tunable
 * @returns SI570_OK, SI570_LARGE_STEP, SI570_TUNE_LIMITED if ok, SI570_TUNE_IMPOSSIBLE if not OK
 */
/*
 * @brief Used to update the individual vfo displays, not meant to be called directly except when changing LO
 * @brief parameters (in this case use (true,0)), use UiDriver_FrequencyUpdateLOandDisplay(full_update) instead
 *
 * @param force_update true = unconditionally update synthesizer EVEN IF frequency did not change
 * @param mode  =0 automatic, 1=force large, 2=force small, upper (RX), 3 = small, lower (TX)
 *
 * WARNING:  If called with "mode = 3", you must ALWAYS call again with "mode = 2" to reset internal variables.
 */
/*
 * @brief change LO freq to match df.tune_new freq according to mode without updating the ui
 *
 * @param trx_mode The mode which the frequency is being used for (TRX_MODE_TX/TRX_MODE_RX)
 */ //*******************************************************************************************************************

void  UiDriver_UpdateFrequency ( bool force_update, enum UpdateFrequencyMode_t mode )
{

	// FIXME: Don't like the handling of lo_result if in Split mode and transmitting
	uint32_t		               dial_freq;
	Oscillator_ResultCodes_t       lo_result            = OSC_OK;
	bool                           lo_change_not_pending = true;

	if(mode == UFM_SMALL_TX)
		// are we updating the TX frequency (small, lower display)?
	{
		dial_freq = RadioManagement_GetTXDialFrequency() ;
		lo_result = RadioManagement_ValidateFrequencyForTX( dial_freq );   // we check with the si570 code if the frequency is tunable, we do not tune to it.
	}
	else
	{
		dial_freq = df.tune_new;

		lo_change_not_pending =  RadioManagement_ChangeFrequency ( force_update, dial_freq, ts.txrx_mode );
		lo_result = ts.last_lo_result;   // use last ts.lo_result
	}

	if (mode == UFM_SMALL_RX && ts.txrx_mode == TRX_MODE_TX )
		// we are not going to show the tx frequency here (aka dial_freq) so we cannot use dial_freq
	{
		dial_freq = RadioManagement_GetRXDialFrequency() ;

		// lo_result = RadioManagement_ValidateFrequencyForTX(dial_freq);  // we check with the si570 code if the frequency is tunable, we do not tune to it.
	}

	// ALL UI CODE BELOW
	{
		uint32_t clr;

		if (lo_change_not_pending)
		{

			if (mode != UFM_SMALL_TX)
			{
				UiDriver_DisplayBandForFreq(dial_freq);
				// check which band in which we are currently tuning and update the display

				UiDriver_UpdateLcdFreq(RadioManagement_GetRXDialFrequency()  ,White, UFM_SECONDARY);
				// set mode parameter to UFM_SECONDARY to update secondary display (it shows real RX frequency if RIT is being used)
				// color argument is not being used by secondary display
			}

			switch(lo_result)
			{
				case OSC_TUNE_IMPOSSIBLE: clr = Orange; break; // orange if there was a problem setting frequency
				case OSC_TUNE_LIMITED   : clr = Yellow; break; // yellow if there was a problem setting frequency exactly
				case OSC_LARGE_STEP     :
				case OSC_OK             : clr = White; 	break; // Normal
				default                 : clr = Red;          // a serious error happened, i.e. I2C not working etc.
			}
		}
		else
		{
			// we did not execute the change, so we show the freq in Blue.
			// this will turn into the appropriate color the moment the tuning
			// happens.
			// Use white in releases, many complained about the  Blue digits
			clr = White; // Blue;
		}
		// Update frequency display
		UiDriver_UpdateLcdFreq(dial_freq, clr, mode);
	}
}



static void UiDriver_UpdateFreqDisplay(ulong dial_freq, uint8_t* dial_digits, ulong pos_x_loc, ulong font_width, ulong pos_y_loc, ushort color, uchar digit_size)
{
	{

#define MAX_DIGITS 9
		ulong dial_freq_temp;
		int8_t pos_mult[MAX_DIGITS] = {9, 8, 7, 5, 4, 3, 1, 0, -1};
		uint32_t idx;
		uint8_t digits[MAX_DIGITS];
		char digit[2];
		uint8_t last_non_zero = 0;

		// Terminate string for digit
		digit[1] = 0;
		// calculate the digits
		dial_freq_temp = dial_freq;

		for (idx = 0; idx < MAX_DIGITS; idx++)
		{
			digits[idx]      = dial_freq_temp % 10;
			dial_freq_temp  /= 10;
			if (digits[idx] != 0)  last_non_zero = idx;
		}

		for (idx = 0; idx < MAX_DIGITS; idx++)
		{
								// -----------------------
								// See if digit needs update
			if ((digits[idx] != dial_digits[idx]) || ts.refresh_freq_disp)
			{
				bool noshow = idx > last_non_zero;
								// don't show leading zeros, except for the 0th digits
				digit[0] = noshow?' ':0x30 + (digits[idx] & 0x0F);
								// Update segment
				if ( ( idx > 2 ) || is_splitmode() )

				{ UiLcdHy28_PrintText( (pos_x_loc + pos_mult[idx] * font_width), pos_y_loc+1, digit, color, Black, digit_size); }
				else
				{ UiLcdHy28_PrintText( (pos_x_loc + pos_mult[idx] * font_width), pos_y_loc+1, digit, Yellow_Light, Black, digit_size ); }
			}
		}

		for ( idx = 3; idx < MAX_DIGITS;  idx += 3)
		{
			bool noshow = last_non_zero < idx;
			digit[0]    = noshow?' ':'.';
			UiLcdHy28_PrintText( pos_x_loc+ (pos_mult[idx]+1) * font_width,pos_y_loc,digit,color,Black,digit_size );
		}

	}
}

//*----------------------------------------------------------------------------
//* Function Name       : UiDriverUpdateLcdFreq
//* Object              : this function will split LCD freq display control
//* Object              : and update as it is 7 segments indicator
//* Input Parameters    : freq=freq (Hz), color=color, mode: 0 = auto, 1= force normal (large digits), 2= force upper, small, 3 = force lower, small, 4 = secondary display
//* Output Parameters   :
//* Functions called    :
//*----------------------------------------------------------------------------
static void UiDriver_UpdateLcdFreq(ulong dial_freq,ushort color, ushort mode)
{
	uchar		digit_size;
	ulong		pos_y_loc;
	ulong		pos_x_loc;
	ulong		font_width;
	uint8_t*	digits_ptr;

	if(ts.frequency_lock)  	{ color = Grey; }	// Frequency is locked - change color of display

	if(mode == UFM_AUTOMATIC)
	{
		if   (is_splitmode())  { mode = UFM_SMALL_RX;}	// yes - update upper, small digits (receive frequency)
		else                   { mode = UFM_LARGE;   }	// NOT in split mode:  large, normal-sized digits
	}

	// if (mode != UFM_SECONDARY) {
									ts.refresh_freq_disp = true;  // } //because of coloured digits...

	if(ts.xverter_mode)	 	// transverter mode active?
	{
		dial_freq    *= (ulong)ts.xverter_mode;	// yes - scale by LO multiplier
		dial_freq    += ts.xverter_offset;	// add transverter frequency offset
		if(dial_freq  > 1000000000)		// over 1000 MHz?
		   dial_freq -= 1000000000;		// yes, offset to prevent overflow of display
		if(ts.xverter_mode && mode != UFM_SECONDARY)  color = Yellow;	// if in transverter mode, frequency is yellow unless we do the secondary display

	}

	// Handle frequency display offset in "CW RX" modes
	if(ts.dmod_mode == DEMOD_CW)	 		// In CW mode?
	{
		switch(ts.cw_offset_mode)
		{
			case CW_OFFSET_LSB_RX:	// Yes - In an LSB mode with display offset?
			case CW_OFFSET_USB_RX:	// In a USB mode with display offset?
			case CW_OFFSET_AUTO_RX:	// in "auto" mode with display offset?
									if (ts.cw_lsb) { dial_freq -= ts.cw_sidetone_freq; }  // yes - LSB - lower display frequency by sidetone amount
									else           { dial_freq += ts.cw_sidetone_freq; }  // yes - USB - raise display frequency by sidetone amount
									break;
		}
	}

	switch(mode)
	{
		case UFM_SMALL_RX:
							digits_ptr  = df.dial_digits;
							digit_size  = 0;
							pos_y_loc   = ts.Layout->TUNE_FREQ.y;
							pos_x_loc   = ts.Layout->TUNE_SPLIT_FREQ_X;
							font_width  = SMALL_FONT_WIDTH;
							break;
		case UFM_SMALL_TX:					// small digits in lower location
							digits_ptr  = df.dial_digits;
							digit_size  = 0;
							pos_y_loc   = ts.Layout->TUNE_SPLIT_FREQ_Y_TX;
							pos_x_loc   = ts.Layout->TUNE_SPLIT_FREQ_X;
							font_width  = SMALL_FONT_WIDTH;
							break;
		case UFM_SECONDARY:
							digits_ptr  = df.sdial_digits;
							digit_size  = 0;
							pos_y_loc   = ts.Layout->TUNE_SFREQ.y;
							pos_x_loc   = ts.Layout->TUNE_SFREQ.x;
							font_width  = SMALL_FONT_WIDTH;
							break;
		case UFM_LARGE:
							if ((ts.dynamic_tune_activ_counter > 0) && (ts.txrx_mode == TRX_MODE_RX))	{ color = White ; }  // Yellow; }

		default:			// default:  normal sized (large) digits

							digits_ptr  = df.dial_digits;
#ifdef USE_8bit_FONT
							digit_size  = ts.FreqDisplayFont==0?1:5;
#else
							digit_size  = 1;
#endif
							pos_y_loc   = ts.Layout->TUNE_FREQ.y;
							pos_x_loc   = ts.Layout->TUNE_FREQ.x;
							font_width  = LARGE_FONT_WIDTH;
	}
	// in SAM mode, never display any RIT etc., but
	// use small display for display of the carrier frequency that the PLL has locked to
	if((( (ts.dmod_mode == DEMOD_SAM)   && mode == UFM_SMALL_RX) || ((ts.dmod_mode == DEMOD_SAM)  && mode == UFM_SECONDARY)))
	{
		/*
		digits_ptr  = df.sdial_digits;
		digit_size  = 0;
		pos_y_loc   = ts.Layout->TUNE_SFREQ.y;
		pos_x_loc   = ts.Layout->TUNE_SFREQ.x;
		font_width  = SMALL_FONT_WIDTH;
		UiDriver_UpdateFreqDisplay(dial_freq + ads.carrier_freq_offset, digits_ptr, pos_x_loc, font_width, pos_y_loc, Yellow, digit_size);
		*/

		char outxt[12];
		snprintf( outxt, 12, "%5dHz",  ads.carrier_freq_offset );  /// NIZZZ
		UiLcdHy28_PrintTextRight( ts.Layout->SNAP_CARRIER.x + 31, ts.Layout->SNAP_CARRIER.y - 13, outxt, White, Black, 0);
	}
	 else    // pour ne pas afficher la 2em frequence en haut en minuscule
	{
		if (mode != UFM_SECONDARY) UiDriver_UpdateFreqDisplay(dial_freq, digits_ptr, pos_x_loc, font_width, pos_y_loc, color, digit_size);
	}
}
///********************************************************
///
void     NIZZ_UpdateLcdFreq(ulong dial_freq,ushort color,ushort mode)
{
	UiDriver_UpdateLcdFreq ( dial_freq, color, mode );
}
//*----------------------------------------------------------------------------
//* Function Name       : UiDriverChangeTuningStep
//* Object              : Change tunning step
//* Input Parameters    :
//* Output Parameters   :
//* Functions called    :
//*----------------------------------------------------------------------------
void UiDriver_ChangeTuningStep(uchar is_up)
{
	ulong 	idx = df.selected_idx;
	uint8_t idx_limit = T_STEP_MAX_STEPS -1;

	if((!ts.xvtr_adjust_flag) && (!ts.xverter_mode))
	{
		// are we NOT in "transverter adjust" or transverter mode *NOT* on?
		idx_limit = T_STEP_100KHZ_IDX;
	}

	if ( is_up )
	{
		idx = (idx>=idx_limit)?0:idx+1;
						// 9kHz step only on MW and LW
		if(idx == T_STEP_9KHZ_IDX && ((df.tune_old) > 1600001))
			idx ++;
	}
	else
	{
		idx = (idx==0)?idx_limit:idx-1;
						// 9kHz step only on MW and LW
		if(idx == T_STEP_9KHZ_IDX && ((df.tune_old) > 1600001))
			idx --;
	}

	df.tuning_step	= tune_steps[idx];  Gett_Tune_ordre_integer(df.tuning_step);
	df.selected_idx = idx;

	// Update step on screen
	UiDriver_DisplayFreqStepSize();

}


/*----------------------------------------------------------------------------
 * @brief Scans buttons 0-16:  0-15 are normal buttons, 16 is power button, 17 touch
 * @param button_num - 0-BUTTON_NUM - 1
 * @returns true if button is pressed
 */

static bool UiDriver_IsButtonPressed(uint32_t button_num)
{
	// FIXME: This is fragile code, as it depends on being called multiple times in short periods (ms)
	// This works, since regularily the button matrix is queried.
	/*
    if ( button_num == TOUCHSCREEN_ACTIVE )
    {
    	UiLcdHy28_TouchscreenDetectPress();
    	if ( mchf_touchscreen.state == TP_DATASETS_VALID)  return true ;
    } */

    UiLcdHy28_TouchscreenDetectPress();
	return Keypad_IsKeyPressed(button_num);
}
///*******************************************************************************
///
static void UiDriver_WaitForButtonPressed(uint32_t button_num)
{
    while (true)
    {
        Keypad_Scan();
        if ( UiDriver_IsButtonPressed(button_num) ) { break; }
        HAL_Delay(20);
    }
}
///*********************************************************************************
///


bool UiDriver_KeyboardProcessOldClicks()    // appelée systématiquement toutes les 10ms
{
    static  uchar    press_hold_release_delay = 0;  // temporisateur après relachement bouton pressé avec maintien ..
    static uint32_t  debounce_delay           = 100;


	if ( ! ks.button_execution_Wait ) // s'il n'y a pas d'exécution en cours d'une fonction de fond
		                              // attachée a un bouton ou une touche tactile
	{
		Keypad_Scan();               // balayer l etat physique GPIO's  de tous les boutons + IRQ-tactile

		if ( ! ks.button_pressed )   //  si'il n'y a pas eu de bouton déja pressé
		                             //  alors on test si il y a un nouveau bouton qui vient d'étre pressé
		{
			for ( int i =  0;  i < BUTTON_NUM; i++ )
			{
				if ( UiDriver_IsButtonPressed ( i ) )   // s'il y a un nouveau bouton pressé on initialise les
					                                    // indicateurs booleans et autres.
				{
					ks.button_pressed           = true;  // pour mentionner que le click est pris en compte et a suivre prochainement
					ks.button_id                = i;     // numero du buton  (17 = touchscreen tactil)
					ks.debounce_time            = 0;
					ks.debounce_checked         = 0;
					ks.press_holded             = 0;

					if (i==17)                          { debounce_delay =  77; ts.enable_display_on_screen = false; ts.FFT_sampling_enable = false;  sd.samp_ptr = 0; sd.state = 0; } // si touchescreen  ne plus afficher spectre sur ecrant
					else                                { debounce_delay = 100; }
					if ( ts.scope_scheduler     < 393 ) { ts.scope_scheduler     = 393; }  // pour retarder l'affichage du spectre
					if ( ts.waterfall.scheduler < 371 ) { ts.waterfall.scheduler = 371; }  // pour retarder l'affichage du spectre
														  ts.switch_pause        = 650;    // pour suspendre momentan�ment un calcul non prioritaire de reajustements des correction IQ

					ads.adc_quarter_clip        = false;  // ceci pour ne pas changer le gain Codec en cours de toushscreen
					ads.adc_half_clip           = 0;
					ads.adc_clip                = 0;
					break;    // sortir carrement de la boucke for
				}
			}
		}
		     // si il y a  un  bouton qui est  déja  pressé
		else if ( (!ks.debounce_checked)  && (ks.debounce_time >= debounce_delay ) )  // filtre de rebondissement 50ms environ
		{
			if ( UiDriver_IsButtonPressed ( ks.button_id ) )        // button still pressed?
			{
			 	ks.debounce_checked  = 1;       // indicate that the debounce check was completed
			}
			else     // Si Rebondissment alors on fait comme si rien ne s'est passé : on retourne au test clavier de depart
			{
				ks.button_pressed           = false;  // on retourne au test clavier de depart
				ts.enable_display_on_screen = true; ts.FFT_sampling_enable = true;  sd.samp_ptr = 0; sd.state = 0;
			}
		}
		else if ( (!ks.press_holded) && (ks.debounce_time >= 1000) )   // si le bouton est maintenu pressé au dela de 1 seconde
			                                                           // alors on lance la fonction de fond attachée
			                                                           // et on attend le relachement du buton avant de
			                                                           // retourner au test clavier de depart
		{
			ks.press_holded                = 1;
			ks.button_execution_Wait       = 1;   // on lance la fonction de fond attachée au bouton pressé avec maintien
			press_hold_release_delay       = 10; // PRESS_HOLD_RELEASE_DELAY_TIME; // Set up a bit of delay for when press-and-hold is released
		}
		else if ( ks.press_holded && (!UiDriver_IsButtonPressed(ks.button_id)) )   // on attend le relachement bouton pressé avec maintien
			                                                                       // avec une certaine temperisation
		{
			if ( press_hold_release_delay ) { press_hold_release_delay--;} // attente leger tempo  après relachement bouton pressé avec maintien
			else                            { ks.button_pressed    = 0;  } // Si tempo expiré après relachement bouton pressé avec maintien
				                                                           // alors On retourne  au test clavier de depart
		}
		else if ( !ks.press_holded &&  ( !UiDriver_IsButtonPressed (ks.button_id) )  ) // attente relachement bouton normal
			                                                                           // pressé sans  maintien
		{
			ks.button_execution_Wait  = 1;   // on lance la tache de fond attachée au bouton pressé
			ks.button_pressed         = 0;   // On retourne  au test clavier de depart

		//	if ( ts.scope_scheduler        < 40 )   ts.scope_scheduler     = 40;    //  pour retarder l'affichage du spectre
		//	if ( ts.waterfall.scheduler    < 70 )   ts.waterfall.scheduler = 70;    //  pour retarder l'affichage du spectre
			if (ts.switch_pause            < 300)   ts.switch_pause       += 300;   //  pour suspendre momentan�ment le calcul non prioritaire de reajustement des correction IQ, besoin du processeur

		}

		//
		// Handle press-and-hold tuning step adjustment
		//
		/*
		if ( (ts.tune_step != STEP_PRESS_OFF) && ( !ks.press_holded ) )    // are we in press-and-hold step size mode and did the button get released?
		{
			ts.tune_step      = STEP_PRESS_OFF;                     // yes, cancel offset
			df.selected_idx   = ts.tune_step_idx_holder;            // restore previous setting
			df.tuning_step    = tune_steps [ df.selected_idx ];  Gett_Tune_ordre_integer( df.tuning_step );
			UiDriver_DisplayFreqStepSize ();
		}*/
	}

    return     ks.button_pressed;
}

///*********************************************************************************************
///

enum TRX_States_t
{
	TRX_STATE_TX_TO_RX,//!< TRX_STATE_TX_TO_RX
	TRX_STATE_RX,      //!< TRX_STATE_RX
	TRX_STATE_RX_TO_TX,//!< TRX_STATE_RX_TO_TX
	TRX_STATE_TX,      //!< TRX_STATE_TX
};

static void UiDriver_TxRxUiSwitch(enum TRX_States_t state)
{
	static uchar enc_one_mode   =   ENC_ONE_MODE_AUDIO_GAIN;    // stores modes of encoder when we enter TX
	static uchar enc_two_mode   =   ENC_TWO_MODE_RF_GAIN;       // stores modes of encoder when we enter TX
	static uchar enc_three_mode =   ENC_THREE_MODE_CW_SPEED;    // stores modes of encoder when we enter TX

	{
		if(state == TRX_STATE_RX_TO_TX)
		{
			UiDriver_DeleteSMeterLabels();
			UiDriver_DrawPowerMeterLabels();

			if((ts.flags1 & FLAGS1_TX_AUTOSWITCH_UI_DISABLE) == false)                // If auto-switch on TX/RX is enabled
			{
				// change display related to encoder one to TX mode (e.g. Sidetone gain or Compression level)
				enc_one_mode   = ts.enc_one_mode;
				enc_two_mode   = ts.enc_two_mode;
				enc_three_mode = ts.enc_thr_mode;

				// we reconfigure the encoders according to the currently selected mode
				// for now this is only relevant for CW
				if (ts.dmod_mode    == DEMOD_CW)
				{
					ts.enc_one_mode  = ENC_ONE_MODE_ST_GAIN;
					ts.enc_thr_mode  = ENC_THREE_MODE_CW_SPEED;
				}
				else if (ts.dmod_mode == DEMOD_DIGI && ts.digital_mode == DigitalMode_BPSK)
				{
					ts.enc_one_mode = ENC_ONE_MODE_ST_GAIN;
					ts.enc_thr_mode = ENC_THREE_MODE_PSK_SPEED;
				}
				else // for all other modes we activate the compressor setting and input gain control
				{
					ts.enc_one_mode = ENC_ONE_MODE_CMP_LEVEL;
					ts.enc_thr_mode = ENC_THREE_MODE_INPUT_CTRL;
				}
			}

			ts.enc_thr_mode = ENC_THREE_MODE_RIT;  // pour forcer l'affichage du "POW" en 3em pav� en haut de l'ecran durant le Tx
			// force redisplay of Encoder boxes and values

			UiDriver_RefreshEncoderDisplay();
		}
		else if (state == TRX_STATE_TX_TO_RX)
		{
			ts.old_rfg_calc = 100;  // NIZZ pour forcer l'ecriture du  gain Codec du I/Q  Rx

			UiDriver_DeleteSMeterLabels();
			UiDriver_DrawSMeterLabels();
			if((ts.flags1 & FLAGS1_TX_AUTOSWITCH_UI_DISABLE) == false)                // If auto-switch on TX/RX is enabled
			{
				ts.enc_one_mode = enc_one_mode;
				ts.enc_two_mode = enc_two_mode;
				ts.enc_thr_mode = enc_three_mode;
			}

			// force redisplay of Encoder boxes and values
			UiDriver_RefreshEncoderDisplay();
		}
	}

	UiDriver_UpdateBtmMeter(0,0);        // clear bottom meter of any outstanding indication when going back to RX
	if((ts.menu_mode))              // update menu when we are (or WERE) in MENU mode
	{
		UiMenu_RenderMenu(MENU_RENDER_ONLY);
	}
}

/**
 * adjust volume and return to RX from TX and other time-related functions,
 * has to be called regularly
 */
static void UiDriver_TimeScheduler()
{
	static bool	 audio_spkr_volume_update_request = true;
	static bool  audio_spkr_delayed_unmute_active = false;

	static bool   old_squelch             = 0;	// used to detect change-of-state of squelch
	static bool   old_tone_det            = 0;	// used to detect change-of-state of tone decoder
	static bool   old_tone_det_enable     = 0;	// used to detect change-of-state of tone decoder enabling
	static bool   old_burst_active        = 0; // used to detect state of change of tone burst generator
	static bool   startup_done_flag       = 0;
	static bool	  dsp_rx_reenable_flag    = 0;
	static ulong  dsp_rx_reenable_timer   = 0;
	static enum   TRX_States_t last_state = TRX_STATE_RX; // we assume everything is
	       enum   TRX_States_t state;



	// let us figure out if we are in a stable state or if this
	// is the first run after a mode change
	if (ts.txrx_mode == TRX_MODE_TX)
	{
		if (last_state != TRX_STATE_TX)  { 	state = TRX_STATE_RX_TO_TX;	}
		else 							 {	state = TRX_STATE_TX;		}
		last_state = TRX_STATE_TX;
	}
	else
	{
		if (last_state != TRX_STATE_RX)
		{
			state       = TRX_STATE_TX_TO_RX;
		}
		else
		{
			state  = TRX_STATE_RX;
		}
		last_state = TRX_STATE_RX;
	}


	/*** RX MODE ***/
	if(ts.txrx_mode == TRX_MODE_RX)
	{
		if (state == TRX_STATE_TX_TO_RX)
		{
			audio_spkr_delayed_unmute_active = true;
		}

		if(audio_spkr_delayed_unmute_active  && ts.audio_spkr_unmute_delay_count == 0)	 	// did timer hit zero
		{
			audio_spkr_delayed_unmute_active = false;
			audio_spkr_volume_update_request = true;
		}



		audio_spkr_volume_update_request |= ts.rx_gain[RX_AUDIO_SPKR].value != ts.rx_gain[RX_AUDIO_SPKR].value_old;


		if( audio_spkr_volume_update_request)	 	//   in normal mode - calculate volume normally
		{

			ts.rx_gain[RX_AUDIO_SPKR].value_old    = ts.rx_gain[RX_AUDIO_SPKR].value;
			ts.rx_gain[RX_AUDIO_SPKR].active_value = 1;		// software gain not active - set to unity

			float32_t  vvolume =  ts.rx_gain[RX_AUDIO_SPKR].value; // NIZZZ  ajout� pour affiner la resolution du son

			int8_t ggg = (int) (vvolume);
			//if (ts.last_volume == 0)  ts.cmpt_volume = 13;
			//if (ts.cmpt_volume  > 0)  ts.cmpt_volume--;
			//ts.last_volume = ggg;

			// ts.audio_processor_input_mute_counter =  0 ;  // Mute entr�e Codec vers Rx_processing
			// ts.audio_dac_muting_buffer_count      =  0;

			//if ( ts.cmpt_volume == 0 )
			{
				 if      ((ts.last_volume <  15) && (ggg >=15))             ts.last_volume = 15;
				 else if ( ts.last_volume < ggg)               {  ts.last_volume++;  if (( ts.last_volume < ggg) && ( ts.last_volume < 22))  ts.last_volume++; }
				 else if ( ts.last_volume > ggg)               {  ts.last_volume = ggg; }


				if ( ts.last_volume   <= CODEC_SPEAKER_MAX_VOLUME )  		    // Note:  Gain > 16 adjusted in audio_driver.c via software
				{
					Codec_VolumeSpkr ( ts.last_volume );
				}
				else  	// are we in the "software amplification" range?
				{
					Codec_VolumeSpkr ( CODEC_SPEAKER_MAX_VOLUME );	 	// set to fixed "maximum" gain
					ts.rx_gain [ RX_AUDIO_SPKR ].active_value = (uint8_t) (( (float) (ts.last_volume)/8) - 4);	// 2.5/5.35 to float , toujours > 1
				}

				if ( ts.last_volume == ggg )  // si on a fini de mettre a jour progressivement le volume du speaker
				{
					audio_spkr_volume_update_request = false; // si on atteind le volume voulu
				}

				if ( ts.last_volume > ( ggg - 11 ) )              // 12
				{
					dsp_rx_reenable_flag  = true;		       // indicate that we need to re-enable the DSP soon
					dsp_rx_reenable_timer = ts.sysclock - 1 ;  // DSP_REENABLE_DELAY; // 13	// ( unit� de 10ms)  establish time at which we re-enable the DSP
				}
			}
		}

		// Check to see if we need to re-enable DSP after return to RX
		if ( dsp_rx_reenable_flag )	 	// have we returned to RX after TX?
		{
			if ( ts.sysclock > dsp_rx_reenable_timer )	 	// yes - is it time to re-enable DSP?
			{
				ts.dsp_inhibit       = false;	// yes - re-enable DSP
				dsp_rx_reenable_flag = false;	// clear flag so we don't do this again
			}
		}

		// update the on-screen indicator of squelch/tone detection (the "FM" mode text) if there is a change of state of squelch/tone detection
		if ( ( old_squelch != ads.fm_squelched ) || ( old_tone_det        != ads.fm_subaudible_tone_detected        )
												 || ( old_tone_det_enable != (bool)ts.fm_subaudible_tone_det_select )) // did the squelch or tone detect state just change?
		{

			UiDriver_DisplayDemodMode();                           // yes - update on-screen indicator to show that squelch is open/closed
			old_squelch         = ads.fm_squelched;
			old_tone_det        = ads.fm_subaudible_tone_detected;
			old_tone_det_enable = (bool)ts.fm_subaudible_tone_det_select;
		}
	}

	/*** TX MODE ONLY ***/
	if ( ts.txrx_mode == TRX_MODE_TX )
	{

		if( ( state == TRX_STATE_RX_TO_TX ) )
		{
			// we just switched to TX
			if((ts.dmod_mode != DEMOD_CW))        // did we just enter TX mode in voice mode?
			{
				ads.alc_val    = 1; // re-init AGC value
				ads.peak_audio = 0; // clear peak reading of audio meter
			}
		}


		// Has the timing for the tone burst expired?
		if ( ts.sysclock > ts.fm_tone_burst_timing )
		{
			ads.fm_tone_burst_active = 0;               // yes, turn the tone off
		}

		if ( ads.fm_tone_burst_active != old_burst_active )       // did the squelch or tone detect state just change?
		{
			UiDriver_DisplayDemodMode();                           // yes - update on-screen indicator to show that tone burst is on/off
			old_burst_active = ads.fm_tone_burst_active;
		}
	}

	/*** TX+RX STATE CHANGE ONLY ***/
	// if we do change modes, some visuals need an update
	if(state == TRX_STATE_RX_TO_TX || state == TRX_STATE_TX_TO_RX)
	{
		// now update display according to the changed state
		UiDriver_TxRxUiSwitch( state );
	}

	/*** ALWAYS ***/
	UiDriver_LcdBlankingProcessTimer();


	// This delays the start-up of the DSP for several seconds to minimize the likelihood that the LMS function will get "jammed"
	// and stop working.


    //****************************************************************************************//
	//************************* ONCE AFTER STARTUP DELAY *************************************//
	//****************************************************************************************//
	if( startup_done_flag == false && (ts.sysclock > DSP_STARTUP_DELAY))       // has it been long enough after startup?
	{
		startup_done_flag = true;                  // set flag so that we do this only once

		UiDriver_DisplayEncoderTwoMode();

		ts.dsp_inhibit = 0;                         // allow DSP to function

		audio_spkr_volume_update_request = 1;      // set unmute flag to force audio to be un-muted - just in case it starts up muted!

		Codec_MuteDAC( false );                      // make sure that audio is un-muted
	}
}


/*
 * Tells you which SSB Demod Mode is the preferred one for a given frequency in Hertz
 */
//*----------------------------------------------------------------------------
//* Function Name       : UiDriverChangeDemodMode
//* Object              : change demodulator mode
//* Input Parameters    : "noskip", if TRUE, disabled modes are to be included
//* Output Parameters   :
//* Functions called    :
//*----------------------------------------------------------------------------

/**
 * This function is responsible for make the changes to the UI layout
 * as required for a give new mode, such as enabling the right set of encoder boxes etc.
 */
typedef struct
{
	int16_t encoder_modes[3];
} encoder_mode_store_t;

void UiDriver_SetDemodMode(uint8_t new_mode)
{
	RadioManagement_SetDemodMode(new_mode);

#if 0
	static encoder_mode_store_t demod_modes[] =
	{
			{ ENC_ONE_MODE_AUDIO_GAIN, ENC_TWO_MODE_RF_GAIN   , ENC_THREE_MODE_RIT      }, // USB, LSB,(S)AM,FM,FreeDV
			{ ENC_ONE_MODE_ST_GAIN   ,                      -1, ENC_THREE_MODE_CW_SPEED }, // CW
			{ ENC_ONE_MODE_RTTY_SPEED, ENC_TWO_MODE_RTTY_SHIFT, -1                      }, // RTTY
	};
#endif

	switch(ts.dmod_mode)
	{
	case DEMOD_DIGI:
					{
						switch(ts.digital_mode)
						{
						case DigitalMode_RTTY:
							if (ts.enc_one_mode != ENC_ONE_MODE_AUDIO_GAIN)
							{
								ts.enc_one_mode = ENC_ONE_MODE_RTTY_SPEED;
							}

							if (ts.enc_two_mode != ENC_TWO_MODE_RF_GAIN)
							{
								ts.enc_two_mode = ENC_TWO_MODE_RTTY_SHIFT;
							}
							break;

						case DigitalMode_BPSK:
							if (ts.enc_thr_mode != ENC_THREE_MODE_RIT)
							{
								ts.enc_thr_mode = ENC_THREE_MODE_PSK_SPEED;
							}
							break;
						}
					}
					break;

	case DEMOD_CW:
					{
						if (ts.enc_one_mode != ENC_ONE_MODE_AUDIO_GAIN)
						{
							ts.enc_one_mode = ENC_ONE_MODE_ST_GAIN;
						}
						if (ts.enc_thr_mode != ENC_THREE_MODE_RIT)
						{
							ts.enc_thr_mode = ENC_THREE_MODE_CW_SPEED;
						}
					}
					break;

	default:
					if (ts.enc_thr_mode != ENC_THREE_MODE_RIT)
					{
						ts.enc_thr_mode = ENC_THREE_MODE_INPUT_CTRL;
					}
					break;
	}
	UiDriver_UpdateDisplayAfterParamChange();
}

static void UiDriver_ChangeToNextDemodMode(bool select_alternative_mode)
{
	uint8_t  new_mode = ts.dmod_mode;	// copy to local, so IRQ is not affected
	if (select_alternative_mode)   	{  new_mode = RadioManagement_NextAlternativeDemodMode(new_mode); }
	else	                        {  new_mode = RadioManagement_NextNormalDemodMode     (new_mode); }

	// TODO: We call this always, since we may have switched sidebands or the digital mode
	// if we would remember that, we would decide if to call this.
	UiDriver_SetDemodMode(new_mode);
}

/**
 * @brief band change
 * @param vfo_sel	VFO A/B
 * @param curr_band_index
 * @param new_band_index
 */
void UiDriver_UpdateBand(uint16_t vfo_sel, uint8_t curr_band_index, uint8_t new_band_index)
{

		// TODO: There is a strong similarity to code in UiDriverProcessFunctionKeyClick around line 2053
		// Load frequency value - either from memory or default for
		// the band if this is first band selection
		vfo [vfo_sel].band[curr_band_index].dial_value = df.tune_new;   /// NIZZZ pour sauvegarder la derni�re frequence de la bande � quitter
		vfo [vfo_sel].band[curr_band_index].decod_mode = ts.dmod_mode;

		if( vfo [vfo_sel].band[new_band_index].dial_value != 0xFFFFFFFF )
		{
			df.tune_new = vfo[vfo_sel].band[new_band_index].dial_value;	// Load value from VFO
		}
		else
		{
			df.tune_new = bandInfo[curr_band_index].tune; 			// Load new frequency from startup
		}

		bool      new_lsb          = RadioManagement_CalculateCWSidebandMode();
		uint16_t  new_dmod_mode    = vfo[vfo_sel].band[new_band_index].decod_mode;
		uint16_t  new_digital_mode = vfo[vfo_sel].band[new_band_index].digital_mode;
		bool      isNewDigitalMode = ts.digital_mode != new_digital_mode && new_dmod_mode == DEMOD_DIGI;


		// we need to mute here since changing bands may cause audible click/pops
		RadioManagement_MuteTemporarilyRxAudio();

		ts.digital_mode = new_digital_mode;

		if(ts.dmod_mode != new_dmod_mode || (new_dmod_mode == DEMOD_CW && ts.cw_lsb != new_lsb) || isNewDigitalMode)
		{
			// Update mode
			ts.cw_lsb = new_lsb;
			RadioManagement_SetDemodMode ( new_dmod_mode );
		}

		// Finally update public flag
		ts.band = new_band_index;

		UiDriver_UpdateDisplayAfterParamChange();    // because mode/filter may have changed
		UiVk_Redraw();		//virtual keypads call (refresh purpose)
		// RadioManagement_SetBandPowerFactor(ts.band); // inutile puisque ceci se fait aussi lors du switch Rx-->Tx
}

/**
 * @brief initiate band change.
 * @param is_up select the next higher band, otherwise go to the next lower band
 */
static void UiDriver_ChangeBand(uchar is_up)
{

	// Do not allow band change during TX
	if(ts.txrx_mode != TRX_MODE_TX)
	{



		uint16_t vfo_sel = is_vfo_b()?VFO_B:VFO_A;

		uint8_t curr_band_index = ts.band; // index in band table of currently selected band


		// Save old band values
		if(curr_band_index < (MAX_BANDS) && ts.cat_band_index == 255)
		{
			// Save dial
			vfo[vfo_sel].band[curr_band_index].dial_value   = df.tune_old;
			vfo[vfo_sel].band[curr_band_index].decod_mode   = ts.dmod_mode;
			vfo[vfo_sel].band[curr_band_index].digital_mode = ts.digital_mode;
		}
		else
		{
			ts.cat_band_index = 255;
		}

		uint8_t   new_band_index = curr_band_index;     // index of the new selected band
		// in case of no other band enabled, we stay in this band

		// Handle direction
		if(is_up)
		{
		    // we start checking the index following the current one
		    // until we reach an enabled band
		    for (int idx  = 1; idx <= MAX_BANDS; idx++)
		    {
		        uint32_t test_idx = (curr_band_index + idx) % MAX_BANDS;
		        if (vfo[vfo_sel].enabled[test_idx])
		        {
		            new_band_index = test_idx;
		            break; // we found the first enabled band following the current one
		        }
		    }
		}
		else
		{
            // we start checking the index before the current one
            // until we reach an enabled band
            for (int idx = MAX_BANDS-1; idx >= 0; idx--)
            {
                uint32_t test_idx = (curr_band_index + idx) % MAX_BANDS;
                if (vfo[vfo_sel].enabled[test_idx])
                {
                    new_band_index = test_idx;
                    break; // we found the first enabled band before the current one
                }
            }
		}

		UiDriver_UpdateBand(vfo_sel, curr_band_index, new_band_index);
	}
}
									   // Speed 1,  2,   3,   4,   5,   6,  7,

static int16_t  enc_multiplier_Array[7][7] = {{ 1, 10,  50, 100, 200, 250, 300},  //    1 Hz   NIZZZ
											  { 1, 10,  20,  30,  40,  60, 120},  //    5 Hz
											  { 1, 10,  20,  30,  40,  50,  60},  //   10 Hz
											  { 1,  1,   2,   3,   4,   5,  10},  //  100 Hz
											  { 1,  1,   2,   3,   4,   5,   6},  //  500 Hz
											  { 1,  1,   1,   2,   3,   4,   5},  //   1 Khz
											  { 1,  1,   1,   2,   2,   2,   3}}; //   5 Khz

/**
 * @brief Read out the changes in the frequency encoder and initiate frequency change by setting a global variable.
 *
 * @returns true if a frequency change was detected and a new tuning frequency was set in a global variable.
 */
static bool UiDriver_CheckFrequencyEncoder()
{
	int 		    pot_diff;
	bool		    retval                   = true;
	       int	    enc_multiplier;
	static int      last_Skeep_freq          = 10;
	static float 	enc_speed_avg            = 0.0;  //keeps the averaged encoder speed
	int		        delta_t, enc_speed;
	static bool     first_dynamic_tune_activ = true;

	pot_diff = UiDriverEncoderRead( ENCFREQ );


	if (pot_diff != 0)
	{
		delta_t              = ts.audio_int_counter;  // get ticker difference since last enc. change
		ts.audio_int_counter = 0;		              //reset tick counter

		UiDriver_LcdBlankingStartTimer();	// calculate/process LCD blanking timing

	}

	if (    pot_diff                  != 0
			&& ((ts.txrx_mode         == TRX_MODE_RX )
#ifndef TUNISIANN
			|| ts.tune
#endif
			)
			// && ks.button_continued_pressed == false   // NIZZZ on l'a omis pour voir si elle est indispensable
			&& ts.frequency_lock      == false        ) // allow tuning only if in rx mode, no freq lock,
	{

		if (delta_t > 300)
		{
			enc_speed_avg = 0;    //when leaving speedy turning set avg_speed to 0
		}

		enc_speed = div(4000,delta_t).quot * pot_diff;  // app. 4000 tics per second -> calc. enc. speed.

		     if (enc_speed >  500) { enc_speed =  500;}   //limit calculated enc. speed
		else if (enc_speed < -500) { enc_speed = -500;}

		enc_speed_avg = 0.1 * enc_speed + 0.9 * enc_speed_avg; // averaging to smooth encoder speed

		enc_multiplier = 1; //set standard speed

		if (((ts.flags1 & FLAGS1_DYN_TUNE_ENABLE) || (df.tuning_step == 5) || (df.tuning_step == 10)) &&  (! ts.tune) )   // check if dynamic tuning has been activated by touchscreen
		{

			uint16_t speed_ordre = 0;
			if      ((enc_speed_avg <  10) && (enc_speed_avg > ( -10))) { speed_ordre = 0; first_dynamic_tune_activ = true;} // NIZZZ
			else if ((enc_speed_avg <  45) && (enc_speed_avg > (- 45))) { speed_ordre = 1;}
			else if ((enc_speed_avg <  70) && (enc_speed_avg > (- 70))) { speed_ordre = 2;}
			else if ((enc_speed_avg <  95) && (enc_speed_avg > (- 95))) { speed_ordre = 3;}
			else if ((enc_speed_avg < 130) && (enc_speed_avg > (-130))) { speed_ordre = 4;}
			else if ((enc_speed_avg < 170) && (enc_speed_avg > (-170))) { speed_ordre = 5;}
			else                                                        { speed_ordre = 6;}

			if (speed_ordre)  { ts.dynamic_tune_activ_counter = 35;}
			else              { ts.dynamic_tune_activ_counter =  0;}
			if ((first_dynamic_tune_activ ) && ( speed_ordre > 0))  // pour anticiper l'affichage Jaune de la frequence avant le dynamic tune
			{
				first_dynamic_tune_activ = false;
				speed_ordre              =     0;
			}

			enc_multiplier =  enc_multiplier_Array[step_ordre][speed_ordre];



		/*	if ((enc_speed_avg > 50) || (enc_speed_avg < (-50)))
			{
				enc_multiplier = 2;   //10;    // turning medium speed -> increase speed by 10
				if (df.tuning_step <= 10)    { enc_multiplier = 5; }
				if (df.tuning_step >= 10000) { enc_multiplier = 1; }
			}

			if ((enc_speed_avg > 100) || (enc_speed_avg < (-120)))
			{
				enc_multiplier = 5;  // 40;    //turning fast speed -> increase speed by 100
			}

			if ((enc_speed_avg > 300) || (enc_speed_avg < (-300)))
			{
				enc_multiplier = 20;  /// 100;    //turning fast speed -> increase speed by 100
			}

			if ((df.tuning_step >= 500) && (enc_multiplier > 2))
			{
				enc_multiplier = 2;    //limit speed to 100000kHz/step
			}

			if ((df.tuning_step >= 10000) && (enc_multiplier > 1))
			{
				enc_multiplier = 1;    //limit speed to 100000kHz/step
			} */

		}


		// Finally convert to frequency incr/decr
		int32_t  Skeep_freq =  df.tuning_step * enc_multiplier ;
#ifndef TUNISIANN
		if ( ts.tune )  Skeep_freq = 5000;  // un saut de 5Khz en cas de mesure de SWR en fonction de la bande  avec la fonction tune en cours de Tx
#endif
		if ( pot_diff > 0 )
		{
			df.tune_new += Skeep_freq;
			//itoa(enc_speed,num,6);
			//UiSpectrumClearDisplay();			// clear display under spectrum scope
			//UiLcdHy28_PrintText(110,156,num,Cyan,Black,0);
			if (Skeep_freq > last_Skeep_freq)
			{
				df.tune_new = Skeep_freq * div(df.tune_new,Skeep_freq).quot;    // keep last digit to zero
			}
		}
		else
		{
			df.tune_new -= Skeep_freq;

			if (Skeep_freq > last_Skeep_freq)
			{
				if ( div( df.tune_new , Skeep_freq ).rem != 0) df.tune_new += Skeep_freq;
				df.tune_new = Skeep_freq * div( df.tune_new , Skeep_freq ).quot;    /// keep last digit to zero
			}
		}
		last_Skeep_freq = Skeep_freq;
		retval          = false;
	}
	return retval;
}



//*----------------------------------------------------------------------------
//* Function Name       : UiDriverCheckEncoderOne
//* Object              :
//* Input Parameters    :
//* Output Parameters   :
//* Functions called    :
//*----------------------------------------------------------------------------

static void UiDriver_CheckEncoderOne()
{
	int32_t pot_diff = UiDriverEncoderRead(ENC1);

	if (pot_diff)
	{
		int8_t pot_diff_step = pot_diff < 0?-1:1;

		UiDriver_LcdBlankingStartTimer();	// calculate/process LCD blanking timing
		// Take appropriate action
		switch(ts.enc_one_mode)
		{
			case ENC_ONE_MODE_RTTY_SPEED:
									// Convert to Audio Gain incr/decr
									rtty_ctrl_config.speed_idx = change_and_limit_int(rtty_ctrl_config.speed_idx,pot_diff_step,0,RTTY_SPEED_NUM-1);
									RttyDecoder_Init();
									UiDriver_DisplayRttySpeed(true);
									break;
									// Update audio volume
			case ENC_ONE_MODE_AUDIO_GAIN:
									if (ts.RFG_wait >400) ts.RFG_wait = 400;
									ts.rx_gain[RX_AUDIO_SPKR].value = (uint8_t) change_and_limit_uint(ts.rx_gain[RX_AUDIO_SPKR].value, pot_diff_step, 0, ts.rx_gain[RX_AUDIO_SPKR].max);
									UiDriver_DisplayAfGain(1);
									break;
			case ENC_ONE_MODE_ST_GAIN:
									ts.cw_sidetone_gain = change_and_limit_uint(ts.cw_sidetone_gain,pot_diff_step,0,SIDETONE_MAX_GAIN);
									Codec_TxSidetoneSetgain(ts.txrx_mode);
									UiDriver_DisplaySidetoneGain(true);
									break;
			case ENC_ONE_MODE_CMP_LEVEL:
									ts.tx_comp_level = change_and_limit_int  ( (int) ts.tx_comp_level, pot_diff_step, 0, 1);
									AudioManagement_CalcTxCompLevel();		// calculate values for selection compression level
									UiDriver_DisplayCmpLevel(true);	        // update on-screen display
									ts.AFG_wait = 9000;
									break;
			default:   				break;
		}
	}
}
//
//*----------------------------------------------------------------------------
//* Function Name       : UiDriverCheckEncoderTwo
//* Object              :
//* Input Parameters    :
//* Output Parameters   :
//* Functions called    :
//*----------------------------------------------------------------------------
static void UiDriver_CheckEncoderTwo()
{
	int32_t pot_diff = UiDriverEncoderRead(ENC2);

	if (pot_diff != 0)
	{
		UiDriver_LcdBlankingStartTimer();	// calculate/process LCD blanking timing

		if(ts.menu_mode)
		{
			UiMenu_RenderChangeItem(pot_diff);
		}
		else
		{
			int8_t pot_diff_step = pot_diff < 0?-1:1;


			if(ts.txrx_mode == TRX_MODE_RX)
			{
				// dynamic encoder speed , used for notch and peak
				static float    enc_speed_avg = 0.0;  //keeps the averaged encoder speed
				int             delta_t, enc_speed;
				float32_t       enc_multiplier;

				delta_t = ts.audio_int_counter;  // get ticker difference since last enc. change
				ts.audio_int_counter = 0;        //reset tick counter

				if (delta_t > 300)
				{
					enc_speed_avg = 0;    //when leaving speedy turning set avg_speed to 0
				}

				enc_speed = div(4000,delta_t).quot*pot_diff;  // app. 4000 tics per second -> calc. enc. speed.

				if (enc_speed >  500) {	enc_speed =  500; }   //limit calculated enc. speed
				if (enc_speed < -500) {	enc_speed = -500; }

				enc_speed_avg  = 0.1*enc_speed + 0.9*enc_speed_avg; // averaging to smooth encoder speed

				enc_multiplier = 1; //set standard speed

				if ((enc_speed_avg >  80) || (enc_speed_avg < (-80 ))) 	{ enc_multiplier = 10; }  // turning medium speed -> increase speed by 10
				if ((enc_speed_avg > 150) || (enc_speed_avg < (-150))) 	{ enc_multiplier = 30; }  // turning fast speed -> increase speed by 100
				if ((enc_speed_avg > 300) || (enc_speed_avg < (-300)))	{ enc_multiplier = 100;}  // turning fast speed -> increase speed by 100

				// used for notch and peak
				float32_t MAX_FREQ = 5000.0;

				if      (FilterPathInfo[ts.filter_path].sample_rate_dec == RX_DECIMATION_RATE_24KHZ) 	{ MAX_FREQ = 10000.0; }
				else if (FilterPathInfo[ts.filter_path].sample_rate_dec == RX_DECIMATION_RATE_12KHZ)	{ MAX_FREQ =  5000.0; }

				if ( ts.enc_two_mode != ENC_TWO_MODE_RF_GAIN ) ts.RFG_wait = 9000; // tempo avant retour affichage "RFG"

				switch(ts.enc_two_mode)
				{
					case ENC_TWO_MODE_RTTY_SHIFT:
							rtty_ctrl_config.shift_idx = change_and_limit_int ( rtty_ctrl_config.shift_idx, pot_diff_step, 0, RTTY_SHIFT_NUM - 1 );
							RttyDecoder_Init();
							UiDriver_DisplayRttyShift(1);
							break;

					case ENC_TWO_MODE_RF_GAIN:  // AGC
							if(ts.dmod_mode != DEMOD_FM)	 	// is this *NOT* FM?  Change AGC gain
							{

									ts.agc_wdsp_thresh = change_and_limit_int(ts.agc_wdsp_thresh,pot_diff_step,-20,120);
									AudioDriver_SetupAgcWdsp();
							}
							else	 		// it is FM - change squelch setting
							{
								ts.fm_sql_threshold = change_and_limit_uint(ts.fm_sql_threshold,pot_diff_step,0,FM_SQUELCH_MAX);
							}

							UiDriver_DisplayRfGain(1);    // change on screen
							break;

						// Update DSP/NB setting
					case ENC_TWO_MODE_SIG_PROC:
								// Signal processor setting
								// this is AGC setting OR noise blanker setting
								if(is_dsp_nb()) // noise blanker is active (ts.nb_setting > 0)
								{
									ts.nb_setting        = change_and_limit_uint ( ts.nb_setting, pot_diff_step, 0, MAX_NB_SETTING );
									ts.nb_setting_float  = ( 15 - ts.nb_setting ) * 0.5  + 3 ;
								}
								else // AGC mode setting
								{
									//                    ts.agc_wdsp_tau_decay = change_and_limit_int(ts.agc_wdsp_tau_decay,pot_diff_step * 100,100,5000);
									ts.agc_wdsp_mode = change_and_limit_uint( ts.agc_wdsp_mode,pot_diff_step, 0, 5);
									ts.agc_wdsp_switch_mode = 1; // set flag, so that mode switching really takes place in AGC_prep
									AudioDriver_SetupAgcWdsp();
								}
								UiDriver_DisplayNoiseBlanker(1);
								break;

					case ENC_TWO_MODE_NR:
								if (is_dsp_nr())        // only allow adjustment if DSP NR is active
								{	//
									uint8_t nr_step = DSP_NR_STRENGTH_STEP;
									if(ts.dsp_nr_strength >= 180) 	{ nr_step = 1; }
									ts.dsp_nr_strength = change_and_limit_uint(ts.dsp_nr_strength,pot_diff_step * nr_step,DSP_NR_STRENGTH_MIN,DSP_NR_STRENGTH_MAX);
									if(ts.dsp_nr_strength > 200)  { 	ts.dsp_nr_strength = 200; }

									// this causes considerable noise
									//AudioDriver_SetRxAudioProcessing(ts.dmod_mode, false);
									// we do this instead
									ts.nr_alpha = 0.000 + (((float32_t)ts.dsp_nr_strength + 00) /200.1);
								}
								// Signal processor setting
								UiDriver_DisplayDSPMode(1);
								break;

					case ENC_TWO_MODE_NOTCH_F:
										if (is_dsp_mnotch())   // notch f is only adjustable when notch is enabled
										{
											if(pot_diff < 0)
											{
												ts.notch_frequency = ts.notch_frequency - 5.0 * enc_multiplier;
											}
											if(pot_diff > 0)
											{
												ts.notch_frequency = ts.notch_frequency + 5.0 * enc_multiplier;
											}

											if(ts.notch_frequency > MAX_FREQ)
											{
												ts.notch_frequency = MAX_FREQ;
											}
											if(ts.notch_frequency < MIN_PEAK_NOTCH_FREQ)
											{
												ts.notch_frequency = MIN_PEAK_NOTCH_FREQ;
											}
											// display notch frequency
											// set notch filter instance
											AudioDriver_SetRxAudioProcessing(ts.dmod_mode, false);
											UiDriver_DisplayDSPMode(1);
										}
										break;

					case ENC_TWO_MODE_BASS_GAIN:
										ts.bass_gain = change_and_limit_int ( ts.bass_gain, pot_diff_step, MIN_BASS, MAX_BASS );
										// set filter instance
										AudioDriver_SetRxAudioProcessing ( ts.dmod_mode, false );
										// display bass gain
										UiDriver_DisplayTone(true);
										break;

					case ENC_TWO_MODE_TREBLE_GAIN:
										ts.treble_gain = change_and_limit_int ( ts.treble_gain, pot_diff_step, MIN_TREBLE, MAX_TREBLE );
										// set filter instance
										AudioDriver_SetRxAudioProcessing ( ts.dmod_mode, false );
										// display treble gain
										UiDriver_DisplayTone( true );
										break;

					case ENC_TWO_MODE_PEAK_F:
									if (is_dsp_mpeak())   // peak f is only adjustable when peak is enabled
									{
										if(pot_diff < 0)
										{
											ts.peak_frequency = ts.peak_frequency - 5.0 * enc_multiplier;
										}
										if(pot_diff > 0)
										{
											ts.peak_frequency = ts.peak_frequency + 5.0 * enc_multiplier;
										}
										if(ts.peak_frequency > MAX_FREQ)
										{
											ts.peak_frequency = MAX_FREQ;
										}
										if(ts.peak_frequency < MIN_PEAK_NOTCH_FREQ)
										{
											ts.peak_frequency = MIN_PEAK_NOTCH_FREQ;
										}
										// set notch filter instance
										AudioDriver_SetRxAudioProcessing(ts.dmod_mode, false);
										// display peak frequency
										UiDriver_DisplayDSPMode(1);
									}
									break;
					default: 		break;
					}
				}
				else { // in TX case only bass & treble gain can be adjusted with encoder TWO

					// Take appropriate action
					switch(ts.enc_two_mode)
					{
						case ENC_TWO_MODE_BASS_GAIN:
											ts.tx_bass_gain = change_and_limit_int(ts.tx_bass_gain,pot_diff_step,MIN_TX_BASS,MAX_TX_BASS);
											// set filter instance
											AudioDriver_SetRxAudioProcessing(ts.dmod_mode, false);
											// display bass gain
											UiDriver_DisplayTone(true);
											break;

						case ENC_TWO_MODE_TREBLE_GAIN:
											ts.tx_treble_gain = change_and_limit_int(ts.tx_treble_gain,pot_diff_step, MIN_TX_TREBLE, MAX_TX_TREBLE);
											// set filter instance
											AudioDriver_SetRxAudioProcessing(ts.dmod_mode, false);
											// display treble gain
											UiDriver_DisplayTone(true);
											break;

						default: 			break;
				}
			}
		}
	}
}

//
//*----------------------------------------------------------------------------
//* Function Name       : UiDriverCheckEncoderThree
//* Object              :
//* Input Parameters    :
//* Output Parameters   :
//* Functions called    :
//*----------------------------------------------------------------------------
static void UiDriver_CheckEncoderThree()
{
	int 	pot_diff;

	pot_diff = UiDriverEncoderRead(ENC3);


	if (pot_diff)
	{
		int8_t pot_diff_step = pot_diff < 0?-1:1;

		UiDriver_LcdBlankingStartTimer();	// calculate/process LCD blanking timing
		if (filter_path_change)
		{
			AudioFilter_NextApplicableFilterPath(PATH_ALL_APPLICABLE | (pot_diff < 0?PATH_DOWN:PATH_UP),AudioFilter_GetFilterModeFromDemodMode(ts.dmod_mode),ts.filter_path);
			// we store the new filter in the current active filter location
			AudioDriver_SetRxAudioProcessing(ts.dmod_mode, false);
			// we activate it (in fact the last used one, which is the newly selected one);

			UiDriver_UpdateDisplayAfterParamChange();
		}
		else  if(ts.menu_mode)
		{
			UiMenu_RenderChangeItemValue(pot_diff);
		}
		else
		{
			// Take appropriate action
			switch(ts.enc_thr_mode)
			{
			// Update RIT value
			case ENC_THREE_MODE_RIT:
								if(ts.txrx_mode == TRX_MODE_RX)
								{
									int16_t old_rit_value = ts.rit_value;
									ts.rit_value = change_and_limit_int ( ts.rit_value, pot_diff_step, MIN_RIT_VALUE, MAX_RIT_VALUE);

									ts.dial_moved = ts.rit_value != old_rit_value;

									// Update RIT
									UiDriver_DisplayRit(1);
									// Change frequency
									UiDriver_FrequencyUpdateLOandDisplay(false);
								}
								else if(ts.txrx_mode == TRX_MODE_TX)
								{
									volatile uint8_t* adj_ptr;
									if ( RadioManagement_GetBand(df.tune_old) < MAX_BANDS) // dans les bandes
									{
										adj_ptr = &ts.pwr_adj [ts.power_level == PA_LEVEL_FULL?ADJ_FULL_POWER:ADJ_5W][ts.band];
									}
									else // en dehors des bandes
									{
										adj_ptr = &ts.pwr_out_of_band_factor;
									}

								/*	    if((band_mode == RadioManagement_GetBand(df.tune_old)) && (ts.power_level == pa_level))
										{
											tchange = UiDriverMenuItemChangeUInt8(var, mode, adj_ptr,
																				  TX_POWER_FACTOR_MIN,
																				  RadioManagement_IsPowerFactorReduce(df.tune_old)?TX_POWER_FACTOR_MAX:TX_POWER_FACTOR_MAX/2,

																				  TX_POWER_FACTOR_MIN,
																				  1
																				 );

											if(tchange)	{  RadioManagement_SetBandPowerFactor(ts.band); }	// yes, update the power factor if any adjustement changes

										} */
									ts.current_pwr_adj = *adj_ptr;
									ts.current_pwr_adj = change_and_limit_uint (  ts.current_pwr_adj, pot_diff_step, 3, 110 );
									*adj_ptr           = ts.current_pwr_adj;
									UiDriver_DisplayPowerAdjust ( 1 );
									RadioManagement_SetBandPowerFactor ( ts.band );
								}
								break;
				// Keyer speed
			case ENC_THREE_MODE_CW_SPEED:
								// Convert to Audio Gain incr/decr
								ts.cw_keyer_speed = change_and_limit_int(ts.cw_keyer_speed,pot_diff_step,CW_KEYER_SPEED_MIN,CW_KEYER_SPEED_MAX);
								CwGen_SetSpeed();
								UiDriver_DisplayKeyerSpeed(1);
								break;
			case ENC_THREE_MODE_PSK_SPEED:
								psk_ctrl_config.speed_idx = change_and_limit_int(psk_ctrl_config.speed_idx,pot_diff_step,0,PSK_SPEED_NUM-1);
								UiDriver_TextMsgClear();
								PskDecoder_Init();
								UiDriver_DisplayPskSpeed(true);
								break;
				// Update audio volume
			case ENC_THREE_MODE_INPUT_CTRL:
									// in voice mode, adjust audio input gain
								{
									uint16_t gain_max = ts.tx_audio_source == TX_AUDIO_MIC? MIC_GAIN_MAX : LINE_GAIN_MAX;
									uint16_t gain_min = ts.tx_audio_source == TX_AUDIO_MIC? MIC_GAIN_MIN : LINE_GAIN_MIN;

									ts.tx_gain[ts.tx_audio_source] = change_and_limit_int ( ts.tx_gain[ts.tx_audio_source], pot_diff_step, gain_min, gain_max);

									if (ts.tx_audio_source == TX_AUDIO_MIC)
									{
										Codec_SwitchMicTxRxMode(ts.txrx_mode);
									}
									UiDriver_DisplayLineInModeAndGain(1);
								}
								break;
			default:
								break;
			}
		}
	}
}

static bool UiDriver_IsApplicableEncoderOneMode(uint8_t mode)
{
	bool retval = true;
	switch(mode)
	{
		case ENC_ONE_MODE_RTTY_SPEED:
									// only switch to rtty adjustment, if rtty enabled!
									retval = is_demod_rtty();
									break;
		case ENC_ONE_MODE_ST_GAIN:
									retval = ts.dmod_mode == DEMOD_CW || (ts.dmod_mode == DEMOD_DIGI && ts.digital_mode == DigitalMode_BPSK);
									break;
		case ENC_ONE_MODE_CMP_LEVEL:
									retval = ts.dmod_mode != DEMOD_CW && ts.dmod_mode != DEMOD_DIGI;
									break;
	}
	return retval;
}


static void UiDriver_DisplayEncoderOneMode()
{
	// upper box
	UiDriver_DisplayAfGain(ts.enc_one_mode == ENC_ONE_MODE_AUDIO_GAIN);

	// lower box
	switch(ts.enc_one_mode)
	{
		case ENC_ONE_MODE_RTTY_SPEED: 	UiDriver_DisplayRttySpeed   (1);		                break;
		case ENC_ONE_MODE_ST_GAIN   :	UiDriver_DisplaySidetoneGain(1);	                    break;
		case ENC_ONE_MODE_CMP_LEVEL :	UiDriver_DisplayCmpLevel    (1);   ts.AFG_wait = 9000; 	break;
		default                     :
										// what to display if lower box is not active
										if (is_demod_rtty())
										{
											UiDriver_DisplayRttySpeed(0);
										}
										else if(ts.dmod_mode == DEMOD_CW || (ts.dmod_mode == DEMOD_DIGI && ts.digital_mode == DigitalMode_BPSK))
										{
											UiDriver_DisplaySidetoneGain(0);
										}
										else
										{
											UiDriver_DisplayCmpLevel(0);
										}
	}
}

static  bool  UiDriver_IsApplicableEncoderTwoMode ( uint8_t mode )
{
	bool retval = true;
	switch(mode)
	{
		case ENC_TWO_MODE_RTTY_SHIFT:   retval = is_demod_rtty();   break;  // only switch to rtty adjustment, if rtty enabled!
		case ENC_TWO_MODE_NOTCH_F   :	retval = is_dsp_mnotch();	break;
		case ENC_TWO_MODE_PEAK_F    :	retval = is_dsp_mpeak();	break;
	}
	return retval;
}


static void UiDriver_DisplayEncoderTwoMode()
{

	uint8_t inactive = ts.menu_mode?0:1;
	// we use this to disable all active displays once in menu mode
	switch(ts.enc_two_mode)
	{
		case ENC_TWO_MODE_RF_GAIN:
								UiDriver_DisplayRfGain(inactive);
								UiDriver_DisplayNoiseBlanker(0);
								UiDriver_DisplayDSPMode(0);
								break;
		case ENC_TWO_MODE_SIG_PROC:
								UiDriver_DisplayRfGain(0);
								//		UiDriver_DisplayNoiseBlanker(inactive);
								UiDriver_DisplayNoiseBlanker(1);
								UiDriver_DisplayDSPMode(0);
								ts.RFG_wait   = 8000;    // ajout� pour retourner au "RFG" apr�s tempo
								break;
		case ENC_TWO_MODE_PEAK_F:
		case ENC_TWO_MODE_NOTCH_F:
		case ENC_TWO_MODE_NR:
								UiDriver_DisplayRfGain(0);
								UiDriver_DisplayNoiseBlanker(0);
								UiDriver_DisplayDSPMode(inactive);
								break;
		case ENC_TWO_MODE_BASS_GAIN:
								UiDriver_DisplayDSPMode(0);
								UiDriver_DisplayTone(inactive);
								break;
		case ENC_TWO_MODE_TREBLE_GAIN:
								UiDriver_DisplayDSPMode(0);
								UiDriver_DisplayTone(inactive);
								break;
		case ENC_TWO_MODE_RTTY_SHIFT:
								UiDriver_DisplayRfGain(0);
								UiDriver_DisplayDSPMode(0);
								UiDriver_DisplayRttyShift(1);
								break;
		default:
								UiDriver_DisplayRfGain(0);
								UiDriver_DisplayNoiseBlanker(0);
								UiDriver_DisplayDSPMode(0);
								break;
	}

}

//*****************************************************************************************************


static bool UiDriver_IsApplicableEncoderThreeMode(uint8_t mode)
{
	bool retval = true;
	switch(mode)
	{
		case ENC_THREE_MODE_CW_SPEED: 	retval = ts.dmod_mode == DEMOD_CW;											break;
		case ENC_THREE_MODE_PSK_SPEED:	retval = is_demod_psk();													break;
		case ENC_THREE_MODE_INPUT_CTRL:	retval = ts.dmod_mode != DEMOD_DIGI || ts.digital_mode != DigitalMode_BPSK; break;
			//retval = ts.dmod_mode != DEMOD_DIGI || (ts.digital_mode != DigitalMode_BPSK && ts.digital_mode != DigitalMode_RTTY);
	}
	return retval;
}

//*********************************************************************************************************

static void UiDriver_DisplayEncoderThreeMode()
{
	// upper box
	if (ts.txrx_mode == TRX_MODE_RX) { 	UiDriver_DisplayRit        (ts.enc_thr_mode == ENC_THREE_MODE_RIT);}
	else                             {  UiDriver_DisplayPowerAdjust(ts.enc_thr_mode == ENC_THREE_MODE_RIT);}

	// lower box
	switch(ts.enc_thr_mode)
	{
		case ENC_THREE_MODE_CW_SPEED  :	   UiDriver_DisplayKeyerSpeed(1);		    break;
		case ENC_THREE_MODE_PSK_SPEED :    UiDriver_DisplayPskSpeed(1);      		break;
		case ENC_THREE_MODE_INPUT_CTRL:    UiDriver_DisplayLineInModeAndGain(1);   	break;
		default:
						// this defines what is shown if the lower box is not actively selected
						if (ts.dmod_mode == DEMOD_CW)
						{
							UiDriver_DisplayKeyerSpeed(0);
						}
						else if (ts.dmod_mode == DEMOD_DIGI && ts.digital_mode == DigitalMode_BPSK)
						{
							UiDriver_DisplayPskSpeed(0);
						}
						else
						{
							UiDriver_DisplayLineInModeAndGain(0);
						}
						break;
	}
}


/**
 * Handles the execution of the change encoder logic for the 3 encoders
 */

static void UiDriver_ChangeEncoderMode(volatile uint8_t* mode_ptr, uint8_t num_modes, bool (*is_applicable_f)(uint8_t), void(*display_encoder_f)())
{
	if(ts.menu_mode == false)   // changes only when not in menu mode
	{
		uint8_t new_enc_mode = *mode_ptr;
		do
		{
			new_enc_mode++;
			new_enc_mode %= num_modes;
		} while ((*is_applicable_f)(new_enc_mode)  == false && new_enc_mode != *mode_ptr );
		if (new_enc_mode != *mode_ptr)
		{
			*mode_ptr = new_enc_mode;
			(*display_encoder_f)();
		}
	}
}

static void UiDriver_ChangeEncoderOneMode()
{
	UiDriver_ChangeEncoderMode ( &ts.enc_one_mode, ENC_ONE_NUM_MODES, UiDriver_IsApplicableEncoderOneMode, UiDriver_DisplayEncoderOneMode);
}

static void UiDriver_ChangeEncoderTwoMode()
{
	UiDriver_ChangeEncoderMode(&ts.enc_two_mode, ENC_TWO_NUM_MODES, UiDriver_IsApplicableEncoderTwoMode, UiDriver_DisplayEncoderTwoMode);
}

static void UiDriver_ChangeEncoderThreeMode()
{
	UiDriver_ChangeEncoderMode(&ts.enc_thr_mode, ENC_THREE_NUM_MODES, UiDriver_IsApplicableEncoderThreeMode, UiDriver_DisplayEncoderThreeMode);
}

/**
 * @brief Displays audio speaker volume
 */
static void UiDriver_DisplayAfGain ( bool encoder_active )
{
	UiDriver_EncoderDisplaySimple(0,0,"VOL", encoder_active, ts.rx_gain[RX_AUDIO_SPKR].value);
}

/**
 * @brief Display CW Sidetone gain (used during CW TX or training)
 */
static void UiDriver_DisplaySidetoneGain ( bool  encoder_active )
{
	UiDriver_EncoderDisplaySimple(1,0,"STG", encoder_active, ts.cw_sidetone_gain);
}

/**
 * @brief Display TX Compressor Level
 */
static void UiDriver_DisplayCmpLevel ( bool encoder_active )
{
	ushort 	     color = encoder_active?  White:Grey;
	const char*  outs;

	if (ts.tx_comp_level != 1 )   /// TX_AUDIO_COMPRESSION_MIN)  NIZZ pour reduire le Tx compressor a OFF et ON seulement
			{	outs = " OFF"; }
	else 	 					// 	display numbers for all but the highest value
			{	outs = " 1.0";}

	UiDriver_EncoderDisplay ( 1, 0, "CMP" , encoder_active,  outs, color);
}



uint32_t UiDriver_GetActiveDSPFunctions()
{
	return  ts.dsp_active & ( DSP_NOTCH_ENABLE | DSP_NR_ENABLE | DSP_MNOTCH_ENABLE | DSP_MPEAK_ENABLE );
}

static void UiDriver_DisplayDSPMode(bool encoder_active)
{
	uint32_t clr       = White;
	uint32_t clr_val   = White;

	char val_txt[7]    = { 0x0 };
	bool txt_is_value  = false;
	const char* txt[2] = { "DSP", NULL };

	//uint32_t dsp_functions_active = ts.dsp_active & (DSP_NOTCH_ENABLE|DSP_NR_ENABLE|DSP_MNOTCH_ENABLE|DSP_MPEAK_ENABLE);
	uint32_t dsp_functions_active = UiDriver_GetActiveDSPFunctions();

	 UiVk_Redraw();			//virtual keypads call (refresh purpose)

	switch (dsp_functions_active)
	{
		case 0            :  clr = Grey2; txt[1] = "OFF";	break;// all off

		case DSP_NR_ENABLE: 			  txt[0] = "NR" ;
							 snprintf ( val_txt, 7, "%5u", ts.dsp_nr_strength);
										  txt[1] = val_txt;
										  txt_is_value = true;
							 break;

		case DSP_NOTCH_ENABLE:  txt[1] = "A-NOTC"; 	break;

		case DSP_NOTCH_ENABLE|DSP_NR_ENABLE:
											txt[0]       = "NR+NOT";
											snprintf ( val_txt, 7, "%5u", ts.dsp_nr_strength);
											txt[1]       = val_txt;
											txt_is_value = true;
											break;
		case DSP_MNOTCH_ENABLE:
							txt[0]       = "M-NOTC";
							snprintf ( val_txt, 7, "%5lu", ts.notch_frequency);
							txt[1]       = val_txt;
							txt_is_value = true;
							break;

		case DSP_MPEAK_ENABLE:
							txt[0]       = "PEAK";
							snprintf ( val_txt, 7, "%5lu", ts.peak_frequency );
							txt[1]       = val_txt;
							txt_is_value = true;
							break;

		default: // unsupported combination of DSP functions yield error display now
							clr    = Grey2;
							txt[1] = "ERROR";
							break;
	}

	UiDriver_LeftBoxDisplay ( 1, txt[0], encoder_active,txt[1], clr, clr_val, txt_is_value);
}


static void UiDriver_DisplayKeyerSpeed(bool encoder_active)
{
	uint16_t 	color = encoder_active?White:Grey;
	const char* txt;
	char  txt_buf[5];

	txt = "WPM";
	snprintf(txt_buf,5,"%3d",ts.cw_keyer_speed);

	UiDriver_EncoderDisplay(1,2,txt, encoder_active, txt_buf, color);
}

static void UiDriver_DisplayRttySpeed(bool encoder_active)
{
	uint16_t  color = encoder_active?White:Grey;
	UiDriver_EncoderDisplay(1,0,"BD", encoder_active, rtty_speeds[rtty_ctrl_config.speed_idx].label, color);
}

static void UiDriver_DisplayPskSpeed(bool encoder_active)
{
	uint16_t  color = encoder_active?White:Grey;
	UiDriver_EncoderDisplay(1,2,"PSK", encoder_active, psk_speeds[psk_ctrl_config.speed_idx].label, color);
}

static void UiDriver_DisplayRttyShift(bool encoder_active)
{
	uint16_t  color = encoder_active?White:Grey;
	UiDriver_EncoderDisplay(1,1,"SFT", encoder_active, rtty_shifts[rtty_ctrl_config.shift_idx].label, color);
}


static void UiDriver_DisplayLineInModeAndGain(bool encoder_active)
{
	ushort 	color = encoder_active? White:Grey;
	const char* txt;
	char  txt_buf[5];

	bool   gain_external_control = false;
	// if true, gain is controlled externally and ENC3 encoder does not do anything.

	switch ( ts.tx_audio_source )
	{
		case TX_AUDIO_MIC     :   txt = "MIC";	                              break;
		case TX_AUDIO_LINEIN_L:	  txt = "L>L";	                              break;	// Line gain
		case TX_AUDIO_LINEIN_R:	  txt = "L>R";                                break;	// Line gain
		case TX_AUDIO_DIG_EX  :	  txt = "DIG";	gain_external_control = true; break;	// Line gain
		case TX_AUDIO_DIG_IQ  :	  txt = "I/Q";	gain_external_control = true; break;
		default               :   txt = "???";
	}

	if (gain_external_control == true)
	{
		snprintf(txt_buf,5,"EXT");
	}
	else
	{
		if ((ts.tx_audio_source== TX_AUDIO_MIC) && ts.codec_mic_boost_enable) {snprintf( txt_buf, 5,"%3d+", ts.tx_gain[ts.tx_audio_source]);}
		else                                                                   snprintf( txt_buf, 5, "%4d", ts.tx_gain[ts.tx_audio_source]);
	}

	UiDriver_EncoderDisplay(1,2,txt, encoder_active, txt_buf, color);
}

static  void  UiDriver_DisplayRfGain ( bool  encoder_active )  /// seuement afffichage Bouton "RFG"
{
	uint32_t color = encoder_active? White : Grey;

	char	 temp[5];
	const    char* label = "???";
	int32_t  value;
	if ( ts.dmod_mode != DEMOD_FM )  {	label = "AGC";	value = ts.agc_wdsp_thresh;  } // NOT FM
	else      					     {	label = "SQL";	value = ts.fm_sql_threshold; } // use SQL for FM


	/*
    if(ts.dmod_mode != DEMOD_FM) // && !ts.agc_wdsp)	 	// If not FM, use RF gain
    {
        if()
        if(encoder_active)
        {
            //
            // set color as warning that RX sensitivity is reduced
            //
            if(ts.rf_gain < 20)
                color = Red;
            else if(ts.rf_gain < 30)
                color = Orange;
            else if(ts.rf_gain < 40)
                color = Yellow;
        }
        value = ts.rf_gain;
    }
    else if(!ts.agc_wdsp)	 						// it is FM, display squelch instead
    {
        value = ts.fm_sql_threshold;
    }
    else // it is WDSP AGC and NOT FM
    {
        value = ts.agc_wdsp_thresh;
    }

	 */
	snprintf( temp, 5, " %02ld", value );

	UiDriver_EncoderDisplay ( 0, 1, label, encoder_active, temp, color);

}

//*----------------------------------------------------------------------------
//* Function Name       : UiDriverChangeSigProc
//* Object              : Display settings related to signal processing - DSP NR or Noise Blanker strength
//* Input Parameters    :
//* Output Parameters   :
//* Functions called    :
//*----------------------------------------------------------------------------
static  void  UiDriver_DisplayNoiseBlanker ( bool encoder_active )
{
	uint32_t 	color = encoder_active? White : Grey;
	char	temp[5];
	const char *label, *val_txt;
	int32_t value = 0;
//	bool is_active = false;
	//label = "NB";
	//    label = "DEC";
//#if 0
		//
		// Noise blanker settings display
		//
		if(is_dsp_nb())	 	// is noise blanker to be displayed
		{
			if(encoder_active)
			{
				if     (ts.nb_setting >= NB_WARNING3_SETTING)  		color = Red;		// above this value, make it red
				else if(ts.nb_setting >= NB_WARNING2_SETTING)		color = Orange;		// above this value, make it orange
				else if(ts.nb_setting >= NB_WARNING1_SETTING)		color = Yellow;		// above this value, make it yellow
				else												color = White;		// Otherwise, make it white
			}
			label   = "NB";
			value   = ts.nb_setting;
			snprintf( temp, 5, "%3ld", value );
			val_txt = temp;
		}

		else
		{
//#endif
			switch ( ts.agc_wdsp_mode )
			{
				case 0: 		label = "vLO";		break;
				case 1:			label = "LON";		break;
				case 2:			label = "SLO";		break;
				case 3:			label = "MED";		break;
				case 4:			label = "FAS";		break;
				case 5:			label = "OFF";		break;
				default:		label = "???";		break;
			}
			value   = (int32_t) ( ts.agc_wdsp_tau_decay[ ts.agc_wdsp_mode ] );   ////     / 10.0);
			snprintf( temp, 5, "%4ld", value );  /// NIZZZ
			val_txt = temp;

		}
		UiDriver_EncoderDisplay ( 1, 1, label, encoder_active, val_txt, color );
}

#define  NOTCH_DELTA_Y ( 2 * ENC_ROW_H )

//*----------------------------------------------------------------------------
//* Function Name       : UiDriverDisplayBass
//* Object              : Display settings related to bass & treble filter
//* Input Parameters    :
//* Output Parameters   :
//* Functions called    :
//*----------------------------------------------------------------------------
static  void  UiDriver_DisplayTone ( bool  encoder_active )
{

	// UiLcdHy28_DrawFullRect(POS_AG_IND_X, POS_AG_IND_Y + NOTCH_DELTA_Y, 16 + 12 , 112, Black);


	char temp[5];
	int  bas, tre;


	if ( ts.txrx_mode == TRX_MODE_TX ) { bas = ts.tx_bass_gain;  tre = ts.tx_treble_gain; } // if in TX_mode, display TX bass gain instead of RX_bass gain!
	else                               { bas = ts.bass_gain; 	 tre = ts.treble_gain;    }

	bool enable = ( ts.enc_two_mode == ENC_TWO_MODE_BASS_GAIN );
	if  (enable)
	{
		if ( ts.txrx_mode == TRX_MODE_RX ) ts.RFG_wait = 9000;  // ajout� pour retourner au "RFG" apr�s tempo
		snprintf ( temp, 5, "%3d", bas );

		// use 2,1 for placement below existing boxes
		UiDriver_EncoderDisplay ( 0, 1, "BAS", enable && encoder_active, temp, White );
	}


	if ((enable = (ts.enc_two_mode == ENC_TWO_MODE_TREBLE_GAIN)))
	{
		if ( ts.txrx_mode == TRX_MODE_RX ) ts.RFG_wait = 9000;  // ajout� pour retourner au "RFG" apr�s tempo
		snprintf( temp, 5, "%3d", tre );

		// use 2,2 for placement below existing boxes
		UiDriver_EncoderDisplay ( 1, 1, "TRB", enable && encoder_active, temp, White );
	}

} // end void UiDriverDisplayBass

//
//*----------------------------------------------------------------------------
//* Function Name       : UiDriverChangeRit
//* Object              :
//* Input Parameters    :
//* Output Parameters   :
//* Functions called    :
//*----------------------------------------------------------------------------
static  void  UiDriver_DisplayRit( bool  encoder_active )
{
	char	  temp[6];
	uint32_t  color = ts.rit_value? Green : (encoder_active ? White : Grey);
	snprintf( temp, 6, ts.rit_value?"%+4i":"%4i", ts.rit_value * 10 );    /// %+3  NIZZZ
	UiDriver_EncoderDisplay( 0, 2, "RIT", encoder_active, temp, color );
}

///**************************************************************************
static  void  UiDriver_DisplayPowerAdjust ( bool  encoder_active )
{
	char	  temp[5];
	uint32_t  color = White;  ///ts.rit_value? Green : (encoder_active ? White:Grey);
	snprintf( temp, 5, "%4d", ts.current_pwr_adj );
	UiDriver_EncoderDisplay ( 0, 2, "PWR",  encoder_active, temp, color );
}
///***************************************************************************
///
/*
static void UiDriver_DisplayModulationType()  /// masqu� pour ne pas afficher le Pad en bas ou il est marqu� SSB
{

	ushort bgclr     = ts.dvmode?Orange:Blue;
	ushort color     = digimodes[ts.digital_mode].enabled?(ts.dvmode?Black:White):Grey2;
	char txt_empty[] = "       ";
	char txt_SSB[]   = "SSB";
	char txt_CW []   = "CW";

	//const char* txt = digimodes[ts.digital_mode].label;
	const char* txt;
	switch(ts.dmod_mode)
	{
		case DEMOD_DIGI:	txt = digimodes[ts.digital_mode].label; 	break;
		case DEMOD_LSB:
		case DEMOD_USB: 	txt = txt_SSB;		                        break;
		case DEMOD_CW:		txt = txt_CW;		                        break;
		default:		    txt = txt_empty;
	}

	// Draw line for box
	UiLcdHy28_DrawStraightLine(ts.Layout->DIGMODE.x,(ts.Layout->DIGMODE.y - 1),ts.Layout->DIGMODE.w,LCD_DIR_HORIZONTAL,bgclr);
	UiLcdHy28_PrintTextCentered(ts.Layout->DIGMODE.x,ts.Layout->DIGMODE.y,ts.Layout->DIGMODE.w,txt,color,bgclr,0);
	if(disp_resolution==RESOLUTION_480_320)
	{
		UiLcdHy28_DrawStraightLineDouble(ts.Layout->DIGMODE.x,ts.Layout->DIGMODE.y+12,ts.Layout->DIGMODE.w,LCD_DIR_HORIZONTAL,bgclr);
		UiLcdHy28_DrawStraightLine(ts.Layout->DIGMODE.x,ts.Layout->DIGMODE.y+14,ts.Layout->DIGMODE.w,LCD_DIR_HORIZONTAL,Blue);
	}
	//fdv_clear_display();
}
 */



static void UiDriver_DisplayPowerLevel()
{
	ushort color = White;
	const  char*  txt;

	switch(ts.power_level)
	{
		case PA_LEVEL_5W: 	txt = "5W";		break;
		case PA_LEVEL_2W:	txt = "2W";		break;
		case PA_LEVEL_1W:	txt = "1W";		break;
		case PA_LEVEL_0_5W:	txt = "0.5W";	break;
		default:		    txt = "FULL";	break;
	}
	// Draw top line
	UiLcdHy28_PrintTextCentered (  (ts.Layout->PW_IND.x),  (ts.Layout->PW_IND.y),  ts.Layout->PW_IND.w, txt,  color,  Blue,  0);
}
		///**************************************************************************

static void UiDriver_HandleSMeter  ()   // (bool  IQCodecGain_Execute)  // chaque 40ms
{

	// Only in RX mode
	if(ts.txrx_mode == TRX_MODE_RX)
	{


		// ONLY UI CODE BELOW
		//
		// This makes a portion of the S-meter go red if A/D clipping occurs
		//
		{
			static bool         clip_indicate = 0;

// we fixed the S-Meter to display ONLY dBm, NOT dBm/Hz
			/*
			if (ts.s_meter == DISPLAY_S_METER_DBM) // based on dBm calculation
			{
				sm.gain_calc = sm.dbm;
			}
			else // based on dBm/Hz calculation
			{
				sm.gain_calc = sm.dbmhz;
			}
*/

			sm.gain_calc = sm.dbm;
			const float *S_Meter_Cal_Ptr = S_Meter_Cal_dbm;

			// find corresponding signal level
			for ( sm.s_count = 0;	(sm.gain_calc >= S_Meter_Cal_Ptr[sm.s_count]) && (sm.s_count < S_Meter_Cal_Size);	 sm.s_count++)
			{	 }  // nothing to do here

			if ( ads.adc_clip > 10 )	 		// did clipping occur many times ?
			{
				if( !clip_indicate )	 	// have we seen it clip before?
				{
					UiDriver_DrawSMeter( Red );		// No, make the first portion of the S-meter red to indicate A/D overload
					clip_indicate = true;		// set flag indicating that we saw clipping and changed the screen (prevent continuous redraw)
				}
				ads.adc_clip = 0;		// reset clip detect flag
			}
			else	 		// clipping NOT occur?
			{
				if( clip_indicate )	 	// had clipping occurred since we last visited this code?
				{
					UiDriver_DrawSMeter( White );		// yes - restore the S meter to a white condition
					clip_indicate = false;				// clear the flag that indicated that clipping had occurred
				}
			}

			// make sure that the S meter always reads something!
			UiDriver_UpdateTopMeterA ( (sm.s_count>0) ? sm.s_count : 1 );
		}
	}
}



/**
 *
 * Power, SWR, ALC and Audio indicator handling
 */
static void UiDriver_HandleTXMeters()
{
	// Only in TX mode
	if ( ts.txrx_mode != TRX_MODE_TX )
	{
	// swrm.vswr_dampened =  0;		// reset averaged readings when not in TX mode
		swrm.fwd_pwr_avg   = -1;
		swrm.rev_pwr_avg   = -1;
        swrm.p_curr        =  0;
        swrm.fwd_calc      =  0;
        swrm.rev_calc      =  0;

	}
	else
	{
		static uint8_t    old_power_level = 99;

		// display FWD, REV power, in milliwatts - used for calibration - IF ENABLED
		if ( swrm.pwr_meter_disp )
		{
			if((swrm.fwd_pwr_avg < 0) || (ts.power_level != old_power_level))  	// initialize with current value if it was zero (e.g. reset) or power level changed
			{
				swrm.fwd_pwr_avg = swrm.fwd_pwr;
			}
			else
			{
				swrm.fwd_pwr_avg = (swrm.fwd_pwr_avg * (1-PWR_DAMPENING_FACTOR)) + swrm.fwd_pwr * PWR_DAMPENING_FACTOR;	// apply IIR smoothing to forward power reading
			}

			if((swrm.rev_pwr_avg < 0) || (ts.power_level != old_power_level))  	// initialize with current value if it was zero (e.g. reset) or power level changed
			{
				swrm.rev_pwr_avg = swrm.rev_pwr;
			}
			else
			{
				swrm.rev_pwr_avg = (swrm.rev_pwr_avg * (1-PWR_DAMPENING_FACTOR)) + swrm.rev_pwr * PWR_DAMPENING_FACTOR; // apply IIR smoothing to reverse power reading
			}

			old_power_level = ts.power_level;		// update power level change detector
		}

		{
			char txt[16];
			const  char*  txp = NULL;
			if ( swrm.pwr_meter_disp )
			{
				snprintf ( txt, 16, "%5d/%4d", (int)(swrm.fwd_pwr_avg*1000), (int)(swrm.rev_pwr_avg*1000) );  // scale to display power in milliwatts
				txp = txt;
				swrm.pwr_meter_was_disp = 1;	// indicate the power meter WAS displayed
			}
			else if( swrm.pwr_meter_was_disp )	// had the numerical display been enabled - and it is now disabled?
			{
				txp = "           ";            // yes - overwrite location of numerical power meter display to blank it
				swrm.pwr_meter_was_disp = 0;	// clear flag so we don't do this again
			}
			if ( txp != NULL )
			{
				UiLcdHy28_PrintText ( ts.Layout->PWR_NUM_IND.x,  ts.Layout->PWR_NUM_IND.y, txp, Grey, Black, 0 );
			}
		}


		// Do selectable meter readings
		float     btm_mtr_val       = 0.0;
		uint32_t  btm_mtr_red_level = 18;


		if (ts.tx_meter_mode == METER_SWR)
		{
			if  ( ts.dynamic_tune_activ_counter < 2 )  // pour temporiser un peu afin de stabiliser les tensions avant mesure
			{
				if ( swrm.fwd_pwr >= SWR_MIN_CALC_POWER )	 		// is the forward power high enough for valid VSWR calculation?
				{
					// (Do nothing/freeze old data if below this power level)
					if(swrm.vswr_dampened < 1 )	// initialize averaging if this is the first time (e.g. VSWR <1 = just returned from RX)
					{
						swrm.vswr_dampened = swrm.vswr;
					}
					else
					{
						swrm.vswr_dampened = swrm.vswr_dampened * (1 - VSWR_DAMPENING_FACTOR) + swrm.vswr * VSWR_DAMPENING_FACTOR;
					}

					if       ( swrm.vswr_dampened > 7 ) { swrm.vswr_dampened =   7; }
					else if  ( swrm.vswr_dampened < 1 ) { swrm.vswr_dampened = 0.7; }


					if ( swrm.vswr_dampened < 2.0 )   // 1.75
					{
						btm_mtr_val = swrm.vswr_dampened  * 28 - 24;
						if ( ! SWR_Zoom2 )
						{
							SWR_Zoom2 = true;
							SWR_Zoom1 = false;
							UiLcdHy28_DrawFullRect ( ts.Layout->SM_IND.x + 65   ,  ts.Layout->SM_IND.y + 47, 10, 120, Black );
							UiLcdHy28_PrintText    ((ts.Layout->SM_IND.x + 176) , (ts.Layout->SM_IND.y + 59 - BTM_MINUS), "2", White, Black, 4);
						}
					}
					else if ((swrm.vswr_dampened < 2.1) && SWR_Zoom2) { btm_mtr_val = swrm.vswr_dampened  * 28 - 24;   }
					else if ( swrm.vswr_dampened < 3.0)  // 2.5
					{
						btm_mtr_val = swrm.vswr_dampened  * 14 - 10;
						if (! SWR_Zoom1)
						{
							SWR_Zoom1 = true;
							SWR_Zoom2 = false;
							UiLcdHy28_DrawFullRect ( ts.Layout->SM_IND.x + 65   ,  ts.Layout->SM_IND.y + 47             ,  10,   120, Black);
							UiLcdHy28_PrintText    ((ts.Layout->SM_IND.x + 106 ), (ts.Layout->SM_IND.y + 59 - BTM_MINUS), "2", White, Black, 4);
							UiLcdHy28_PrintText    ((ts.Layout->SM_IND.x + 176) , (ts.Layout->SM_IND.y + 59 - BTM_MINUS), "3", White, Black, 4);
						}
					}
					else
					{
						if (SWR_Zoom1  || SWR_Zoom2 )
						{
							if (SWR_Zoom1  & (swrm.vswr_dampened < 3.1))  { btm_mtr_val = swrm.vswr_dampened  * 14 - 10; }
							else
							{
								btm_mtr_val = swrm.vswr_dampened  * 7 - 3;  // four dots per unit of VSWR  NIZZZ pour augmenter la resolution du SWR
								SWR_Zoom1   = false;
								SWR_Zoom2   = false;
								// btm_mtr_val = swrm.vswr_dampened  * 6 - 2;  // swrm.vswr_dampened * 4;		// yes - four dots per unit of VSWR  NIZZZ pour augmenter la resolution du SWR
								UiLcdHy28_DrawFullRect ( ts.Layout->SM_IND.x + 105, ts.Layout->SM_IND.y + 47, 10, 45, Black );
								UiLcdHy28_PrintText ( (ts.Layout->SM_IND.x +  71), (ts.Layout->SM_IND.y + 59 - BTM_MINUS), "2", White, Black, 4 );
								UiLcdHy28_PrintText ( (ts.Layout->SM_IND.x + 106), (ts.Layout->SM_IND.y + 59 - BTM_MINUS), "3", White, Black, 4 );
								UiLcdHy28_PrintText ( (ts.Layout->SM_IND.x + 141), (ts.Layout->SM_IND.y + 59 - BTM_MINUS), "4", White, Black, 4 );
								UiLcdHy28_PrintText ( (ts.Layout->SM_IND.x + 176), (ts.Layout->SM_IND.y + 59 - BTM_MINUS), "5", White, Black, 4 );
							}
						}
						else {  btm_mtr_val = swrm.vswr_dampened  * 7 - 3;  }

					}
				}
			}
		}
		else if(ts.tx_meter_mode == METER_ALC)
		{
			btm_mtr_val  = ads.alc_val;		     // get TX ALC value
			btm_mtr_val *= btm_mtr_val;		     // square the value
			btm_mtr_val  = log10f(btm_mtr_val);	 // get the log10
			btm_mtr_val *= -10;		             // convert it to DeciBels and switch sign and then scale it for the meter
		}
		else if(ts.tx_meter_mode == METER_AUDIO)
		{
			btm_mtr_val    = ads.peak_audio/10000;	 // get a copy of the peak TX audio (maximum reference = 30000)
			ads.peak_audio = 0;					     // reset the peak detect
			btm_mtr_val   *= btm_mtr_val;			 // square the value
			btm_mtr_val    = log10f(btm_mtr_val);	 // get the log10
			btm_mtr_val   *= 10;					 // convert to DeciBels and scale for the meter
			btm_mtr_val   += 11;					 // offset for meter

			btm_mtr_red_level = 22;	// setting the "red" threshold
		}
		// calculate and display RF power reading
		UiDriver_UpdateTopMeterA ( swrm.fwd_pwr * 3);
		// show selected bottom meter value
		UiDriver_UpdateBtmMeter ( btm_mtr_val, btm_mtr_red_level );   // update the meter, setting the "red" threshold
	}
}


static void UiDriver_CreateVoltageDisplay()
{                                       /// NIZZ
	// Create voltage
	// UiLcdHy28_PrintTextCentered ( ts.Layout->PWR_IND.x - 4   ,ts.Layout->PWR_IND.y, ts.Layout->LEFTBOXES_IND.w,   "--.- V",  COL_PWR_IND, Black, 0);
}

static bool UiDriver_SaveConfiguration()
{
	bool savedConfiguration = true;

	const char* txp;
	uint16_t    txc;

	switch ( ts.configstore_in_use )
	{
		case CONFIGSTORE_IN_USE_FLASH: 	txp = "Saving settings to Flash Memory";	                      break;
		case CONFIGSTORE_IN_USE_I2C:	txp = "Saving settings to I2C EEPROM";		                      break;
		default:						txp = "Detected problems: Not saving";	savedConfiguration = false;
	}
	UiLcdHy28_PrintTextCentered( sd.Slayout->full.x, sd.Slayout->full.h/2+sd.Slayout->full.y-6,sd.Slayout->full.w, txp, Blue, Black, 0);

	if (savedConfiguration)
	{
		// save settings
		if (UiConfiguration_SaveEepromValues() == 0)
		{
			txp = "Saving settings finished";
			txc = Green;
		}
		else
		{
			txp = "Saving settings failed";
			txc = Red;
			savedConfiguration = false;
		}
		UiLcdHy28_PrintTextCentered( sd.Slayout->full.x, sd.Slayout->full.h/2+sd.Slayout->full.y+6, sd.Slayout->full.w, txp, txc, Black, 0 );
	}
	return savedConfiguration;
}


/*
 * @brief displays the visual information that power down is being executed and saves EEPROM if requested
 */
static void UiDriver_PowerDownCleanup( bool saveConfiguration )
{
	const char* txp;
	// Power off all - high to disable main regulator

	ts.powering_down = 1;   // indicate that we should be powering down

	UiSpectrum_Clear();   // clear display under spectrum scope

	// hardware based mute
	Codec_MuteDAC(true);  // mute audio when powering down

	txp = " ";

	UiLcdHy28_PrintTextCentered ( 60, 148, 240, txp              , Blue2, Black, 0 );
	UiLcdHy28_PrintTextCentered ( 60, 156, 240, "Powering off...", Blue2, Black, 0 );
	UiLcdHy28_PrintTextCentered ( 60, 168, 240, txp              , Blue2, Black, 0 );

	if ( saveConfiguration )
	{
		UiDriver_SaveConfiguration();
	}
	else
	{
		UiLcdHy28_PrintTextCentered( 60, 176, 260, "...without saving settings...", Blue, Black , 0 );
	}


	if ( saveConfiguration )
	{
		UiConfiguration_SaveEepromValues();     // save EEPROM values
	}

	HAL_Delay( 3000 );
}



/*
 * @brief Display external voltage
 */
static void UiDriver_DisplayVoltage()    // NIZZZ
{
	uint32_t low_power_threshold = ((ts.low_power_config & LOW_POWER_THRESHOLD_MASK) + LOW_POWER_THRESHOLD_OFFSET) * 10;
	// did we detect a voltage change?

	uint32_t col = COL_PWR_IND;  // Assume normal voltage, so Set normal color

	if      (pwmt.voltage < low_power_threshold +  50)  { col = Red;    }
	else if (pwmt.voltage < low_power_threshold + 100)	{ col = Orange; }
	else if (pwmt.voltage < low_power_threshold + 150)	{ col = Yellow; }

	static uint8_t voltage_blink = 0;
	// in case of low power shutdown coming, we let the voltage blink with 1hz
	if ( pwmt.undervoltage_detected == true && voltage_blink < 1 ) 	{ col = Black; }
	voltage_blink++;
	if ( voltage_blink == 2 ) { voltage_blink = 0; }

	char digits[7];
	snprintf ( digits, 7, "%2ldv%02ld", pwmt.voltage/100, pwmt.voltage%100 );
	UiLcdHy28_PrintText ( ts.Layout->PWR_IND.x, ts.Layout->PWR_IND.y, digits, col, Black, 0 );
}

/**
 * @brief Measures Voltage and controls undervoltage detection
 * @returns true if display update is required, false if not
 */
static bool UiDriver_HandleVoltage()
{
	bool retval = false;
	// if this is set to true, we should update the display because something relevant for the user happened.

	// Collect samples
	if(pwmt.p_curr < POWER_SAMPLES_CNT)
	{
		// Add to accumulator
		pwmt.pwr_aver = pwmt.pwr_aver + HAL_ADC_GetValue(&hadc1);
		pwmt.p_curr++;
	}
	else
	{

		// Get average
		uint32_t val_p  = ((pwmt.pwr_aver/POWER_SAMPLES_CNT) * (ts.voltmeter_calibrate + 900))/2500;

		// Reset accumulator
		pwmt.p_curr     = 0;
		pwmt.pwr_aver   = 0;


		retval = pwmt.voltage != val_p;

		pwmt.voltage = val_p;


		uint32_t low_power_threshold        = ((ts.low_power_config & LOW_POWER_THRESHOLD_MASK) + LOW_POWER_THRESHOLD_OFFSET) * 10;
		bool     low_power_shutdown_enabled = ( ts.low_power_config & LOW_POWER_ENABLE_MASK) == LOW_POWER_ENABLE;

		if (low_power_shutdown_enabled && (val_p < low_power_threshold ))
		{
			// okay, voltage is too low, we should indicate
			pwmt.undervoltage_detected = true;
			retval = true;

			if (ts.txrx_mode == TRX_MODE_RX)
			{
				if (ts.sysclock > ts.low_power_shutdown_time )         // only allow power-off in RX mode
				{
					UiDriver_PowerDownCleanup(true);
				}
			}
			else
			{
				ts.low_power_shutdown_time = ts.sysclock + LOW_POWER_SHUTDOWN_DELAY_TIME;
				// in tx mode, we extend the waiting time during the transmit, so that we don't switch off
				// right after a transmit but let the battery some time to "regenerate"
			}
		}
		else
		{
			if (pwmt.undervoltage_detected == true)
			{
				retval = true;
				pwmt.undervoltage_detected = false;
				Board_GreenLed(LED_STATE_ON);
			}
			ts.low_power_shutdown_time = ts.sysclock + LOW_POWER_SHUTDOWN_DELAY_TIME;
		}
	}

	return retval;
}

#if 0
/*
 * @brief Displays temp compensation value in a bar
 */

static  void  UiDriverUpdateLoMeter ( uchar val,  uchar active )
{
	static  int       last_active     =  99;
	static  uint32_t  last_active_val =  99;
	uchar 	          i,v_s           =  3;
	int		          clr             =  White;

	if (last_active != active)
	{
		last_active     = active;
		last_active_val = val;
		// Full redraw
		for(i = 1; i < 26; i++)
		{
			if (active)
			{
				clr = val==i?Blue:White;
			}
			else
			{
				clr = Grey;
			}
			UiLcdHy28_DrawStraightLineTriple(((POS_TEMP_IND_X + 1) + i*4),((POS_TEMP_IND_Y + 21) - v_s),v_s,LCD_DIR_VERTICAL,clr);
		}
	}
	else if ( active && (last_active_val != val) )
	{
		// Partial redraw
		if ( val>1 && val < 26 )
		{
			UiLcdHy28_DrawStraightLineTriple(((POS_TEMP_IND_X + 1) + val*4),((POS_TEMP_IND_Y + 21) - v_s),v_s,LCD_DIR_VERTICAL,Blue);
		}
		if ( last_active_val > 1 && last_active_val < 26)
		{
			UiLcdHy28_DrawStraightLineTriple(((POS_TEMP_IND_X + 1) + last_active_val*4),((POS_TEMP_IND_Y + 21) - v_s), v_s, LCD_DIR_VERTICAL, White);
		}
		last_active_val = val;
	}
}
#endif

/**
 * \brief draws the the TCXO temperature display, has to be called once
 *
 * @param create set to true in order to draw the static parts of the UI too.
 * @param enabled set to true in order to enable actual display of temperature
 */
#define TEMP_DATA 43
void UiDriver_CreateTemperatureDisplay()
{
	const char *label, *txt;
	uint32_t label_color, txt_color;

	bool enabled = lo.sensor_present == true && RadioManagement_TcxoIsEnabled();

	label       = "TCXO";
	label_color = Black;

	// Top part - name and temperature display
	UiLcdHy28_DrawEmptyRect(ts.Layout->TEMP_IND.x, ts.Layout->TEMP_IND.y,13,109,Grey);

	if (enabled)
	{
		txt       = RadioManagement_TcxoIsFahrenheit()?"*---.-F":"*---.-C";
		txt_color = Grey;
	}
	else
	{
		if   (lo.sensor_present == false)	{	txt = "*  OFF ";	txt_color = Grey; }  // {	txt = "SENSOR!";	txt_color = Red;  }
		else								{	txt = "*  OFF ";	txt_color = Grey; }
	}

	// Label
	UiLcdHy28_PrintText((ts.Layout->TEMP_IND.x + 1), (ts.Layout->TEMP_IND.y + 1),label,label_color,Grey,0);
	// Lock Indicator
	UiLcdHy28_PrintText(ts.Layout->TEMP_IND.x + TEMP_DATA,(ts.Layout->TEMP_IND.y + 1), txt,txt_color,Black,0);	// show base string
}

/**
 * @brief display measured temperature and current state of TCXO
 * @param temp in tenth of degrees Celsius (10 == 1 degree C)
 */
static void UiDriver_DisplayTemperature(int temp)
{
	static int last_disp_temp = -100;


	// if ( (ts.switch_pause == 0) || (ts.txrx_mode == TRX_MODE_TX ) )
	{
		uint32_t clr =  RadioManagement_TcxoGetMode() ==TCXO_ON ? Blue:Green ;  // Red;

		UiLcdHy28_PrintText(ts.Layout->TEMP_IND.x + TEMP_DATA,(ts.Layout->TEMP_IND.y + 1),"*",clr,Black,0);

		if (temp != last_disp_temp)
		{
			char  out[10];
			char* txt_ptr;
			if((temp < 0) || (temp > 1000))  // is the temperature out of range?
			{
				txt_ptr = "RANGE!";
			}
			else
			{
				last_disp_temp = temp;

				int32_t  ttemp = last_disp_temp;
				if(RadioManagement_TcxoIsFahrenheit())
				{
					ttemp = ((ttemp *9)/5) + 320;			// multiply by 1.8 and add 32 degrees
				}
				snprintf ( out, 10, "%3ld.%1ld", ttemp/10, (ttemp)%10 );
				txt_ptr = out;
			}
			UiLcdHy28_PrintText(ts.Layout->TEMP_IND.x + TEMP_DATA + SMALL_FONT_WIDTH*1,(ts.Layout->TEMP_IND.y + 1),txt_ptr,Grey,Black,0);
		}
	}

	if ( ts.show_debug_info )  /// NIZZZ  affichage des coeff de correction IQ amplitude et phase en cas de besoins
	{
		char  out[10];
		snprintf ( out, 10, "%6d", (int)((1 - ts.rx_iq_auto_Ampl_correc )*10000));   UiLcdHy28_PrintText ( 10,  95, out, Grey, Black, 0);
		snprintf ( out, 10, "%6d", (int)((ts.rx_iq_auto_Phas_correc     )*10000));   UiLcdHy28_PrintText ( 10, 108, out, Grey, Black, 0);
	}
}

//*----------------------------------------------------------------------------
//* Function Name       : UiDriverHandleLoTemperature
//* Object              : display LO temperature and compensate drift
//* Input Parameters    :
//* Output Parameters   :
//* Functions called    :
//*----------------------------------------------------------------------------
static void UiDriver_HandleLoTemperature()
{
	if (SoftTcxo_HandleLoTemperatureDrift())
	{
		UiDriver_DisplayTemperature( lo.temp / 1000 ); // precision is 0.1 represent by lowest digit
	}
}



//*----------------------------------------------------------------------------
//* Function Name       : UiDriverEditMode
//* Object              :
//* Input Parameters    :
//* Output Parameters   :
//* Functions called    :
//*----------------------------------------------------------------------------
/*static void UiDriverEditMode()
{
	char symb[2];

	// Is edit mode ?
	if(kbs.set_mode != 1)
		return;

	// Key pressed
	if(kbs.last_char == 0)
		return;

	//printf("key = %02x ",kbs.last_char);

	// Handle CR
	if(kbs.last_char == 0x0a)
	{
		kbs.edit_item_id++;
		if(kbs.edit_item_id == 3)
			kbs.edit_item_id = 0;

		// Switch items
		switch(kbs.edit_item_id)
		{
			case 0:
			{
				UiLcdHy28_PrintText((POS_SPECTRUM_IND_X + 5),(POS_SPECTRUM_IND_Y +  0),"Call:  ",White,Black,0);
				UiLcdHy28_PrintText((POS_SPECTRUM_IND_X + 5),(POS_SPECTRUM_IND_Y + 15),"Loc:   ",Grey, Black,0);
				UiLcdHy28_PrintText((POS_SPECTRUM_IND_X + 5),(POS_SPECTRUM_IND_Y + 30),"Power: ",Grey, Black,0);
				break;
			}

			case 1:
			{
				UiLcdHy28_PrintText((POS_SPECTRUM_IND_X + 5),(POS_SPECTRUM_IND_Y +  0),"Call:  ",Grey,Black,0);
				UiLcdHy28_PrintText((POS_SPECTRUM_IND_X + 5),(POS_SPECTRUM_IND_Y + 15),"Loc:   ",White, Black,0);
				UiLcdHy28_PrintText((POS_SPECTRUM_IND_X + 5),(POS_SPECTRUM_IND_Y + 30),"Power: ",Grey, Black,0);
				break;
			}

			case 2:
			{
				UiLcdHy28_PrintText((POS_SPECTRUM_IND_X + 5),(POS_SPECTRUM_IND_Y +  0),"Call:  ",Grey,Black,0);
				UiLcdHy28_PrintText((POS_SPECTRUM_IND_X + 5),(POS_SPECTRUM_IND_Y + 15),"Loc:   ",Grey, Black,0);
				UiLcdHy28_PrintText((POS_SPECTRUM_IND_X + 5),(POS_SPECTRUM_IND_Y + 30),"Power: ",White, Black,0);
				break;
			}

			default:
				break;
		}

		// Reset hor ptr
		kbs.edit_item_hor = 0;
	}
	else
	{
		symb[0] = kbs.last_char;
		symb[1] = 0;

		// Print items
		switch(kbs.edit_item_id)
		{
			case 0:
			{
				// Add to buffer
				kbs.item_0[kbs.edit_item_hor] = kbs.last_char;

				UiLcdHy28_PrintText((POS_SPECTRUM_IND_X + 80 + (kbs.edit_item_hor*12)),(POS_SPECTRUM_IND_Y +  0),symb,Grey,Black,0);
				break;
			}

			case 1:
			{
				// Add to buffer
				kbs.item_1[kbs.edit_item_hor] = kbs.last_char;

				UiLcdHy28_PrintText((POS_SPECTRUM_IND_X + 80 + (kbs.edit_item_hor*12)),(POS_SPECTRUM_IND_Y + 15),symb,Grey, Black,0);
				break;
			}

			case 2:
			{
				// Add to buffer
				kbs.item_2[kbs.edit_item_hor] = kbs.last_char;

				UiLcdHy28_PrintText((POS_SPECTRUM_IND_X + 80 + (kbs.edit_item_hor*12)),(POS_SPECTRUM_IND_Y + 30),symb,Grey, Black,0);
				break;
			}

			default:
				break;
		}

		// Move cursor right
		kbs.edit_item_hor++;
		if(kbs.edit_item_hor == 10)
			kbs.edit_item_hor = 0;
	}

	// Clear public
	kbs.last_char = 0;
}*/

typedef enum
{
	CONFIG_DEFAULTS_KEEP = 0,
	CONFIG_DEFAULTS_LOAD_FREQ,
	CONFIG_DEFAULTS_LOAD_ALL
} CONFIG_DEFAULTS;


static void UiDriver_WaitForBandMAndBandPorPWR()
{
    while((((UiDriver_IsButtonPressed(BUTTON_BNDM_PRESSED)) && (UiDriver_IsButtonPressed(BUTTON_BNDP_PRESSED))) == false) && UiDriver_IsButtonPressed(BUTTON_PWR_PRESSED) == false)
    {
        HAL_Delay(20);
        Keypad_Scan();
    }
}


/*
 * @brief Handles the loading of the configuration at startup (including the load of defaults if requested)
 * @returns false if it is a normal startup, true if defaults have been loaded
 */
static bool UiDriver_LoadSavedConfigurationAtStartup()
{

	bool              retval    = false;
	CONFIG_DEFAULTS   load_mode = CONFIG_DEFAULTS_KEEP;
	HAL_Delay(180);  // 1500 //1800

	if (UiDriver_IsButtonPressed(BUTTON_F1_PRESSED) && UiDriver_IsButtonPressed(BUTTON_F3_PRESSED) && UiDriver_IsButtonPressed(BUTTON_F5_PRESSED))
	{
		load_mode = CONFIG_DEFAULTS_LOAD_ALL;
	}
/*	else if (UiDriver_IsButtonPressed(BUTTON_F2_PRESSED) && UiDriver_IsButtonPressed(BUTTON_F4_PRESSED))
	{
		HAL_Delay(1500);  // 1500 on sait jamais peut etre qu'il faut encore attendre le hardware se stabiliser
	}*/

	else if (UiDriver_IsButtonPressed(BUTTON_F2_PRESSED) && UiDriver_IsButtonPressed(BUTTON_F4_PRESSED))
	{
		load_mode = CONFIG_DEFAULTS_LOAD_FREQ;
	}

	if(load_mode != CONFIG_DEFAULTS_KEEP)
	{
		// let us make sure, the user knows what he/she is doing
		// in case of change of mindes, do normal configuration load

		uint32_t   clr_fg,  clr_bg;
		const char*  top_line;

		switch (load_mode)
		{
			case CONFIG_DEFAULTS_LOAD_ALL:
											clr_bg   = Red;
											clr_fg   = White;
											top_line = "ALL DEFAULTS";
											break;

			case CONFIG_DEFAULTS_LOAD_FREQ:
											clr_bg   = Yellow;
											clr_fg   = Black;
											top_line = "FREQ/MODE DEFAULTS";
											break;
			default:
											break;
		}


		UiLcdHy28_LcdClear( clr_bg );							// clear the screen
		// now do all of the warnings, blah, blah...
		UiLcdHy28_PrintTextCentered(2,05, 316, top_line,clr_fg,clr_bg,1);
		UiLcdHy28_PrintTextCentered(2,35, 316, "-> LOAD REQUEST <-",clr_fg,clr_bg,1);

		UiLcdHy28_PrintTextCentered(2,70, 316,	"If you don't want to do this\n"
												"press POWER button to start normally.",clr_fg,clr_bg,0);

		UiLcdHy28_PrintTextCentered(2,120, 316,	"If you want to load default settings\n"
												"press and hold BAND+ AND BAND-.\n"
												"Settings will be saved at POWEROFF",clr_fg,clr_bg,0);

		// On screen delay									// delay a bit...
		HAL_Delay(5000);

		// add this for emphasis
		UiLcdHy28_PrintTextCentered(2,195, 316,
				"Press BAND+ and BAND-\n"
				"to confirm loading",clr_fg,clr_bg,0);

		UiDriver_WaitForBandMAndBandPorPWR();

		const char* txp;

		if(UiDriver_IsButtonPressed(BUTTON_PWR_PRESSED))
		{
			clr_bg    = Black;							// clear the screen
			clr_fg    = White;
			txp       = "...performing normal start...";
			load_mode = CONFIG_DEFAULTS_KEEP;
			retval    = false;
		}
		else
		{
			txp = "...loading defaults in progress...";
			// call function to load values - default instead of EEPROM
			retval              = true;
			ts.menu_var_changed = true;
		}
		UiLcdHy28_LcdClear(clr_bg);                         // clear the screen
		UiLcdHy28_PrintTextCentered( 2, 108, 316, txp, clr_fg, clr_bg, 0 );

		HAL_Delay(5000);
	}

	switch (load_mode)
	{
	case CONFIG_DEFAULTS_LOAD_ALL:  ts.load_eeprom_defaults    = true; break;  // yes, set flag to indicate that defaults will be loaded instead of those from EEPROM
	case CONFIG_DEFAULTS_LOAD_FREQ:	ts.load_freq_mode_defaults = true; break;
	     default:		                                               break;
	}

	UiConfiguration_LoadEepromValues();
	ts.load_eeprom_defaults    = false;
	ts.load_freq_mode_defaults = false;


	return retval;
}

//*----------------------------------------------------------------------------
//* Function Name       : UiCheckForPressedKey
//* Object              : Used for testing keys on the front panel
//* Input Parameters    :
//* Output Parameters   :
//* Functions called    :
//* Comments            : If a button (other than "POWER") is held during power-up a screen is displayed that shows which key (if any)
//  Comments            : is being pressed.  If multiple keys are being pressed, only the one with the highest precedence is displayed.  The order of decreasing
//  Comments            : precedence is:  M2, G3, G2, BNDM, G4, M3, STEPM, STEPP, M1, M3, F1, F2, F4, BNDP, F5, G1 and POWER.  [KA7OEI October, 2015]
//*----------------------------------------------------------------------------
static void UiDriver_KeyTestScreen()
{
	ushort     p_o_state     = 0,  rb_state    = 0,   new_state  = 0;
	uint32_t   poweroffCount = 0,  rebootCount = 0;

	// int direction;

	uint32_t   keyScanState = 0;

    Keypad_Scan(); // read and map the keys to their logical buttons
    // not all keys may have a button or some keys may go to the same button

	if (UiDriver_IsButtonPressed( TOUCHSCREEN_ACTIVE ))
	{
	    UiLcdHy28_TouchscreenHasProcessableCoordinates();
	    // touchscreen was pressed:
	    // wait a little and see if touch is still pressed
	    // some TRX seem to see touchscreen events even if not pressed
	    HAL_Delay(500);
	}

	Keypad_Scan();

    keyScanState = Keypad_KeyStates();	// remember which one was pressed

	if(Keypad_IsAnyKeyPressed()) {			// at least one button was pressed

        char           txt_buf[40];
        const  char*   txt;
        uint32_t       encoderCount = 0;
        const uint32_t clr_fg       = White;
        const uint32_t clr_bg       = Blue;

		UiLcdHy28_LcdClear(Blue);							// clear the screen
        UiLcdHy28_PrintTextCentered ( 2, 05, ts.Layout->Size.x-4, "INPUT TEST SCREEN", clr_fg, clr_bg, 1 );

		snprintf( txt_buf, 40, "Keys Initial: %08lx", keyScanState );
		UiLcdHy28_PrintTextCentered( 0, 30, ts.Layout->Size.x, txt_buf, White, Blue, 0 );

		UiLcdHy28_PrintTextCentered( 0, 70, ts.Layout->Size.x, "press & hold POWER button to poweroff\npress & hold BAND- button to reboot",White,Blue,0);

		for(;;)	 		// get stuck here for test duration
		{
			uint32_t  idxFirstPressedButton = 99;		// load with flag value
			uint32_t  numOfPressedButtons   = 0;

			// we slow down the loop a bit so that our wait counters take some time to go down.
            HAL_Delay(10);

	        Keypad_Scan();
	        // read and map the hw keys to their logical buttons
	        // not all keys may have a button or some keys may go to the same button


	        uint32_t newKeyScanState = Keypad_KeyStates();   // check which hw keys are pressed

	        if (newKeyScanState != keyScanState)
	        {
	            keyScanState = newKeyScanState;
	            snprintf( txt_buf, 40, "Keys Current: %08lx", keyScanState );
	            UiLcdHy28_PrintTextCentered( 0, 45, ts.Layout->Size.x, txt_buf, White, Blue, 0 );
	        }


	        //  now find out which buttons are pressed (logical buttons, not hw keys)
			for( int buttonIdx = 0; buttonIdx < BUTTON_NUM ; buttonIdx++ )
			{
				// scan all buttons
				if(UiDriver_IsButtonPressed(buttonIdx))
				{
					// is this button pressed?
					numOfPressedButtons++;
					if(idxFirstPressedButton == 99)						// is this the first button pressed?
					{
						idxFirstPressedButton = buttonIdx;						// save button number
					}
				}
			}

			if(idxFirstPressedButton == BUTTON_BNDM_PRESSED && new_state == 0)	// delay if BANDM was used to enter button test mode
			{
				rebootCount = 0;
				new_state   = 1;
			}


			// now find out if an encoder was moved (we detect this by seeing a encoder value != 0)
			int32_t encoderDirection;

			uint32_t encoderIdx;
			for(encoderIdx = 0; encoderIdx < ENC_MAX; encoderIdx++)
			{
				encoderDirection = UiDriverEncoderRead(encoderIdx);
				if(encoderDirection != 0)
				{
					encoderCount = 200;
					break;
				}
			}

			if(encoderIdx != ENC_MAX)
			{
				snprintf(txt_buf,40," Encoder %ld <%s>", encoderIdx+1, encoderDirection>0 ? "right":"left");		// building string for encoders
				idxFirstPressedButton = BUTTON_NUM+encoderIdx;					// add encoders behind buttons;
			}

			if (idxFirstPressedButton < BUTTON_NUM)
			{
				txt = buttons[idxFirstPressedButton].label;
			}
			else
			{
				txt = NULL;
			}
			switch(idxFirstPressedButton)	 				// decode keyPin to text
			{
			case	BUTTON_PWR_PRESSED:
										if(poweroffCount > 75)
										{
											txt = "powering off...";
											p_o_state = 1;
										}
										poweroffCount++;
										break;
			case	BUTTON_BNDM_PRESSED:
										if(rebootCount > 75)
										{
											txt = "rebooting...";
											rb_state = 1;
										}
										rebootCount++;
										break;
			case	TOUCHSCREEN_ACTIVE:

				if (UiLcdHy28_TouchscreenHasProcessableCoordinates())
				{

					snprintf(txt_buf,40,"x/y: %04d/%04d x/y raw: %04x/%04x",ts.tp->hr_x,ts.tp->hr_y,ts.tp->xraw,ts.tp->yraw);	//show touched coordinates
					UiLcdHy28_PrintTextCentered(2,216,ts.Layout->Size.x-4,txt_buf,White,Blue,0);           // identify button on screen
					UiLcdHy28_DrawColorPoint(ts.tp->hr_x,ts.tp->hr_y,White);

					txt = "Touch";
				}
				else
				{
					if (mchf_touchscreen.present)
					{
						txt = "Touch (no coord.)";
					}
					else
					{
						txt = "Touch (no cntrlr)";
					}
				}
				break;
			case	BUTTON_NUM+ENC1:							// handle encoder event
			case	BUTTON_NUM+ENC2:
			case	BUTTON_NUM+ENC3:
            case    BUTTON_NUM+ENCFREQ:
										txt = txt_buf;
										break;
			default:
										if (txt == NULL)
										{
											if(encoderCount == 0)
											{
												txt = "<no key>";				// no keyPin pressed
											}
											else
											{
												encoderCount--;
											}
											poweroffCount = 0;
											rebootCount   = 0;
										}
			}

			if(txt != NULL)
			{
				UiLcdHy28_PrintTextCentered(0,120,ts.Layout->Size.x,txt,White,Blue,1);			// identify button on screen
			}

			snprintf(txt_buf,40, "# of buttons pressed: %ld  ", numOfPressedButtons);
			UiLcdHy28_PrintTextCentered(0,160,ts.Layout->Size.x,txt_buf,White,Blue,0);			// show number of buttons pressed on screen

			if(ts.tp->present)			// show translation of touchscreen if present
			{
				txt = "Touch Coordinates:";
			}
			else
			{
				txt = "Touch Controller not present";
			}

			UiLcdHy28_PrintTextCentered(0,200,ts.Layout->Size.x,txt,White,Blue,0);

			if(p_o_state == 1)
			{
                if(idxFirstPressedButton != BUTTON_PWR_PRESSED)
                {
                    Board_Powerdown();
                }
				// never reached
			}
			if(rb_state == 1)
			{
				if(idxFirstPressedButton != BUTTON_BNDM_PRESSED)
				{
					Board_Reboot();
				}
			}
		}
	}
}
//cross size definitions, must be odd
#define CrossSizeH 11
#define CrossSizeV 11
static  void  DrawCross( int16_t*  coord, uint16_t  color )
{
	UiLcdHy28_DrawStraightLine( coord[0]-(CrossSizeH/2), coord[1], CrossSizeH,        LCD_DIR_HORIZONTAL,color);
	UiLcdHy28_DrawStraightLine( coord[0], coord[1]-(CrossSizeV/2), CrossSizeV,        LCD_DIR_VERTICAL  ,color);
}


/*
 * @brief Touchscreen Calibration function
 * @returns false if it is a normal startup, true if touchscreen has been calibrated
 */

#define ARM_MATH_MATRIX_CHECK
#define Touch_ShowTestscreen

static void UiDriver_TouchscreenCalibrationRun()
{
    UiLcdHy28_TouchscreenReadCoordinates();
    ts.tp->state    = TP_DATASETS_NONE;
    uint16_t  MAX_X = ts.Layout->Size.x;
    uint16_t  MAX_Y = ts.Layout->Size.y;

    int16_t cross[5][4] =
    {
            {      20,       20, 0, 0 },
            {MAX_X-20,       20, 0, 0 },
            {      20, MAX_Y-20, 0, 0 },
            {MAX_X-20, MAX_Y-20, 0, 0 },
            { MAX_X/2, MAX_Y/2 , 0, 0 },
    };

    //reset calibration coefficients before acquiring points
    for(int16_t m=0; m<6; m++)
    {
        ts.tp->cal[m]=0;
    }

    ts.tp->cal[0]=65536;
    ts.tp->cal[4]=65536;

    for (int16_t idx = 0; idx < 5; idx++)
    {
        UiDriver_DoCrossCheck(cross[idx]);
    }

    //calibration algorithm based on publication:
    //"Calibration in touch-screen systems" Texas Instruments
    //Analog Applications Journal 3Q 2007

    /*//test vectors
    int16_t cross[0][4] = {     128,     384,1698,2258};
    int16_t cross[1][4] = {      64,     192, 767,1149};
    int16_t cross[2][4] = {     192,     192,2807,1327};
    int16_t cross[3][4] = {     192,     576,2629,3367};
    int16_t cross[4][4] = {      64,     576, 588,3189};*/

    //matrices field definitions
    float mA[3*5];
    float mAT[3*5];
    float mATAinv[3*3];
    float mbuff[3*3];
    float mcom[3*5];
    float mX[5];
    float mY[5];
    float mABC[3];
    float mDEF[3];

    //matrix data init
    for (int m=0; m < 5; m++)
    {
        mA[3*m + 0] = cross[m][2];
        mA[3*m + 1] = cross[m][3];
        mA[3*m + 2] = 1.0;
        mX[m]     = cross[m][0];
        mY[m]     = cross[m][1];
    }

    //create matrices instances
    arm_matrix_instance_f32  m_A, m_AT, m_ATAinv, m_X, m_Y, m_ABC, m_DEF, m_buff, m_com;

    //init of matrices
    arm_mat_init_f32 ( &m_A     , 5, 3, mA     );
    arm_mat_init_f32 ( &m_AT    , 3, 5, mAT    );
    arm_mat_init_f32 ( &m_ATAinv, 3, 3, mATAinv);
    arm_mat_init_f32 ( &m_X     , 5, 1, mX     );
    arm_mat_init_f32 ( &m_Y     , 5, 1, mY     );
    arm_mat_init_f32 ( &m_ABC   , 3, 1, mABC   );
    arm_mat_init_f32 ( &m_DEF   , 3, 1, mDEF   );
    arm_mat_init_f32 ( &m_buff  , 3, 3, mbuff  );
    arm_mat_init_f32 ( &m_com   , 3, 5, mcom   );

    //real computation
    arm_mat_trans_f32  ( &m_A     , &m_AT             );  //  A^T           size 5x3 -> 3x5
    arm_mat_mult_f32   ( &m_AT    , &m_A     , &m_buff);  //  A^T x A   size 3x5 * 5x3 -> 3x3
    arm_mat_inverse_f32( &m_buff  , &m_ATAinv         );  // (A^T x A)^-1  size 3x3
    arm_mat_mult_f32   ( &m_ATAinv, &m_AT    , &m_com );  // (A^T x A)^-1 x A^T   m_com is common matrix for estimating coefficients for X and Y      size 3x3 * 3x5 -> 3x5

    arm_mat_mult_f32( &m_com, &m_X, &m_ABC);   //calculating the coefficients for X data    size 3x5 * 5x1  -> 3x1
    arm_mat_mult_f32( &m_com, &m_Y, &m_DEF);   //calculating the coefficients for Y data    size 3x5 * 5x1  -> 3x1

    //store cal parameters
    for (int m=0; m < 3; m++)
    {
        ts.tp->cal[m  ] = mABC[m] * 65536;
        ts.tp->cal[m+3] = mDEF[m] * 65536;
    }
}

static bool UiDriver_TouchscreenCalibration()
{
	bool retval    = false;
	uint16_t MAX_X = ts.Layout->Size.x;
	uint16_t MAX_Y = ts.Layout->Size.y;

    bool run_calibration = false;

    const uint32_t clr_bg = Black;
    const uint32_t clr_fg = White;

    Keypad_Scan();

    //if (UiDriver_IsButtonPressed(TOUCHSCREEN_ACTIVE) && UiDriver_IsButtonPressed(BUTTON_F5_PRESSED))
    if (UiDriver_IsButtonPressed(TOUCHSCREEN_ACTIVE))
    {
        //wait for a moment to filter out some unwanted spikes
        HAL_Delay(500);
        Keypad_Scan();

        if(UiDriver_IsButtonPressed(TOUCHSCREEN_ACTIVE))
        {

            UiLcdHy28_LcdClear(clr_bg);

            if (ts.tp->present)
            {
                // now do all of the warnings, blah, blah...
                UiLcdHy28_PrintTextCentered(2,05,MAX_X-4,"TOUCH CALIBRATION",clr_fg,clr_bg,1);
                UiLcdHy28_PrintTextCentered(2, 70, MAX_X-4, "If you don't want to do this\n"
                        "press POWER button to start normally.\n"
                        " Settings will be saved at POWEROFF"
                        ,clr_fg,clr_bg,0);

                // delay a bit...
                HAL_Delay(3000);

                // add this for emphasis
                UiLcdHy28_PrintTextCentered(2, 195, MAX_X-4, "Press BAND+ and BAND-\n"
                        "to start calibration",clr_fg,clr_bg,0);

                UiDriver_WaitForBandMAndBandPorPWR();

                if (UiDriver_IsButtonPressed(BUTTON_PWR_PRESSED))
                {
                    UiLcdHy28_LcdClear(Black);							// clear the screen
                    UiLcdHy28_PrintTextCentered(2,108,MAX_X-4,"      ...performing normal start...",White,Black,0);
                    HAL_Delay(3000);
                }
                else
                {
                    run_calibration = true;
                }
            }
            else
            {
                UiLcdHy28_PrintTextCentered( 2, 05, MAX_X-4, "TOUCHSCREEN ERROR", clr_fg, clr_bg, 1 );
                UiLcdHy28_PrintTextCentered( 2, 70, MAX_X-4, "A touchscreen press was detected\n"
                        "but no touchscreen controller found\n"
                        "Calibration cannot be executed!"
                        ,clr_fg,clr_bg,0);
                // delay a bit...
                HAL_Delay(3000);
            }
		}
	}

	if (run_calibration)
	{
	    UiLcdHy28_LcdClear(clr_bg);
	    UiLcdHy28_PrintTextCentered(2,70, MAX_X-4,
	            "On the next screen crosses will appear.\n"
	            "Touch as exact as you can on the middle\n"
	            "of each cross. After three valid\n"
	            "samples position of cross changes.\n"
	            "Repeat until the five test positions\n"
	            "are finished.",clr_fg,clr_bg,0);

	    UiLcdHy28_PrintTextCentered(2,195,MAX_X-4,"Touch at any position to start.",clr_fg,clr_bg,0);

 	    UiDriver_WaitForButtonPressed(TOUCHSCREEN_ACTIVE);

	    UiLcdHy28_LcdClear(clr_bg);
	    UiLcdHy28_PrintTextCentered( 2, 100, MAX_X-4, "Wait one moment please...", Yellow, clr_bg, 0 );
	    HAL_Delay(1000);

	    UiDriver_TouchscreenCalibrationRun();

	    UiLcdHy28_LcdClear(clr_bg);

#ifdef Touch_ShowTestscreen
	    UiLcdHy28_PrintTextCentered(2, 195, MAX_X-4, "Press BAND+ and BAND-\n"
	            "to run drawing on screen\n"
	            "or POWER to boot",clr_fg,clr_bg,0);

	    UiDriver_WaitForBandMAndBandPorPWR();

	    if (UiDriver_IsButtonPressed(BUTTON_PWR_PRESSED))
	    {
	        UiLcdHy28_LcdClear(Black);                          // clear the screen
	        UiLcdHy28_PrintTextCentered(2,108,MAX_X-4,"      ...performing normal start...",White,Black,0);
	        HAL_Delay(150); ///  NIZZZ  ajout� pour d�boger le problem absence son au d�marrage
	    }
	    else
        {
            UiLcdHy28_LcdClear(clr_bg);
            UiLcdHy28_PrintTextCentered(2, MAX_Y/2-8, MAX_X-4, "Test screen.\n"
                    "You can draw by pressing the screen.\n"
                    "Press Power to boot",clr_fg,clr_bg,0);
            while(1)
            {
                do
                {
                    HAL_Delay(10);
                    Keypad_Scan();
                } while (UiDriver_IsButtonPressed(TOUCHSCREEN_ACTIVE) == false && UiDriver_IsButtonPressed(BUTTON_PWR_PRESSED) == false);

                if(UiDriver_IsButtonPressed(BUTTON_PWR_PRESSED) == true)
                {
                    UiLcdHy28_LcdClear(Black);                          // clear the screen
                    UiLcdHy28_PrintTextCentered(2,108,MAX_X-4,"      ...performing normal start...",White,Black,0);

                    HAL_Delay(200); ///  NIZZZ  ajout� pour d�boger le problem absence son au d�marrage

                    break;
                }

                if (UiLcdHy28_TouchscreenHasProcessableCoordinates())
                {
                    //          *xt_corr += (ts.tp->hr_x - cross[0]);
                    //          *yt_corr += (ts.tp->hr_y - cross[1]);
                    UiLcdHy28_DrawColorPoint( ts.tp->hr_x, ts.tp->hr_y, White );
                }

            }
        }
#endif
	    HAL_Delay(2000);
	    retval              = true;
	    ts.menu_var_changed = true;
	}
	return retval;
}

#define    CrossCheckCount    10      // 3

void UiDriver_DoCrossCheck ( int16_t cross[] )
{
	uint16_t  MAX_X = ts.Layout->Size.x;
	uint32_t  clr_fg, clr_bg;
	clr_bg          = Black;
	clr_fg          = White;

	UiLcdHy28_LcdClear( clr_bg );
	DrawCross(  cross,  clr_fg );

	char   txt_buf[ 40 ];
	uchar  datavalid = 0;  // samples = 0;

	// int16_t*  xt_corr = &cross[2];
	// int16_t*  yt_corr = &cross[3];

	int32_t  xt_corr = 0;
	int32_t  yt_corr = 0;

	do
	{
	    UiDriver_WaitForButtonPressed ( TOUCHSCREEN_ACTIVE );

		if ( UiLcdHy28_TouchscreenHasProcessableCoordinates() )
		{
			//if(abs(ts.tp->hr_x - cross[0]) < MaxTouchError && abs(ts.tp->hr_y - cross[1]) < MaxTouchError)
			//{
				datavalid++;
				xt_corr += ts.tp->hr_x;
				yt_corr += ts.tp->hr_y;
				clr_fg    = Green;
				snprintf ( txt_buf, 40, "Try (%d) error: x = %+d / y = %+d", datavalid, ts.tp->hr_x - cross[0], ts.tp->hr_y - cross[1]);	//show misajustments
			/*}
			else
			{
				clr_fg = Red;
				snprintf(txt_buf,40,"Try (%d) BIG error: x = %+d / y = %+d",samples,ts.tp->hr_x-cross[0],ts.tp->hr_y-cross[1]);	//show misajustments
			}*/
			// samples++;
			UiLcdHy28_PrintTextCentered( 2, 70, MAX_X-4, txt_buf, clr_fg, clr_bg, 0 );

			snprintf( txt_buf, 40, "RAW: x = %+d / y = %+d", ts.tp->xraw, ts.tp->yraw );	//show misajustments
			UiLcdHy28_PrintTextCentered( 2, 85, MAX_X-4, txt_buf, clr_fg, clr_bg, 0 );
			ts.tp->state = TP_DATASETS_PROCESSED;
		}
	}
	while( datavalid < CrossCheckCount );

	UiLcdHy28_PrintTextCentered( 2, 100, MAX_X-4, "Wait one moment please...", Yellow, clr_bg, 0 );

	xt_corr  /=  CrossCheckCount;   cross[2] = xt_corr; //average the data
	yt_corr  /=  CrossCheckCount;   cross[3] = yt_corr;

	HAL_Delay( 2000 );
}


static  uint16_t  startUpScreen_nextLineY;
static  bool      startUpError = false;

/**
 * @brief use this method to report initialization problems on splash screen, may only be used during splash screen presence (!)
 *
 * @param isProblem if set to true, the problem is reported on screen, otherwise nothing is done
 * @param txt pointer to a text string characterizing the problem detected
 *
 */
void UiDriver_StartupScreen_LogIfProblem(bool isProblem, const char* txt)
{
	if (isProblem)
	{
		startUpScreen_nextLineY = UiLcdHy28_PrintTextCentered( ts.Layout->StartUpScreen_START.x, startUpScreen_nextLineY, 320, txt, Black, Red2, 0);
		startUpError = true;
	}
}

static  uint16_t   fw_version_number_major   = 0;    // save new F/W version
static  uint16_t   fw_version_number_release = 0;
static  uint16_t   fw_version_number_minor   = 0;

/**
 * @returns true if the firmware version is different from version in loaded configuraton settings.
 */
static bool UiDriver_FirmwareVersionCheck()
{

	fw_version_number_major   = atoi(UHSDR_VER_MAJOR  );    // save new F/W version
	fw_version_number_release = atoi(UHSDR_VER_RELEASE);
	fw_version_number_minor   = atoi(UHSDR_VER_MINOR  );

	return ((ts.version_number_major != fw_version_number_major) || (ts.version_number_release != fw_version_number_release) || (ts.version_number_minor != fw_version_number_minor));        // Yes - check for new version
}
/**
 * @brief basically does nothing but settiSng the firmware number of configuration to number of running fw
 */
static void UiDriver_FirmwareVersionUpdateConfig()
{

	if (UiDriver_FirmwareVersionCheck())
	{
		ts.version_number_major   = fw_version_number_major;    // save new F/W version
		ts.version_number_release = fw_version_number_release;
		ts.version_number_minor   = fw_version_number_minor;

	}
}


/**
 * @brief show initial splash screen, we run AFTER the configuration storage has been initialized!
 * @param hold_time how long the screen to be shown before proceeding (in ms)
 */
void UiDriver_StartUpScreenInit()
{
	char     tx[100];
	uint32_t clr;
	// Clear all
	UiLcdHy28_LcdClear(Black);
	uint16_t    nextY =  ts.Layout->StartUpScreen_START.y;
	snprintf( tx, 100, "%s", DEVICE_STRING );
	nextY  =  UiLcdHy28_PrintTextCentered ( ts.Layout->StartUpScreen_START.x,  nextY, 320, tx, Cyan, Black, 1 );

#ifdef TRX_HW_LIC
	snprintf ( tx, 100, "Hardware License: %s", TRX_HW_LIC );
	nextY = UiLcdHy28_PrintTextCentered ( ts.Layout->StartUpScreen_START.x, nextY + 3, 320,            tx, White, Black, 0 );
#endif
#ifdef TRX_HW_CREATOR
	nextY = UiLcdHy28_PrintTextCentered ( ts.Layout->StartUpScreen_START.x, nextY    , 320, TRX_HW_CREATOR, White, Black, 0);
#endif

	snprintf( tx, 100, "%s%s","UHSDR + Vers. ", UiMenu_GetSystemInfo(&clr, INFO_FW_VERSION ));
	nextY = UiLcdHy28_PrintTextCentered ( ts.Layout->StartUpScreen_START.x, nextY + 8, 320, tx, Yellow, Black, 1);

	nextY = UiLcdHy28_PrintTextCentered ( ts.Layout->StartUpScreen_START.x, nextY + 3, 320, "Firmware License: " UHSDR_LICENCE , White, Black, 0);

	// show important error status
	startUpScreen_nextLineY = nextY + 8; // reset y coord to first line of error messages

	UiLcdHy28_BacklightEnable(true);

}

void UiDriver_StartUpScreenFinish()
{
	const     char*  txp;
	char      tx[100];
	uint32_t  clr, fg_clr;

	uint32_t hold_time;

	UiDriver_StartupScreen_LogIfProblem ( osc->isPresent() == false, "Local Oscillator NOT Detected!" );

/*	if(!Si5351a_IsPresent())
	{
		UiDriver_StartupScreen_LogIfProblem ( lo.sensor_present == false, "MCP9801 Temp Sensor NOT Detected!");
	}
*/
	if(ts.configstore_in_use == CONFIGSTORE_IN_USE_ERROR)                                   // problem with EEPROM init
	{
#ifdef USE_CONFIGSTORAGE_FLASH
	    UiDriver_StartupScreen_LogIfProblem ( ts.ee_init_stat != HAL_OK, "Config Flash Error");
#endif
	    if ( SerialEEPROM_eepromTypeDescs [ ts.ser_eeprom_type ].size == 0)
	    {
	        snprintf ( tx, 100, "Config EEPROM: %s", SerialEEPROM_eepromTypeDescs[ts.ser_eeprom_type].name );
	        UiDriver_StartupScreen_LogIfProblem ( true, tx );
	    }
	}

	if(!Si5351a_IsPresent())
	{
	  UiDriver_StartupScreen_LogIfProblem ( (HAL_ADC_GetValue(&hadc2) > MAX_VSWR_MOD_VALUE) && (HAL_ADC_GetValue(&hadc3) > MAX_VSWR_MOD_VALUE),
			"SWR Bridge resistor mod NOT completed!");
	}
	if (UiDriver_FirmwareVersionCheck())
	{
		hold_time               = 10000; // 15s
		txp                     = "Firmware change detected!\nPlease review settings!";
		startUpScreen_nextLineY = UiLcdHy28_PrintTextCentered(ts.Layout->StartUpScreen_START.x,startUpScreen_nextLineY + 10,320,txp,White,Black,0);

		UiDriver_FirmwareVersionUpdateConfig();
	}

	if(startUpError == true)
	{
		hold_time = 15000; // 15s
		txp       = "Boot Delay because of Errors or Warnings";
		clr       = Red3;
		fg_clr    = Black;
	}
	else
	{
		hold_time = 3000; // 3s
		txp       = "...starting up normally...";
		clr       =  Black;
		fg_clr    = Green;
	}

	UiLcdHy28_PrintTextCentered(ts.Layout->StartUpScreen_START.x,startUpScreen_nextLineY + 10,320,txp,fg_clr,clr,0);

	HAL_Delay(hold_time);

	UiDriver_CreateDesktop();
	UiDriver_UpdateDisplayAfterParamChange();
}

// UiAction_... are typically small functions to execute a specific ui function initiate by a key press or touch event
// they take no argument and return nothing. If possible, try to keep the function atomic and independent from the
// key or touch region it is assigned to. I.e. it is better not to implement 2 functions based on menu mode or not here
// this logic should done separately so that the resulting action is reusable in different activation scenarios (touch or key or even CAT)
// If execution of an action is not applicable all the times it is necessary to check if the function should check that
// by itself and become a "No operation" or even issues a message, or if this is to be implicitly handled by the use of the
// function only in certain modes of operation through the modes tables.

// TODO: Make Atomic
static void UiAction_ChangeLowerMeterDownOrSnap()
{

	sc.snap = 1;

	// Not in MENU mode - select the METER mode
//	decr_wrap_uint8( &ts.tx_meter_mode, 0, METER_MAX - 1 );
//	UiDriver_DeleteMeters();
//	UiDriver_CreateMeters();    // redraw meter
}

void UiAction_ChangeLowerMeterUp()
{
	incr_wrap_uint8( &ts.tx_meter_mode, 0, METER_MAX - 1 );
	UiDriver_DeleteMeters();
	UiDriver_CreateMeters();	// redraw meter
	if (ks.button_id == 17 ) AudioManagement_KeyBeep();  // Beep if touchscreen commande
}

///******************************************************************
void UiDriver_UpdateDSPmode()
{
	// bool hold_paddles = ts.paddles_active;   ts.paddles_active = false;

	ts.audio_dac_muting_buffer_count += 50;   // mute qq temps le  Rx processing  du son
	//  ads.adc_clip = 0;   ads.adc_half_clip = 0;   ads.adc_quarter_clip = false; // pour ne pas causer de changement de gain codec non souhait�
	if ( ts.switch_pause < 300 )  ts.switch_pause  += 300;


	//loop for detection of first possible DSP function to switch it on if others are disabled/not allowed
	for ( int i = 0; i < DSP_SWITCH_MAX;  i++ )
	{
		//
		// prevent certain modes to prevent CPU crash
		//

		if ( ts.dsp_mode == DSP_SWITCH_NR_AND_NOTCH && ts.dmod_mode == DEMOD_CW )          ts.dsp_mode++; // prevent NR AND NOTCH, when in CW
		if ( ts.dsp_mode == DSP_SWITCH_NOTCH        && ts.dmod_mode == DEMOD_CW )          ts.dsp_mode++; // prevent NOTCH, when in CW
		if ( ts.dsp_mode == DSP_SWITCH_NR_AND_NOTCH &&(ts.dmod_mode == DEMOD_AM )                        // prevent NR AND NOTCH, when in AM and decimation rate
	    && ( FilterPathInfo[ts.filter_path].sample_rate_dec == RX_DECIMATION_RATE_24KHZ )) ts.dsp_mode++;// equals 2 --> high CPU load)
		if ( ts.dsp_mode >= DSP_SWITCH_MAX )                                               ts.dsp_mode = DSP_SWITCH_OFF; // flip round
		if(((ts.dsp_mode_mask & ( 1 << ts.dsp_mode))==0))                                  ts.dsp_mode++;
		else break;
		if ( ts.dsp_mode >= DSP_SWITCH_MAX)                                                ts.dsp_mode = DSP_SWITCH_OFF; // flip round
		if ( ts.dsp_mode == 0 )	  break;	//safe exit because there is always DSP OFF option at which we can stop
	}




	switch ( ts.dsp_mode )
	{

		case DSP_SWITCH_OFF: // switch off everything
							ts.dsp_active   =  (ts.dsp_active & ~(DSP_NR_ENABLE|DSP_NOTCH_ENABLE|DSP_MNOTCH_ENABLE|DSP_MPEAK_ENABLE));
							ts.enc_two_mode =  ENC_TWO_MODE_RF_GAIN;
							break;

		case DSP_SWITCH_NR:
							ts.dsp_active   =  DSP_NR_ENABLE | (ts.dsp_active & ~(DSP_NOTCH_ENABLE | DSP_MNOTCH_ENABLE | DSP_MPEAK_ENABLE));
							ts.enc_two_mode = ENC_TWO_MODE_NR;
							break;

		case DSP_SWITCH_NOTCH:
							ts.dsp_active   =  DSP_NOTCH_ENABLE | (ts.dsp_active & ~(DSP_NR_ENABLE|DSP_MNOTCH_ENABLE|DSP_MPEAK_ENABLE));
							ts.enc_two_mode =  ENC_TWO_MODE_RF_GAIN;
							break;

		case DSP_SWITCH_NR_AND_NOTCH:
							ts.dsp_active   =  DSP_NOTCH_ENABLE | DSP_NR_ENABLE | (ts.dsp_active & ~(DSP_MNOTCH_ENABLE|DSP_MPEAK_ENABLE));
							ts.enc_two_mode =  ENC_TWO_MODE_NR;
							break;

		case DSP_SWITCH_NOTCH_MANUAL:
							ts.dsp_active   =  DSP_MNOTCH_ENABLE | (ts.dsp_active & ~(DSP_NR_ENABLE|DSP_NOTCH_ENABLE|DSP_MPEAK_ENABLE));
							ts.enc_two_mode =  ENC_TWO_MODE_NOTCH_F;
							break;

		case DSP_SWITCH_PEAK_FILTER:
							ts.dsp_active   =  DSP_MPEAK_ENABLE | (ts.dsp_active & ~(DSP_NR_ENABLE|DSP_NOTCH_ENABLE|DSP_MNOTCH_ENABLE));
							ts.enc_two_mode =  ENC_TWO_MODE_PEAK_F;
							break;

		default:  			break;
	}

	ts.dsp_active_toggle = ts.dsp_active;  // save update in "toggle" variable



	// reset DSP NR coefficients
	AudioDriver_SetRxAudioProcessing ( ts.dmod_mode, true );        // update DSP/filter settings
	UiDriver_DisplayEncoderTwoMode ();                              // DSP control is mapped to column 2
	if ( ts.dsp_mode != DSP_SWITCH_OFF )  ts.RFG_wait = 9000;

	non_os_delay(); non_os_delay(); non_os_delay(); non_os_delay(); non_os_delay(); // pour ne pas bloquer les 2 equaliseurs parametriques lors

	// du 1er d�marrage de la fonction DSP

	// ts.paddles_active = hold_paddles ;
}
//*******************************************************************************************************************

static void UiAction_ChangeToNextDspMode()
{
	 ts.audio_dac_muting_buffer_count += 25;  // muter qq temps le processing Rx du son

	static bool  first_init = true;   //  variable ajout�e pour initialiser le bouton DSP (G2) au lancement

	if (ts.switch_pause     < 500 )   ts.switch_pause += 500;           //  pour suspendre momentanement les calculs correc IQ NIZZZ
	if ( ts.dmod_mode != DEMOD_FM )	  //  allow selection/change of DSP only if NOT in FM
	{

		if (first_init)
		{
			first_init = false;
			if ( is_dsp_nr() )
			{
				if  (is_dsp_notch ())  { ts.dsp_mode = DSP_SWITCH_NR_AND_NOTCH;  }
				else                   { ts.dsp_mode = DSP_SWITCH_NR;            }
			}
			else if (is_dsp_notch ())  { ts.dsp_mode = DSP_SWITCH_NOTCH;         }
			else if (is_dsp_mnotch())  { ts.dsp_mode = DSP_SWITCH_NOTCH_MANUAL;  }
			else if (is_dsp_mpeak ())  { ts.dsp_mode = DSP_SWITCH_PEAK_FILTER;   }
		}

		//
		// I think we should alter this to use a counter
		// What do we want to switch here:
		// NR ON/OFF		ts.dsp_active |= DSP_NR_ENABLE;	 // 	ts.dsp_active &= ~DSP_NR_ENABLE;
		// NOTCH ON/OFF		ts.dsp_active |= DSP_NOTCH_ENABLE; // 	ts.dsp_active &= ~DSP_NOTCH_ENABLE;
		// Manual Notch		ts.dsp_active |= DSP_MNOTCH_ENABLE
		// BASS				ts.bass // always "ON", gain ranges from -20 to +20 dB, "OFF" = 0dB
		// TREBLE			ts.treble // always "ON", gain ranges from -20 to +20 dB, "OFF" = 0dB

		ts.dsp_mode ++; // switch mode
		// 0 = everything OFF, 1 = NR, 2 = automatic NOTCH, 3 = NR + NOTCH, 4 = manual NOTCH, 5 = BASS adjustment, 6 = TREBLE adjustment
		if ( ts.dsp_mode >= DSP_SWITCH_MAX ) ts.dsp_mode = DSP_SWITCH_OFF; // flip round

		UiDriver_UpdateDSPmode();
	}   else first_init = false;
}
//********************************************************************************************************************

void UiAction_ChangeSpectrumSize()
{
    ts.menu_var_changed = 1;
    if (ts.spectrum_size == SPECTRUM_BIG)
    {
        ts.spectrum_size = SPECTRUM_NORMAL;
    }
    else
    {
        ts.spectrum_size = SPECTRUM_BIG;
    }

    UiDriver_SpectrumChangeLayoutParameters();
}


void UiAction_ChangeSpectrumZoomLevelDown()
{
	ts.menu_var_changed = 1;
	decr_wrap_uint8( &sd.magnify, MAGNIFY_MIN, MAGNIFY_MAX );

	UiDriver_SpectrumChangeLayoutParameters();
}

void UiAction_ChangeSpectrumZoomLevelUp()
{
	ts.menu_var_changed = 1;
	incr_wrap_uint8( &sd.magnify, MAGNIFY_MIN, MAGNIFY_MAX );
	UiDriver_SpectrumChangeLayoutParameters();
}

void UiAction_ChangeFrequencyToNextKhz()
{

	df.tune_new = floor((df.tune_new + 500)/ 1000) * 1000;	// set last three digits to "0"
	// UiDriver_FrequencyUpdateLOandDisplay(true);
	if (ks.button_id == 17 ) AudioManagement_KeyBeep(); // Beep if touchscreen commande
}

void   UiAction_ChangeFrequency_Sub500Hz()     /// NIZZ
{
	df.tune_new = floor((df.tune_new - 500)/ 500) * 500;	// set last three digits to "0"

	if (ks.button_id == 17 ) AudioManagement_KeyBeep(); // Beep if touchscreen commande
}

void   UiAction_ChangeFrequency_Add500Hz()
{
	df.tune_new = floor((df.tune_new + 500)/ 500) * 500;	// set last three digits to "0"

	if (ks.button_id == 17 ) AudioManagement_KeyBeep(); // Beep if touchscreen commande
}

void UiAction_ToggleWaterfallScopeDisplay()
{
	ts.switch_pause    += 300; // pour suspendre momentanement les calculs correc IQ
    SpectrumMode_t temp = UiDriver_GetSpectrumMode();

    if (temp != SPECTRUM_BLANK)
    {
        // we want range 0 - 2 instead of the normal 1 - 3
        temp--;
    }
    temp++;
    temp%=3;
    temp++;
    UiDriver_SetSpectrumMode(temp);
    UiSpectrum_ResetSpectrum();
    UiSpectrum_Init();   // init spectrum display
}
///***********************************************************************
///
void UiAction_ChangeDemodMode()
{
	if((!ts.tune) && (ts.txrx_mode == TRX_MODE_RX))	 	// do NOT allow mode change in TUNE mode or transmit mode
	{
		// bool hold_paddles = ts.paddles_active;
		// ts.paddles_active = false;

		ts.switch_pause                   = 700; // pour suspendre momentan�ment le calcul non prioritaire de reajustement des correction IQ, besoin du processeur
		ts.audio_dac_muting_buffer_count += 50;

		UiDriver_ChangeToNextDemodMode(0);
		Board_RedLed(LED_STATE_OFF);
		RadioManagement_Check_Initialise_Tx_decimated_filters ( false );
		ts.nr_first_time =  1;
		// ts.paddles_active = hold_paddles ;
		if (ks.button_id == 17 ) AudioManagement_KeyBeep(); // if touchscreen press then Beep
	}
}
///********************************************************************************************************

static void UiAction_ChangeDemodModeToAlternativeMode()
{
	if((!ts.tune) && (ts.txrx_mode == TRX_MODE_RX))	 	// do NOT allow mode change in TUNE mode or transmit mode
	{
		// bool hold_paddles = ts.paddles_active;
		// ts.paddles_active = false;

		ts.audio_dac_muting_buffer_count += 50;

		UiDriver_ChangeToNextDemodMode(1);

		ts.nr_first_time =  1;
		// ts.paddles_active = hold_paddles ;
	}
}

//********************************************************************************************************

void UiAction_ChangePowerLevel()
{
	UiDriver_HandlePowerLevelChange(ts.power_level+1);

	if (ks.button_id == 17 ) AudioManagement_KeyBeep(); // if touchscreen then Beep
}

void UiAction_ChangeAudioSource()
{

	if(ts.dmod_mode != DEMOD_CW)
	{
		if (ts.enc_thr_mode == ENC_THREE_MODE_INPUT_CTRL)  incr_wrap_uint8(&ts.tx_audio_source,0,TX_AUDIO_MAX_ITEMS);
		ts.enc_thr_mode      = ENC_THREE_MODE_INPUT_CTRL; //   ENC_THREE_MODE_RIT;  NIZZZ pour activer le Pad par touchscreen
		UiDriver_DisplayEncoderThreeMode();

		if (ks.button_id == 17 ) {  AudioManagement_KeyBeep(); } // if touchscreen then Beep
	}

}
//***********************************************************************************
#ifdef ATTENUATOR_6DB

void UiAction_Attenuator_switch()
{
	if ( ts.display->use_spi == true  )
	{
		if ( !ts.show_debug_info)
		{
			ts.antenna_attenuator_enable ^=  0x01;
			if (ts.antenna_attenuator_enable) { GPIO_ResetBits( ATTENUATION_6DB_PIO, ATTENUATION_6DB );} // activer Attenuateur -6dB
			else                              { GPIO_SetBits  ( ATTENUATION_6DB_PIO, ATTENUATION_6DB );} // inhiber Attenuateur -6dB
		}
		else
		{
			ts.show_debug_info = 0;
			UiDriver_DebugInfo_DisplayEnable ( ts.show_debug_info );
		}
		ts.old_rfg_calc = 100;
	}
	else
	{
		ts.antenna_attenuator_enable = 0;  // forcer le variable a False si display parallêle
	}
}

#endif
//************************************************************************************************************

void UiAction_Debug_switch()
{
	ts.show_debug_info = ! ts.show_debug_info;
	UiDriver_DebugInfo_DisplayEnable ( ts.show_debug_info );
	if ( !ts.show_debug_info) { ts.old_rfg_calc = 100; }
}

//**************************************************************************************************************

void UiAction_RAZ_RIT_And_Display()  //  NIZZZ  nouvelle fonction ajout� pour mettre a zero le RIT par touchscreen
{
	// if (ts.rit_value != 0)
	{
		ts.dial_moved   = true;
		ts.rit_value    = 0;
		ts.enc_thr_mode = ENC_THREE_MODE_RIT;
		UiDriver_DisplayEncoderThreeMode();   // UiDriver_DisplayRit(1);
		//  if ( is_splitmode() )
		UiDriver_FrequencyUpdateLOandDisplay ( false );

		if (ks.button_id == 17 ) AudioManagement_KeyBeep(); // if touchscreen then Beep
	}
}

		// TODO: Decide if we really want to switch
		// order like for the normal buttons
void UiAction_ChangeBandDownOrUp()
{
	UiDriver_HandleBandButtons(BUTTON_BNDM);
}


void UiAction_ChangeBandUpOrDown()
{
	UiDriver_HandleBandButtons( BUTTON_BNDP );
}

static void UiAction_SaveConfigurationToMemory()
{
	ts.switch_pause += 950; // pour suspendre momentan�ment le calcul non prioritaire de reajustement des correction IQ, besoin du processeur
	if(ts.txrx_mode == TRX_MODE_RX)	 				// only allow EEPROM write in receive mode
	{
		UiSpectrum_Clear();
		UiDriver_SaveConfiguration();
		HAL_Delay(3000);

		ts.menu_var_changed = 0;                    // clear "EEPROM SAVE IS NECESSARY" indicators
		UiDriver_DisplayFButton_F1MenuExit();

		if(ts.menu_mode)
		{
			UiMenu_RenderMenu(MENU_RENDER_ONLY);    // update menu display, was destroyed by message
		}
		else
		{
			UiSpectrum_Init();          // not in menu mode, redraw spectrum scope
		}
	}
}

static void UiDriver_DrawGraticule_Rect(bool show)
{
	uint16_t pos_y=sd.Slayout->graticule.y;
	if(show)
	{

		UiLcdHy28_PrintText     (sd.Slayout->graticule.x+5                         ,pos_y+4, "CHOOSE NEW POSITION"    , White,Black              , 4);
		UiLcdHy28_PrintText     (sd.Slayout->graticule.x+sd.Slayout->graticule.w-65,pos_y+4, "| OK |"                 , Green,Black              , 4);
		UiLcdHy28_PrintText     (sd.Slayout->graticule.x+sd.Slayout->graticule.w-20,pos_y+4, "X"                      , Orange,Black             , 4);
		UiLcdHy28_DrawEmptyRect (sd.Slayout->graticule.x+1                         ,pos_y+1, sd.Slayout->graticule.h-3, sd.Slayout->graticule.w-3, White);
	}
	else
	{
		//clear the graticule area
		UiLcdHy28_DrawFullRect(sd.Slayout->graticule.x+1,pos_y+1,sd.Slayout->graticule.h-2,sd.Slayout->graticule.w-2,Black);
	}
}

/*
 * Special actions for long pressed spectrum/waterfall area
 */
void UiAction_CheckSpectrumTouchActions()
{
	//UiArea_t* ScGRAT_area=UiSpectrum_GetSpectrumGraticule();	//fetch the current scope graticule area pointer

	if(UiDriver_CheckTouchRegion(&sd.Slayout->graticule)			//check for spectrum/waterfall size ratio change
			&&ts.txrx_mode == TRX_MODE_RX)
	{
		if(ts.SpectrumResize_flag==0)
		{
			UiSpectrum_Clear();
			UiDriver_DrawGraticule_Rect(true);		//draw new graticule control
			UiLcdHy28_DrawEmptyRect(sd.Slayout->full.x,sd.Slayout->full.y,sd.Slayout->full.h-1,sd.Slayout->full.w-1,White);
			ts.SpectrumResize_flag = 1;
			return;
		}
	}
}

//SpectrumVirtualKeys_flag
///***********************************************************************************************
///
///
///
 static  uint8_t   delay_to_change_freq_by_touch = 0;    /// NIZZZ


void UiAction_Skeep48KhzUpByTouch()
{

	if (delay_to_change_freq_by_touch == 0)
	{
		ts.switch_pause               = 500;// X 0.666 ms pour suspendre momentanement les calculs correc IQ NIZZZ
		delay_to_change_freq_by_touch = 40; // X10 ms pour temporiser avant la prochaine touschscreen function
		ts.dynamic_tune_activ_counter = 42; // X10 ms pour coloration jaune des digits de la frequence
		df.tune_new                  += ( 48000 >> sd.magnify ) ;
		// UiDriver_FrequencyUpdateLOandDisplay ( true );
		AudioManagement_KeyBeep ();
		ts.tempo_avg_FFT = 75;
	}
}
//********************************************************
void  UiAction_Skeep48KhzDownByTouch()
{

	if (delay_to_change_freq_by_touch == 0)
	{
		ts.switch_pause               = 500; //X 0.666 ms pour suspendre momentanement les calculs correc IQ NIZZZ
		delay_to_change_freq_by_touch = 40; // X10 ms pour temporiser avant la prochaine touschscreen function
		ts.dynamic_tune_activ_counter = 48; // X10 ms pour coloration jaune des digits de la frequence
		df.tune_new                  -= (48000 >> sd.magnify);
		// UiDriver_FrequencyUpdateLOandDisplay(true);
		AudioManagement_KeyBeep();
		ts.tempo_avg_FFT = 75;
	}
}

//*************************************************************************************************


void UiAction_ChangeFrequencyByTouch()    /// NIZZZ
{


	if (  delay_to_change_freq_by_touch == 0)
	{
		ts.tempo_avg_FFT = 75;
		// AudioManagement_KeyBeep(); // if touchscreen then Beep
		if (ts.frequency_lock == false)
		{
			  delay_to_change_freq_by_touch =  40; //38  // X10 ms  pour temporiser avant la prochaine touschscreen function

								  int32_t step = 1000;		// adjust to 1000Hz
			if      (sd.magnify == 1)	{ step =  500; }	// adjust to 500Hz
			else if (sd.magnify == 2)   { step =  100; }	// adjust to 100Hz
			else if (sd.magnify == 3)	{ step =   50; }	// adjust to 50Hz
			else if (sd.magnify == 4)	{ step =    1; }	// adjust to 1Hz

			if      (ts.dmod_mode == DEMOD_AM || ts.dmod_mode == DEMOD_SAM)	{ step =  5000;}	// adjust to 5KHz
			else if (ts.dmod_mode == DEMOD_CW )  	                        { step =     1;}	// adjust to 1 Hz

			//int16_t line =sd.marker_pos[0] + UiSpectrum_GetSpectrumStartX();

			// int16_t line = sd.marker_pos[0] + sd.Slayout->scope.x - sd.line_offset;

		 	 int16_t  line = 159;
            if ( sd.magnify == 0)  // X1 Zoom
            {
            	switch(ts.iq_freq_mode)
				{
					case FREQ_IQ_CONV_M12KHZ  :	line   =  239;   break;
					case FREQ_IQ_CONV_M6KHZ   : line   =  199;   break;
					case FREQ_IQ_CONV_P12KHZ  :	line   =   79;   break;
					case FREQ_IQ_CONV_P6KHZ   :	line   =  119;   break;
				}
            }


			/*int16_t line =sd.rx_carrier_pos;
			if(ts.dmod_mode == DEMOD_CW)
			{
				line=sd.marker_pos[0];
			}*/

			//uint32_t tunediff = ((IQ_SAMPLE_RATE/slayout.scope.w)/(1 << sd.magnify))*(ts.tp->hr_x-line);

		//	int32_t tunediff       =  sd.hz_per_pixel*(ts.tp->hr_x - line);

			int32_t  tunediff   ;
		/*	if (ts.dmod_mode == DEMOD_USB )
			{
				 tunediff =  sd.hz_per_pixel * UISpectrum_track_optimal_USB_Tune( ts.tp->hr_x - line);
			}
			else if (ts.dmod_mode == DEMOD_LSB )
			{
				 tunediff =  sd.hz_per_pixel * UISpectrum_track_optimal_LSB_Tune( ts.tp->hr_x - line);
			}
			else tunediff =  sd.hz_per_pixel*(ts.tp->hr_x-line);
	*/
			// tunediff     =  ( sd.hz_per_pixel * UISpectrum_track_optimal_SSB_Tune( ts.tp->hr_x - line)  )/16;  // line-->64

			tunediff     =  ( sd.hz_per_pixel * 1.25 * UISpectrum_track_optimal_SSB_Tune( 0.8 * ( ts.tp->hr_x - line) )   )/16;  // line-->64

			if (ts.dmod_mode  == DEMOD_CW )
			{
				int32_t         add_offset   = ts.cw_sidetone_freq;
				if (ts.cw_lsb) 	{ add_offset = - add_offset ; }
				df.tune_new                 += tunediff - add_offset;
			}
			else
			{
				df.tune_new  =  (( tunediff  + df.tune_new +  (step >> 1) - 1 ) / step) * step;
			}
		// 	UiDriver_FrequencyUpdateLOandDisplay ( true );
			AudioManagement_KeyBeep();
			delay_to_change_freq_by_touch +=  4;   // pour temporiser avant la prochaine touschscreen function

					/* if ( ! ts.show_debug_info )
			    	{
			    		char txt[12];
			    		snprintf ( txt, 12, "%4d", line);
			    		UiLcdHy28_PrintText ( ts.Layout->DisplayDbm.x,   ts.Layout->DisplayDbm.y, txt, White, Black, 0);
			    	} */
		}
	}
}
///******************************************************************************************
///
void UiAction_ChangeDigitalMode()
{

	incr_wrap_uint8(&ts.digital_mode,0,DigitalMode_Num_Modes-1);
	// We limit the reachable modes to the ones truly available
	// which is FreeDV1, RTTY, BPSK for now
	UiDriver_ToggleDigitalMode();
	if (ks.button_id == 17 ) AudioManagement_KeyBeep(); // if touchscreen then Beep
}

void UiAction_ChangeDynamicTuning()
{

	if (!(ts.flags1 & FLAGS1_DYN_TUNE_ENABLE))			// is it off??
	{
		ts.flags1 |= FLAGS1_DYN_TUNE_ENABLE;	// then turn it on
	}
	else
	{
		ts.flags1 &= ~FLAGS1_DYN_TUNE_ENABLE;	// then turn it off
	}

	UiDriver_DisplayFreqStepSize();
	if (ks.button_id == 17 ) AudioManagement_KeyBeep(); // if touchscreen then Beep
}

//*********************************************************************************

void UiAction_ChangeDebugInfoDisplay()
{
	UiDriver_DebugInfo_DisplayEnable(!ts.show_debug_info);
}



static void UiAction_ChangeFilterBW()
{
	if (ts.switch_pause < 500 ) ts.switch_pause += 600;   // pour suspendre momentan�ment le calcul non prioritaire de reajustement des correction IQ, besoin du processeur
	ts.audio_dac_muting_buffer_count            += 50;

	if(!ts.tune)
	{
		if (filter_path_change == true)
		{
			filter_path_change = false;
		}
		else
		{
			AudioFilter_NextApplicableFilterPath( PATH_USE_RULES, AudioFilter_GetFilterModeFromDemodMode(ts.dmod_mode), ts.filter_path );
			// we store the new filter in the current active filter location
			AudioDriver_SetRxAudioProcessing( ts.dmod_mode, false );
			// we activate it (in fact the last used one, which is the newly selected one);
		}
		// Change filter
		UiDriver_UpdateDisplayAfterParamChange();		// re-init for change of filter including display updates
		UiDriver_DisplayEncoderThreeMode();
	}
}

//*****************************************************************************************************


void UiAction_ChangeFilterBW_CALL()
{
	UiAction_ChangeFilterBW();
	if (ks.button_id == 17 )   AudioManagement_KeyBeep();   // if touchscreen then Beep
}


static void UiAction_ChangeTuningStepDownOrUp()
{
	UiDriver_ChangeTuningStep( ts.freq_step_config & FREQ_STEP_SWAP_BTN? true: false );
}

static void UiAction_ChangeTuningStepUpOrDown()
{
	UiDriver_ChangeTuningStep(ts.freq_step_config & FREQ_STEP_SWAP_BTN? false: true);
}

static void UiAction_ChangeBacklightBrightness()
{
	// incr_wrap_uint8(&ts.lcd_backlight_brightness,0,5);
	 incr_wrap_uint8( &ts.lcd_backlight_brightness, LCD_DIMMING_LEVEL_MIN, LCD_DIMMING_LEVEL_MAX );
}

static void UiAction_ToggleTxDisable()
{
	if(ts.txrx_mode == TRX_MODE_RX)			// do NOT allow mode change in TUNE mode or transmit mode
	{
		if( RadioManagement_IsTxDisabledBy(TX_DISABLE_USER) == false)
		{
			ts.tx_disable |= TX_DISABLE_USER;
		}
		else
		{
			ts.tx_disable &= ~TX_DISABLE_USER;
		}
		UiDriver_FButton_F5Tune();
	}
}

static void UiAction_ToggleVfoMem()
{
	RadioManagement_ToggleVfoMem();
	UiDriver_FButton_F3MemSplit();
}

static void UiAction_ToggleDspEnable()
{
	if(ts.dmod_mode != DEMOD_FM)	 		// do not allow change of mode when in FM
	{
		if(is_dsp_nr()|| is_dsp_notch() || is_dsp_mnotch() || is_dsp_mpeak())	 			// is any DSP function active?
		{
			ts.dsp_active_toggle =  ts.dsp_active;	// save setting for future toggling
			ts.dsp_active       &= ~(DSP_NR_ENABLE | DSP_NOTCH_ENABLE |DSP_MNOTCH_ENABLE | DSP_MPEAK_ENABLE);				// turn off NR and notch
			ts.enc_two_mode      =  ENC_TWO_MODE_RF_GAIN;
		}
		else	 		// neither notch or NR was active
		{
			if(ts.enc_two_mode != ENC_TWO_MODE_RF_GAIN)  ts.RFG_wait = 9000;
			if(ts.dsp_active_toggle != 0xff)	 	// has this holder been used before?
			{
				ts.dsp_active = ts.dsp_active_toggle;	// yes - load value
			}
		}
		AudioDriver_SetRxAudioProcessing( ts.dmod_mode, false );	// update DSP settings
		UiDriver_DisplayEncoderTwoMode();
	}
}

// TODO: Split into separate actions and a composition
static void UiAction_ChangeRxFilterOrFmToneBurst()
{
	if((!ts.tune) && (ts.txrx_mode == TRX_MODE_RX))	  // only allow in receive mode and when NOT in FM
	{
		filter_path_change = true;
		UiDriver_DisplayFilter();
		UiDriver_DisplayEncoderThreeMode();
	}
	else if((ts.txrx_mode == TRX_MODE_TX) && (ts.dmod_mode == DEMOD_FM))
	{
		if(ts.fm_tone_burst_mode != FM_TONE_BURST_OFF)	 	// is tone burst mode enabled?
		{
			ads.fm_tone_burst_active = 1;					// activate the tone burst
			ts.fm_tone_burst_timing = ts.sysclock + FM_TONE_BURST_DURATION;	// set the duration/timing of the tone burst
		}
	}
}

static void UiAction_ToggleNoiseblanker()
{
	 // ((uint8_t) ts.dsp_active) ^= DSP_NB_ENABLE;	// toggle whether or not DSP or NB is to be displayed
    ts.dsp_active ^= DSP_NB_ENABLE;
	UiDriver_DisplayEncoderTwoMode();
}

static void UiAction_ToggleTuneMode()
{
	ts.tune = RadioManagement_Tune(!ts.tune);
	UiDriver_DisplayPowerLevel();           // tuning may change power level temporarily
	UiDriver_FButton_F5Tune();
}

static void UiAction_PlayKeyerBtnN(int8_t n)
{
	uint8_t   *pmacro;
	uint16_t   c = 0;

	if (ts.keyer_mode.button_recording == KEYER_BUTTON_NONE)
	{
		pmacro = (uint8_t *)ts.keyer_mode.macro[n];
		if (*pmacro != '\0') // If there is a macro
		{
			while (*pmacro != '\0')
			{
				DigiModes_TxBufferPutChar(*pmacro++);
			}

			if ((ts.dmod_mode == DEMOD_CW && ts.cw_keyer_mode != CW_KEYER_MODE_STRAIGHT) || is_demod_psk())
			{
				ts.ptt_req = true;
			}
		}

		UiDriver_TextMsgPutChar('>');
	}
	else if (ts.keyer_mode.button_recording == n)
	{
		ts.cw_text_entry = false;
		pmacro = (uint8_t *) ts.keyer_mode.macro[n];
		c = 0;
		while (DigiModes_TxBufferHasData())
		{
			if (++c > KEYER_MACRO_LEN - 1) {
				break;
			}
			DigiModes_TxBufferRemove(pmacro++);
		}
		*pmacro = '\0';

		UiConfiguration_UpdateMacroCap();
		UiDriver_TextMsgPutChar('<');
		ts.keyer_mode.button_recording = KEYER_BUTTON_NONE;
	}
	UiDriver_CreateFunctionButtons(false);
}

static void UiAction_PlayKeyerBtn1()
{
	UiAction_PlayKeyerBtnN(KEYER_BUTTON_1);
}

static void UiAction_PlayKeyerBtn2()
{
	UiAction_PlayKeyerBtnN( KEYER_BUTTON_2 );
}

static void UiAction_PlayKeyerBtn3()
{
	UiAction_PlayKeyerBtnN( KEYER_BUTTON_3 );
}

static void UiAction_RecordKeyerBtnN( int8_t n )
{
	if (ts.keyer_mode.button_recording == KEYER_BUTTON_NONE && ts.txrx_mode == TRX_MODE_RX && !ts.cw_text_entry && ts.dmod_mode == DEMOD_CW)
	{
		ts.cw_text_entry                = true;
		ts.keyer_mode.button_recording  = n;
		DigiModes_TxBufferReset();
		UiDriver_TextMsgPutChar(':');
		UiDriver_CreateFunctionButtons(false);
	}
}

static void UiAction_RecordKeyerBtn1()
{
	UiAction_RecordKeyerBtnN(KEYER_BUTTON_1);
}

static void UiAction_RecordKeyerBtn2()
{
	UiAction_RecordKeyerBtnN(KEYER_BUTTON_2);
}

static void UiAction_RecordKeyerBtn3()
{
	UiAction_RecordKeyerBtnN(KEYER_BUTTON_3);
}

static void UiAction_ToggleBufferedTXMode()
{
	ts.buffered_tx  = ! ts.buffered_tx;
	UiDriver_FButton_F5Tune();
}

//*************************************************************************************

static void UiAction_ToggleTxRx()
{
	if   (ts.txrx_mode == TRX_MODE_RX)  { ts.ptt_req     = true; }
	else	                            { ts.tx_stop_req = true; }
	UiDriver_FButton_F5Tune();
}

//*************************************************************************************

static void UiAction_ToggleSplitModeOrToggleMemMode()
{
	if (ts.switch_pause < 400) ts.switch_pause += 500; // pour suspendre momentan�ment le calcul non prioritaire de reajustement des correction IQ, besoin du processeur
	if(!ts.vfo_mem_flag)	 		// update screen if in VFO (not memory) mode
	{
		UiDriver_SetSplitMode( !is_splitmode() );
	}
	else	 		// in memory mode
	{
		UiSpectrum_Clear();		// always clear display
		ts.mem_disp = !ts.mem_disp;
		if (ts.mem_disp == 0 )
		{
			UiSpectrum_Init();	// init spectrum scope
		}
	}
}

static void UiAction_ToggleMenuMode()
{
	if(!ts.mem_disp)	 			// allow only if NOT in memory display mode
	{
		if(ts.switch_pause < 400)  ts.switch_pause += 500; // pour suspendre momentanement les calculs correc IQ
		if(ts.menu_mode == false)	 	// go into menu mode if NOT already in menu mode and not to halt on startup
		{
			ts.menu_mode        = true;
			ts.encoder3state    = filter_path_change;
			filter_path_change  = false;			// deactivate while in menu mode
			UiDriver_DisplayFilter();
			UiSpectrum_Clear();
			UiDriver_DisplayFButton_F1MenuExit();
			UiDriver_DrawFButtonLabel(2,"PREV",Yellow);
			UiDriver_DrawFButtonLabel(3,"NEXT",Yellow);
			UiDriver_DrawFButtonLabel(4,"DEFLT",Yellow);
			//
			//
			// Grey out adjustments and put encoders in known states
			//
			UiDriver_RefreshEncoderDisplay();

			ts.menu_var = 0;
			//
			UiMenu_RenderMenu(MENU_RENDER_ONLY);	// Draw the menu the first time
			UiMenu_RenderMenu(MENU_PROCESS_VALUE_CHANGE);	// Do update of the first menu item
		}
		else	 	// already in menu mode - we now exit
		{
			ts.menu_mode       = false;
			filter_path_change = ts.encoder3state;
			UiDriver_DisplayFilter();
			UiSpectrum_Init();			// init spectrum scope

			// Restore encoder displays to previous modes
			UiDriver_RefreshEncoderDisplay();
			UiDriver_DisplayFilter();	// update bandwidth display
			UiDriver_CreateFunctionButtons(false);
		}
	}
}

static void UiAction_ToggleKeyerMode()
{
	ts.keyer_mode.active = !ts.keyer_mode.active;
	UiDriver_CreateFunctionButtons(false);
}

static void UiAction_BandMinusHold()
{
	if(UiDriver_IsButtonPressed(BUTTON_PWR_PRESSED))	 	// and POWER button pressed-and-held at the same time?
	{
		UiDriver_LcdBlankingStealthSwitch();
	}
	else if(UiDriver_IsButtonPressed(BUTTON_BNDP_PRESSED))	 	// and BAND-UP pressed at the same time?
	{
		if(!ts.menu_mode)	 			// do not do this in menu mode!
		{
			UiAction_ToggleWaterfallScopeDisplay();
		}
	}
}

static void UiAction_BandPlusHold()
{
	if(UiDriver_IsButtonPressed(BUTTON_BNDM_PRESSED))	 	// and BAND-DOWN pressed at the same time?
	{
		if(!ts.menu_mode)	 		// do not do this if in menu mode!
		{
			UiAction_ToggleWaterfallScopeDisplay();
		}
	}
	if(UiDriver_IsButtonPressed(BUTTON_PWR_PRESSED))	 	// and POWER button pressed-and-held at the same time?
	{
		UiDriver_PowerDownCleanup(false); // do not save the configuration
	}
}

static void UiAction_PowerHold()
{
	if(UiDriver_IsButtonPressed(BUTTON_BNDM_PRESSED))	 	// was keyPin BAND- pressed at the same time?
	{
		UiDriver_LcdBlankingStealthSwitch();
	}
	else
	{
		// ONLY the POWER button was pressed
		if(ts.txrx_mode == TRX_MODE_RX)  		// only allow power-off in RX mode
		{
			UiDriver_PowerDownCleanup(true);
		}
	}
}

static void UiAction_StepMinusHold()
{
	if(UiDriver_IsButtonPressed(BUTTON_STEPP_PRESSED))	 	// was keyPin STEP+ pressed at the same time?
	{
		ts.frequency_lock = !ts.frequency_lock;
		// update frequency display
		UiDriver_FrequencyUpdateLOandDisplay(true);
	}
	else
	{
		if(!(ts.freq_step_config & FREQ_STEP_SWAP_BTN))	  // keyPin swap NOT enabled
		{
			UiDriver_PressHoldStep(0);	// decrease step size
		}
		else  		// keyPin swap enabled
		{
			UiDriver_PressHoldStep(1);	// increase step size
		}
	}
}

static void UiAction_MenuSetDefaultValue()
{
	UiMenu_RenderMenu(MENU_PROCESS_VALUE_SETDEFAULT);
}

/*
static void UiAction_StepPlusHold()
{
	if(UiDriver_IsButtonPressed(BUTTON_STEPM_PRESSED))	 	// was keyPin STEP- pressed at the same time?
	{
		ts.frequency_lock = !ts.frequency_lock;
		// update frequency display
		UiDriver_FrequencyUpdateLOandDisplay(true);
	}
	else
	{
		if(!(ts.freq_step_config & FREQ_STEP_SWAP_BTN))  	// keyPin swap NOT enabled
		{
			UiDriver_PressHoldStep(1);	// increase step size
		}
		else  		// keyPin swap enabled
		{
			UiDriver_PressHoldStep(0);	// decrease step size
		}
	}
}

*/

static bool UiDriver_Process_WFscope_RatioChange()
{
	ts.switch_pause     +=  500; // pour suspendre momentanement les calculs correc IQ NIZZZ
	bool TouchProcessed  =  false;

	UiArea_t     OKArea;
	OKArea.x   = sd.Slayout->graticule.x+sd.Slayout->graticule.w-65;
	OKArea.y   = sd.Slayout->graticule.y;
	OKArea.h   = sd.Slayout->graticule.h;
	OKArea.w   = 25;
	UiArea_t     ExitArea;
	ExitArea.x = sd.Slayout->graticule.x+sd.Slayout->graticule.w-30;
	ExitArea.y = sd.Slayout->graticule.y;
	ExitArea.h = sd.Slayout->graticule.h;
	ExitArea.w = 30;

	if(UiDriver_CheckTouchRegion(&OKArea))
	{
		ts.SpectrumResize_flag  = 0;
		ts.graticulePowerupYpos = sd.Slayout->graticule.y;		//store current graticule position for future eeprom save
		UiDriver_DisplayFButton_F1MenuExit();		//redraw the menu button to indicate the changed item
		ts.menu_var_changed     = 1;
		ts.flags1              |= FLAGS1_SCOPE_ENABLED;
		ts.flags1              |= FLAGS1_WFALL_ENABLED;
		UiSpectrum_Init();
		TouchProcessed=true;
	}
	else if(UiDriver_CheckTouchRegion(&ExitArea))
	{
		ts.SpectrumResize_flag = 0;
		UiSpectrum_Init();
		TouchProcessed         = true;
	}
	else if(UiDriver_CheckTouchRegion(&sd.Slayout->full))
	{
		UiDriver_DrawGraticule_Rect(false); 	 //clear graticule current area
		sd.Slayout->graticule.y = UiSprectrum_CheckNewGraticulePos(ts.tp->hr_y);
		UiDriver_DrawGraticule_Rect(true);		//draw new graticule control
		TouchProcessed          = true;
	}
	return TouchProcessed;
}
#define TOUCH_SHOW_REGIONS_AND_POINTS	//this definition enables the drawing of boxes of regions and put the pixel in touch point

static void UiDriver_HandleTouchScreen(bool is_long_press)
{
	if(is_touchscreen_pressed())
	{
		uint32_t touchaction_idx = ts.menu_mode == true?1:0;

		if (false) // (ts.show_debug_info)					// show coordinates for coding purposes
		{
			char text[14];
			snprintf( text, 14, "%04d%s%04d%s", ts.tp->hr_x, " : ", ts.tp->hr_y, "  " );

    #ifdef TOUCH_SHOW_REGIONS_AND_POINTS
			UiLcdHy28_DrawColorPoint ( ts.tp->hr_x, ts.tp->hr_y, White );

			uint16_t x, y, w, h;
			for(int n=0;n<ts.Layout->touchaction_list[touchaction_idx].size;n++)
			{
				x = ts.Layout->touchaction_list[touchaction_idx].actions[n].region.x;
				y = ts.Layout->touchaction_list[touchaction_idx].actions[n].region.y;
				w = ts.Layout->touchaction_list[touchaction_idx].actions[n].region.w;
				h = ts.Layout->touchaction_list[touchaction_idx].actions[n].region.h;
				UiLcdHy28_DrawEmptyRect ( x, y, h, w, Red );
			}
    #endif


			UiLcdHy28_PrintText( 0, ts.Layout->LOADANDDEBUG_Y, text, White, Black, 0 );
		}

		bool  TouchProcessed       = 0;
		if(ts.SpectrumResize_flag == true
				&&  ts.menu_mode  == 0)
		{
			TouchProcessed = UiDriver_Process_WFscope_RatioChange();
		}
		else if(ts.VirtualKeysShown_flag)
		{
			TouchProcessed = UiVk_Process_VirtualKeypad(is_long_press);
		}

		if(!TouchProcessed)
		{
			UiDriver_ProcessTouchActions(&ts.Layout->touchaction_list[touchaction_idx], is_long_press);
		}

		ts.tp->state = TP_DATASETS_PROCESSED;							// set statemachine to data fetched
	}
}
///*******************************************************************************************
///
///
static void UiDriver_HandleTouchScreenShortPress(bool is_long_press)
{
    UiDriver_HandleTouchScreen(false);
}

//************************************************************************************************
//***

static void UiDriver_HandleTouchScreenLongPress(bool is_long_press)
{
    UiDriver_HandleTouchScreen(true);
}
///************************************************************************************************
///
static const keyaction_descr_t keyactions_normal[] = // Bouton
{
		{ TOUCHSCREEN_ACTIVE, 	UiDriver_HandleTouchScreenShortPress,       UiDriver_HandleTouchScreenLongPress },
		{ BUTTON_F1_PRESSED, 	UiAction_ToggleMenuMode, 					UiAction_SaveConfigurationToMemory },
		{ BUTTON_F2_PRESSED, 	UiAction_ChangeLowerMeterDownOrSnap, 		UiAction_ChangeLowerMeterUp },
		{ BUTTON_F3_PRESSED, 	UiAction_ToggleSplitModeOrToggleMemMode, 	UiAction_ToggleVfoMem },
		{ BUTTON_F4_PRESSED, 	UiAction_ToggleVfoAB, 						UiAction_CopyVfoAB },
		{ BUTTON_F5_PRESSED, 	UiAction_ToggleTuneMode,					UiAction_ToggleTxDisable },
		{ BUTTON_G1_PRESSED, 	UiAction_ChangeDemodMode,					UiAction_ChangeDemodModeToAlternativeMode },
		{ BUTTON_G2_PRESSED, 	UiAction_ChangeToNextDspMode,				UiAction_ToggleDspEnable },
		{ BUTTON_G3_PRESSED, 	UiAction_ChangePowerLevel,					KEYACTION_NOP },
		{ BUTTON_G4_PRESSED, 	UiAction_ChangeFilterBW,					UiAction_ChangeRxFilterOrFmToneBurst },
		{ BUTTON_M1_PRESSED, 	UiDriver_ChangeEncoderOneMode,				UiAction_ToggleKeyerMode },
		{ BUTTON_M2_PRESSED, 	UiDriver_ChangeEncoderTwoMode,				UiAction_ToggleNoiseblanker },
		{ BUTTON_M3_PRESSED, 	UiDriver_ChangeEncoderThreeMode,			UiAction_ChangeAudioSource },
		{ BUTTON_STEPM_PRESSED, UiAction_ChangeTuningStepDownOrUp,			KEYACTION_NOP }, // UiAction_StepMinusHold },
		{ BUTTON_STEPP_PRESSED, UiAction_ChangeTuningStepUpOrDown,			KEYACTION_NOP }, // UiAction_StepPlusHold },
		{ BUTTON_BNDM_PRESSED, 	UiAction_ChangeBandDownOrUp,				UiAction_BandMinusHold },
		{ BUTTON_BNDP_PRESSED,  UiAction_ChangeBandUpOrDown,				UiAction_BandPlusHold },
		{ BUTTON_PWR_PRESSED,   UiAction_ChangeBacklightBrightness,			UiAction_PowerHold },
};

static const keyaction_descr_t keyactions_menu[] =
{
		{ BUTTON_F2_PRESSED, 	(void(*)())UiMenu_RenderPrevScreen, 		UiMenu_RenderFirstScreen },
		{ BUTTON_F3_PRESSED, 	(void(*)())UiMenu_RenderNextScreen, 		UiMenu_RenderLastScreen },
		{ BUTTON_F4_PRESSED, 	UiAction_MenuSetDefaultValue,				KEYACTION_NOP },
};

static const keyaction_descr_t keyactions_keyer[] =
{
		{ BUTTON_F1_PRESSED, 	UiAction_PlayKeyerBtn1, 		UiAction_RecordKeyerBtn1  },
		{ BUTTON_F2_PRESSED, 	UiAction_PlayKeyerBtn2, 		UiAction_RecordKeyerBtn2  },
		{ BUTTON_F3_PRESSED, 	UiAction_PlayKeyerBtn3, 		UiAction_RecordKeyerBtn3  },
		{ BUTTON_F4_PRESSED, 	KEYACTION_NOP, 		            KEYACTION_NOP             },
		{ BUTTON_F5_PRESSED, 	UiAction_ToggleTxRx,		UiAction_ToggleBufferedTXMode },
};

static const keyaction_list_descr_t key_sets[] =
{
		// ATTENTION: the size calculation only works for true arrays, not for pointers!
		{ keyactions_normal, sizeof(keyactions_normal) / sizeof(*keyactions_normal)  },
		{ keyactions_menu  , sizeof(keyactions_menu  ) / sizeof(*keyactions_menu  )  },
		{ keyactions_keyer , sizeof(keyactions_keyer ) / sizeof(*keyactions_keyer )  },
};
///*******************************************************************************************
///
static  void   UiDriver_HandleKeyboard()
{
	if ( ks.button_execution_Wait)
	{
		UiDriver_LcdBlankingStartTimer();	// calculate/process LCD blanking timing


		bool keyIsProcessed      =  false;
		if (ts.keyer_mode.active == true)
		{
			keyIsProcessed = UiDriver_ProcessKeyActions ( &key_sets[2] );
		}


		if (keyIsProcessed == false && ts.menu_mode == true)
		{
			keyIsProcessed = UiDriver_ProcessKeyActions ( &key_sets[1] );
		}


		if (keyIsProcessed == false)
		{
			keyIsProcessed = UiDriver_ProcessKeyActions ( &key_sets[0] );
		}

		ts.FFT_stand_by = 7;  // tempo 70ms avant reprise echantillonnage et affichag sur ecran
		sd.state        = 0;  // NIZZ pour echatiollnner FFT du debut de nouveau


			// Reset flag, allow other buttons to be checked
		ks.button_execution_Wait = 0;
		ks.debounce_time	     = 0;
		if ( ks.button_id != 17 ) AudioManagement_KeyBeep();   // il me semble qu'elle est doubl�e ici reste � verifier        // make keyboard beep, if enabled
	}
}
///***************************************************************

typedef enum
{
	SCTimer_ENCODER_KEYS = 0, //       10ms
	SCTimer_RTC,              // 100 * 10ms
	SCTimer_LODRIFT,          // 64  * 10ms
	SCTimer_VOLTAGE,          // 23  * 10ms
	SCTimer_SMETER,           // 4   * 10ms
	SCTimer_MAIN,             // 1   * 10ms
	SCTimer_LEDBLINK,         // 64  * 10ms
    SCTimer_SAM,              // 25  * 10ms
	SCTimer_NUM
} SysClockTimers;

// uint32_t next_expected_sysclock [SCTimer_NUM] = { 1, 100,  64, 23,  7, 1,  67, 25 }; // tous *10ms delay
   uint32_t next_expected_sysclock [SCTimer_NUM] = { 2, 200, 124, 46, 14, 2, 134, 50 }; // tous *5 ms delay


/*
 * Implements a simple timeout timer.
 * Returns true if the current sysclock differs by equal or more than divider cycles.
 * Dividers should be powers of 2 to generate optimal code
 */
bool UiDriver_TimerIsExpired ( SysClockTimers sct, uint32_t now ) // delay en 10ms
{
	return (now >= next_expected_sysclock[sct] );
}

/**
 * @brief Implements a simple timeout timer. Sets the time to now/divider, so it will expire in now+divider cycles
 * Dividers should be powers of 2 to generate optimal code
 */
void UiDriver_TimerRewind ( SysClockTimers sct, uint32_t now, uint32_t delay)  // delay en 10ms
{
	next_expected_sysclock[sct] = now + delay;
}

/**
 * @brief Implements a simple timeout timer. If expired the timer is automatically restarted.
 *
 * @param sct the timer data structure
 * @param divider should be powers of 2 to generate optimal code
 * @param now current sysclock value
 *
 * @returns true if the now differs by equal or more than divider cycles.
 *
 */
bool UiDriver_TimerExpireAndRewind ( SysClockTimers sct, uint32_t now, uint32_t delay )
{
	if (now >= next_expected_sysclock[sct] )
	{
		next_expected_sysclock[sct] = now + delay;
		return true;
	}
	else return false;
}



#ifdef     USE_USBHOST
#include  "usb_host.h"
#include  "usbh_core.h"
#include  "usbh_hid_keybd.h"
extern    USBH_HandleTypeDef  hUsbHostHS;
#endif

/**
 * This handler is activated by the Audio-Interrupt with a frequency of 1500 Hz via PendSV interrupt.
 * However, since this handler may run longer than 0.66mS it cannot be used to keep track of time
 * or you cannot count on it called every 0.66mS. It will be next activate no longer than 0.66mS after
 * last finishing the handler.
 *
 * This handler is executed as PendSV interrupt handler with lowest possible priority, i.e. its execution
 * is interrupted by all other interrupts with a higher priority.
 *
 * Please note: Do not call any UI related functions here, essentially we should only include high prio
 * tasks such as longer running audio processing (e.g. FreeDV or the NR processing) which should be executed
 * with as low latency as possible but take too long for being included in the audio interrupt handler itself
 * Also relevant: If this handler function does take too long, the UI handling including display updates
 * will feel sluggish or will not work at all since all processing time is spend here then and
 * the main tasks will never get executed.
 *
 * In a nutshell, unless there is a very, very good reason, do not add or change anything here.
 *
 */
//*******************************************************************************************************

void UiDriver_TaskHandler_HighPrioTasks()
{
    // READ THE LENGTHY COMMENT ABOVE BEFORE CHANGING ANYTHING BELOW!!!
    // YES, READ IT! Thank you!
#ifdef USE_FREEDV
    if (ts.dmod_mode == DEMOD_DIGI && ts.digital_mode == DigitalMode_FreeDV)
    {
        FreeDv_HandleFreeDv();
    }
#endif // USE_FREEDV

#ifdef USE_ALTERNATE_NR
    // if (( ( ts.nb_setting > 0) ||   ( ts.dsp_active & DSP_NR_ENABLE )) && ( ads.decimation_rate == 4 ))
    if ( ts.txrx_mode == TRX_MODE_RX )
    {
		if ( ( ts.dsp_active & DSP_NR_ENABLE ) && ( ads.decimation_rate == 4 ))
		{
			alternateNR_handle();      // pour traiter les derniers 256 points cumulés s'ils sont prets
		}
    }
#endif

   // if ( RadioManagement_TxRxSwitching_IsEnabled() )
	//  && osc->readyForIrqCall() )
	//  && Codec_ReadyForIrqCall() )
	//  && radioManagement_SwitchTxRx_running == false )


/*    if ( ( ts.dmod_mode == DEMOD_CW ) && !ts.tx_disable )  // NIZZZ a verifier avec CW pour voir si cela marche
     {
        RadioManagement_Handle_Ptt_On_Off_Switch();
     }  */
/*
#ifdef USE_CONVOLUTION
    	convolution_handle();
#endif
*/
}

///**************************************************************************************************

void UiDriver_TaskHandler_MainTasks()  /// NIZZZ boucle principale infini, fonctionne aussi bien en Rx qu'en TX
{

	uint32_t now = ts.sysclock;
	//        HAL_GetTick()/10;

	CatDriver_HandleProtocol();

#ifndef USE_PENDSV_FOR_HIGHPRIO_TASKS

	UiDriver_TaskHandler_HighPrioTasks();    // sans attendre une temporisation multiple de 10 ms
	                                         // Free-DV , NR ,

#endif

#ifdef USE_USBHOST
	MX_USB_HOST_Process();  // Traitement du clavier si il est au USB_HOST pour le CW

#ifdef USE_USBKEYBOARD
	if(USBH_HID_GetDeviceType(&hUsbHostHS) == HID_KEYBOARD)
	{

		HID_KEYBD_Info_TypeDef *k_pinfo;
		char kbdChar;
		k_pinfo = USBH_HID_GetKeybdInfo ( &hUsbHostHS );

		if(k_pinfo != NULL)
		{
			kbdChar = USBH_HID_GetASCIICode ( k_pinfo );
			switch ( k_pinfo->keys[0] )
			{
				case KEY_F1:  ts.ptt_req     = true;	  break;
				case KEY_F2:  ts.tx_stop_req = true;	  break;
			}
			if ( kbdChar != '\0' )
			{
				if ( is_demod_rtty() || is_demod_psk() )
				{
					DigiModes_TxBufferPutChar ( kbdChar );
				}
				UiDriver_TextMsgPutChar ( kbdChar );
			}
		}
	}
#endif
#endif
    // START CALLED AS OFTEN AS POSSIBLE
    	if ( ts.tx_stop_req == true  || ts.ptt_req == true  ) { RadioManagement_Handle_Ptt_On_Off_Switch(); }  // END CALLED AS OFTEN AS POSSIBLE
	// || ts.dmod_mode == DEMOD_CW



	// BELOW ALL CALLING IS BASED ON SYSCLOCK 10ms clock
	if ( UiDriver_TimerExpireAndRewind ( SCTimer_ENCODER_KEYS, now, 2 ) )      // 10ms
	{
				// 10ms have elapsed.
				// Now process events which should be handled regularly at a rate of 100 Hz
				// Remember to keep this as short as possible since this is executed in addition
				// to all other processing below.


		UiDriver_CheckEncoderOne();      // Nizou
		UiDriver_CheckEncoderTwo();
		UiDriver_CheckEncoderThree();
		UiDriver_CheckFrequencyEncoder();

		UiDriver_KeyboardProcessOldClicks();
		UiDriver_HandleKeyboard();                   // NIZZ rajouté ici pour traiter les taches de fond aussitot que IRQ-TP est detecté par la fonction Old_Clicks

		if ( ts.dynamic_tune_activ_counter > 1 )  { ts.dynamic_tune_activ_counter--;       }
		if ( delay_to_change_freq_by_touch     )  { delay_to_change_freq_by_touch--;       }
		if ( ts.tempo_avg_FFT                  )  { ts.tempo_avg_FFT--;                    }
		if ( ts.txrx_mode == TRX_MODE_TX       )  { RadioManagement_UpdatePowerAndVSWR();  }

		RadioManagement_Handle_Ptt_On_Off_Switch();  // PTT Button & Dah

		if ( ( ts.txrx_mode == TRX_MODE_RX ) && ! ts.menu_mode)  UiSpectrum_Redraw(); // Boucle done continue pour  calculer et afficher FFT  si le buffer 256/512 est rempli
	}

	// Expect the code below to be executed around every 40 - 80ms.
	// The exact time between two calls is unknown and varies with different
	// display options  (waterfall/scope, DSP settings etc.)
	// Nothing with short intervals < 100ms  and/or need for very regular intervals between calls
	// should be placed in here.

	if ( UiDriver_TimerIsExpired ( SCTimer_MAIN, now ) )         // 10 ms + qq ms selon charge   // bail out if it is not time to do this task
	{

		switch ( drv_state )     /// sequencement par  rafale de 7 chaque  0.6666ms et ce  toutes les 10ms
		{
			//**********************************************************************************************************
			case STATE_S_METER:
							               // we update all the meters (either TX or RX) no more than 25 times a second
							if ( UiDriver_TimerExpireAndRewind ( SCTimer_SMETER, now, 8 ) )   // toutes les 40 ms  / 25 Hz
							{
								if ( ts.txrx_mode == TRX_MODE_RX )
								{
									RadioManagement_HandleRxIQSignalCodecGain(); // pour corriger le gain hardware du Codec automatiquement de 0..-31db
									if ( (ts.dmod_mode == DEMOD_USB) || (ts.dmod_mode == DEMOD_LSB) || ( ts.dmod_mode == DEMOD_CW ))
									{
										sm.dbm =  50 * log10f_fast ( ts.Sm_peak_moyen ) + (float32_t) ts.dbm_constant - 122  - (float32_t) ts.old_rfg_calc * 3.82 ;  //35 /103 / 2.9 NIZZZ Le gain codec est pris en compte pour l'affichage correct du smetre SSB
									}
									if (ts.enable_display_on_screen)  UiDriver_HandleSMeter ();
								}
								else UiDriver_HandleTXMeters();
#ifdef USE_FREEDV
								if ( ts.dmod_mode == DEMOD_DIGI && ts.digital_mode == DigitalMode_FreeDV )
								{
									FreeDv_DisplayUpdate();
								}
#endif // USE_FREEDV

							}
							break;

			//************************************************************************************************************
			case STATE_HANDLE_POWERSUPPLY:
										Board_HandlePowerDown();


										break;
			// ***********************************************************************************************************
			case STATE_LO_TEMPERATURE:
									if ( UiDriver_TimerExpireAndRewind ( SCTimer_RTC, now, 200 ) )  // 1 seconde
									{
										if ( ( ts.rtc_present ) && ts.enable_display_on_screen )
										{
											RTC_TimeTypeDef   sTime;
											MchfRtc_GetTime ( &hrtc, &sTime,  RTC_FORMAT_BIN );
											char str[10];
											snprintf ( str, 10, "%2u:%02u:%02u", sTime.Hours, sTime.Minutes, sTime.Seconds );
											UiLcdHy28_PrintText ( ts.Layout->RTC_IND.x, ts.Layout->RTC_IND.y, str, White, Black, 0 );
										}
									}
									else if ( UiDriver_TimerExpireAndRewind ( SCTimer_LODRIFT, now, 128 ) )  // 0.64 seconde  a verifier si le delay fait changer la valeur de charge processeur
									{
										// if ( ts.switch_pause == 0)
										if (ts.enable_display_on_screen) 	UiDriver_HandleLoTemperature(); // pour suspendre la communication I2S avec la puce de temperature en cas de besoin
										if ( ts.show_debug_info  )   /// NIZZZ  a inclure a cette boucle les lignes plus haut pour optimiser l'execution
										{
											ProfilingTimedEvent*  pe_ptr = profileTimedEventGet ( ProfileAudioInterrupt );

											// Percent audio interrupt load  = Num of cycles per audio interrupt  / ((max num of cycles between two interrupts ) / 100 )
											//
											// Num of cycles per audio interrupt = cycles for all counted interrupts / number of interrupts
											// Max num of cycles between two interrupts / 100 = HCLK frequency / Interruptfrequenz -> e.g. 168 000 000 / 1500 / 100 = 1120
											// NIZZ                                                     Pour STM32F427VIT6  (180 Mhz)      180 000 000 /1500  / 100 = 1200
											// FIXME: Need to figure out which clock is being used, 168 000 000 in mcHF, I40 UI = 168.000.000 or 216.000.000 or something else...
                                            if (pe_ptr->count == 0) { pe_ptr->count = 1; }
											uint32_t  load =  pe_ptr->duration / ( pe_ptr->count * (112) );
											profileTimedEventReset ( ProfileAudioInterrupt );

											// char str[20]; snprintf ( str, 20, "L%3u%%", (unsigned int) load );
											char str[20]; snprintf ( str, 20, "%2ld.%01ld%%", (unsigned int) load /10, (unsigned int) load % 10 );
											// snprintf ( digits, 7, "%2ldv%02ld", pwmt.voltage/100, pwmt.voltage%100 );
											UiLcdHy28_PrintText ( ts.Layout->LOAD_X,  ts.Layout->LOADANDDEBUG_Y, str, White, Black, 0 );
										}
									}
								break;
			// **************************************************************************************************************
			case STATE_TASK_CHECK:  UiDriver_TimeScheduler();    // Handles live update of Calibrate between TX/RX and volume control

								break;
			// *************************************************************************************************************
			case STATE_UPDATE_FREQUENCY:
								/* at this point we handle request for changing the frequency
								 * either from a difference in dial freq or a temp change
								 *  */
								if( ( df.tune_old != df.tune_new) )
								{
									UiDriver_FrequencyUpdateLOandDisplay ( false );
									//  UiDriver_DisplayMemoryLabel();				// this is because a frequency dialing via CAT must be indicated if "CAT in sandbox" is active

								}
								else if (df.temp_factor_changed  || ts.tune_freq != ts.tune_freq_req)
								{
									// this handles the cases where the dial frequency remains the same but the
									// LO tune frequency needs adjustment, e.g. in CW mode  or if temp of LO changes
									RadioManagement_ChangeFrequency ( false, df.tune_new, ts.txrx_mode );
								}
								break;

			// ***************************************************************************************************************

			case STATE_PROCESS_KEYBOARD:	// UiDriver_HandleKeyboard();  // y compris le touchscreen IRQ
			                                // Executer les taches de fond commandées par les boutons et les touches tactiles

											if ( UiDriver_TimerExpireAndRewind ( SCTimer_VOLTAGE, now, 16 ) && ts.enable_display_on_screen )  // 80 ms
											{
												if ( UiDriver_HandleVoltage() )  { UiDriver_DisplayVoltage();	}

												if ( pwmt.undervoltage_detected == true)
												{
													if ( UiDriver_TimerExpireAndRewind ( SCTimer_LEDBLINK, now, 128) )
													{
														Board_GreenLed ( LED_STATE_TOGGLE );
													}
												}
												UiDriver_TextMsgDisplay();  // Affichage  texte decode en CW ou digital RTTY PSK ...
											}

											break;

			// ***************************************************************************************************************

			case STATE_DISPLAY_SAM_CARRIER:
										if ( UiDriver_TimerExpireAndRewind ( SCTimer_SAM, now, 62 ) )   // 27
										{
											if (ts.dmod_mode == DEMOD_SAM)   // NIZZZ Plus la peine d'afficher la frequence SAM en miniature
											{
												UiDriver_UpdateLcdFreq ( df.tune_old, Yellow, UFM_SECONDARY );
											}


											/*	else if (ts.dmod_mode == DEMOD_CW && cw_decoder_config.snap_enable)
											{
												//UiDriver_UpdateLcdFreq(ads.snap_carrier_freq, Green, UFM_SECONDARY);
											}
											*/

											// static float32_t last_tx_alc_agc_min = 1;  /// NIZZZ

											else if (ts.txrx_mode == TRX_MODE_TX )  // Affichage du taux de compression Tx en temps reel
											{
												if (ts.tx_comp_level)
												{
													if ( ( ts.tx_audio_source == TX_AUDIO_MIC ) && (ts.dmod_mode !=  DEMOD_CW ))
													{
														//last_tx_alc_agc_min += (ts.tx_alc_agc_min - last_tx_alc_agc_min)/2;
														//if ( ts.tx_alc_agc_min != last_tx_alc_agc_min )
														{
															// ts.tx_alc_agc_min /= 28000;
															if ( ts.tx_alc_agc_min < 0.1428 ) ts.tx_alc_agc_min = 0.1428; // 1/7
															char  out[10];
															snprintf ( out, 10, "%4.1f", 1/ts.tx_alc_agc_min );
															UiLcdHy28_PrintTextRight ( (ts.Layout->ENCODER_IND.x + ENC_COL_W - 4), (ts.Layout->ENCODER_IND.y + 1 + ENC_ROW_H + ENC_ROW_2ND_OFF),  out,  Grey, Black,  0);
															// last_tx_alc_agc_min = ts.tx_alc_agc_min;
														}
														ts.tx_alc_agc_min = 1;
													}
												}
											}



												// display AGC box and AGC state
												// we have 5 states -> We can collapse 1 and 2 -> you see this in the box title anyway
												// we use an asterisk to indicate action
												// 1 OFF -> WDSP AGC not active
												// 2 ON + NO_HANG + NO ACTION		no asterisk
												// 3 ON + HANG_ACTION + NO ACTION 	white asterisk
												// 4 ON + ACTION                	green asterisk
												// 5 ON + ACTION + HANG_ACTION  	blue asterisk
											if (ts.enable_display_on_screen)
											{
												const char*  txt    = "   ";
												uint16_t AGC_bg_clr = Black;
												uint16_t AGC_fg_clr = Black;

												if (ts.agc_wdsp_hang_action == 1 && ts.agc_wdsp_hang_enable == 1) { AGC_bg_clr = White; AGC_fg_clr = Black;}
												else															  { AGC_bg_clr = Blue;  AGC_fg_clr = White;}
												if (ts.agc_wdsp_action == 1)  { txt = "AGC"; }

								//				UiLcdHy28_PrintTextCentered(ts.Layout->DEMOD_MODE_MASK.x - 41,ts.Layout->DEMOD_MODE_MASK.y,ts.Layout->DEMOD_MODE_MASK.w-6,txt,AGC_fg_clr,AGC_bg_clr,0);
												 UiLcdHy28_PrintTextCentered ( ts.Layout->AGC_MASK.x, ts.Layout->AGC_MASK.y, ts.Layout->AGC_MASK.w, txt, AGC_fg_clr, AGC_bg_clr, 0);
												// display CW decoder WPM speed
												if ( ts.cw_decoder_enable && ts.dmod_mode == DEMOD_CW )
												{
													CwDecoder_WpmDisplayUpdate ( false );
												}
											}
										}
										break;
		    // **************************************************************************************************************

			default:    			    break;
			//***************************************************************************************************************
		}

		if (drv_state < STATE_MAX)
		{
			drv_state++;       // advance to next state
		}
		else
		{
			UiDriver_TimerRewind ( SCTimer_MAIN, ts.sysclock, 2 );  // 10ms=2*5ms expected time is now_clock + 10ms
			drv_state = 0;    // wrap state to first state

			     /// on va profiter de la boucle vide N�7 pour faire nos propre taches suplementaires  ajoutes
			if (ts.dynamic_tune_activ_counter == 1)   // NIZZZ  pour recolorer la frequence en blanc apr�s tempo
			{
				ts.dynamic_tune_activ_counter  = 0;
				//  if (ts.txrx_mode == TRX_MODE_RX) UiDriver_UpdateLcdFreq ( df.tune_old, White, UFM_LARGE );
			}

			else if (ts.RFG_wait == 1)     // NIZZZ retour temporis� pour l'activation et l'affichage du pav� RFG apr�s tempo
			{
				ts.RFG_wait      =     0;
				ts.enc_two_mode  =     ENC_TWO_MODE_RF_GAIN;
				UiDriver_DisplayEncoderTwoMode();
			}
			else if (ts.AFG_wait == 1)     // NIZZ retour temporis� pour l'activation  et l'affichage du pav� AFG apr�s tempo
			{
				ts.AFG_wait      =     0;
				ts.enc_one_mode  =     ENC_ONE_MODE_AUDIO_GAIN;
				UiDriver_DisplayEncoderOneMode();
			}
		}
	}
}

/*
 * This handler creates a software pwm for the LCD backlight. It needs to be called
 * very regular to work properly. Right now it is activated from the audio interrupt
 * at a rate of 1.5khz The rate itself is not too critical,
 * just needs to be high and very regular.
 */

#define LCD_DIMMING_PWM_COUNTS 16

void UiDriver_BacklightDimHandler()
{
	//  static uchar lcd_dim = 0, lcd_dim_prescale = 0;

	    static uchar lcd_dim = 0;
		static const uint16_t dimming_pattern_map[1 + LCD_DIMMING_LEVEL_MAX - LCD_DIMMING_LEVEL_MIN] =
		{
		        0xffff, // 16/16
		        0x3f3f, // 12/16
		        0x0f0f, // 8/16
		        0x0303, // 4/16
		        0x0101, // 2/16
		        0x0001, // 1/1
		};
	    // most of the patterns generate a 1500/8 =  187.5 Hz noise, lowest 1500/16 = 93.75 Hz.

		static uint16_t dim_pattern = 0xffff; // gives us the maximum brightness


	if(!ts.lcd_blanking_flag)       // is LCD *NOT* blanked?
	{
	/*    if(!lcd_dim_prescale)       // Only update dimming PWM counter every fourth time through to reduce frequency below that of audible range
		{
			UiLcdHy28_BacklightEnable(lcd_dim >= ts.lcd_backlight_brightness);   // LCD backlight off or on

			lcd_dim++;
			lcd_dim %= 6;   // limit brightness PWM count to 0-3
		}
		lcd_dim_prescale++;
		lcd_dim_prescale %= 6;  // limit prescale count to 0-3  */

		 if (lcd_dim == 0 )
		{
			dim_pattern = dimming_pattern_map[ts.lcd_backlight_brightness - LCD_DIMMING_LEVEL_MIN];
		}
		 UiLcdHy28_BacklightEnable((dim_pattern & 0x001) == 1);   // LCD backlight off or on

		dim_pattern >>=  1;
		lcd_dim++;
		lcd_dim %= LCD_DIMMING_PWM_COUNTS;   // limit brightness PWM count to 0-3

	}
	else if(!ts.menu_mode)
	{ // LCD is to be blanked - if NOT in menu mode
		UiLcdHy28_BacklightEnable(false);
	}
}

