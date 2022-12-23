/*  -*-  mode: c; tab-width: 4; indent-tabs-mode: t; c-basic-offset: 4; coding: utf-8  -*-  */
/************************************************************************************
 **                                                                                **
 **                                        UHSDR                                   **
 **               a powerful firmware for STM32 based SDR transceivers             **
 **                                                                                **
 **--------------------------------------------------------------------------------**
 **                                                                                **
 **  File name:		ui_lcd_layouts.c                                               **
 **  Description:   Layout definitions file                                        **
 **  Licence:		GNU GPLv3                                                      **
 **  Author: 		Slawomir Balon/SP9BSL                                          **
 ************************************************************************************/

#ifndef BOOTLOADER_BUILD
#include "uhsdr_board.h"
#include "ui_lcd_layouts.h"
#include "ui_driver.h"

//------------------------------------------------------------------------------------------------------------------------------------------
//some local 480x320 calculations to make life easier in case of change something

#define R480320_TUNE_FREQ_X             		280
#define R480320_TUNE_FREQ_Y             		36
#define R480320_TUNE_SPLIT_FREQ_X           	(R480320_TUNE_FREQ_X+80)//Shift with a small split to the right to close the frequency digits.
#define R480320_TUNE_SPLIT_MARKER_X         	(R480320_TUNE_FREQ_X+40)
#define R480320_TUNE_SPLIT_FREQ_Y_TX        	(R480320_TUNE_FREQ_Y+12)
#define R480320_TUNE_SFREQ_X            		(R480320_TUNE_FREQ_X + 120)// Second frequency display control
#define R480320_TUNE_SFREQ_Y            		(R480320_TUNE_FREQ_Y - 20)

#define R480320_DisplayDbm_X					(R480320_TUNE_FREQ_X+45)
#define R480320_DisplayDbm_Y					(R480320_TUNE_FREQ_Y-36)

#define R480320_MEMORYLABEL_X 					(R480320_TUNE_FREQ_X+45+(SMALL_FONT_WIDTH * 11)+4)
#define R480320_MEMORYLABEL_Y 					(R480320_TUNE_FREQ_Y-36)

#define R480320_BAND_MODE_X             		(R480320_TUNE_FREQ_X + 160)
#define R480320_BAND_MODE_Y             		(R480320_TUNE_FREQ_Y + 7)
#define R480320_BAND_MODE_MASK_X            	(R480320_BAND_MODE_X - 1)
#define R480320_BAND_MODE_MASK_Y            	(R480320_BAND_MODE_Y - 1)
#define R480320_BAND_MODE_MASK_H            	13
#define R480320_BAND_MODE_MASK_W            	33

// Demodulator mode control
#define R480320_DEMOD_MODE_X            		(R480320_TUNE_FREQ_X -61)
#define R480320_DEMOD_MODE_Y            		(R480320_TUNE_FREQ_Y - 20)
#define R480320_DEMOD_MODE_MASK_X           	R480320_DEMOD_MODE_X
#define R480320_DEMOD_MODE_MASK_Y           	(R480320_DEMOD_MODE_Y - 1)
#define R480320_DEMOD_MODE_MASK_H           	13
#define R480320_DEMOD_MODE_MASK_W           	58	//same as R480320_DIGMODE_IND_W

// AGC display box
//#define R480320_AGC_MASK_X (R480320_DEMOD_MODE_MASK_X-41)
#define R480320_AGC_MASK_X (R480320_TUNE_FREQ_X)
#define R480320_AGC_MASK_Y R480320_DEMOD_MODE_MASK_Y
#define R480320_AGC_MASK_W 41
#define R480320_AGC_MASK_H R480320_DEMOD_MODE_MASK_H

// Tunning step control
#define R480320_TUNE_STEP_X             		(R480320_TUNE_FREQ_X + 45)
#define R480320_TUNE_STEP_Y             		(R480320_TUNE_FREQ_Y - 21)
#define R480320_TUNE_STEP_MASK_H            	15
#define R480320_TUNE_STEP_MASK_W            	(SMALL_FONT_WIDTH*7)

#define R480320_PW_IND_X                		(R480320_TUNE_FREQ_X)
#define R480320_PW_IND_Y                		(R480320_DEMOD_MODE_Y - 16)
#define R480320_PW_IND_W						41

#define R480320_SM_IND_X                		0
#define R480320_SM_IND_Y                		0
#define R480320_SM_IND_W 						(200)
#define R480320_SM_IND_H 						(72 - 14)

#define R480320_LEFTBOXES_IND_X              	0
#define R480320_LEFTBOXES_IND_Y              	64
#define R480320_LEFTBOX_WIDTH 					58 // used for the lower left side controls
#define R480320_LEFTBOX_ROW_H  					31
#define R480320_LEFTBOX_ROW_2ND_OFF  			15

#define R480320_ENCODER_IND_X                	128
#define R480320_ENCODER_IND_Y                	64

#define R480320_DIGMODE_IND_X              		R480320_DEMOD_MODE_MASK_X  // 218
#define R480320_DIGMODE_IND_Y              		0
#define R480320_DIGMODE_IND_W					R480320_DEMOD_MODE_MASK_W

//------------------------------------------------------------------------------------------------------------------------------------------
//some local 320x240 calculations to make life easier in case of change something
#define R320240_TUNE_FREQ_X             		116
#define R320240_TUNE_FREQ_Y             		100
#define R320240_TUNE_SPLIT_FREQ_X           	(R320240_TUNE_FREQ_X+80)//Shift with a small split to the right to close the frequency digits.
#define R320240_TUNE_SPLIT_MARKER_X         	(R320240_TUNE_FREQ_X+40)
#define R320240_TUNE_SPLIT_FREQ_Y_TX        	(R320240_TUNE_FREQ_Y+12)
#define R320240_TUNE_SFREQ_X            		(R320240_TUNE_FREQ_X + 120)// Second frequency display control
#define R320240_TUNE_SFREQ_Y            		(R320240_TUNE_FREQ_Y - 20)

#define R320240_DisplayDbm_X					(R320240_TUNE_FREQ_X+45+109)
#define R320240_DisplayDbm_Y					(R320240_TUNE_FREQ_Y-36+15-1-1)

#define R320240_MEMORYLABEL_X 					(R320240_TUNE_FREQ_X+45+(SMALL_FONT_WIDTH * 11)+4)
#define R320240_MEMORYLABEL_Y 					(R320240_TUNE_FREQ_Y-36)

#define R320240_BAND_MODE_X             		(R320240_TUNE_FREQ_X + 160 -216)
#define R320240_BAND_MODE_Y             		(R320240_TUNE_FREQ_Y + 7 +9)
#define R320240_BAND_MODE_MASK_X            	(R320240_BAND_MODE_X - 1)
#define R320240_BAND_MODE_MASK_Y            	(R320240_BAND_MODE_Y - 1)
#define R320240_BAND_MODE_MASK_H            	13
#define R320240_BAND_MODE_MASK_W            	33

// Demodulator mode control
#define R320240_DEMOD_MODE_X            		(R320240_TUNE_FREQ_X + 2)
#define R320240_DEMOD_MODE_Y            		(R320240_TUNE_FREQ_Y - 20 -1 -1)
#define R320240_DEMOD_MODE_MASK_X           	(R320240_DEMOD_MODE_X - 1)
#define R320240_DEMOD_MODE_MASK_Y           	(R320240_DEMOD_MODE_Y - 1)
#define R320240_DEMOD_MODE_MASK_H           	13
#define R320240_DEMOD_MODE_MASK_W           	41

// AGC display box
#define R320240_AGC_MASK_X (R320240_DEMOD_MODE_MASK_X-42)
#define R320240_AGC_MASK_Y R320240_DEMOD_MODE_MASK_Y
#define R320240_AGC_MASK_W (R320240_DEMOD_MODE_MASK_W-6)
#define R320240_AGC_MASK_H R320240_DEMOD_MODE_MASK_H

// Tunning step control
#define R320240_TUNE_STEP_X             		(R320240_TUNE_FREQ_X + 45)
#define R320240_TUNE_STEP_Y             		(R320240_TUNE_FREQ_Y - 21)
#define R320240_TUNE_STEP_MASK_H            	15
#define R320240_TUNE_STEP_MASK_W            	(SMALL_FONT_WIDTH*7)

#define R320240_PW_IND_X                		(R320240_DEMOD_MODE_X -1)
#define R320240_PW_IND_Y                		(R320240_DEMOD_MODE_Y - 16 -1)
#define R320240_PW_IND_W						41

#define R320240_SM_IND_X                		116
#define R320240_SM_IND_Y                		0
#define R320240_SM_IND_W 						(200)
#define R320240_SM_IND_H 						(72 - 14)

#define R320240_LEFTBOXES_IND_X              	159     // 156
#define R320240_LEFTBOXES_IND_Y              	62  // 63      // 64     // 130
#define R320240_LEFTBOX_WIDTH 					58 // used for the lower left side controls
#define R320240_LEFTBOX_ROW_H  					28
#define R320240_LEFTBOX_ROW_2ND_OFF  			13

#define R320240_ENCODER_IND_X                	0
#define R320240_ENCODER_IND_Y                	16-1

#define R320240_DIGMODE_IND_X              		0
#define R320240_DIGMODE_IND_Y              		191
#define R320240_DIGMODE_IND_W					58
//------------------------------------------------------------------------------------------------------------------------------------------
//Touchscreen definitions for 480x320
static const touchaction_descr_t R480320_touchactions_normal[] =
{
	//	{ {R480320_BAND_MODE_X,R480320_BAND_MODE_Y,R480320_BAND_MODE_MASK_W/2,R480320_BAND_MODE_MASK_H},                              UiAction_ChangeBandDownOrUp,   NULL }, // Left Part Band Display: Band down
	//	{ {R480320_BAND_MODE_X+R480320_BAND_MODE_MASK_W*3/4,R480320_BAND_MODE_Y,R480320_BAND_MODE_MASK_W/2,R480320_BAND_MODE_MASK_H}, UiAction_ChangeBandUpOrDown,   NULL }, // Right Part Band Display: Band up
	//	{ {R480320_BAND_MODE_X,R480320_BAND_MODE_Y,R480320_BAND_MODE_MASK_W/2+1 ,R480320_BAND_MODE_MASK_H},                               UiAction_Skeep48KhzDownByTouch,   NULL }, // Left Part Band Display: Band down
	//	{ {R480320_BAND_MODE_X+R480320_BAND_MODE_MASK_W*3/4 ,R480320_BAND_MODE_Y,R480320_BAND_MODE_MASK_W/2 ,R480320_BAND_MODE_MASK_H}, UiAction_Skeep48KhzUpByTouch  ,   NULL }, // Right Part Band Display: Band up
		{ {192, 297, 96, 23}, UiAction_Skeep48KhzDownByTouch, NULL }, // Left Part Band Display: Band down
		{ {288, 297, 96, 23}, UiAction_Skeep48KhzUpByTouch  , NULL }, // Right Part Band Display: Band up

		{ {0,105,480,176}, UiAction_ChangeFrequencyByTouch, NULL}, // Scope Draw Area: Tune to Touch

		{ {R480320_SM_IND_X,R480320_SM_IND_Y,R480320_SM_IND_W,R480320_SM_IND_H}, UiAction_ChangeLowerMeterUp,             NULL },  // Lower Meter: Meter Toggle
	//	{ {0,110,160,16}, UiAction_ToggleWaterfallScopeDisplay,    UiAction_ChangeSpectrumSize }, // Spectrum Bar Left Part: WaterfallScope Toggle
	//	{ {(480/2)-16,110,48,16}, UiAction_ChangeSpectrumZoomLevelDown,    UiAction_CheckSpectrumTouchActions }, // Spectrum Bar Middle Part: Decrease Zoom Level
	//	{ {(480/2)+100,110,48,16}, UiAction_ChangeSpectrumZoomLevelUp,      UiAction_CheckSpectrumTouchActions }, // Spectrum Bar Right Part: Increase Zoom Level
		{ {R480320_TUNE_FREQ_X+16*7,R480320_TUNE_FREQ_Y,16*3,24}, UiAction_ChangeFrequencyToNextKhz,       NULL }, // Frequency display, last three digits :Set last 3 digits to zero
		{ {R480320_TUNE_FREQ_X,R480320_TUNE_FREQ_Y,16*7,24}, UiVk_BndSelVirtualKeys, UiVk_BndFreqSetVirtualKeys }, // Frequency display, first digits :Select the band
		{ {R480320_DEMOD_MODE_X,R480320_DEMOD_MODE_Y,R480320_DEMOD_MODE_MASK_W,R480320_DEMOD_MODE_MASK_H}, UiVk_ModSelVirtualKeys,                NULL }, // Demod Mode Box: mode switch
		{ {R480320_PW_IND_X,R480320_PW_IND_Y,64,30},								 UiAction_ChangePowerLevel,               NULL }, // Power Box: TX Power Increase
		{ {R480320_ENCODER_IND_X+ENC_COL_W*5+Xspacing*2,R480320_ENCODER_IND_Y,ENC_COL_W,ENC_ROW_H}, UiAction_ChangeAudioSource,              NULL }, // Audio In Box: Switch Source

		{ {R480320_LEFTBOXES_IND_X,R480320_LEFTBOXES_IND_Y,R480320_LEFTBOX_WIDTH,R480320_LEFTBOX_ROW_H}, UiVk_DSPVirtualKeys, NULL } , // Codec_RestartI2S }, // DSP Box: Restart I2S
		{ {R480320_DIGMODE_IND_X,R480320_DIGMODE_IND_Y,R480320_DIGMODE_IND_W,16}, UiVk_ModSelVirtualKeys,              NULL }, // Digital Mode Box: Switch Digi Mode
		{ {R480320_TUNE_STEP_X,R480320_TUNE_STEP_Y,R480320_TUNE_STEP_MASK_W,R480320_TUNE_STEP_MASK_H}, UiAction_ChangeDynamicTuning,            NULL }, // Step Box: Dynamic Tuning Toggle
	//  { {R480320_TUNE_STEP_X,R480320_TUNE_STEP_Y,R480320_TUNE_STEP_MASK_W,R480320_TUNE_STEP_MASK_H}, UiAction_ChangeDynamicTuning,            NULL }, // Step Box: Dynamic Tuning Toggle
};

// this is the map for menu mode, right now only used for debugging/experimental purposes
static const touchaction_descr_t R480320_touchactions_menu[] =
{
		{ { R480320_SM_IND_X+R480320_SM_IND_W-16,R480320_SM_IND_Y,16,16 }, UiAction_ChangeDebugInfoDisplay}, // S-Meter db: toggle show tp coordinates
};

static const touchaction_list_descr_t R480320_touch_regions[] =
{
		// ATTENTION: the size calculation only works for true arrays, not for pointers!
		{ R480320_touchactions_normal, sizeof(R480320_touchactions_normal)/sizeof(*R480320_touchactions_normal) },
		{ R480320_touchactions_menu  , sizeof(R480320_touchactions_menu  )/sizeof(*R480320_touchactions_menu  ) },
};

//------------------------------------------------------------------------------------------------------------------------------------------
//Touchscreen definitions for 320x240
static const touchaction_descr_t R320240_touchactions_normal[] =
{
	//	{ {R320240_BAND_MODE_X,R320240_BAND_MODE_Y,R320240_BAND_MODE_MASK_W/2,R320240_BAND_MODE_MASK_H},                              UiAction_ChangeBandDownOrUp    ,NULL }, // Left Part Band Display: Band down
	//	{ {R320240_BAND_MODE_X+R320240_BAND_MODE_MASK_W*3/4,R320240_BAND_MODE_Y,R320240_BAND_MODE_MASK_W/2,R320240_BAND_MODE_MASK_H}, UiAction_ChangeBandUpOrDown    ,NULL }, // Right Part Band Display: Band up
	//	{ {R320240_BAND_MODE_X,R320240_BAND_MODE_Y,R320240_BAND_MODE_MASK_W/2+1 ,R320240_BAND_MODE_MASK_H},                              UiAction_Skeep48KhzDownByTouch ,NULL }, // Left Part Band Display: Band down
	//	{ {R320240_BAND_MODE_X+R320240_BAND_MODE_MASK_W*3/4 ,R320240_BAND_MODE_Y,R320240_BAND_MODE_MASK_W/2 ,R320240_BAND_MODE_MASK_H}, UiAction_Skeep48KhzUpByTouch   ,NULL }, // Right Part Band Display: Band up
		{ {  0, 128,320, 94}, UiAction_ChangeFrequencyByTouch, NULL }, // Scope Draw Area: Tune to Touch
		{ {128, 223, 64, 17}, UiAction_Skeep48KhzDownByTouch , NULL }, // Left Part Band Display: Band down
		{ {192, 223, 64, 17}, UiAction_Skeep48KhzUpByTouch   , NULL }, // Right Part Band Display: Band up



		{ {R320240_SM_IND_X,R320240_SM_IND_Y,R320240_SM_IND_W,R320240_SM_IND_H},UiAction_ChangeLowerMeterUp,NULL },  // Lower Meter: Meter Toggle
	//	{ {64,128,60,16}, UiAction_ToggleWaterfallScopeDisplay,UiAction_ChangeSpectrumSize }, // Spectrum Bar Left Part: WaterfallScope Toggle
	//	{ {180,128,40,16}, UiAction_ChangeSpectrumZoomLevelDown,UiAction_CheckSpectrumTouchActions }, // Spectrum Bar Middle Part: Decrease Zoom Level
	//	{ {280,128,40,16}, UiAction_ChangeSpectrumZoomLevelUp,UiAction_CheckSpectrumTouchActions }, // Spectrum Bar Right Part: Increase Zoom Level
		{ {R320240_TUNE_FREQ_X+16*6 + 4 ,R320240_TUNE_FREQ_Y,16*2 ,24}, UiAction_ChangeFrequency_Sub500Hz  ,NULL }, // /*UiAction_ChangeFrequencyToNextKhz*/Tune button:Set last 3 digits to last 500Hz

		{ {R320240_TUNE_FREQ_X+16*8 + 5 ,R320240_TUNE_FREQ_Y,16+11 ,24}, UiAction_ChangeFrequency_Add500Hz ,NULL }, // Tune button:Set last 3 digits to next 500Hz

		{ {R320240_TUNE_FREQ_X -2,R320240_TUNE_FREQ_Y,16*6,24}, UiVk_BndSelVirtualKeys,       NULL }, // Frequency display, first digits :Select the band
		{ {R320240_DEMOD_MODE_X - 5,R320240_DEMOD_MODE_Y,R320240_DEMOD_MODE_MASK_W -1,R320240_DEMOD_MODE_MASK_H}, UiAction_ChangeDemodMode,NULL }, // Demod Mode Box: mode switch
		{ {R320240_PW_IND_X - 2,R320240_PW_IND_Y,42,16},UiAction_ChangePowerLevel,NULL }, // Power Box: TX Power Increase
		{ {R320240_ENCODER_IND_X+ENC_COL_W*2,R320240_ENCODER_IND_Y+ENC_ROW_H,ENC_COL_W,ENC_ROW_H}, UiAction_ChangeAudioSource  , NULL }, // Audio In Box: Switch Source
		{ {R320240_ENCODER_IND_X+ENC_COL_W*2,R320240_ENCODER_IND_Y          ,ENC_COL_W,ENC_ROW_H}, UiAction_RAZ_RIT_And_Display, NULL }, // Effacer le RIT par touchscreen = 0

		{ {R320240_LEFTBOXES_IND_X + R320240_LEFTBOX_WIDTH   ,R320240_LEFTBOXES_IND_Y  ,R320240_LEFTBOX_WIDTH+2 , R320240_LEFTBOX_ROW_H}, UiVk_DSPVirtualKeys, NULL }, // Codec_RestartI2S }, // DSP Box: Restart I2S
		{ {R320240_LEFTBOXES_IND_X                           ,R320240_LEFTBOXES_IND_Y , R320240_LEFTBOX_WIDTH   , R320240_LEFTBOX_ROW_H}, UIVK_BWTouschscreen, NULL }, // DSP Box: Restart I2S
	//	{ {R320240_DIGMODE_IND_X,R320240_DIGMODE_IND_Y,R320240_DIGMODE_IND_W,16}, UiAction_ChangeDigitalMode,NULL }, // Digital Mode Box: Switch Digi Mode
		{ {R320240_TUNE_STEP_X + 118,R320240_TUNE_STEP_Y + 23 ,R320240_TUNE_STEP_MASK_W -15,R320240_TUNE_STEP_MASK_H}, UiAction_ChangeDynamicTuning,NULL }, // Step Box: Dynamic Tuning Toggle
		{ {278   ,61 ,43 , 14}, UiAction_Debug_switch, NULL }, //  enable/disable  info debug boolean

#ifdef ATTENUATOR_6DB
		{ {278   ,76 ,43 , 14}, UiAction_Attenuator_switch, NULL }, //  enable/disable  Antenna attenuator
#endif

};

// this is the map for menu mode, right now only used for debugging/experimental purposes
static const touchaction_descr_t R320240_touchactions_menu[] =
{
		{ { R320240_SM_IND_X+R320240_SM_IND_W-16,R320240_SM_IND_Y,16,16 }, UiAction_ChangeDebugInfoDisplay}, // S-Meter db: toggle show tp coordinates
};

static const touchaction_list_descr_t R320240__touch_regions[] =
{
					// ATTENTION: the size calculation only works for true arrays, not for pointers!
		{ R320240_touchactions_normal, sizeof(R320240_touchactions_normal)/sizeof(*R320240_touchactions_normal)},
		{ R320240_touchactions_menu  , sizeof(R320240_touchactions_menu  )/sizeof(*R320240_touchactions_menu  )},
};
//------------------------------------------------------------------------------------------------------------------------------------------
const LcdLayout LcdLayouts[LcdLayoutsCount]=
{
		//-----------------------------------------------------------------
		{		//320x240
				.Size                  = { 320, 240 },
				.StartUpScreen_START   = { 0, 10 },
				.SpectrumWindow        = {  .x = 0, .y = 126, .w = 320, .h = 98  },  // {  .x = 58, .y = 128, .w = 260, .h = 94  },98

				.SpectrumWindowPadding = 0,

				.TUNE_FREQ             = { R320240_TUNE_FREQ_X       -1, R320240_TUNE_FREQ_Y },
				.TUNE_SPLIT_FREQ_X     = R320240_TUNE_SPLIT_FREQ_X   -1,
				.TUNE_SPLIT_MARKER_X   = R320240_TUNE_SPLIT_MARKER_X -1,
				.TUNE_SPLIT_FREQ_Y_TX  = R320240_TUNE_SPLIT_FREQ_Y_TX,
				.TUNE_SFREQ            = { R320240_TUNE_SFREQ_X, R320240_TUNE_SFREQ_Y },
				.DisplayDbm            = { R320240_DisplayDbm_X + 9 , R320240_DisplayDbm_Y },
				.MEMORYLABEL           = { R320240_MEMORYLABEL_X, R320240_MEMORYLABEL_Y} ,

				.BAND_MODE             = { R320240_BAND_MODE_X           ,      R320240_BAND_MODE_Y  },
				.BAND_MODE_MASK        = { .x = R320240_BAND_MODE_MASK_X , .y = R320240_BAND_MODE_MASK_Y , .h = R320240_BAND_MODE_MASK_H, .w = R320240_BAND_MODE_MASK_W },

				.DEMOD_MODE_MASK       = { .x = R320240_DEMOD_MODE_MASK_X - 3, .y = R320240_DEMOD_MODE_MASK_Y, .h = R320240_DEMOD_MODE_MASK_H, .w = R320240_DEMOD_MODE_MASK_W },
				.AGC_MASK              = {.x = R320240_AGC_MASK_X, .y = R320240_AGC_MASK_Y, .h = R320240_AGC_MASK_H,
			    .w                     = R320240_AGC_MASK_W},
				.TUNE_STEP             = {.x=R320240_TUNE_STEP_X + 118 , .y=R320240_TUNE_STEP_Y + 23 , .h=R320240_TUNE_STEP_MASK_H, .w=R320240_TUNE_STEP_MASK_W - 15 },

				.ENCODER_IND           = { R320240_ENCODER_IND_X, R320240_ENCODER_IND_Y},   // Les boutons VOL, AGC, RIT
				.ENCODER_MODE          = MODE_VERTICAL,

				.DIGMODE               = {.x=R320240_DIGMODE_IND_X,.y=R320240_DIGMODE_IND_Y,.w=R320240_DIGMODE_IND_W},

				.LEFTBOXES_IND         = { .x=R320240_LEFTBOXES_IND_X , .y=R320240_LEFTBOXES_IND_Y , .w=R320240_LEFTBOX_WIDTH , .h=R320240_LEFTBOX_ROW_H},
				.LEFTBOXES_MODE        = MODE_HORIZONTAL ,                      // MODE_VERTICAL,
				.LEFTBOXES_ROW_2ND_OFF =  R320240_LEFTBOX_ROW_2ND_OFF,

				.PW_IND                = { .x=R320240_PW_IND_X - 3, .y=R320240_PW_IND_Y, .w=R320240_PW_IND_W },
				.TEMP_IND              = {.x = 0, .y = 0},
				.RTC_IND               = {.x= 5, .y = 78-1},   /// pour affichage de l'horloge RTC  NIZZZ

				.LOADANDDEBUG_Y        = 77 ,    // 79,  95, NIZZZZ  position d'affichage du Load du processeur d�cal� plus haut
				.DEBUG_X               = 0,
				.LOAD_X                = 279,

				.PWR_NUM_IND           = { 1, 91},   /// affichage numerique de Power reverse et forward

				.CW_DECODER_WPM        = { 0, 105},

				.SNAP_CARRIER          = { 27, 119},

				.TextMsgLine           = { 5, 91},   // 92
				.TextMsg_buffer_max    = 44,
				.TextMsg_font          = 4,

				.FREEDV_SNR            = { 5, 116},
				.FREEDV_BER            = { 5, 104},
				.FREEDV_FONT           = 4,

				.SM_IND                = {.x=R320240_SM_IND_X,.y=R320240_SM_IND_Y,.h=R320240_SM_IND_H,.w=R320240_SM_IND_W},
				.PWR_IND               = {.x = 0 +279, .y = 193 -129 -1 -1},  // .x = 4, .y = 193 + 35 NIZZZ pour affichage voltage de la tension

				.BOTTOM_BAR            = {.x=0,.y=228, .h=16, .w=62}, // {.x=0,.y=228, .h=16, .w=62}la barre au fond d 'ecran pour les boutons F1 F2 F3 F4 et F5.

				.MENUSIZE           = 7,
				.MENU_IND           = { 60, 128},
				.MENU_CHANGE_X      = 244,
				.MENU_CURSOR_X      = 311,
				.MENU_TEXT_SIZE_MAX = 34,

				.touchaction_list   = R320240__touch_regions
		},

		//-----------------------------------------------------------------
		{		//480x320
				.Size = { 480, 320},
				.StartUpScreen_START ={ 80, 60},
				.SpectrumWindow={ .x = 0, .y = 110, .w = 480, .h = 176 },
				.SpectrumWindowPadding=0,

				.TUNE_FREQ= { R480320_TUNE_FREQ_X, R480320_TUNE_FREQ_Y},
				.TUNE_SPLIT_FREQ_X=R480320_TUNE_SPLIT_FREQ_X,
				.TUNE_SPLIT_MARKER_X=R480320_TUNE_SPLIT_MARKER_X,
				.TUNE_SPLIT_FREQ_Y_TX=R480320_TUNE_SPLIT_FREQ_Y_TX,
				.TUNE_SFREQ = { R480320_TUNE_SFREQ_X, R480320_TUNE_SFREQ_Y},
				.DisplayDbm = { R480320_DisplayDbm_X, R480320_DisplayDbm_Y},
				.MEMORYLABEL = { R480320_MEMORYLABEL_X, R480320_MEMORYLABEL_Y},

				.BAND_MODE = { R480320_BAND_MODE_X, R480320_BAND_MODE_Y},
				.BAND_MODE_MASK = { .x = R480320_BAND_MODE_MASK_X, .y = R480320_BAND_MODE_MASK_Y, .h = R480320_BAND_MODE_MASK_H, .w= R480320_BAND_MODE_MASK_W},

				.DEMOD_MODE_MASK = { .x = R480320_DEMOD_MODE_MASK_X, .y = R480320_DEMOD_MODE_MASK_Y, .h = R480320_DEMOD_MODE_MASK_H,
                .w = R480320_DEMOD_MODE_MASK_W},
				.AGC_MASK = {.x = R480320_AGC_MASK_X, .y = R480320_AGC_MASK_Y, .h = R480320_AGC_MASK_H,
		                .w = R480320_AGC_MASK_W},
				.TUNE_STEP={.x=R480320_TUNE_STEP_X, .y=R480320_TUNE_STEP_Y, .h=R480320_TUNE_STEP_MASK_H, .w=R480320_TUNE_STEP_MASK_W},

				.ENCODER_IND = { R480320_ENCODER_IND_X, R480320_ENCODER_IND_Y},
				.ENCODER_MODE=MODE_HORIZONTAL,

				.DIGMODE={.x=R480320_DIGMODE_IND_X,.y=R480320_DIGMODE_IND_Y,.w=R480320_DIGMODE_IND_W},

				.LEFTBOXES_IND         = { .x=R480320_LEFTBOXES_IND_X, .y=R480320_LEFTBOXES_IND_Y, .w=R480320_LEFTBOX_WIDTH, .h=R480320_LEFTBOX_ROW_H},
				.LEFTBOXES_MODE        = MODE_HORIZONTAL,
				.LEFTBOXES_ROW_2ND_OFF = R480320_LEFTBOX_ROW_2ND_OFF,

				.PW_IND              = { .x=R480320_PW_IND_X, .y=R480320_PW_IND_Y, .w=R480320_PW_IND_W},
				.TEMP_IND            = {.x = 370,.y = 64},
				.RTC_IND             = {.x = 415,.y = 80},

				.LOADANDDEBUG_Y      = 96,
				.DEBUG_X             = 0,
				.LOAD_X              = 280,

				.PWR_NUM_IND         = { 320, 96},

				.CW_DECODER_WPM      = { 420, 290},

				.SNAP_CARRIER        = { 242, 29},

				.TextMsgLine         = { 0, 290 },
				.TextMsg_buffer_max  = 50,
				.TextMsg_font        = 0,

				//.FREEDV_SNR         = { 280, 28},
				//.FREEDV_BER         = {380, 28},
				.FREEDV_SNR           = {410,288},
				.FREEDV_BER           = {410,297},
				.FREEDV_FONT          = 4,

				.SM_IND={.x = R480320_SM_IND_X, .y = R480320_SM_IND_Y, .h = R480320_SM_IND_H, .w = R480320_SM_IND_W},
				.PWR_IND={ .x = 420  , .y = 307  },

#ifdef UI_BRD_OVI40
				.BOTTOM_BAR={.x=0,.y=308, .h=16, .w=74},
#else
				.BOTTOM_BAR={.x=32,.y=308, .h=16, .w=74},
#endif
				.MENUSIZE           = 14,
				.MENU_IND           = { 80, 110 },
				.MENU_CHANGE_X      = 280,
				.MENU_CURSOR_X      = 360,
				.MENU_TEXT_SIZE_MAX = 40,

				.touchaction_list   = R480320_touch_regions
		}
};


disp_resolution_t disp_resolution;
#endif
