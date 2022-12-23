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

#ifndef __MCHF_HW_I2S_H
#define __MCHF_HW_I2S_H

// #define BUFF_LEN (2*(32*2)) // == 128     // le buffer dma en boucle  est le double du buffer a traiter
#define BUFF_LEN 128

/*
 * BUFF_LEN is derived from 2* 32 LR Samples per Audio-Interrupt (== 64 * int16_t )
 * since we get half of the buffer in each DMA Interrupt for processing
 */

typedef struct
{
    int32_t out[BUFF_LEN];       // NIZ32  int16_t    le bufer DMA stereo est le double du buffer a traiter
    int32_t in [BUFF_LEN];
}   dma_audio_buffer_t;


#if defined(UI_BRD_MCHF)
#define DMA_AUDIO_NUM   1
#elif defined(UI_BRD_OVI40)
#define DMA_AUDIO_NUM   1    //   2     RS928p est une fausse OVI40 a cause de la presence de  SI5351A
#endif

void UhsdrHwI2s_Codec_StartDMA();
void UhsdrHwI2s_Codec_StopDMA();
void UhsdrHwI2s_Codec_ClearTxDmaBuffer();

#endif

