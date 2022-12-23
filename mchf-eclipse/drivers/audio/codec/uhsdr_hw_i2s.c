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

// Common
#include "uhsdr_board.h"
#include "profiling.h"
#include "uhsdr_hw_i2s.h"

#include "audio_driver.h"

#ifdef UI_BRD_MCHF
#include "i2s.h"
#endif

#ifdef UI_BRD_OVI40
#include "sai.h"
#endif




__UHSDR_DMAMEM __IO   dma_audio_buffer_t    audio_buf [ DMA_AUDIO_NUM ];


static uint32_t szbuf;


#ifdef USE_24_BITS
typedef int32_t audio_data_t;
#else
typedef int16_t audio_data_t;
#endif

#ifdef UI_BRD_MCHF
#define CODEC_IQ_IDX 0
#define CODEC_ANA_IDX  0
#endif

#ifdef UI_BRD_OVI40
#define CODEC_IQ_IDX 1
#define CODEC_ANA_IDX 0
#endif

void UhsdrHwI2s_Codec_ClearTxDmaBuffer()
{
    memset( (void*) &audio_buf[ CODEC_IQ_IDX ].out,  0 , sizeof( audio_buf[ CODEC_IQ_IDX ].out ));
}


//******************************************************************************************************************


static void MchfHw_Codec_HandleBlock( uint16_t which )
{
#ifdef PROFILE_EVENTS
													// we stop during interrupt
													// at the end we start again
													// profileCycleCount_stop();
	if ( ts.show_debug_info  )  profileTimedEventStart( ProfileAudioInterrupt );
#endif

    static  audio_data_t *src, *dst;


#ifdef EXEC_PROFILING

    GPIOE->BSRRL = GPIO_Pin_10;         // Profiling pin (high level)
#endif

    ts.audio_int_counter++;   			// generating a time base for encoder handling

										// Transfer complete interrupt
										// Point to 2nd half of buffers

    uint16_t  offset = which == 0?64:0;     // soit debut du buffer zero , soit après 64 data de 32 bits stereo

    src = (audio_data_t*) &audio_buf [ CODEC_IQ_IDX ].in [offset];
    dst = (audio_data_t*) &audio_buf [ CODEC_ANA_IDX].out[offset];

    AudioDriver_I2S_Callback ( src, dst );   // Handle

#ifdef EXEC_PROFILING

    GPIOE->BSRRH = GPIO_Pin_10;           // Profiling pin (low level)
#endif
#ifdef PROFILE_EVENTS
														// we stopped during interrupt
														// now we start again
														// profileCycleCount_start();
    if ( ts.show_debug_info  )  profileTimedEventStop ( ProfileAudioInterrupt );
#endif
}

#ifdef UI_BRD_MCHF
/**
 * @brief HAL Handler for Codec DMA Interrupt
 */
void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
    MchfHw_Codec_HandleBlock(0);
}

/**
 * @brief HAL Handler for Codec DMA Interrupt
 */
void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
    MchfHw_Codec_HandleBlock(1);
}
#endif

#ifdef UI_BRD_OVI40
void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hi2s)
{
    if (hi2s == &hsai_BlockA2)
    {
        MchfHw_Codec_HandleBlock(0);
    }
}




void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hi2s)
{
    if (hi2s == &hsai_BlockA2)
    {
        MchfHw_Codec_HandleBlock(1);
    }
}
#endif

void UhsdrHwI2s_Codec_StartDMA()
{
    szbuf =   BUFF_LEN ;   // NIZZZ 2 foix memoires de 2x32 ;  2*2*32 =128;

#ifdef UI_BRD_MCHF
    HAL_I2SEx_TransmitReceive_DMA ( &hi2s3,  (uint16_t*) audio_buf [0]. out,  (uint16_t*) audio_buf [0].in, szbuf );
#endif
#ifdef UI_BRD_OVI40
    // we clean the buffers since we don't know if we are in a "cleaned" memory segement
    memset( (void*)audio_buf, 0, sizeof( audio_buf ) );

    HAL_SAI_Receive_DMA ( &hsai_BlockA1, (uint8_t*) audio_buf[0].in,  szbuf );
    HAL_SAI_Transmit_DMA( &hsai_BlockB1, (uint8_t*) audio_buf[0].out, szbuf );

    HAL_SAI_Receive_DMA ( &hsai_BlockA2, (uint8_t*) audio_buf[1].in,  szbuf );
    HAL_SAI_Transmit_DMA( &hsai_BlockB2, (uint8_t*) audio_buf[1].out, szbuf );

#endif
}


void UhsdrHwI2s_Codec_StopDMA(void)
{
#ifdef UI_BRD_MCHF
    HAL_I2S_DMAStop(&hi2s3);
#endif
#ifdef UI_BRD_OVI40
    HAL_SAI_DMAStop(&hsai_BlockA1);
    HAL_SAI_DMAStop(&hsai_BlockB1);
    HAL_SAI_DMAStop(&hsai_BlockA2);
    HAL_SAI_DMAStop(&hsai_BlockB2);
#endif
}