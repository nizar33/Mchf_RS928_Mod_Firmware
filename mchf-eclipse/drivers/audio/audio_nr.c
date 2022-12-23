/************************************************************************************
 **                                                                                 **
 **                               UHSDR Firmware                                    **
 **                                                                                 **
 **---------------------------------------------------------------------------------**
 **                                                                                 **
 **  Description:                                                                   **
 **  Licence:       GNU GPLv3                                                       **
 ************************************************************************************/

#include "uhsdr_board.h"
#include "audio_nr.h"
#include "arm_const_structs.h"
#include "profiling.h"

//#define debug_alternate_NR

#ifdef USE_ALTERNATE_NR

NoiseReduction  __MCHF_SPECIALMEM 	NR; // definition
NoiseReduction2 __MCHF_SPECIALMEM	NR2; // definition

__IO int32_t  NR_in_head  = 0;
__IO int32_t  NR_in_tail  = 0;
__IO int32_t  NR_out_head = 0;
__IO int32_t  NR_out_tail = 0;

NR_Buffer*   NR_in_buffers [ NR_BUFFER_FIFO_SIZE ];
NR_Buffer*   NR_out_buffers[ NR_BUFFER_FIFO_SIZE ];


 const float32_t SQRT_von_Hann_256[256]
= { 0          ,0.01231966 ,0.024637449,0.036951499,0.049259941,0.061560906,0.073852527,0.086132939,
	0.098400278,0.110652682,0.122888291,0.135105247,0.147301698,0.159475791,0.171625679,0.183749518,
	0.195845467,0.207911691,0.219946358,0.231947641,0.24391372 ,0.255842778,0.267733003,0.279582593,
	0.291389747,0.303152674,0.314869589,0.326538713,0.338158275,0.349726511,0.361241666,0.372701992,
	0.384105749,0.395451207,0.406736643,0.417960345,0.429120609,0.440215741,0.451244057,0.462203884,
	0.473093557,0.483911424,0.494655843,0.505325184,0.515917826,0.526432163,0.536866598,0.547219547,
	0.557489439,0.567674716,0.577773831,0.587785252,0.597707459,0.607538946,0.617278221,0.626923806,
	0.636474236,0.645928062,0.65528385 ,0.664540179,0.673695644,0.682748855,0.691698439,0.700543038,
	0.709281308,0.717911923,0.726433574,0.734844967,0.743144825,0.75133189 ,0.759404917,0.767362681,
	0.775203976,0.78292761 ,0.790532412,0.798017227,0.805380919,0.812622371,0.819740483,0.826734175,
	0.833602385,0.840344072,0.846958211,0.853443799,0.859799851,0.866025404,0.872119511,0.878081248,
	0.883909710,0.889604013,0.895163291,0.900586702,0.905873422,0.911022649,0.916033601,0.920905518,
	0.925637660,0.930229309,0.934679767,0.938988361,0.943154434,0.947177357,0.951056516,0.954791325,
	0.958381215,0.961825643,0.965124085,0.968276041,0.971281032,0.974138602,0.976848318,0.979409768,
	0.981822563,0.984086337,0.986200747,0.988165472,0.989980213,0.991644696,0.993158666,0.994521895,
	0.995734176,0.996795325,0.99770518 ,0.998463604,0.999070481,0.99952572 ,0.99982925 ,0.999981027,
	0.999981027,0.99982925 ,0.99952572 ,0.999070481,0.998463604,0.99770518 ,0.996795325,0.995734176,
	0.994521895,0.993158666,0.991644696,0.989980213,0.988165472,0.986200747,0.984086337,0.981822563,
	0.979409768,0.976848318,0.974138602,0.971281032,0.968276041,0.965124085,0.961825643,0.958381215,
	0.954791325,0.951056516,0.947177357,0.943154434,0.938988361,0.934679767,0.930229309,0.925637660,
	0.920905518,0.916033601,0.911022649,0.905873422,0.900586702,0.895163291,0.889604013,0.883909710,
	0.878081248,0.872119511,0.866025404,0.859799851,0.853443799,0.846958211,0.840344072,0.833602385,
	0.826734175,0.819740483,0.812622371,0.805380919,0.798017227,0.790532412,0.78292761 ,0.775203976,
	0.767362681,0.759404917,0.75133189 ,0.743144825,0.734844967,0.726433574,0.717911923,0.709281308,
	0.700543038,0.691698439,0.682748855,0.673695644,0.664540179,0.65528385 ,0.645928062,0.636474236,
	0.626923806,0.617278221,0.607538946,0.597707459,0.587785252,0.577773831,0.567674716,0.557489439,
	0.547219547,0.536866598,0.526432163,0.515917826,0.505325184,0.494655843,0.483911424,0.473093557,
	0.462203884,0.451244057,0.440215741,0.429120609,0.417960345,0.406736643,0.395451207,0.384105749,
	0.372701992,0.361241666,0.349726511,0.338158275,0.326538713,0.314869589,0.303152674,0.291389747,
	0.279582593,0.267733003,0.255842778,0.24391372 ,0.231947641,0.219946358,0.207911691,0.195845467,
	0.183749518,0.171625679,0.159475791,0.147301698,0.135105247,0.122888291,0.110652682,0.098400278,
	0.086132939,0.073852527,0.061560906,0.049259941,0.036951499,0.024637449,0.01231966 ,          0};
/*
 const float32_t INV_SQRT_von_Hann_256[128]
 = {  1.409877843, 1.39292853 , 1.376588357, 1.360831257, 1.34563273 , 1.330969727, 1.31682055 , 1.303164754,
	  1.289983064, 1.277257292, 1.264970271, 1.25310578 , 1.241648487, 1.230583892, 1.219898274, 1.209578641,
	  1.19961269 , 1.18998876 , 1.180695797, 1.171723318, 1.163061378, 1.154700538, 1.14663184 , 1.138846778,
	  1.131337272, 1.124095649, 1.11711462 , 1.110387259, 1.103906987, 1.097667551, 1.091663012, 1.085887727,
	  1.080336338, 1.075003755, 1.069885147, 1.064975927, 1.060271747, 1.055768482, 1.051462224, 1.047349273,
	  1.043426127, 1.039689477, 1.036136198, 1.032763342, 1.029568134, 1.026547965, 1.023700386, 1.021023103,
	  1.018513974, 1.016171003, 1.013992336, 1.011976261, 1.010121199, 1.008425704, 1.00688846 , 1.00550828 ,
	  1.004284099, 1.003214978, 1.002300098, 1.00153876 , 1.000930384, 1.000474505, 1.000170779, 1.000018973,
	  1.000018973, 1.000170779, 1.000474505, 1.000930384, 1.00153876 , 1.002300098, 1.003214978, 1.004284099,
	  1.00550828 , 1.00688846 , 1.008425704, 1.010121199, 1.011976261, 1.013992336, 1.016171003, 1.018513974,
	  1.021023103, 1.023700386, 1.026547965, 1.029568134, 1.032763342, 1.036136198, 1.039689477, 1.043426127,
	  1.047349273, 1.051462224, 1.055768482, 1.060271747, 1.064975927, 1.069885147, 1.075003755, 1.080336338,
	  1.085887727, 1.091663012, 1.097667551, 1.103906987, 1.110387259, 1.11711462 , 1.124095649, 1.131337272,
	  1.138846778, 1.14663184 , 1.154700538, 1.163061378, 1.171723318, 1.180695797, 1.18998876 , 1.19961269,
	  1.209578641, 1.219898274, 1.230583892, 1.241648487, 1.25310578 , 1.264970271, 1.277257292, 1.289983064,
	  1.303164754, 1.31682055 , 1.330969727, 1.34563273 , 1.360831257, 1.376588357, 1.39292853 , 1.409877843 };
*/

/*
 const float32_t INV_SQRT_von_Hann_256[128]
= {
	 1.987755532, 1.940249891, 1.894995504, 1.851861709, 1.810727443, 1.771480414, 1.734016361, 1.698238376,
	 1.664056304, 1.631386191, 1.600149787, 1.570274096, 1.541690965, 1.514336715, 1.488151798, 1.46308049 ,
	 1.439070607, 1.41607325 , 1.394042566, 1.372935535, 1.352711769, 1.333333333, 1.314764578, 1.296971983,
	 1.279924022, 1.263591028, 1.247945074, 1.232959865, 1.218610635, 1.204874052, 1.191728131, 1.179152156,
	 1.167126604, 1.155633074, 1.144654227, 1.134173726, 1.124176178, 1.114647088, 1.105572809, 1.0969405  ,
	 1.088738083, 1.080954209, 1.07357822 , 1.066600121, 1.060010543, 1.053800725, 1.04796248 , 1.042488177,
	 1.037370715, 1.032603506, 1.028180458, 1.024095953, 1.020344837, 1.0169224  , 1.013824371, 1.0110469  ,
	 1.008586551, 1.006440292, 1.004605487, 1.003079888, 1.001861633, 1.000949236, 1.000341587, 1.000037946,
	 1.000037946, 1.000341587, 1.000949236, 1.001861633, 1.003079888, 1.004605487, 1.006440292, 1.008586551,
	 1.0110469  , 1.013824371, 1.0169224  , 1.020344837, 1.024095953, 1.028180458, 1.032603506, 1.037370715,
	 1.042488177, 1.04796248 , 1.053800725, 1.060010543, 1.066600121, 1.07357822 , 1.080954209, 1.088738083,
	 1.0969405  , 1.105572809, 1.114647088, 1.124176178, 1.134173726, 1.144654227, 1.155633074, 1.167126604,
	 1.179152156, 1.191728131, 1.204874052, 1.218610635, 1.232959865, 1.247945074, 1.263591028, 1.279924022,
	 1.296971983, 1.314764578, 1.333333333, 1.352711769, 1.372935535, 1.394042566, 1.41607325 , 1.439070607,
	 1.46308049 , 1.488151798, 1.514336715, 1.541690965, 1.570274096, 1.600149787, 1.631386191, 1.664056304,
	 1.698238376, 1.734016361, 1.771480414, 1.810727443, 1.851861709, 1.894995504, 1.940249891, 1,987755532 };
 */

//  NIZZ 06 Nov 2022 :
//  fenetre de Hanning (0.50 + 0.50*cos(phi)) originale a été remplacée par
//  fenetre de Hamming (0.54 + 0.46*cos(phi))




int NR_in_buffer_peek(NR_Buffer** c_ptr)
{
    int ret = 0;

    if (NR_in_head != NR_in_tail)
    {
        NR_Buffer* c = NR_in_buffers[NR_in_tail];
        *c_ptr = c;
        ret++;
    }
    return ret;
}


int NR_in_buffer_remove(NR_Buffer** c_ptr)
{
    int ret = 0;

    if (NR_in_head  != NR_in_tail)
    {
        NR_Buffer* c = NR_in_buffers[NR_in_tail];
        NR_in_tail   = (NR_in_tail + 1) % NR_BUFFER_FIFO_SIZE;
        *c_ptr       = c;
        ret++;
    }
    return ret;
}

/* no room left in the buffer returns 0 */
int NR_in_buffer_add(NR_Buffer* c)
{
    int ret = 0;
    int32_t next_head = (NR_in_head + 1) % NR_BUFFER_FIFO_SIZE;

    if (next_head != NR_in_tail)
    {
        /* there is room */
        NR_in_buffers[NR_in_head] = c;
        NR_in_head = next_head;
        ret ++;
    }
    return ret;
}

void NR_in_buffer_reset()
{
    NR_in_tail = NR_in_head;
}

int8_t NR_in_has_data()
{
    int32_t len = NR_in_head - NR_in_tail;
    return  len < 0?len+NR_BUFFER_FIFO_SIZE:len;
}

int32_t NR_in_has_room()
{
    // FIXME: Since we cannot completely fill the buffer
    // we need to say full 1 element earlier
    return NR_BUFFER_FIFO_SIZE - 1 - NR_in_has_data();
}


//*********Out Buffer handling

int NR_out_buffer_peek(NR_Buffer** c_ptr)
{
    int ret = 0;

    if (NR_out_head != NR_out_tail)
    {
        NR_Buffer* c = NR_out_buffers[NR_out_tail];
        *c_ptr = c;
        ret++;
    }
    return ret;
}


int NR_out_buffer_remove ( NR_Buffer**  c_ptr )
{
    int ret = 0;

    if ( NR_out_head != NR_out_tail )
    {
        NR_Buffer* c =  NR_out_buffers [ NR_out_tail ];
        NR_out_tail  = (NR_out_tail + 1) % NR_BUFFER_FIFO_SIZE;
        *c_ptr = c;
        ret++;
    }
    return ret;
}

/* no room left in the buffer returns 0 */
int NR_out_buffer_add(NR_Buffer* c)
{
    int ret = 0;
    int32_t next_head = (NR_out_head + 1) % NR_BUFFER_FIFO_SIZE;

    if (next_head != NR_out_tail)
    {
        /* there is room */
        NR_out_buffers [NR_out_head ] = c;
        NR_out_head = next_head;
        ret ++;
    }
    return ret;
}

void NR_out_buffer_reset()
{
    NR_out_tail = NR_out_head;
}

int8_t NR_out_has_data()
{
    int32_t len = NR_out_head - NR_out_tail;
    return  len < 0? len + NR_BUFFER_FIFO_SIZE : len;
}

int32_t NR_out_has_room()
{
    // FIXME: Since we cannot completely fill the buffer
    // we need to say full 1 element earlier
    return NR_BUFFER_FIFO_SIZE - 1 - NR_out_has_data();
}



void alternateNR_handle()
{
    static uint16_t  NR_current_buffer_idx = 0;
    static bool      NR_was_here           = false;

    if (NR_was_here == false)
    {
        NR_was_here           = true;
        NR_current_buffer_idx = 0;
        NR_in_buffer_reset();
        NR_out_buffer_reset();
    }

    if ( NR_in_has_data() && NR_out_has_room())
    {   // audio data is ready to be processed

        NR_current_buffer_idx %= NR_BUFFER_NUM;

        NR_Buffer* input_buf = NULL;
        NR_in_buffer_remove(&input_buf); //&input_buffer points to the current valid audio data

        // inside here do all the necessary noise reduction stuff!!!!!
        // here are the current input samples:  input_buf->samples
        // NR_output samples have to be placed here: fdv_iq_buff[NR_current_buffer_idx].samples
        // but starting at an offset of NR_FFT_SIZE as we are using the same buffer for in and out
        // here is the only place where we are referring to fdv_iq... as this is the name of the used freedv buffer

        //profileTimedEventStart(ProfileTP8);

        do_alternate_NR( &input_buf->samples[0].real, &mmb.nr_audio_buff[NR_current_buffer_idx].samples[NR_FFT_SIZE].real);

        //profileTimedEventStop(ProfileTP8);

        NR_out_buffer_add( &mmb.nr_audio_buff[ NR_current_buffer_idx ] );
        NR_current_buffer_idx++;

    }

}


void  do_alternate_NR ( float32_t*  inputsamples, float32_t*  outputsamples )
{

   // float32_t* Energy=0;

    /*
    if(ts.nb_setting > 0)      ///   NIZ32  masque pour retourner au Noise Blanker ancien
    {
        alt_noise_blanking ( inputsamples, NR_FFT_SIZE, Energy );  // la nouvelle fonction Noise-Blanker
    }
     */

    //    if(( ts.dsp_active & DSP_NR_ENABLE ) || ( ts.dsp_active & DSP_NOTCH_ENABLE ))
    if ( ts.dsp_active  &  DSP_NR_ENABLE )
    {
		// profileTimedEventStart ( ProfileTP8 );

		/*	// spectral_noise_reduction_2(inputsamples);
		if (ts.nr_mode == 0)
		  spectral_noise_reduction(inputsamples);
		else
		  if (ts.nr_mode == 1)
			spectral_noise_reduction_2(inputsamples);
		  else
			  */

		spectral_noise_reduction_3 ( inputsamples );

		// profileTimedEventStop ( ProfileTP8 );
    }

    for (int k=0; k < NR_FFT_SIZE;  k++)
    {
        outputsamples[k] = inputsamples[k];
    }

}


void spectral_noise_reduction_3 ( float* in_buffer )
{
				////////////////////////////////////////////////////////////////////////////////////////
				// Frank DD4WH & Michael DL2FW, November 2017
				// NOISE REDUCTION BASED ON SPECTRAL SUBTRACTION
				// following Romanin et al. 2009 on the basis of Ephraim & Malah 1984 and Hu et al. 2001
				// detailed technical description of the implemented algorithm
				// can be found in our WIKI
				// https://github.com/df8oe/UHSDR/wiki/Noise-reduction
				//
				// half-overlapping input buffers (= overlap 50%)
				// Hann window on 128 or 256 samples
				// FFT128 - inverse FFT128 or FFT256 / iFFT256
				// overlap-add


float32_t NR_sample_rate = 12000.0;
				// we use further decimation to 6ksps, when filter bandwidth is < 2901Hz
if( ts.NR_decimation_enable && (FilterInfo[ FilterPathInfo[ ts.filter_path ].id].width < 2901))
{
	NR_sample_rate = 6000.0;
}

static  uint8_t   NR_init_counter =  0;
        uint8_t   VAD_low         =  0;
        uint8_t   VAD_high        =  63;

float32_t  width   = FilterInfo     [ FilterPathInfo [ ts.filter_path ].id ].width;
float32_t  offset  = FilterPathInfo [ ts.filter_path ].offset;
float32_t  lf_freq = ( offset - width/2 ) / ( NR_sample_rate / ts.NR_FFT_L );  // bin BW is 93.75Hz [12000Hz / 128 bins]
float32_t  uf_freq = ( offset + width/2 ) / ( NR_sample_rate / ts.NR_FFT_L );  // ( offset + width/2 ) / ( NR_sample_rate / ts.NR_FFT_L );

//const float32_t tinc = 0.00533333; // frame time 5.3333ms
//const float32_t tax  = 0.071;	     // noise output smoothing time constant - absolut value in seconds
//const float32_t tap  = 0.152;	     // speech prob smoothing time constant  - absolut value in seconds


//const float32_t asnr   = 15; 	     // active SNR in dB

#define       psthr        0.99		 // threshold for smoothed speech probability [0.99]
#define       pnsaf        0.01		 // noise probability safety value            [0.01]
#define       psini        0.50      // initial speech probability                [0.50]
#define       pspri        0.50      // prior speech probability                  [0.50]
#define       NR2_ax       0.7405    // for 6ksps and FFT256
#define       NR2_ap       0.8691

// for 12ksps and FFT128
// NR2_ax = 0.9276; 		//expf(-tinc / tax);
// NR2_ap = 0.9655; 		//expf(-tinc / tap);


//NR2.xih1          = 31.62; 	  // powf(10, (float32_t)NR2.asnr / 10.0);
NR2.xih1            = powf( 10, (float32_t)NR2.asnr / 10.0 );
NR2.xih1r           =  1.0 / ( 1.0 + NR2.xih1 ) - 1.0;
NR2.pfac            = (1.0 / pspri - 1.0) * ( 1.0 + NR2.xih1 );
#define  NR2_snr_prio_min     0.001      //	//powf(10, - (float32_t)NR2.snr_prio_min_int / 10.0);  //range should be down to -30dB min
NR2.power_threshold = (float32_t) ( NR2.power_threshold_int )/ 100.0;  // A rendre "NR2.power_threshold_int" en fonction de la force du signal
static  float32_t  pslp [ NR_FFT_SIZE ];
static  float32_t  xt   [ NR_FFT_SIZE ];
        float32_t  xtr;
        float32_t  ph1y [ NR_FFT_SIZE ];



//ax and ap adjustment according to FFT-size and decimation
// for 12KS and 128'er FFT we are doing well with the above values
// This table shows the tinc values in ms for the calculation of the coefficients ax and ap
//
// 	   FFT Size:  |	128		256
//    		______|________________________
// Samplerate:  12KS  | 5.333		10.666
//		      |
//		 6KS  |10.666		21.333


/*if (ts.nr_fft_256_enable && ts.NR_decimation_enable)
  {
    NR2.ax = 0.7405;
    NR2.ap = 0.8691;
  }
else
  if (ts.nr_fft_256_enable || ts.NR_decimation_enable)
    {
      NR2.ax = 0.8605;
      NR2.ap = 0.9322;
    } */


			//********************************************************************************************************
			//*************************   Initialisation des variables du Noise Reduction NR *************************
			//********************************************************************************************************

	int NR_FFT_LDD = ts.NR_FFT_L >> 1;

    if( ts.nr_first_time == 1 )      // TODO: properly initialize all the variables
    {
		for( int bindx = 0; bindx < NR_FFT_LDD ; bindx++ )
			{
				  NR.last_sample_buffer_L[bindx]    = 0.0;
				  NR.Hk                  [bindx]    = 1.0;
			//    xu                     [bindx]    = 1.0;  // has to be replaced by other variable
				  NR.Hk_old              [bindx]    = 1.0;  // old gain or xu in development mode
				  NR.Nest                [bindx][0] = 0.0;
				  NR.Nest                [bindx][1] = 1.0;
				  pslp                   [bindx]    = 0.5;
		    //    NR2.long_tone_gain     [bindx]    = 1.0;
			}
        ts.nr_first_time = 2; // we need to do some more a bit later down
    }





    // NR_FFT_buffer is 256 floats big
    // interleaved r, i, r, i . . .
    // fill first half of FFT_buffer with last events audio samples

					//********************************************************************************************
					//*****************  Placement des nouveaux  128 points temporelles avec ceux décalés ********
					//********************************************************************************************

	  for ( int i = 0; i < NR_FFT_LDD; i++ )
	  {
		NR.FFT_buffer [(i << 1)    ] = NR.last_sample_buffer_L[i]; // real
		NR.FFT_buffer [(i << 1) + 1] = 0.0; // imaginary
	  }

	  for ( int i = 0; i < NR_FFT_LDD ; i++ )  // copy recent samples to last_sample_buffer for next time!
	  {
		 NR.last_sample_buffer_L [i] = in_buffer[ i  ];
	  }

	  for (  int i = 0; i < NR_FFT_LDD; i++ )  // now fill recent audio samples into second half of FFT_buffer
	  {
		  NR.FFT_buffer[ ts.NR_FFT_L + (i << 1 )    ] = in_buffer[i]; // real
		  NR.FFT_buffer[ ts.NR_FFT_L + (i << 1 ) + 1] = 0.0;
	  }


				   //***************************************************************************************
				   //*********  Fenetrage SQRT-VON-Hanning et FFT direct 256 points ************************
				   //***************************************************************************************
	  for ( int idx = 0; idx < ts.NR_FFT_L; idx++ )
	  {
		  NR.FFT_buffer[idx << 1] *= SQRT_von_Hann_256[idx];         // FFT_buffer [re, im, re, im, re, im . . .]
	  }
	  arm_cfft_f32 ( &arm_cfft_sR_f32_len256, NR.FFT_buffer, 0, 1 ); // FFT_256   function


                     //**************************************************************************************
                     //*********** Calcul Amplitude au carré ************************************************
                     //**************************************************************************************
	 for ( int bindx = 0; bindx < NR_FFT_LDD; bindx++ )           //here we need squared magnitude
	 {
		int   bindxx = bindx << 1;
		NR2.X[bindx][0] = (NR.FFT_buffer[bindxx] * NR.FFT_buffer[bindxx] + NR.FFT_buffer[bindxx + 1] * NR.FFT_buffer[bindxx + 1]);
	 }


	  	  	  	  	//***************************************************************************************
	                //***********   Moyennage du registre "Nest"  et Initialisation du registre "xt"   ******                                                   ****************
	                //***************************************************************************************
	 if( ts.nr_first_time == 2 )
	 {
		 for( int bindx = 0; bindx < NR_FFT_LDD; bindx++ )
		 {
			 NR.Nest[bindx][0] += 0.05 * NR2.X[bindx][0];  // we do it 20 times to average over 20 frames for app. 100ms only on NR_on/bandswitch/modeswitch,...
		 }

		 NR_init_counter++;
		 if ( NR_init_counter > 19 )//average over 20 frames for app. 100ms
		 {
			 for( int bindx = 0; bindx < NR_FFT_LDD; bindx++ ) { xt[bindx] = psini * NR.Nest[bindx][0]; }
			 NR_init_counter  = 0;
			 ts.nr_first_time = 3;  //all necessary initialization was done , we can start the Noise Reduction function
		 }
	 }

                  //*****************************************************************************************
                  //***********************   Traitement permanent du Noise Reduction ***********************
                  //*****************************************************************************************

     if (ts.nr_first_time == 3)
     {
    	        //*************************************************************************
    	 	 	/////////******** New Noise Estimation  MMSE based !!!  ******/////////////
    	 	 	/// ***********************************************************************

		for ( int bindx = 0; bindx <NR_FFT_LDD; bindx++ )// 1. Step of NR - calculate the SNR's
    	{
		      ph1y[bindx] = 1.0 / (1.0 + NR2.pfac * expf(NR2.xih1r * NR2.X[bindx][0]/xt[bindx]));
		      pslp[bindx] = NR2_ap * pslp[bindx] + (1.0 - NR2_ap)  * ph1y[bindx];


		      if ( pslp[bindx] > psthr ) { ph1y[bindx] = 1.0 - pnsaf;              }
		      else		                 { ph1y[bindx] = fmin( ph1y[bindx] , 1.0); }

		      xtr       = (1.0 - ph1y[bindx]) * NR2.X[bindx][0] + ph1y[bindx] * xt[bindx];
		      xt[bindx] = NR2_ax * xt[bindx] + (1.0 - NR2_ax) * xtr;
        }


	    for( int bindx = 0; bindx < NR_FFT_LDD; bindx++)// 1. Step of NR - calculate the SNR's
		{
		   NR.SNR_post[bindx] = fmax(fmin(NR2.X[bindx][0] / xt[bindx],1000.0), NR2_snr_prio_min); // limited to +30 /-15 dB, might be still too much of reduction, let's try it?

		   NR.SNR_prio[bindx] = fmax(ts.nr_alpha * NR.Hk_old[bindx] + (1.0 - ts.nr_alpha) * fmax(NR.SNR_post[bindx] - 1.0, 0.0), 0.0);
		}

		VAD_low  = (int) lf_freq;

		VAD_high = (int) uf_freq;

		if ( VAD_low == VAD_high)    { VAD_high++;  }
		if ( VAD_low  < 1       )    { VAD_low = 1; }
		else
		if       ( VAD_low  > NR_FFT_LDD - 2)  { VAD_low  = NR_FFT_LDD - 2; }
		if       ( VAD_high < 1             )  { VAD_high = 1;              }
		else  if ( VAD_high > NR_FFT_LDD    )  { VAD_high = NR_FFT_LDD;     }


  	            // 4    calculate v = SNRprio(n, bin[i]) / (SNRprio(n, bin[i]) + 1) * SNRpost(n, bin[i]) (eq. 12 of Schmitt et al. 2002, eq. 9 of Romanin et al. 2009)
				//*************************************************************************************
				//************  Calculer les coeffs  du Filtre FFT , les  HK's ************************
				//*************************************************************************************

		for( int bindx = VAD_low; bindx < VAD_high; bindx++ )// maybe we should limit this to the signal containing bins (filtering!!)
		{
			  float32_t  v     = NR.SNR_prio[bindx] * NR.SNR_post[bindx] / (1.0 + NR.SNR_prio[bindx]);

			  NR.Hk    [bindx] = fmax(1.0 / NR.SNR_post[bindx] * sqrtf((0.7212 * v + v * v)),0.001); //limit HK's to 0.001'

			  NR.Hk_old[bindx] = NR.SNR_post[bindx] * NR.Hk[bindx] * NR.Hk[bindx]; //

		}
				//****************************************************************************************
				//*** Musical Noise "artefact" reduction by dynamic averaging - depending on SNR ratio ***
				//****************************************************************************************

		NR2.pre_power    =  0.0;
		NR2.post_power   =  0.0;
		for ( int bindx  =  VAD_low; bindx < VAD_high; bindx++ )
		{
		    NR2.pre_power  += NR2.X[bindx][0];
		    NR2.post_power += NR.Hk[bindx] * NR.Hk[bindx]  * NR2.X[bindx][0];
		}

		NR2.power_ratio     = NR2.post_power / NR2.pre_power;
		if (NR2.power_ratio > NR2.power_threshold)
		{
			NR2.power_ratio =  1.0;
			NR2.NN          =  1;
		}
		else
		{
			NR2.NN = 1 + 2 * (int)( 0.5 + NR2.width * (1.0 - NR2.power_ratio / NR2.power_threshold));
		}

		int16_t  NRNN = (NR2.NN >> 1);

		for ( int bindx = VAD_low + NRNN ;  bindx < VAD_high - NRNN ;  bindx++ )
		{
		    NR.Nest[bindx][0] = 0.0;
		    for (  int m = bindx - NRNN ; m <= bindx + NRNN ; m++)  {  NR.Nest[bindx][0] += NR.Hk[m];  }
		    NR.Nest[bindx][0] /= (float32_t)NR2.NN;
		}

				//********************************************************************************
				//********************  Edges  Treatment   **************************************
				//********************************************************************************
				// and now the edges - only going NN steps forward and taking the average

		for( int bindx = VAD_low; bindx < VAD_low + (NR2.NN >> 1); bindx++ )
		  {
		    NR.Nest[bindx][0] = 0.0;
		    for ( int m = bindx;  m < (bindx + NR2.NN); m++ )    {  NR.Nest[bindx][0] += NR.Hk[m];  }
		    NR.Nest[bindx][0]  /= (float32_t)NR2.NN;
		 }

		// upper edge - only going NN steps backward and taking the average
		for ( int bindx = VAD_high - NR2.NN; bindx < VAD_high; bindx++ )
		 {
		    NR.Nest[bindx][0] = 0.0;
		    for( int m = bindx; m > (bindx - NR2.NN); m-- )     {  NR.Nest[bindx][0] += NR.Hk[m];   }
		    NR.Nest[bindx][0] /= (float32_t) NR2.NN;
		 }

				//********************************************************************************
				//******************** end of edge treatment   ***********************************
				//********************************************************************************

		for( int bindx = VAD_low + (NR2.NN >> 1); bindx < VAD_high - (NR2.NN >> 1); bindx++ )
		  {
		    NR.Hk[bindx] = NR.Nest[bindx][0];
		  }
				//******************************************************************************
				//******** End of musical noise reduction  *************************************
				//******************************************************************************
	}	//end of "if ts.nr_first_time == 3"


		  // FINAL SPECTRAL WEIGHTING : Multiply current FFT results with NR_FFT_buffer for 64 bins
		  // with the 64 bin-specific gain factors
		  // only do this for the bins inside the filter passband
		  // if you do this for all the bins, you will get distorted audio: plopping !
		  //     for ( int bindx = 0; bindx < ts.NR_FFT_L / 2; bindx++ ) // plopping !!!!
		  //*******************************************************************************************
		  //*******************  Application du Filtrage FFT par Pondération adaptative ***************
		  //*******************************************************************************************

    int NR_FFT_LII = ( ts.NR_FFT_L << 1 ) - 2;

    for ( int bindx = VAD_low;    bindx < VAD_high;  bindx++ ) // no plopping
    {
	  int                   bindxx    =  bindx   << 1;
	  int                   NR_FFT_LL =  NR_FFT_LII - bindxx;
	  float32_t             HKK = NR.Hk[ bindx ];   /// const
   // float32_t             HKD = HKK * 0.5;

	  NR.FFT_buffer [ bindxx       ] *= HKK; // real part
	  NR.FFT_buffer [ bindxx    + 1] *= HKK; // imag part
	  NR.FFT_buffer [ NR_FFT_LL    ] *= HKK; // real part conjugate symmetric
	  NR.FFT_buffer [ NR_FFT_LL + 1] *= HKK; // imag part conjugate symmetric

	  /*
	  if ( NR.FFT_buffer [bindxx     ] < N_seuil ) { NR.FFT_buffer[bindxx     ] *= HKD; }   // real part
	  else                                         { NR.FFT_buffer[bindxx     ] *= HKK; }

	  if ( NR.FFT_buffer [bindxx + 1 ] < N_seuil ) { NR.FFT_buffer[bindxx + 1 ] *= HKD; }   // imag part
	  else                                         { NR.FFT_buffer[bindxx + 1 ] *= HKK; }

	  if ( NR.FFT_buffer [NR_FFT_LL  ] < N_seuil ) { NR.FFT_buffer[NR_FFT_LL  ] *= HKD; } // real part conjugate symmetric
	  else                                         { NR.FFT_buffer[NR_FFT_LL  ] *= HKK; }

	  if ( NR.FFT_buffer [NR_FFT_LL+1] < N_seuil ) { NR.FFT_buffer[NR_FFT_LL+1] *= HKD; } // imag part conjugate symmetric
	  else                                         { NR.FFT_buffer[NR_FFT_LL+1] *= HKK; }
	  */

    }

							 /*****************************************************************
							 * NOISE REDUCTION ESTIMATION ENDS HERE
							 *****************************************************************/
               //****************************************************************************************
               //*************   FFT-inverse et Fenetrage SQRT-Von-Hanning   256 points *****************
               //****************************************************************************************

    arm_cfft_f32 ( &arm_cfft_sR_f32_len256, NR.FFT_buffer, 1, 1 );  /// FFT-INVERSE

    for ( int idx = 0; idx <  ts.NR_FFT_L ; idx++ )
    {
    	NR.FFT_buffer[ idx  << 1 ] *= SQRT_von_Hann_256[idx];
    }


				//*********************************************************************************************
				//*****************************  Reassemblage des 128 points temporelles **********************
				//*********************************************************************************************
				// do the overlap & add

	for ( int i = 0;  i < NR_FFT_LDD;  i++ )
	{ // take real part of first half of current iFFT result and add to 2nd half of last iFFT_result
	   in_buffer[ i  ] = NR.FFT_buffer[ i << 1 ] + NR.last_iFFT_result[i];
	}


	for ( int i = 0; i < NR_FFT_LDD; i++ )
	{
		NR.last_iFFT_result[i] = NR.FFT_buffer[ ts.NR_FFT_L + (i << 1) ];
	}







       /*   //   if (ts.show_debug_info)    else
          {

			  for ( int idx = 0; idx < NR_FFT_LDD; idx++ )
			  {
				  // a essayer d'omettre cette fonction pour voir effet sur la qualité audio (effet de +3 db)
				  // l'omission réduit aussi la charge du processeur
				  in_buffer[ idx  ] =  NR.FFT_buffer[ (idx + 64 ) << 1 ] * INV_SQRT_von_Hann_256[idx];
			  }
          }*/



      /*else
      {
          arm_cfft_f32(&arm_cfft_sR_f32_len128, NR.FFT_buffer, 1, 1);
          for (int idx = 0; idx < ts.NR_FFT_L; idx++)
          {
        	  NR.FFT_buffer[idx * 2] *= SQRT_van_hann[idx];
          }
      }*/

}

//*****************************************************************************************************
//*****************************************************************************************************

//alt noise blanking is trying to localize some impulse noise within the samples and after that
//trying to replace corrupted samples by linear predicted samples.
//therefore, first we calculate the lpc coefficients which represent the actual status of the
//speech or sound generating "instrument" (in case of speech this is an estimation of the current
//filter-function of the voice generating tract behind our lips :-) )
//after finding this function we inverse filter the actual samples by this function
//so we are eliminating the speech, but not the noise. Then we do a matched filtering an thereby detecting impulses
//After that we threshold the remaining samples by some
//level and so detecting impulse noise's positions within the current frame - if one (or more) impulses are there.
//finally some area around the impulse position will be replaced by predicted samples from both sides (forward and
//backward prediction)
//hopefully we have enough processor power left....

/*     // masqué car on est retourné a l'ancienne fonction de  Noise blanker wide bandwidth developpé par Nizar

void alt_noise_blanking ( float* insamp,  int  Nsam,  int  order,  float*  E )  //Nsam = 128
{

#ifdef debug_alternate_NR

const  float32_t   NR_test_samp[128] =
       { 853.472351, 629.066223, 864.270813, 1012.3078 , 738.378113, 446.268219, 635.763123, 1062.71118,
         955.245667, 22.6679211,-1130.45386,-1737.12817,-1728.79114,-1594.82227,-1545.75671,-1208.91003,
        -252.898315, 993.880493, 1820.26538, 1915.65186, 1597.90259, 1248.58838, 809.456909, 28.6509247,
        -961.62677 ,-1604.66443,-1499.18225,-824.882935,-85.1342163, 432.899261, 782.52063 , 1029.38452,
         1040.57166, 692.128662, 138.820541,-286.785767,-420.356415,-384.165161,-348.958527,-308.304718,
        -171.111633, 4.52698851,-5.53196001,-368.999939,-1031.19165,-1766.01074,-2290.01587,-2293.98853,
        -1514.0238 , 23.0157223, 1797.16394, 3018.3894 , 3231.77148, 2702.38745, 2085.92676, 1685.99255,
         1145.43176,-31.9259377,-1722.42847,-3112.2937 ,-3453.61426,-2790.31763,-1812.12769,-1028.70874,
        -1812      , 897.985779, 2375.50903, 3409.33472, 3332.44238, 2293.16602, 1067.26196, 183.806381,
        -548.479553,-1549.47034,-2692.18213,-3288.44702,-2873.70239,-1761.34033,-636.71936 , 250.664383,
         1198.7804 , 2336.43726, 3121.80615, 2848.64355, 1556.67969, 110.084801,-724.328186,-1013.82141,
        -1265.38879,-1506.06091,-1177.04529,-35.6577721, 1209.823  , 1520.28088, 679.406555,-514.541626,
        -1245.55945,-1508.29407,-1707.93408,-1736.12427,-965.137085, 752.618347, 2518.7168 , 3185.57031,
         2563.83838, 1472.3927 , 613.243835,-172.269989,-1311.97058,-2534.06421,-2982.73169,-2282.05859,
        -1025.64673, 12.714426 , 809.696228, 1828.12854, 2977.01709, 3388.77612, 2460.82178, 751.800781,
        -567.183105,-1026.46143,-1190.80762,-1635.05701,-2060.84619,-1785.74683,-841.740173,-62.468441   };



const float32_t NR_test_sinus_samp[128] = {
        0, 765.3668647302, 1414.2135623731, 1847.7590650226, 2000, 1847.7590650226, 1414.2135623731, 765.3668647302,
        0, -765.3668647302, -1414.2135623731, -1847.7590650226, -2000, -1847.7590650226, -1414.2135623731, -765.3668647302,

        0, 765.3668647302, 1414.2135623731, 1847.7590650226, 2000, 1847.7590650226, 1414.2135623731, 765.3668647302,
        0, -765.3668647302, -1414.2135623731, -1847.7590650226, -2000, -1847.7590650226, -1414.2135623731, -765.3668647302,

        0, 765.3668647302, 1414.2135623731, 1847.7590650226, 2000, 1847.7590650226, 1414.2135623731, 765.3668647302,
        0, -765.3668647302, -1414.2135623731, -1847.7590650226, -2000, -1847.7590650226, -1414.2135623731, -765.3668647302,

        0, 765.3668647302, 1414.2135623731, 1847.7590650226, 2000, 1847.7590650226, 1414.2135623731, 765.3668647302,
        0, -765.3668647302, -1414.2135623731, -1847.7590650226, -2000, -1847.7590650226, -1414.2135623731, -765.3668647302,

        0, 765.3668647302, 1414.2135623731, 1847.7590650226, 2000, 1847.7590650226, 1414.2135623731, 765.3668647302,
        0, -765.3668647302, -1414.2135623731, -1847.7590650226, -2000, -1847.7590650226, -1414.2135623731, -765.3668647302,

        0, 765.3668647302, 1414.2135623731, 1847.7590650226, 2000, 1847.7590650226, 1414.2135623731, 765.3668647302,
        0, -765.3668647302, -1414.2135623731, -1847.7590650226, -2000, -1847.7590650226, -1414.2135623731, -765.3668647302,

        0, 765.3668647302, 1414.2135623731, 1847.7590650226, 2000, 1847.7590650226, 1414.2135623731, 765.3668647302,
        0, -765.3668647302, -1414.2135623731, -1847.7590650226, -2000, -1847.7590650226, -1414.2135623731, -765.3668647302,

        0, 765.3668647302, 1414.2135623731, 1847.7590650226, 2000, 1847.7590650226, 1414.2135623731, 765.3668647302,
        0, -765.3668647302, -1414.2135623731, -1847.7590650226, -2000, -1847.7590650226, -1414.2135623731, -765.3668647302
};


#endif

#define boundary_blank 14 // for first trials very large!!!!
#define impulse_length 7 // has to be odd!!!! 7 / 3 should be enough
#define PL             3 // has to be (impulse_length-1)/2 !!!!
#define order         10 // lpc's order
    arm_fir_instance_f32 LPC;
    float32_t lpcs        [ order + 1 ]; // we reserve one more than "order" because of a leading "1"
    float32_t reverse_lpcs[ order + 1 ]; //this takes the reversed order lpc coefficients
    float32_t firStateF32 [ NR_FFT_SIZE + order ];
    float32_t tempsamp[ NR_FFT_SIZE ];
    float32_t sigma2; //taking the variance of the inpo
    float32_t lpc_power;
    float32_t impulse_threshold;
    int impulse_positions[5];  //we allow a maximum of 5 impulses per frame
    int search_pos    = 0;
    int impulse_count = 0;
    //static float32_t last_frame_end[order+PL]; //this takes the last samples from the previous frame to do the prediction within the boundaries
#ifdef debug_alternate_NR
    static int frame_count=0;  //only used for the distortion insertion - can alter be deleted
    int dist_level     =0;//only used for the distortion insertion - can alter be deleted
    int nr_setting = 0;
#endif

    float32_t R[11];  // takes the autocorrelation results
    float32_t k,alfa;

    float32_t any[order+1];  //some internal buffers for the levinson durben algorithm

    float32_t Rfw[impulse_length+order]; // takes the forward predicted audio restauration
    float32_t Rbw[impulse_length+order]; // takes the backward predicted audio restauration
    float32_t Wfw[impulse_length],Wbw[impulse_length]; // taking linear windows for the combination of fwd and bwd

    float32_t s;

    static float32_t working_buffer[NR_FFT_SIZE + 2 * order + 2 * PL]; //we need 128 + 26 floats to work on -
								      //necessary to watch for impulses as close to the frame boundaries as possible

#ifdef debug_alternate_NR  // generate test frames to test the noise blanker function
    // using the NR-setting (0..55) to select the test frame
    // 00 = noise blanker active on orig. audio; threshold factor=3
    // 01 = frame of vocal "a" undistorted
    // 02 .. 05 = frame of vocal "a" with different impulse distortion levels
    // 06 .. 09 = frame of vocal "a" with different impulse distortion levels
    //            noise blanker operating!!
    // ************
    // 01..09 are now using the original received audio and applying a rythmic "click" distortion
    // 06..09 is detecting and removing the click by restoring the predicted audio!!!
    // ************
    // 5 / 9 is the biggest "click" and it is slightly noticeable in the restored audio (9)
    // 10 = noise blanker active on orig. audio threshold factor=3
    // 11  = sinusoidal signal undistorted
    // 12 ..15 = sinusoidal signal with different impulse distortion levels
    // 16 ..19 = sinusoidal signal with different impulse distortion levels
    //            noise blanker operating!!
    // 20 ..50   noise blanker active on orig. audio; threshold factor varying between 3 and 0.26

    nr_setting = (int)ts.dsp_nr_strength;
    // *********************************from here just debug impulse / signal generation
    if ((nr_setting > 0) && (nr_setting < 10)) // we use the vocal "a" frame
    {
        //for (int i=0; i<128;i++)          // not using vocal "a" but the original signal
        //    insamp[i]=NR_test_samp[i];

        if ((frame_count > 19) && (nr_setting > 1))    // insert a distorting pulse
        {
            dist_level=nr_setting;
            if (dist_level > 5) dist_level=dist_level-4; // distortion level is 1...5
            insamp[4]=insamp[4] + dist_level*3000; // overlaying a short  distortion pulse +/-
            insamp[5]=insamp[5] - dist_level*1000;
        }

    }

    if ((nr_setting > 10) && (nr_setting < 20)) // we use the sinus frame
    {
        for (int i=0; i<128;i++)
            insamp[i]=NR_test_sinus_samp[i];

        if ((frame_count > 19) && (nr_setting > 11))    // insert a distorting pulse
        {
            dist_level=nr_setting-10;
            if (dist_level > 5) dist_level=dist_level-4;
            insamp[24]=insamp[24] + dist_level*1000; // overlaying a short  distortion pulse +/-
            insamp[25]=insamp[25] + dist_level*500;
            insamp[26]=insamp[26] - dist_level*200; // overlaying a short  distortion pulse +/-
            insamp[27]=insamp[27] - dist_level*100;


        }



    }


    frame_count++;
    if (frame_count > 20) frame_count=0;

#endif

    // *****************************end of debug impulse generation

    memcpy(&working_buffer[2*PL + 2*order],insamp,NR_FFT_SIZE * sizeof(float32_t));// copy incomming samples to the end of our working bufer


    	//  start of test timing zone

    for (int i=0; i<impulse_length; i++)  // generating 2 Windows for the combination of the 2 predictors
    {                                     // will be a constant window later!
        Wbw[i] = 1.0*i/(impulse_length-1);
        Wfw[impulse_length-i-1]=Wbw[i];
    }

    			// calculate the autocorrelation of insamp (moving by max. of #order# samples)
    for(int i=0; i < (order+1); i++)
    {
		//    arm_dot_prod_f32(&insamp[0],&insamp[i],Nsam-i,&R[i]); // R is carrying the crosscorrelations
		arm_dot_prod_f32(&working_buffer[order+PL+0],&working_buffer[order+PL+i],Nsam-i,&R[i]); // R is carrying the crosscorrelations
    }
    	// end of autocorrelation



    //alternative levinson durben algorithm to calculate the lpc coefficients from the crosscorrelation

    R[0] = R[0] * (1.0 + 1.0e-9);

    lpcs[0] = 1;   //set lpc 0 to 1

    for (int i=1; i < order+1; i++)    lpcs[i]=0;   // fill rest of array with zeros - could be done by memfill

    alfa = R[0];

    for ( int m = 1;  m <= order;  m++ )
    {
        s = 0.0;
        for (int u = 1; u < m; u++)   s = s + lpcs[u] * R[m-u];

        k = -(R[m] + s) / alfa;

        for ( int v = 1;  v < m;  v++)    any[v] = lpcs[v] + k * lpcs[m-v];

        for ( int w = 1;  w < m;  w++)    lpcs[w] = any[w];

        lpcs[m] = k;
        alfa    = alfa * (1 - k * k);
    }

    // end of levinson durben algorithm

    for (int o = 0; o < order+1; o++ )             //store the reverse order coefficients separately
        reverse_lpcs[order-o]=lpcs[o];        // for the matched impulse filter

    arm_fir_init_f32(&LPC,order+1,&reverse_lpcs[0],&firStateF32[0],NR_FFT_SIZE);                                         // we are using the same function as used in freedv

    //arm_fir_f32(&LPC,insamp,tempsamp,Nsam); //do the inverse filtering to eliminate voice and enhance the impulses
    arm_fir_f32(&LPC,&working_buffer[order+PL],tempsamp,Nsam); //do the inverse filtering to eliminate voice and enhance the impulses

    arm_fir_init_f32(&LPC,order+1,&lpcs[0],&firStateF32[0],NR_FFT_SIZE);                                         // we are using the same function as used in freedv

    arm_fir_f32(&LPC,tempsamp,tempsamp,Nsam); // do a matched filtering to detect an impulse in our now voiceless signal


    arm_var_f32(tempsamp,NR_FFT_SIZE,&sigma2); //calculate sigma2 of the original signal ? or tempsignal

    arm_power_f32(lpcs,order,&lpc_power);  // calculate the sum of the squares (the "power") of the lpc's

//  impulse_threshold = (float32_t)ts.nb_setting * 0.5 * sqrtf(sigma2 * lpc_power);  //set a detection level (3 is not really a final setting)
    impulse_threshold = (float32_t)(16 - ts.nb_setting) * 0.5 * sqrtf(sigma2 * lpc_power);  //set a detection level (3 is not really a final setting)

    //if ((nr_setting > 20) && (nr_setting <51))
    //    impulse_threshold = impulse_threshold / (0.9 + (nr_setting-20.0)/10);  //scaling the threshold by 1 ... 0.26

    search_pos = order+PL;  // lower boundary problem has been solved! - so here we start from 1 or 0?
    //search_pos = 1;
    impulse_count=0;

    do {        //going through the filtered samples to find an impulse larger than the threshold

        if ((tempsamp[search_pos] > impulse_threshold)||(tempsamp[search_pos] < (-impulse_threshold)))
        {
            impulse_positions[impulse_count]=search_pos - order;  // save the impulse positions and correct it by the filter delay
            impulse_count++;
            search_pos+=PL;   //  set search_pos a bit away, cause we are already repairing this area later
            //  and the next impulse should not be that close
        }

        search_pos++;

    //} while ((search_pos < NR_FFT_SIZE-boundary_blank) && (impulse_count < 5));// avoid upper boundary
    } while ((search_pos < NR_FFT_SIZE) && (impulse_count < 5));


    // from here: reconstruction of the impulse-distorted audio part:

    // first we form the forward and backward prediction transfer functions from the lpcs
    // that is easy, as they are just the negated coefficients  without the leading "1"
    // we can do this in place of the lpcs, as they are not used here anymore and being recalculated in the next frame!

    arm_negate_f32(&lpcs[1],&lpcs[1],order);
    arm_negate_f32(&reverse_lpcs[0],&reverse_lpcs[0],order);


    for (int j=0; j<impulse_count; j++)
    {
        for (int k = 0; k<order; k++)   // we have to copy some samples from the original signal as
        {                           // basis for the reconstructions - could be done by memcopy

            //if ((impulse_positions[j]-PL-order+k) < 0)// this solves the prediction problem at the left boundary
            //{
            //   Rfw[k]=last_frame_end[impulse_positions[j]+k];//take the sample from the last frame
            //}
            //else
            //{
                //Rfw[k]=insamp[impulse_positions[j]-PL-order+k];//take the sample from this frame as we are away from the boundary
                Rfw[k]=working_buffer[impulse_positions[j]+k];//take the sample from this frame as we are away from the boundary
                //}

            //Rbw[impulse_length+k]=insamp[impulse_positions[j]+PL+k+1];
            Rbw[impulse_length+k]=working_buffer[order+PL+impulse_positions[j]+PL+k+1];



        }     //bis hier alles ok

        for (int i = 0; i < impulse_length; i++) //now we calculate the forward and backward predictions
        {
            arm_dot_prod_f32(&reverse_lpcs[0],&Rfw[i],order,&Rfw[i+order]);
            arm_dot_prod_f32(&lpcs[1],&Rbw[impulse_length-i],order,&Rbw[impulse_length-i-1]);

        }

        arm_mult_f32(&Wfw[0],&Rfw[order],&Rfw[order],impulse_length); // do the windowing, or better: weighing
        arm_mult_f32(&Wbw[0],&Rbw[0],&Rbw[0],impulse_length);



#ifdef debug_alternate_NR
        // in debug mode do the restoration only in some cases
        if (((ts.dsp_nr_strength > 0) && (ts.dsp_nr_strength < 6))||((ts.dsp_nr_strength > 10) && (ts.dsp_nr_strength < 16)))
        {
            // just let the distortion pass at setting 1...5 and 11...15
            //    arm_add_f32(&Rfw[order],&Rbw[0],&insamp[impulse_positions[j]-PL],impulse_length);
        }
        else
        {
            //finally add the two weighted predictions and insert them into the original signal - thereby eliminating the distortion
            //arm_add_f32(&Rfw[order],&Rbw[0],&insamp[impulse_positions[j]-PL],impulse_length);
            arm_add_f32(&Rfw[order],&Rbw[0],&working_buffer[order+PL+impulse_positions[j]-PL],impulse_length);
        }
#else
        //finally add the two weighted predictions and insert them into the original signal - thereby eliminating the distortion
        //arm_add_f32(&Rfw[order],&Rbw[0],&insamp[impulse_positions[j]-PL],impulse_length);
        arm_add_f32(&Rfw[order],&Rbw[0],&working_buffer[order+impulse_positions[j]],impulse_length);

#endif
    }

    //for (int p=0; p<(order+PL); p++)
    //{
    //    last_frame_end[p]=insamp[NR_FFT_SIZE-1-order-PL+p];// store 13 samples from the current frame to use at the next frame
    //}
    //end of test timing zone
memcpy(insamp,&working_buffer[order+PL],NR_FFT_SIZE * sizeof(float32_t));// copy the samples of the current frame back to the insamp-buffer for output
memcpy(working_buffer,&working_buffer[NR_FFT_SIZE],(2*order + 2*PL) * sizeof(float32_t)); // copy
}

*/

#endif
