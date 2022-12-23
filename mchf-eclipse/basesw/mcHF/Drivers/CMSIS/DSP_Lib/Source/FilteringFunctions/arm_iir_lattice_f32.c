/* ----------------------------------------------------------------------    
* Copyright (C) 2010-2014 ARM Limited. All rights reserved.    
*    
* $Date:        19. March 2015
* $Revision: 	V.1.4.5
*    
* Project: 	    CMSIS DSP Library    
* Title:	    arm_iir_lattice_f32.c    
*    
* Description:	Floating-point IIR Lattice filter processing function.    
*    
* Target Processor: Cortex-M4/Cortex-M3/Cortex-M0
*  
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions
* are met:
*   - Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   - Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in
*     the documentation and/or other materials provided with the 
*     distribution.
*   - Neither the name of ARM LIMITED nor the names of its contributors
*     may be used to endorse or promote products derived from this
*     software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.    
* -------------------------------------------------------------------- */

#include "arm_math.h"

/**    
 * @ingroup groupFilters    
 */

/**    
 * @defgroup IIR_Lattice Infinite Impulse Response (IIR) Lattice Filters    
 *    
 * This set of functions implements lattice filters    
 * for Q15, Q31 and floating-point data types.  Lattice filters are used in a     
 * variety of adaptive filter applications.  The filter structure has feedforward and    
 * feedback components and the net impulse response is infinite length.    
 * The functions operate on blocks    
 * of input and output data and each call to the function processes    
 * <code>blockSize</code> samples through the filter.  <code>pSrc</code> and    
 * <code>pDst</code> point to input and output arrays containing <code>blockSize</code> values.    
    
 * \par Algorithm:    
 * \image html IIRLattice.gif "Infinite Impulse Response Lattice filter"    
 * <pre>    
 *    fN(n)   =  x(n)    
 *    fm-1(n) = fm(n) - km * gm-1(n-1)   for m = N, N-1, ...1    
 *    gm(n)   = km * fm-1(n) + gm-1(n-1) for m = N, N-1, ...1    
 *    y(n)    = vN * gN(n) + vN-1 * gN-1(n) + ...+ v0 * g0(n)    
 * </pre>    
 * \par    
 * <code>pkCoeffs</code> points to array of reflection coefficients of size <code>numStages</code>.     
 * Reflection coefficients are stored in time-reversed order.    
 * \par    
 * <pre>    
 *    { kN, kN-1, ....k1}
 * </pre>    
 * <code>pvCoeffs</code> points to the array of ladder coefficients of size <code>(numStages+1)</code>.     
 * Ladder coefficients are stored in time-reversed order.    
 * \par    
 * <pre>    
 *    { vN, vN-1, ...v0}
 * </pre>    
 * <code>pState</code> points to a state array of size <code>numStages + blockSize</code>.    
 * The state variables shown in the figure above (the g values) are stored in the <code>pState</code> array.    
 * The state variables are updated after each block of data is processed; the coefficients are untouched.    
 * \par Instance Structure    
 * The coefficients and state variables for a filter are stored together in an instance data structure.    
 * A separate instance structure must be defined for each filter.    
 * Coefficient arrays may be shared among several instances while state variable arrays cannot be shared.    
 * There are separate instance structure declarations for each of the 3 supported data types.    
  *    
 * \par Initialization Functions    
 * There is also an associated initialization function for each data type.    
 * The initialization function performs the following operations:    
 * - Sets the values of the internal structure fields.    
 * - Zeros out the values in the state buffer.   
 * To do this manually without calling the init function, assign the follow subfields of the instance structure:
 * numStages, pkCoeffs, pvCoeffs, pState. Also set all of the values in pState to zero. 
 *    
 * \par    
 * Use of the initialization function is optional.    
 * However, if the initialization function is used, then the instance structure cannot be placed into a const data section.    
 * To place an instance structure into a const data section, the instance structure must be manually initialized.    
 * Set the values in the state buffer to zeros and then manually initialize the instance structure as follows:    
 * <pre>    
 *arm_iir_lattice_instance_f32 S = { numStages, pState, pkCoeffs, pvCoeffs };
 *arm_iir_lattice_instance_q31 S = { numStages, pState, pkCoeffs, pvCoeffs };
 *arm_iir_lattice_instance_q15 S = { numStages, pState, pkCoeffs, pvCoeffs };
 * </pre>    
 * \par    
 * where <code>numStages</code> is the number of stages in the filter; <code>pState</code> points to the state buffer array;    
 * <code>pkCoeffs</code> points to array of the reflection coefficients; <code>pvCoeffs</code> points to the array of ladder coefficients.    
 * \par Fixed-Point Behavior    
 * Care must be taken when using the fixed-point versions of the IIR lattice filter functions.    
 * In particular, the overflow and saturation behavior of the accumulator used in each function must be considered.    
 * Refer to the function specific documentation below for usage guidelines.    
 */

/**    
 * @addtogroup IIR_Lattice    
 * @{    
 */

/**    
 * @brief Processing function for the floating-point IIR lattice filter.    
 * @param[in]  *S    points to an instance of the floating-point IIR lattice structure.
 * @param[in]  *pSrc points to the block of input data.
 * @param[out] *pDst points to the block of output data.    
 * @param[in]  blockSize number of samples to process.
 * @return none.    
 */

#ifndef ARM_MATH_CM0_FAMILY			// Run the below code for Cortex-M4 and Cortex-M3

void arm_iir_lattice_f32 (  const arm_iir_lattice_instance_f32 * S,  float32_t * pSrc,   float32_t * pDst, uint32_t  blockSize )
{
  float32_t  fnext1, gcurr1, gnext;               // Temporary variables for lattice stages
  float32_t  acc;                                 // Accumlator
  uint32_t   blkCnt, tapCnt;                      // temporary variables for counts
  float32_t  *px1, *px2, *pk, *pv;                // temporary pointers for state and coef
  uint32_t   numStages = S->numStages;            // number of stages
  float32_t  *pState;                             // State pointer
  float32_t  *pStateCurnt;                        // State current pointer
  float32_t  k1, k2;
  float32_t  v1, v2, v3, v4;
  float32_t  gcurr2;
  float32_t  fnext2;


  blkCnt  =  blockSize;          // initialise loop count
  pState  =  &S->pState[0];      // initialise state pointer


  while(blkCnt > 0u)			 // Sample processing
  {
    fnext2  =  *pSrc++;			 // Read Sample from input buffer // fN(n) = x(n)
    pv 		=  &S->pvCoeffs[0];  // Initialize Ladder coeff pointer
    pk 		=  &S->pkCoeffs[0];  // Initialize Reflection coeff pointer
    px1     =  pState;			 // Initialize state read pointer
    px2 	=  pState;			 // Initialize state write pointer
    acc 	=  0.0; 			 // Set accumulator to zero
    tapCnt  = (numStages) >> 2;	 // Loop unrolling.  Process 4 taps at a time.

    while(tapCnt > 0u)
    {
      gcurr1 = *px1;	               // Read gN-1(n-1) from state buffer
      k1     = *pk;                    // read reflection coefficient kN
      fnext1 = fnext2 - (k1 * gcurr1); // fN-1(n) = fN(n) - kN * gN-1(n-1)
      v1     = *pv;					   // read ladder coefficient vN
      k2     = *(pk + 1u);			   // read next reflection coefficient kN-1
      gcurr2 = *(px1 + 1u);			   // Read gN-2(n-1) from state buffer
      v2     = *(pv + 1u);			   // read next ladder coefficient vN-1
      fnext2 = fnext1 - (k2 * gcurr2); // fN-2(n) = fN-1(n) - kN-1 * gN-2(n-1)
      gnext  = gcurr1 + (k1 * fnext1); // gN(n)   = kN * fN-1(n) + gN-1(n-1)
      k1 	 = *(pk + 2u);             // read reflection coefficient kN-2
      *px2++ = gnext;				   // write gN(n) into state for next sample processing
      gcurr1 = *(px1 + 2u);			   // Read gN-3(n-1) from state buffer
      acc 	+= (gnext * v1);           // y(n) += gN(n) * vN
      fnext1 = fnext2 - (k1 * gcurr1); // fN-3(n) = fN-2(n) - kN-2 * gN-3(n-1)
      gnext  = gcurr2 + (k2 * fnext2); // gN-1(n)   = kN-1 * fN-2(n) + gN-2(n-1)
      gcurr2 = *(px1 + 3u);			   // Read gN-4(n-1) from state buffer
      acc   += (gnext * v2);		   // y(n) += gN-1(n) * vN-1
      k2     = *(pk + 3u);			   // read reflection coefficient kN-3
      *px2++ = gnext;				   // write gN-1(n) into state for next sample processing
      fnext2 = fnext1 - (k2 * gcurr2); // fN-4(n) = fN-3(n) - kN-3 * gN-4(n-1)
      gnext  = gcurr1 + (k1 * fnext1); // gN-2(n) = kN-2 * fN-3(n) + gN-3(n-1)
      v3     = *(pv + 2u);			   // read ladder coefficient vN-2
      acc 	+= (gnext * v3);		   // y(n) += gN-2(n) * vN-2
      *px2++ = gnext;				   // write gN-2(n) into state for next sample processing
      pk    += 4u;					   // update pointer
      gnext  = (fnext2 * k2) + gcurr2; // gN-3(n) = kN-3 * fN-4(n) + gN-4(n-1)
      v4     = *(pv + 3u);			   // read next ladder coefficient vN-3
      acc   += (gnext * v4);  		   // y(n) += gN-4(n) * vN-4
      *px2++ = gnext;				   // write gN-3(n) into state for next sample processing
       px1  += 4u; 					   // update pointers
       pv   += 4u;
      tapCnt--;
    }

    tapCnt = (numStages) % 0x4u;	  // If the filter length is not a multiple of 4, compute the remaining filter taps

    while ( tapCnt > 0u )
    {
      gcurr1 = *px1++;

      fnext1 = fnext2 - ((*pk)    * gcurr1);	// Process sample for last taps
      gnext  = (fnext1 * (*pk++)) + gcurr1;
      acc   += (gnext  * (*pv++));				// Output samples for last taps
      *px2++ = gnext;
      fnext2 = fnext1;
      tapCnt--;
    }
    acc    += (fnext2 * (*pv));	// y(n) += g0(n) * v0
    *px2++  = fnext2;
    *pDst++ = acc;				// write out into pDst
    pState  = pState + 1u;		// Advance the state pointer by 4 to process the next group of 4 samples
    blkCnt--;
  }
									  // Processing is complete. Now copy last S->numStages samples to start of the buffer
									  // for the preperation of next frame process
  pStateCurnt = &S->pState[0];		  // Points to the start of the state buffer
  pState      = &S->pState[blockSize];
  tapCnt 	  = numStages >> 2u;


  while(tapCnt > 0u)				  // copy data
  {
    *pStateCurnt++ = *pState++;
    *pStateCurnt++ = *pState++;
    *pStateCurnt++ = *pState++;
    *pStateCurnt++ = *pState++;
    tapCnt--;					// Decrement the loop counter
  }
  tapCnt = (numStages) % 0x4u;	// Calculate remaining number of copies
  while( tapCnt > 0u )			// Copy the remaining q31_t data
  {
    *pStateCurnt++ = *pState++;
    tapCnt--;					// Decrement the loop counter
  }
}

#else

void arm_iir_lattice_f32(
  const arm_iir_lattice_instance_f32 * S,
  float32_t * pSrc,
  float32_t * pDst,
  uint32_t    blockSize)
{
  float32_t  fcurr, fnext = 0, gcurr, gnext;     /* Temporary variables for lattice stages */
  float32_t  acc;                                /* Accumlator */
  uint32_t   blkCnt, tapCnt;                     /* temporary variables for counts */
  float32_t *px1, *px2, *pk, *pv;                /* temporary pointers for state and coef */
  uint32_t   numStages = S->numStages;           /* number of stages */
  float32_t *pState;                             /* State pointer */
  float32_t *pStateCurnt;                        /* State current pointer */


  /* Run the below code for Cortex-M0 */

  blkCnt = blockSize;

  pState = &S->pState[0];

  /* Sample processing */
  while(blkCnt > 0u)
  {
    /* Read Sample from input buffer */
    /* fN(n) = x(n) */
    fcurr = *pSrc++;

    /* Initialize state read pointer */
    px1 = pState;
    /* Initialize state write pointer */
    px2 = pState;
    /* Set accumulator to zero */
    acc = 0.0f;
    /* Initialize Ladder coeff pointer */
    pv = &S->pvCoeffs[0];
    /* Initialize Reflection coeff pointer */
    pk = &S->pkCoeffs[0];


    /* Process sample for numStages */
    tapCnt = numStages;

    while( tapCnt > 0u )
    {
      gcurr = *px1++;
      /* Process sample for last taps */
      fnext = fcurr - ((*pk) * gcurr);
      gnext = (fnext * (*pk++)) + gcurr;

      /* Output samples for last taps */
      acc   += (gnext * (*pv++));
      *px2++ = gnext;
      fcurr  = fnext;

      /* Decrementing loop counter */
      tapCnt--;

    }

    /* y(n) += g0(n) * v0 */
    acc    += (fnext * (*pv));

    *px2++  = fnext;

    /* write out into pDst */
    *pDst++ = acc;

    /* Advance the state pointer by 1 to process the next group of samples */
    pState  = pState + 1u;
    blkCnt--;

  }

  /* Processing is complete. Now copy last S->numStages samples to start of the buffer           
     for the preperation of next frame process */

  /* Points to the start of the state buffer */
  pStateCurnt = &S->pState[     0     ];
  pState      = &S->pState[ blockSize ];

  tapCnt = numStages;

  /* Copy the data */
  while(tapCnt > 0u)
  {
    *pStateCurnt++ = *pState++;
     tapCnt--;     /* Decrement the loop counter */
  }

}

#endif /*   #ifndef ARM_MATH_CM0_FAMILY */

/*
void arm_iir_Nizar_f32 (  const arm_iir_lattice_instance_f32 * S,  float32_t * pSrc,   float32_t * pDst, uint32_t  blockSize )
{
  float32_t  fnext1, gcurr1, gnext;               // Temporary variables for lattice stages
  float32_t  acc;                                 // Accumlator
  uint32_t   blkCnt, tapCnt;                      // temporary variables for counts
  float32_t  *px1, *px2, *pk, *pv;                // temporary pointers for state and coef
  uint32_t   numStages = S->numStages;            // number of stages
  float32_t  *pState;                             // State pointer
  float32_t  *pStateCurnt;                        // State current pointer
  float32_t  k1, k2;
  float32_t  v1, v2, v3, v4;
  float32_t  gcurr2;
  float32_t  fnext2;


  blkCnt  =  blockSize;       // initialise loop count
  pState  =  &S->pState[0];   // initialise state pointer


  while( blkCnt > 0u )			// Sample processing
  {
    fnext2  =  *pSrc++;			 // Read Sample from input buffer // fN(n) = x(n)
    pv 		=  &S->pvCoeffs[0];  // Initialize Ladder coeff pointer
    pk 		=  &S->pkCoeffs[0];  // Initialize Reflection coeff pointer
    px1     =  pState;			 // Initialize state read pointer
    px2 	=  pState;			 // Initialize state write pointer
    acc 	=  0.0; 			 // Set accumulator to zero
    tapCnt  = (numStages) >> 2;	 // Loop unrolling.  Process 4 taps at a time.

    while( tapCnt > 0u )
    {
      gcurr1 = *px1;	               // Read gN-1(n-1) from state buffer
      k1     = *pk;                    // read reflection coefficient kN
      fnext1 = fnext2 - (k1 * gcurr1); // fN-1(n) = fN(n) - kN * gN-1(n-1)
      v1     = *pv;					   // read ladder coefficient vN
      k2     = *(pk + 1u);			   // read next reflection coefficient kN-1
      gcurr2 = *(px1 + 1u);			   // Read gN-2(n-1) from state buffer
      v2     = *(pv + 1u);			   // read next ladder coefficient vN-1
      fnext2 = fnext1 - (k2 * gcurr2); // fN-2(n) = fN-1(n) - kN-1 * gN-2(n-1)
      gnext  = gcurr1 + (k1 * fnext1); // gN(n)   = kN * fN-1(n) + gN-1(n-1)
      k1 	 = *(pk + 2u);             // read reflection coefficient kN-2
      *px2++ = gnext;				   // write gN(n) into state for next sample processing
      gcurr1 = *(px1 + 2u);			   // Read gN-3(n-1) from state buffer
      acc 	+= (gnext * v1);           // y(n) += gN(n) * vN
      fnext1 = fnext2 - (k1 * gcurr1); // fN-3(n) = fN-2(n) - kN-2 * gN-3(n-1)
      gnext  = gcurr2 + (k2 * fnext2); // gN-1(n)   = kN-1 * fN-2(n) + gN-2(n-1)
      gcurr2 = *(px1 + 3u);			   // Read gN-4(n-1) from state buffer
      acc   += (gnext * v2);		   // y(n) += gN-1(n) * vN-1
      k2     = *(pk + 3u);			   // read reflection coefficient kN-3
      *px2++ = gnext;				   // write gN-1(n) into state for next sample processing
      fnext2 = fnext1 - (k2 * gcurr2); // fN-4(n) = fN-3(n) - kN-3 * gN-4(n-1)
      gnext  = gcurr1 + (k1 * fnext1); // gN-2(n) = kN-2 * fN-3(n) + gN-3(n-1)
      v3     = *(pv + 2u);			   // read ladder coefficient vN-2
      acc 	+= (gnext * v3);		   // y(n) += gN-2(n) * vN-2
      *px2++ = gnext;				   // write gN-2(n) into state for next sample processing
      pk    += 4u;					   // update pointer
      gnext  = (fnext2 * k2) + gcurr2; // gN-3(n) = kN-3 * fN-4(n) + gN-4(n-1)
      v4     = *(pv + 3u);			   // read next ladder coefficient vN-3
      acc   += (gnext * v4);  		   // y(n) += gN-4(n) * vN-4
      *px2++ = gnext;				   // write gN-3(n) into state for next sample processing
       px1  += 4u; 					   // update pointers
       pv   += 4u;
      tapCnt--;
    }

    tapCnt = (numStages) % 0x4u;	  // If the filter length is not a multiple of 4, compute the remaining filter taps

    while ( tapCnt > 0u )
    {
      gcurr1 = *px1++;

      fnext1 = fnext2 - ((*pk)    * gcurr1);	// Process sample for last taps
      gnext  = (fnext1 * (*pk++)) + gcurr1;
      acc   += (gnext  * (*pv++));				// Output samples for last taps
      *px2++ = gnext;
      fnext2 = fnext1;
      tapCnt--;
    }
    acc    += (fnext2 * (*pv));	// y(n) += g0(n) * v0
    *px2++  = fnext2;
    *pDst++ = acc;				// write out into pDst
    pState  = pState + 1u;		// Advance the state pointer by 4 to process the next group of 4 samples
    blkCnt--;
  }
									  // Processing is complete. Now copy last S->numStages samples to start of the buffer
									  // for the preperation of next frame process
  pStateCurnt = &S->pState[0];		  // Points to the start of the state buffer
  pState      = &S->pState[blockSize];
  tapCnt 	  = numStages >> 2u;


  while(tapCnt > 0u)				  // copy data
  {
    *pStateCurnt++ = *pState++;
    *pStateCurnt++ = *pState++;
    *pStateCurnt++ = *pState++;
    *pStateCurnt++ = *pState++;
    tapCnt--;					// Decrement the loop counter
  }
  tapCnt = (numStages) % 0x4u;	// Calculate remaining number of copies
  while(tapCnt > 0u)			// Copy the remaining q31_t data
  {
    *pStateCurnt++ = *pState++;
    tapCnt--;					// Decrement the loop counter
  }
}  */

/**    
 * @} end of IIR_Lattice group    
 */
