/**
 * @file    ParametricEQ.h
 * @brief   ParametricEQ - fixed point implementation of a Parametric EQ
 * @author  Patrick Thomas
 * @version 1.0
 * @see
 *
 * Copyright (c) 2016
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef PARAMETRICEQ_H
#define PARAMETRICEQ_H

#include <math.h>

#define M_PI 3.14159265358979323846

// Parameter limits
#define GAIN_DB_MAX 20
#define BW_HZ_MIN 20
#define GBW_MARGIN 0.01
#define MAX_ORDER 4

// Sampling limits
#define SAMPLE_RATE_MAX 192000
#define SAMPLE_RATE_MIN 32000
#define SAMPLE_BITS_MIN 8
#define SAMPLE_BITS_MAX 24

#define PIN(n,min,max) ((n) > (max) ? max : ((n) < (min) ? (min) : (n)))

enum FilterType {
    
    Peaking,
    BandPass,
    BandStop,
    LowShelf,
    HighShelf,
    LowPass,
    HighPass    
};

/** A fixed point parametric EQ implementing Butterworth coefficients in direct form 2 transposed realisation. 
 *  The theory of operation is outlined in <a href="http://www.ece.rutgers.edu/~orfanidi/ece346/hpeq.pdf">this paper</a>
 *
 * Example:
 * @code
 *  #include "mbed.h"
 *  #include "ParametricEQ.h"
 *
 *  ParametricEQ myEQ;
 *  Serial pc(USBTX, USBRX);
 *
 *  int main()
 *  {
 *      while(1) {
 *       
 *      while(pc.readable()) {
 *       
 *          pc.printf("%d\n", myEQ.filter(pc.getc()));
 *      }
 *          
 *      }
 *  }
 * @endcode
 */
 
class ParametricEQ {

class biquad {
    
private:

    // Local accumulators
    int v1;
    int v2;
    int v3;
    int v4;
    int w1;
    int w2;

public:

    // Local coefficients
    int c0;
    int s0;
    int bhat_0;
    int bhat_1;
    int bhat_2;
    int ahat_1;
    int ahat_2;

    // Sampling variables
    int scaling_factor;
    int sample_bounds;

    int execute(int input);
};

    // This module's fx blocks
    biquad fx_blocks[(MAX_ORDER + 1)/2];

    // Parameter variables
    float Gain_dB;
    float GBW_dB;
    int F0_Hz;
    int BW_Hz;
    int Order;
    FilterType Type;
    
    // Sampling variables
    int Sample_rate;
    int Sample_bits;

    // Intermediary parameter variables
    double Gain_amplitude, Bandwidth_gain_amplitude, G0_amplitude, Normalised_centre_frequency, Normalised_bandwidth, 
            g, epsilon, beta, g_squared, beta_squared, f0_cosine, f0_sine, D, phi, si, g0, g0_squared;
    int counter;
    
    // Intermediary sampling variables
    int Scaling_factor, Sample_bounds;
    
    void check_GBW();
    void calculate();
    void update_blocks();

public:
    
    /** Create an EQ
     */
    ParametricEQ();
    
    /** Set the Gain 
     *  @param value The target gain (dB)
     *  @return The actual gain (dB)
     */
    float set_Gain_dB(float value);
    
    /** Set the Bandwidth Gain. NB |GBW_dB| must be less than |Gain_dB|
     *  @param value The target bandwidth gain (dB)
     *  @return The actual bandwidth gain (dB)
     */
    float set_GBW_dB(float value);
    
    /** Set the Centre frequency
     *  @param value The target centre frequency (Hz)
     *  @return The actual centre frequency (Hz)
     */
    int set_F0_Hz(int value);
    
    /** Set the Bandwidth 
     *  @param value The target bandwidth (Hz)
     *  @return The actual bandwidth (Hz)
     */
    int set_BW_Hz(int value);
    
    /** Set the Order. Higher order increases cutoff 
     *  @param value The target order
     *  @return The actual order
     */
    int set_Order(int value);
    
    /** Set the Filter type.
     *  @param value The target type: 0 - Peaking, 1 - BandPass, 2 - BandStop, 3 - LowShelf, 4 - HighShelf, 5 - LowPass, 6 - HighPass
     *  @return The actual type
     */
    FilterType set_Type(FilterType type);
    
    /** Set the Sample rate
     *  @param value The target sample rate
     *  @return The actual sample rate
     */
    int set_Sample_rate(int value);
    
    /** Set the number of Sample bits
     *  @param value The target number of sample bits
     *  @return The actual number of sample bits
     */
    int set_Sample_bits(int value);
    
    /** Pass a sample through the EQ
     *  @param input The input sample
     *  @return The filtered output
     */
    int filter(int input);
};

#endif
