/**
 * @file    ParametricEQ.cpp
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

#include "ParametricEQ.h"

int ParametricEQ::biquad::execute(int input) {

    int block_output = bhat_0*input + v1;
    w1 = bhat_1*input - ahat_1*(block_output/scaling_factor) + v3;
    v1 = c0*(w1/scaling_factor) - s0*(v2/scaling_factor);
    v2 = s0*(w1/scaling_factor) + c0*(v2/scaling_factor);
    w2 = bhat_2*input - ahat_2*(block_output/scaling_factor);
    v3 = c0*(w2/scaling_factor) - s0*(v4/scaling_factor);
    v4 = s0*(w2/scaling_factor) + c0*(v4/scaling_factor);

    return PIN(block_output/scaling_factor, -sample_bounds, sample_bounds);
}

void ParametricEQ::check_GBW() {
    
    // Find upper and lower bounds
    float upper = Gain_amplitude > G0_amplitude ? Gain_amplitude : G0_amplitude;
    float lower = Gain_amplitude < G0_amplitude ? G0_amplitude : Gain_amplitude;
    
    // Convert current GBW_dB into amplitude form
    Bandwidth_gain_amplitude = pow(10, GBW_dB/20);
    
    // Check this against the limits and clip if necessary
    Bandwidth_gain_amplitude = PIN(Bandwidth_gain_amplitude, lower + GBW_MARGIN, upper - GBW_MARGIN);
    
    // Convert the checked value back to decibel form
    GBW_dB = 20*log10(Bandwidth_gain_amplitude);
} 

void ParametricEQ::calculate() {
    
    // Calculate sampling variables
    Scaling_factor = 1 << (30 - Sample_bits);
    Sample_bounds = (1 << (Sample_bits - 1)) - 1;

    // Calculate parameter variables
    Normalised_centre_frequency = (F0_Hz*2*M_PI)/Sample_rate;
    Normalised_bandwidth = (BW_Hz*2*M_PI)/Sample_rate;
    f0_cosine = cos(Normalised_centre_frequency);
    f0_sine = sin(Normalised_centre_frequency);
    g = pow(Gain_amplitude,double(1)/Order);
    g_squared = g*g;
    g0 = pow(G0_amplitude,double(1)/Order);
    g0_squared = g0*g0;
    epsilon = sqrt((Gain_amplitude*Gain_amplitude - Bandwidth_gain_amplitude*Bandwidth_gain_amplitude)/
                   (Bandwidth_gain_amplitude*Bandwidth_gain_amplitude - G0_amplitude*G0_amplitude));
    beta = tan(Normalised_bandwidth/2)/pow(epsilon,double(1)/Order);
    beta_squared = beta*beta;
    counter = (Order + 1)/2;
    
    update_blocks();
}

void ParametricEQ::update_blocks() {

    // Iterate through filter blocks
    for (int i = 0; i < counter; i++)
    {
        // Assign new sampling parameters
        fx_blocks[i].scaling_factor = Scaling_factor;
        fx_blocks[i].sample_bounds = Sample_bounds;

        // Assign new coefficients
        fx_blocks[i].c0 = f0_cosine*Scaling_factor;
        fx_blocks[i].s0 = f0_sine*Scaling_factor;

        if ((Order%2 == 1) && (i == 0))
        {
            D = beta + 1;
            fx_blocks[i].bhat_0 = ((g*beta + g0)/D)*Scaling_factor;
            fx_blocks[i].bhat_1 = ((g*beta - g0)/D)*Scaling_factor;
            fx_blocks[i].bhat_2 = 0;
            fx_blocks[i].ahat_1 = ((beta - 1)/D)*Scaling_factor;
            fx_blocks[i].ahat_2 = 0;
        }

        else
        {
            phi = (((2*i) + 1)*M_PI)/(2*Order);
            si = sin(double(phi));
            D = beta_squared + 2*si*beta + 1;
            fx_blocks[i].bhat_0 = ((g_squared*beta_squared + 2*g*g0*si*beta + g0_squared)/D)*Scaling_factor;
            fx_blocks[i].bhat_1 = (2*(g_squared*beta_squared - g0_squared)/D)*Scaling_factor;
            fx_blocks[i].bhat_2 = ((g_squared*beta_squared - 2*g*g0*si*beta + g0_squared)/D)*Scaling_factor;
            fx_blocks[i].ahat_1 = (2*(beta_squared - 1)/D)*Scaling_factor;
            fx_blocks[i].ahat_2 = ((beta_squared - 2*si*beta + 1)/D)*Scaling_factor;
        }
    }
}

ParametricEQ::ParametricEQ() {

    // Initial parameter values
    Gain_dB = 6;
    GBW_dB = 3;
    F0_Hz = 4000;
    BW_Hz = 1000;
    Order = 2;
    Type = Peaking;
    
    // Initial sampling values
    Sample_rate = 48000;
    Sample_bits = 16;

    // Initialise filter blocks
    calculate();
}

float ParametricEQ::set_Gain_dB(float value) {
    
    Gain_dB = PIN(value, -GAIN_DB_MAX, GAIN_DB_MAX);
    
    set_Type(Type);
    return Gain_dB;
}

float ParametricEQ::set_GBW_dB(float value) {
    
    GBW_dB = PIN(value, -GAIN_DB_MAX, GAIN_DB_MAX);
    
    set_Type(Type);
    return GBW_dB;
}

int ParametricEQ::set_F0_Hz(int value) {
    
    F0_Hz = PIN(value, 0, Sample_rate/2);
    
    set_Type(Type);
    return F0_Hz;    
}

int ParametricEQ::set_BW_Hz(int value) {
        
    BW_Hz = PIN(value, BW_HZ_MIN, Sample_rate/2);
    
    set_Type(Type);
    return BW_Hz; 
}

int ParametricEQ::set_Order(int value) {
    
    Order = PIN(value, 1, MAX_ORDER);
    
    set_Type(Type);
    return Order; 
}

FilterType ParametricEQ::set_Type(FilterType type) {
    
    Type = type;
    
    // Tailor parameters to suit the chosen filter type
    switch (Type) {
        
        case Peaking:
            Gain_amplitude = pow(10, Gain_dB/20);
            G0_amplitude = 1;
            break;
        
        case BandPass:
            Gain_amplitude = 1;
            G0_amplitude = 0;
            break;
            
        case BandStop:
            Gain_amplitude = 0;
            G0_amplitude = 1;
            break;    
        
        case LowShelf:
            Gain_amplitude = pow(10, Gain_dB/20);
            G0_amplitude = 1;
            F0_Hz = 0;
            break;
        
        case HighShelf:
            Gain_amplitude = pow(10, Gain_dB/20);
            G0_amplitude = 1;
            F0_Hz = Sample_rate/2;
            break;
        
        case LowPass:
            Gain_amplitude = 1;
            G0_amplitude = 0;
            F0_Hz = 0;
            break;
            
        case HighPass:
            Gain_amplitude = 1;
            G0_amplitude = 0;
            F0_Hz = Sample_rate/2;
            break;
    }
    
    check_GBW();
    calculate();
    return Type;
}

int ParametricEQ::set_Sample_rate(int value) {
    
    Sample_rate = PIN(value, SAMPLE_RATE_MIN, SAMPLE_RATE_MAX);
    
    set_Type(Type);
    return Sample_rate; 
}

int ParametricEQ::set_Sample_bits(int value) {
    
    Sample_bits = PIN(value, SAMPLE_BITS_MIN, SAMPLE_BITS_MAX);
    
    set_Type(Type);
    return Sample_bits; 
}

int ParametricEQ::filter(int input) {
    
    // Send sample through filter blocks
    for (int i = 0; i < counter; i++)
    {
        input = fx_blocks[i].execute(input);        
    }
    
    return input;
}

