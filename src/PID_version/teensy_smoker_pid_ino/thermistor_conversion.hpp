//
// Hysteretic temperature controller sensing a thermistor
//
//                 NTC
//               Thermistor
//  Vref "vref"---/\/\/-------->ADC
//                          |
//                          \
//                          /
//                          \  Rs "rs"
//                          /
//                          |
//                         GND
//
//  Calibration info:
//  Elevation   Boiling Point of Water ^F
//  2500 	      207.2
//  2750 	      206.7
//  3000          206.2
//
//  Target elevation:  2770 ft ==> 206.66 ^F
//  Calibrate using actual measured resistance and table from manufacturer:
//      Cup of ice water, R = 32650 ohms
//      Boiling water at 2770 ft, R = 741.2
//      Offset and scale measured resistance value to match mfr table at 32F and 206.6F
//      The offset/scaled resistor value will be used in temperature calculation.
//

#ifndef THSTR_CONV_H
#define THSTR_CONV_H

#include <math.h>

#define K2C             273.15      //Add to ^C to get ^K

//Fixed point processing defs
#define SHIFT           12          // Define decimal point location for IIR


typedef struct thstr_vars_t
{
    unsigned char nbits; //ADC resolution
    float fres;

    float beta;
    float r0;
    float t0;
    float rinf;

    float vref;
    float rs;
    float offset; //Offset use for centering approximation error above and below measurement range center point.
    float rcal;  //calibration ratio for actual resistance measurement adjustment, for 2-point user cal

    float r_meas;
    float t_meas;

    //Control criteria
    float set_point_degreesF;
    float set_point_max, set_point_min; //represents hysteresis in control loop

    float process_variable_degreesF;    //Used for control
    unsigned int process_variable_cnts; //Digital filtered ADC reading
    unsigned int process_variable_raw;  //ADC reading

    bool throttle;  //variable tells whether heating element is on or off

    // ADC Noise filter
    unsigned int adc_tc;


} thstr_vars;


//
// Conversion and helper functions
//

// Self-explanatory
float kelvin_to_F(float K)
{
    return (32.0 + (K-K2C)*1.8);
}

// Converts ADC value to floating point temperature in degreesF
float count_to_tempF(thstr_vars* th, unsigned int cnt)
{
    float fcount = lrintf(cnt + 1);
    float fvolt = th->vref*fcount/(powf(2.0, th->fres));

    return ( kelvin_to_F( th->beta/log( (th->rs/th->rinf)*(th->vref/fvolt - 1.0) ) ) - th->offset );
}

//Takes temperature in F and calculates the expected voltage
float temp_to_volt(thstr_vars* th, float temp)
{
    //convert degrees F to Kelvin
    float t = (temp - 32.0 + th->offset)/1.8 + K2C;
    return th->rs*th->vref/(th->rinf*exp(th->beta/t) + th->rs);
}

//Converts a voltage to expected ADC value
unsigned int volt_to_counts(thstr_vars* th, float volt)
{
    return (unsigned int) ( (volt/th->vref)*(powf(2.0, th->fres) ) - 1.0);
}

//
// Setup/Initialization Functions
//

void setup_thstr(thstr_vars* th, float beta, float r0, float t0, float vref, float rs, float offset, unsigned char adc_resolution, float sample_rate, float adc_tau)
{
    th->nbits = adc_resolution;
    th->fres = (float) adc_resolution;

    th->beta = beta;
    th->r0 = r0;
    th->t0 = t0 + K2C;
    th->vref = vref;
    th->rs = rs;

    th->rinf = th->r0*exp(-th->beta/th->t0);

    th->offset = offset;
    th->rcal = 1.0;

    //th->rinf = r0*expf(-th->beta/th->t0);
    th->r_meas = 0.0;
    th->t_meas = 0.0;

    //Control system variables
    th->set_point_degreesF = 200.0;
    th->set_point_max = 0.0;
    th->set_point_min = 0.0;

    th->process_variable_cnts = 0;
    th->process_variable_raw = 0;
    th->process_variable_degreesF = 0.0;
    th->throttle = false;

    // ADC Noise Filter -- first-order IIR
    //   a0 = (2^ADC_TC - 1) / 2^ADC_TC
    //   This results in a time constant of TAU = (2^ADC_TC)/SAMPLE_RATE
    //   or,
    //   ADC_TC = LOG2(TAU*SAMPLE_RATE)
    // Example:
    //   For SAMPLE_RATE = 10, ADC_TC = 6,
    //   Tau = 64/10 = 6.4 seconds
    float adc_tc = log(sample_rate*adc_tau)/log(2.0);
    th->adc_tc = (unsigned int) floorf(adc_tc + 0.5);
    if(th->adc_tc < 1)
        th->adc_tc = 1;
    else if(th->adc_tc > (SHIFT - 4))
        th->adc_tc = SHIFT - 4;
}


//Set up process control set points.
//  target is the desired center temperature.
//  hysteresis is total min-max variation
void set_process_control_target(thstr_vars* th, float target, float hysteresis)
{
    float hyst = hysteresis/2.0;

    th->set_point_degreesF = target;
    th->set_point_max = target + hyst;
    th->set_point_min = target - hyst;

    th->throttle = false;
}

//
// Hysteretic temperature control process
// Give it a pointer to the thermistor control object and a reading from the ADC
//
void run_process_control(thstr_vars* th, unsigned int adc_cnt)
{
    unsigned int pv = th->process_variable_cnts;
    unsigned int raw = adc_cnt;
    th->process_variable_raw = adc_cnt;  //keep unfiltered record of it for troubleshooting

    //Run low-pass averaging filter
    //2^ADC_TC sample time constant
    unsigned int tmp = pv;
    th->process_variable_cnts  = ((pv<<th->adc_tc) + (raw<<SHIFT) - tmp)>>th->adc_tc;
    tmp = th->process_variable_cnts;
    pv = tmp >> SHIFT;

    th->process_variable_degreesF = count_to_tempF(th, pv);

    //Then decide to turn heat on or off
    if(th->throttle)
    {
        if(th->process_variable_degreesF > th->set_point_max)
        {
            th->throttle = false;
        }
    }
    else
    {
        if(th->process_variable_degreesF < th->set_point_min)
        {
            th->throttle = true;
        }
    }
}


#endif //THSTR_CONV_H
