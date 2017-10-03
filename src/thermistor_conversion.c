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


#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define K2C             273.15      //Add to ^C to get ^K
#define B               4050        //Thermistor Beta value
#define R0              835.4       //Thermistor resistance at T0
#define T0              93          //T0 in ^C
#define VREF            3.3
#define RS              820
#define OFST            -0.5         //Degrees F for final output error biasing

//Fixed point processing defs
#define SAMPLE_RATE     32
#define SHIFT           12          // Define decimal point location for IIR
#define ADC_TC          8           // a0 = (2^ADC_TC - 1) / 2^ADC_TC
                                    // This results in a time constant of (2^ADC_TC)/SAMPLE_RATE
                                    // For SAMPLE_RATE = 10, ADC_TC = 6,
                                    // Tau = 64/10 = 6.4 seconds

typedef struct thstr_vars_t
{
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
    unsigned int set_point_max_cnts, set_point_min_cnts; //represents hysteresis in control loop
    unsigned int set_point_curr_cnts;  //Alternates between min and max set points
    unsigned int process_variable_cnts;
    unsigned int process_variable_raw;
    float process_variable_degreesF;
    bool throttle;  //variable tells whether heating element is on or off


} thstr_vars;

void setup_thstr(thstr_vars* th, float beta, float r0, float t0, float vref, float rs, float offset)
{
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
    th->set_point_max_cnts = 0;
    th->set_point_min_cnts = 0;
    th->set_point_curr_cnts = 0;
    th->process_variable_cnts = 0;
    th->process_variable_raw = 0;
    th->process_variable_degreesF = 0.0;
    th->throttle = false;
}

//Returns temperature in degrees F but saves degrees C in t_meas
float volt_to_temp(thstr_vars* th, float volt)
{
    th->r_meas = th->rcal*th->rs*(th->vref/volt - 1.0);
    th->t_meas = th->beta/(logf(th->r_meas/th->rinf)) - K2C;
    return 1.8*th->t_meas + 32.0 + th->offset;
}

//Takes temperature in F and calculates ADC voltage set point
float temp_to_volt(thstr_vars* th, float temp)
{
    //convert degrees F to Kelvin
    float t = (temp - 32.0)/1.8 + K2C;
    return th->rs*th->vref/(th->rinf*exp(th->beta/t) + th->rs);
}

unsigned int volt_to_counts(thstr_vars* th, float volt, unsigned char nbits)
{
    return (unsigned int) ( (volt/th->vref)*(powf(2.0, (float) nbits) ) - 1);
}

//Set up process control set points.
//  target is the desired center temperature.
//  hysteresis is total min-max variation
void set_process_control_target(thstr_vars* th, float target, float hysteresis, unsigned char nbits)
{
    float hyst = hysteresis/2.0;
    float max = temp_to_volt(th, target + hyst);
    float min = temp_to_volt(th, target - hyst);
    //printf("min:  %f\nmax: %f\nt: %f\n", min, max, temp_to_volt(th, target));
    th->set_point_degreesF = target;

    // Left-shift set points to match decimal point location
    // on fixed-point ADC averaging filter
    unsigned int tmp = volt_to_counts(th, max, nbits);
    tmp = tmp << SHIFT;
    th->set_point_max_cnts = tmp;
    tmp = volt_to_counts(th, min, nbits);
    tmp = tmp << SHIFT;
    //printf("mincnt:  %f\n", volt_to_temp(th, 3.3*((float) tmp)/powf(2.0, SHIFT + 12.0)) );
    th->set_point_min_cnts = tmp;
    th->set_point_curr_cnts = th->set_point_min_cnts;
    th->throttle = false;
}

//Give it a pointer to the thermistor control object and a reading from the ADC
void run_process_control(thstr_vars* th, unsigned int adc_cnt)
{
    unsigned int pv = th->process_variable_cnts;
    th->process_variable_raw = adc_cnt;  //keep unfiltered record of it for troubleshooting

    //Run low-pass averaging filter
    //2^ADC_TC sample time constant
    pv = ((pv<<ADC_TC) + (adc_cnt<<SHIFT) - pv)>>ADC_TC;
    th->process_variable_cnts = pv;

    //Then decide to turn heat on or off
    if(th->throttle)
    {
        if(pv > th->set_point_max_cnts)
            th->throttle = false;
    }
    else
    {
        if(pv < th->set_point_min_cnts)
            th->throttle = true;
    }
}

void simulate_control_system(float set_point)
{
    static thstr_vars temp_sensor_sim;
    setup_thstr(&temp_sensor_sim, B, R0, T0, VREF, RS, OFST);
    set_process_control_target(&temp_sensor_sim, set_point, 4.0, 12);

    unsigned int i = 0;
    unsigned int j = 0;
    unsigned int sim_time = 60.0; //Minutes
    unsigned int stop_count = 60*sim_time*SAMPLE_RATE;

    float ambient_temp = 40; //degreesF

    //Digital model of smoker (crude)
    float tc = 3600.0; //Time constant for thermal response time on smoker temp
    float fs = SAMPLE_RATE;
    float a0 = expf(-1.0/(tc*fs)); //1p RC response time
    float pv_temp = ambient_temp; //temperature measured in smoker
    float power = 1500.0; // Heater element wattage
    float Rth = 0.2;  //Thermal resistance of smoker (^F/Watt)
    unsigned int pv_cnt = 0;
    float pv_volt = 0.0;
    for( i=0; i < stop_count; i++ )
    {
        if(temp_sensor_sim.throttle)
            pv_temp = (power*Rth + ambient_temp)*(1.0-a0) + pv_temp*a0;
        else
            pv_temp = ambient_temp*(1.0-a0) + pv_temp*a0;
        pv_volt = temp_to_volt(&temp_sensor_sim, pv_temp);
        pv_cnt = volt_to_counts(&temp_sensor_sim, pv_volt, 12);
        run_process_control(&temp_sensor_sim, pv_cnt);
        if(j >= SAMPLE_RATE)
        {
            printf("%f\t%f\t%f\t%u\t%d\t%f\t%f\t%u\t%u\n",
            ((float)i)/(SAMPLE_RATE*60.0), pv_temp, pv_volt, pv_cnt, temp_sensor_sim.throttle,
            volt_to_temp(&temp_sensor_sim, 3.3*((float) temp_sensor_sim.set_point_min_cnts)/powf(2.0, SHIFT + 12.0)) + 0.5,
            volt_to_temp(&temp_sensor_sim, 3.3*((float) temp_sensor_sim.set_point_max_cnts)/powf(2.0, SHIFT + 12.0)) + 0.5,
            (temp_sensor_sim.set_point_min_cnts>>SHIFT), temp_sensor_sim.set_point_max_cnts>>SHIFT);
            j = 0;
        }
        j++;
    }
}

int main(void)
{
    simulate_control_system(200.0);

    return 0;

    static thstr_vars temp_sensor1;
    setup_thstr(&temp_sensor1, B, R0, T0, VREF, RS, OFST);

    //Test set point conversions:
    float control_set_point = 200.0;  //Degrees F
    printf("Control Temp: %f\nControl Volts: %f\n Control Counts: %u\n", control_set_point, temp_to_volt(&temp_sensor1, control_set_point), volt_to_counts(&temp_sensor1, temp_to_volt(&temp_sensor1, control_set_point), 12) );

    int i = 0;
    float fi = 0.0;
    for(i=0; i<=5000; i++)
    {
        run_process_control(&temp_sensor1, 2040);
        unsigned int pv = temp_sensor1.process_variable_cnts;
        pv = pv >> (SHIFT - 1);
        pv += 1;    //Rounding
        pv = pv >> 1;
        printf("%u\t%u\n", i, pv);

        //if( (fi > 1.6) && (fi < 1.7) )
        //    printf("%f\t%f\n", fi, volt_to_temp(&temp_sensor1, fi));
        fi += 3.3/381;
    }

    return 0;

}
