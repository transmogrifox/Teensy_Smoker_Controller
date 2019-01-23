

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "thermistor_conversion.hpp"

// Thermistor parameters
#define B               4050        //Thermistor Beta value
#define R0              835.4       //Thermistor resistance at T0
#define T0              93          //T0 in ^C

// ADC and hardware configuration
#define VREF            3.3
#define RS              820
#define OFST            -0.5         //Degrees F for final output error biasing

//Fixed point processing defs
#define SAMPLE_RATE     32
#define ADC_RES         12


void simulate_control_system(float set_point)
{
    static thstr_vars temp_sensor_sim;
    setup_thstr(&temp_sensor_sim, B, R0, T0, VREF, RS, OFST, 12, SAMPLE_RATE, 2.0);
    set_process_control_target(&temp_sensor_sim, set_point, 0.25);

    unsigned int i = 0;
    unsigned int j = 0;
    unsigned int sim_time = 80.0; //Minutes
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


//

    printf("TIME\tPV_TEMP\tPV_VOLT\tPV_CNT\tTHROTTLE\tSET_MIN\tSET_MAX\tINT_PV\n");
    for( i=0; i < stop_count; i++ )
    {
        if(temp_sensor_sim.throttle)
            pv_temp = (power*Rth + ambient_temp)*(1.0-a0) + pv_temp*a0;
        else
            pv_temp = ambient_temp*(1.0-a0) + pv_temp*a0;

        pv_volt = temp_to_volt(&temp_sensor_sim, pv_temp);
        pv_cnt = volt_to_counts(&temp_sensor_sim, pv_volt);
        run_process_control(&temp_sensor_sim, pv_cnt);

        if(j >= SAMPLE_RATE)
        {
            printf("%f\t%f\t%f\t%u\t%d\t%f\t%f\t%f\n",
            ((float)i)/(SAMPLE_RATE*60.0), pv_temp, pv_volt, pv_cnt,
            temp_sensor_sim.throttle, temp_sensor_sim.set_point_min,
            temp_sensor_sim.set_point_max, temp_sensor_sim.process_variable_degreesF);
            j = 0;
        }
        j++;
    }
}

int main(void)
{
    simulate_control_system(200.0);

    return 0;

}
