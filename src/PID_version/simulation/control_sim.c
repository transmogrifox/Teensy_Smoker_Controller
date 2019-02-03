

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "../lib/thermistor_conversion.hpp"
#include "../lib/iir_compensator.hpp"

// Thermistor parameters
#define B               4000        //Thermistor Beta value
#define R0              1005       //Thermistor resistance at T0
#define T0              100          //T0 in ^C

// ADC and hardware configuration
#define VREF            3.3
#define RS              1000
#define OFST            0.0         //Degrees F for final output error biasing

//Fixed point processing defs
#define SAMPLE_RATE     100
#define ADC_RES         12


void simulate_control_system(float set_point)
{
    float ref = set_point;
    float err = 0.0;
    float f_throttle = 0.0;
    unsigned int n_throttle = 0;
    unsigned int cyc_cnt = 0;
    unsigned int cyc_max = 512;
    static thstr_vars temp_sensor_sim;
    setup_thstr(&temp_sensor_sim, B, R0, T0, VREF, RS, OFST, ADC_RES, SAMPLE_RATE, 1.0);
    set_process_control_target(&temp_sensor_sim, set_point, 0.25);

    unsigned int i = 0;
    unsigned int j = 0;
    unsigned int sim_time = 80.0; //Minutes
    unsigned int stop_count = 120*sim_time*SAMPLE_RATE;

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

    // Setup control loop compensator
    iir_comp typeIII;
    iir_comp* comp = &typeIII;

    //Control loop sampling frequency
    float Ts = ((float) cyc_max)/SAMPLE_RATE;
    float cfs = 1.0/Ts;
    float cws = 2.0*M_PI*fs;

    //Circuit params
    float k = 1000.0;
    float m = 1.0/k;
    float r1 = 1250.0;
    float r2 = 1.0*k;
    float r3 = 550.0;
    float c1 = 300.0*m;
    float c2 = c1/30.0;
    float c3 = 500*m;
    float pg = cws/(500.0*k);
    init_compensator(comp, cfs);
    set_circuit_params(comp, r1, r2, r3, c1, c2, c3, pg);


    printf("TIME\tPV_TEMP\tPV_VOLT\tPV_CNT\tTHROTTLE\tSET\tRAW_T\tINT_PV\n");
    for( i=0; i < stop_count; i++ )
    {
        pv_temp = (power*Rth + ambient_temp)*(1.0-a0) + pv_temp*a0;

        pv_volt = temp_to_volt(&temp_sensor_sim, pv_temp);
        pv_cnt = volt_to_counts(&temp_sensor_sim, pv_volt);
        run_process_control(&temp_sensor_sim, pv_cnt);
        if(++cyc_cnt > n_throttle)
            power = 0.0;
        else
            power = 1500.0;

        if(j >= cyc_max)
        {
            err = temp_sensor_sim.process_variable_degreesF - ref;
            f_throttle = run_compensator(comp, err);
            float raw_throttle = f_throttle;
            if(f_throttle > 0.5) f_throttle = 0.5;
            else if (f_throttle < -0.5) f_throttle = -0.5;
            f_throttle += 0.5; //shift to range 0...100% duty
            n_throttle = ((unsigned int) f_throttle) * cyc_max;
            cyc_cnt = 0;

            printf("%f\t%f\t%f\t%u\t%d\t%f\t%f\t%f\n",
            ((float)i)/(SAMPLE_RATE*60.0), pv_temp, pv_volt, pv_cnt,
            n_throttle, temp_sensor_sim.set_point_min,
            raw_throttle + 0.5, temp_sensor_sim.process_variable_degreesF);
            j = 0;
        }
        j++;
    }
}

int main(void)
{
    simulate_control_system(160.0);

    return 0;

}
