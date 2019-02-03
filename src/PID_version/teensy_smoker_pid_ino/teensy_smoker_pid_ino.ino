//
//  Teensy temperature controller for an electric smoker
//

#include "thermistor_conversion.hpp"
#include "iir_compensator.hpp"


// Thermistor parameters
#define B               4010        // Thermistor Beta value, cooking chamber
#define Be              4010        // Thermistor Beta value, at element
#define R0              678.6       // Thermistor resistance at T0, cooking chamber
#define R0e             678.6       // R0 for thermistor at element

#define T0              100          // Reference temeperature for thermistor calibration (T0 in ^C), cooking chamber
#define T0e              100         // T0 at element

// ADC and hardware configuration
#define VREF            3.3         // ADC reference voltage
#define RS              1000        // Resistor connected in series with the thermistor, cooking chamber
#define RSe             1000        // RS at the element
#define OFST            0.0         // Degrees F for final output error biasing, cooking chamber thermistor
#define OFST            0.0         // OFST for thermistor at element

//Fixed point processing defs
#define SAMPLE_RATE     120
#define ADC_RES         12

// Setup control loop compensator (PID controller)
iir_comp typeIII;
iir_comp* comp;

volatile int32_t counter;
volatile uint32_t val, valel;

// Settings
volatile int32_t pot_set_point;
volatile int32_t pot_hysteresis;
volatile int32_t pot_timer;
volatile int32_t pot_set_point_l;
volatile int32_t pot_hysteresis_l;
volatile int32_t pot_timer_l;
volatile bool running_hysteretic;

volatile float set_point;
volatile float hysteresis;
volatile float time_remaining;

volatile float err;
volatile float throttle;
volatile float tempF;

elapsedMillis sample_timer;
static uint32_t sample_period;

thstr_vars tempsensor;  // placed in vicinity to food being cooked or smoked
thstr_vars element_temp;  // placed near to the heating element for minimum thermal time constant
                          // Thermistor at element_temp is part of the integration loop for the sigma delta converter 
thstr_vars* ptempsensor = &tempsensor;
thstr_vars* pelement_temp = &element_temp;

const int RELAY = 13;
const int RELAY_ = 12;
  
void setup()
{   
  pinMode(RELAY, OUTPUT);
  digitalWrite(RELAY, 0);
  pinMode(RELAY_, OUTPUT);
  digitalWrite(RELAY_, 1);
  
  analogReadResolution(ADC_RES);           
  Serial.begin(38400);
  counter = 0;

  setup_thstr(ptempsensor, B, R0, T0, VREF, RS, OFST, ADC_RES, SAMPLE_RATE, 1.0);
  setup_thstr(pelement_temp, Be, R0e, T0e, VREF, RSe, OFSTe, ADC_RES, SAMPLE_RATE, 1.0);
  

  set_point = 75.0;
  err = 0.0;
  throttle = 0.0;
  tempF = 0.0;

  // Setup control loop compensator
  comp = &typeIII;
  
  //Control loop sampling frequency
  sample_period = 1000/SAMPLE_RATE;
  float Ts = 1.0/SAMPLE_RATE;
  float cfs = 1.0/Ts;
  float cws = 2.0*M_PI*cfs;

  //Circuit params
  float k = 1000.0;
  float m = 1.0/k;
  float r1 = 5250.0;
  float r2 = 550;
  float r3 = 105.0;
  float c1 = 0.5*m;
  float c2 = c1/100.0;
  float c3 = 20*m;
  float pg = cws/(500.0*k);
  init_compensator(comp, cfs);
  set_circuit_params(comp, r1, r2, r3, c1, c2, c3, pg);

  //Initialize input readings
  pot_set_point = 4096 - analogRead(8);
  pot_hysteresis = 4096 - analogRead(7);
  pot_timer = 4096 - analogRead(9);
  pot_set_point_l = pot_set_point;
  pot_hysteresis_l = pot_hysteresis;
  pot_timer_l = pot_timer; 

  set_point = 50.0 + (pot_set_point/4096.0)*200.0;
  hysteresis = (pot_set_point/4096.0)*20.0;
  time_remaining = 0.0;

}


void loop()                     
{
  if(sample_timer >= sample_period)
  {
    sample_timer = 0;

   
    //Get value from ADC with thermistor
    val = analogRead(2);
    valel = val; //analogRead(1); //Update to correct channel when the element thermistor is connected

    // Convert it to degrees F
    tempF = count_to_tempF(ptempsensor, val);

    // Throttle output is inverted phase at DC, and to be tuned 
    // within the system for correct control
    err = tempF - set_point;
    throttle = run_compensator(comp, err);

    //Unclamped throttle is not meaningful, so it is clamped within something meaningful to the capability of the system.
    if(throttle > 350.0) 
      throttle = 350.0;
    else if (throttle < -20.0) 
      throttle = -20.0;

    // Run control
    if(hysteresis > 1.0)
    {
      //In this case we just want to cycle up and down and don't care about overshoot, etc.
      set_process_control_target(ptempsensor, set_point, hysteresis);
      running_hysteretic = 1;
    }
    else
    {
      // Throttle is amplified error, inverted, so adding it to set point creates a set-point for the 
      // inner-loop hysteretic contoller.  In this case the hysteretic controller functions as a delta-sigma modulator
      set_process_control_target(pelement_temp, set_point + throttle, 0.001);
      running_hysteretic = 0;
    }
    run_process_control(ptempsensor, val);
    run_process_control(pelement_temp, val_el);
     
    digitalWrite(RELAY, pelement_temp->throttle);
    //digitalWrite(RELAY, ptempsensor->throttle);
    digitalWrite(RELAY_, !(pelement_temp->throttle));
    //digitalWrite(RELAY_, !(ptempsensor->throttle));

    
    if(counter++ > 50)
    {
      counter = 0;    
        
      // Save last control knob inputs
      pot_set_point_l = pot_set_point;
      pot_hysteresis_l = pot_hysteresis;
      pot_timer_l = pot_timer; 
      // Get new control knob inputs
      pot_set_point = 4096 - analogRead(8);
      pot_hysteresis = 4096 - analogRead(7);
      pot_timer = 4096 - analogRead(9);
    
      // Evaluate if changed significantly
      if(abs(pot_set_point - pot_set_point_l) > 10)
      {
        set_point = 50.0 + (pot_set_point/4096.0)*200.0;
        Serial.println();
        Serial.print("New Set Point:  ");
        Serial.println(set_point);
      }
      if(abs(pot_hysteresis - pot_hysteresis_l) > 10)
      {
        hysteresis = (pot_hysteresis/4096.0)*20.0;
        Serial.println();
        Serial.print("New Hysteresis:  ");
        Serial.println(hysteresis);
      }
      if(abs(pot_timer - pot_timer_l) > 10)
      {
        time_remaining = 24.0*60.0*(pot_timer)/4096.0; //Setting in minutes, let it go up to 24 hours;
        Serial.println();
        Serial.print("New Timer:  ");
        Serial.println(time_remaining);
      }      
      //Serial.print("analog 3 is: ");
      Serial.print(throttle);
      Serial.print(" ");
      Serial.print(err);
      Serial.print(" ");
      Serial.print(tempF);

      //Output pot readings
      Serial.print(" SP: ");
      Serial.print(pot_set_point);
      Serial.print(" HYST: ");
      Serial.print(pot_hysteresis);
      Serial.print(" TIM: ");
      Serial.print(pot_timer);
      if(running_hysteretic)
        Serial.println(" RUNNING HYSTERETIC");
      else
        Serial.println(" PID CONTROL");
    }
    
  }
  

}
