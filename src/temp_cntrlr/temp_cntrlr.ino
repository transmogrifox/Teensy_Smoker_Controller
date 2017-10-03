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
//  2500         207.2
//  2750        206.7
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


#define K2C             273.15      //Add to ^C to get ^K
#define B               4050        //Thermistor Beta value
#define R0              835.4       //Thermistor resistance at T0
#define T0              93          //T0 in ^C
#define VREF            3.3         //Reference voltage
#define RS              809         //Series reference resistor
#define OFST            -0.5         //Degrees F for final output error biasing

//Fixed point processing defs
#define SAMPLE_RATE     32
#define SHIFT           12          // Define decimal point location for IIR
#define ADC_TC          6           // a0 = (2^ADC_TC - 1) / 2^ADC_TC
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
    uint32_t set_point_max_cnts, set_point_min_cnts; //represents hysteresis in control loop
    uint32_t set_point_curr_cnts;  //Alternates between min and max set points
    uint32_t process_variable_cnts;
    uint32_t process_variable_raw;
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

float count_to_temp(thstr_vars* th, uint32_t cnt, uint8_t nbits)
{
  float v = ((float) cnt)*th->vref/(powf(2.0, (float) nbits) - 1.0);
  return volt_to_temp(th, v);
}

//Takes temperature in F and calculates ADC voltage set point
float temp_to_volt(thstr_vars* th, float temp)
{
    //convert degrees F to Kelvin
    float t = (temp - 32.0)/1.8 + K2C;
    return th->rs*th->vref/(th->rinf*exp(th->beta/t) + th->rs);
}

uint32_t volt_to_counts(thstr_vars* th, float volt, uint8_t nbits)
{
    return (uint32_t) ( (volt/th->vref)*(powf(2.0, (float) nbits) ) - 1);
}

//Set up process control set points.
//  target is the desired center temperature.
//  hysteresis is total min-max variation
void set_process_control_target(thstr_vars* th, float target, float hysteresis, uint8_t nbits)
{
    float hyst = hysteresis/2.0;
    float max = temp_to_volt(th, target + hyst);
    float min = temp_to_volt(th, target - hyst);
    //printf("min:  %f\nmax: %f\nt: %f\n", min, max, temp_to_volt(th, target));
    th->set_point_degreesF = target;

    // Left-shift set points to match decimal point location
    // on fixed-point ADC averaging filter
    uint32_t tmp = volt_to_counts(th, max, nbits);
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
void run_process_control(thstr_vars* th, uint32_t adc_cnt)
{
    uint32_t pv = th->process_variable_cnts;
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

//
//  Main Program Code
//  Handles ADC read interval and output printing functions
//

static thstr_vars temp_sensor2;
volatile int32_t counter;
volatile uint32_t val;

elapsedMillis sample_timer;
elapsedMillis run_time;
static volatile uint32_t seconds;
static volatile uint32_t minutes;
static volatile uint32_t hours;
static volatile uint32_t days;
static volatile bool toggle_display;
static volatile bool toggle_red;
static volatile bool toggle_blue;

static uint32_t sample_period;

volatile bool led_state;
const int ledPin = 13;
const int heaterPin = 12;


// Serial LCD Output
#define HWSERIAL Serial1

void setup()
{   
  analogReadResolution(12);           
  Serial.begin(38400);
  HWSERIAL.begin(9600);
  counter = 0;
  led_state = false;
  pinMode(ledPin, OUTPUT);
  pinMode(heaterPin, OUTPUT);

  sample_period = 1000/SAMPLE_RATE;

  setup_thstr(&temp_sensor2, B, R0, T0, VREF, RS, OFST);
  set_process_control_target(&temp_sensor2, 200.0, 4.0, 12);

  //Clock
  run_time = 0;
  sample_timer = 0;
  seconds = 0;
  minutes = 0;
  hours = 0;
  days = 0;
  toggle_display = false;
  toggle_blue = true;
  toggle_red = true;

  //Clear screen
  HWSERIAL.write(0xFE);
  HWSERIAL.write(0x58);
  delay(10);
  //Home
  HWSERIAL.write(0xFE);
  HWSERIAL.write(0x48);
  delay(10);
  //Contrast
  HWSERIAL.write(0xFE);
  HWSERIAL.write(0x91);
  HWSERIAL.write(180);
  //RGB
  HWSERIAL.write(0xFE);
  HWSERIAL.write(0xD5);
  HWSERIAL.write(160);
  HWSERIAL.write(20);
  HWSERIAL.write(210);
  delay(10);
}


void loop()                     
{
  if(run_time >= 1000)
  {
    
    run_time = run_time - 1000;
    seconds += 1;
    if(seconds >= 60)
    {
      seconds = 0;
      minutes += 1;
      if(minutes >= 60)
      {
        minutes = 0;
        hours += 1;
        if(hours >= 24)
        {
          hours = 0;
          days += 1;
        }
      }
    }
  }
  if(sample_timer >= sample_period)
  {
    sample_timer = 0;
    run_process_control(&temp_sensor2, (uint32_t) analogRead(2));

    if(counter++ > 33)
    {
      char digit[7];
      float current_temperature = count_to_temp(&temp_sensor2, temp_sensor2.process_variable_cnts, 12+SHIFT);
      counter = 0;
      Serial.print("analog 3 temp is: ");
      Serial.println(current_temperature);
      Serial.println(temp_sensor2.process_variable_cnts);

      //Print Temperature
      HWSERIAL.write(0xFE);
      HWSERIAL.write(0x48);
      delay(10);
      HWSERIAL.print("PV: ");

      //Format temperature output (sprint with float seems to not work correctly on LC, so conversion to ints is being used)
      sprintf(digit, "%03d", (int) (floorf(current_temperature)) );
      HWSERIAL.print(digit);
      HWSERIAL.print(".");
      sprintf(digit, "%01d", (int) ((current_temperature - floorf(current_temperature))*10.0) );
      HWSERIAL.print(digit);

      //Degree symbol followed by F      
      HWSERIAL.write(0xDF);
      HWSERIAL.println("F");

      if(toggle_display)
      {
        toggle_display = false;
        //Print Time
        sprintf(digit, "%02u", days);
        HWSERIAL.print(digit);
        HWSERIAL.print(" d, ");
        sprintf(digit, "%02u", hours);
        HWSERIAL.print(digit);
        HWSERIAL.print(":");
        sprintf(digit, "%02u", minutes);
        HWSERIAL.print(digit);
        HWSERIAL.print(":");
        sprintf(digit, "%02u", seconds);
        HWSERIAL.print(digit);
      } else
      {
        toggle_display = true;
        //Print process set point
        HWSERIAL.print("SP: ");
        sprintf(digit, "%03d", (int) (floorf(temp_sensor2.set_point_degreesF)) );
        HWSERIAL.print(digit);
        HWSERIAL.print(".");
        sprintf(digit, "%01d", (int) ((temp_sensor2.set_point_degreesF - floorf(temp_sensor2.set_point_degreesF))*10.0) );
        HWSERIAL.print(digit);
        //Degree symbol followed by F      
        HWSERIAL.write(0xDF);
        HWSERIAL.println("F");
        HWSERIAL.print("     ");
      }

      if(temp_sensor2.throttle)
      {
        digitalWrite(ledPin, HIGH);
        digitalWrite(heaterPin, HIGH);

        if(toggle_red)
        {
          //RGB
          HWSERIAL.write(0xFE);
          HWSERIAL.write(0xD5);
          HWSERIAL.write(225);
          HWSERIAL.write(10);
          HWSERIAL.write(10);
        }
        toggle_red = false;
        toggle_blue = true;
        
      }
      else
      {
         digitalWrite(ledPin, LOW);
         digitalWrite(heaterPin, LOW);

        if(toggle_blue)
        {
          //RGB
          HWSERIAL.write(0xFE);
          HWSERIAL.write(0xD5);
          HWSERIAL.write(160);
          HWSERIAL.write(20);
          HWSERIAL.write(210);
        }
        toggle_blue = false;
        toggle_red = true;
      }
      
    }
    
  }
  
  

}