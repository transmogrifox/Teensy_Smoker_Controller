/*
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
*/

#include <math.h>


#define K2C             273.15      //Add to ^C to get ^K
#define B               4050        //Thermistor Beta value
#define R0              835.4       //Thermistor resistance at T0
#define T0              93          //T0 in ^C
#define VREF            3.3         //Reference voltage
#define RS              666         //Series reference resistor
#define OFST            -0.5         //Degrees F for final output error biasing

//Fixed point processing defs
#define SAMPLE_RATE     33
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
    float set_point_high_degreesF;
    float set_point_low_degreesF;
    uint32_t set_point_max_cnts, set_point_min_cnts; //represents hysteresis in control loop
    uint32_t process_variable_cnts;
    uint32_t process_variable_raw;
    float process_variable_degreesF;
    bool throttle;  //variable tells whether heating element is on or off


} thstr_vars;

void setup_thstr(thstr_vars* th, float beta, float r0, float t0, float vref, float rs, float Toffset, float rcal)
{
    th->beta = beta;
    th->r0 = r0;
    th->t0 = t0 + K2C;
    th->vref = vref;
    th->rs = rs;

    th->rinf = th->r0*exp(-th->beta/th->t0);

    th->offset = Toffset;
    th->rcal = rcal;

    th->r_meas = 0.0;
    th->t_meas = 0.0;

    //Control system variables
    th->set_point_high_degreesF = 200.0;
    th->set_point_low_degreesF = 180.0;
    th->set_point_max_cnts = 0;
    th->set_point_min_cnts = 0;
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
    float t = (temp - 32.0)/1.8 + K2C + th->offset;
    return th->rs*th->vref/(th->rinf*exp(th->beta/t) + th->rs);
}

uint32_t volt_to_counts(thstr_vars* th, float volt, uint8_t nbits)
{
    return (uint32_t) ( (volt/th->vref)*(powf(2.0, (float) nbits)*th->rcal ) - 1);
}

//Set up process control set points.
//  target is the desired center temperature.
//  hysteresis is total min-max variation
void set_process_control_target(thstr_vars* th, float min_, float max_, uint8_t nbits)
{
    float maxval = temp_to_volt(th, max_- th->offset);
    float minval = temp_to_volt(th, min_- th->offset);
    //printf("min:  %f\nmax: %f\nt: %f\n", min, max, temp_to_volt(th, target));
    th->set_point_high_degreesF = max_;
    th->set_point_low_degreesF = min_;

    // Left-shift set points to match decimal point location
    // on fixed-point ADC averaging filter
    uint32_t tmp = volt_to_counts(th, maxval, nbits);
    tmp = tmp << SHIFT;
    th->set_point_max_cnts = tmp;
    tmp = volt_to_counts(th, minval, nbits);
    tmp = tmp << SHIFT;
    //printf("mincnt:  %f\n", volt_to_temp(th, 3.3*((float) tmp)/powf(2.0, SHIFT + 12.0)) );
    th->set_point_min_cnts = tmp;
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
// Switch debouncing routine
// ...because I have my own way I like to do this
//
static const int run_switch = 11;
static const int mode_switch = 12;
static const uint8_t switch_cnt = 10;

static volatile uint8_t run_integrator;
static volatile uint8_t mode_integrator;
static volatile bool run_edge_rising;
static volatile bool run_edge_falling;
static volatile bool mode_edge_rising;
static volatile bool mode_edge_falling;
static volatile bool run_state;
static volatile bool mode_state;

elapsedMillis switch_scan_timer;

void init_switches()
{
  pinMode(run_switch, INPUT);
  pinMode(mode_switch, INPUT);

  switch_scan_timer = 0;
}

// This is such a hackish way to do this I should be embarrassed to make it publicly available...
// ...but I'm not.  Just want to get this thing working.
void run_debounce_switches()
{
  if(digitalRead(run_switch))
  {
    run_integrator++;
    if(run_integrator >= switch_cnt)
    {
      run_integrator = switch_cnt;
      if(!run_state)
      {
        run_edge_falling = false;
        run_edge_rising = true;
      }
      run_state = true;
    }
  }
  else
  {
    run_integrator--;
    if(run_integrator < 1)
    {
      run_integrator = 1;
      if(run_state)
      {
        run_edge_falling = true;
        run_edge_rising = false;
      }
      run_state = false;
    }
  }

  if(digitalRead(mode_switch))
  {
    mode_integrator++;
    if(mode_integrator >= switch_cnt)
    {
      mode_integrator = switch_cnt;
      if(!mode_state)
      {
        mode_edge_falling = false;
        mode_edge_rising = true;
      }
      mode_state = true;
    }
  }
  else
  {
    mode_integrator--;
    if(mode_integrator < 1)
    {
      mode_integrator = 1;
      if(mode_state)
      {
        mode_edge_falling = true;
        mode_edge_rising = false;
      }
      mode_state = false;
    }
  }

}

//
//  Main Program Code
//  Handles ADC read interval and output printing functions
//

#define MAX_MODES                     4
#define MODE_RUNNING                  0
#define MODE_SET_MIN_TEMP             1
#define MODE_SET_MAX_TEMP             2
#define MODE_SET_RUN_TIME             3


static thstr_vars temp_sensor2;
static volatile int32_t counter;
static volatile uint32_t val;
static volatile bool temp_control_enabled;
static volatile uint32_t current_mode;
static volatile float temp_set_high;
static volatile float temp_set_low;

elapsedMillis sample_timer;
elapsedMillis run_time;

static volatile uint32_t ticker;
static volatile uint32_t seconds;
static volatile uint32_t minutes;
static volatile uint32_t hours;
static volatile uint32_t days;
static volatile bool toggle_display;
static volatile bool toggle_red;
static volatile bool toggle_blue;

static uint32_t sample_period;

static volatile bool led_state;
static const int ledPin = 13;


// Serial LCD Output
#define HWSERIAL Serial1

void backlight_red()
{
  HWSERIAL.write(0xFE);
  HWSERIAL.write(0xD5);
  HWSERIAL.write(225);
  HWSERIAL.write(10);
  HWSERIAL.write(10);
}

void backlight_blue()
{
  HWSERIAL.write(0xFE);
  HWSERIAL.write(0xD5);
  HWSERIAL.write(160);
  HWSERIAL.write(20);
  HWSERIAL.write(210);
}

void backlight_green()
{
  HWSERIAL.write(0xFE);
  HWSERIAL.write(0xD5);
  HWSERIAL.write(20);
  HWSERIAL.write(180);
  HWSERIAL.write(20);
}

void lcd_home()
{
  //Home
  HWSERIAL.write(0xFE);
  HWSERIAL.write(0x48);
}

void lcd_clear_home()
{
  HWSERIAL.write(0xFE);
  HWSERIAL.write(0x58);
  delay(10);
  //Home
  lcd_home();
}

void lcd_print_temperature_F(float degreesF)
{
  char digit[17];
  sprintf(digit, "%03d.", (int) (floorf(degreesF)) );
  HWSERIAL.print(digit);
  sprintf(digit, "%01d", (int) ((degreesF - floorf(degreesF))*10.0) );
  HWSERIAL.print(digit);

  //Degree symbol followed by F
  HWSERIAL.write(0xDF);
  HWSERIAL.print("F");
}

void apply_process_state(thstr_vars *th)
{
    if(th->throttle)
    {
      if(temp_control_enabled)
        digitalWrite(ledPin, HIGH);
      else
        digitalWrite(ledPin, LOW);

      if(toggle_red && temp_control_enabled)
      {
        //RGB red
        backlight_red();
        Serial.println("Change red");
      }
      toggle_red = false;
      toggle_blue = true;

    }
    else
    {
       digitalWrite(ledPin, LOW);

      if(temp_control_enabled && toggle_blue)
      {
        //RGB Red
        backlight_blue();
        Serial.println("Change Blue");
      }
      toggle_blue = false;
      toggle_red = true;
    }
}


void secs2dhms(uint32_t ticks_)
{
    uint32_t ds, hrs, mns, scs;
    uint32_t ticks = ticks_;
    ds=hrs=mns=scs = 0;

    if(ticks >= 3600*24)
    {
        ds = ticks/(3600*24);
        ticks %= (3600*24);
    }

    if(ticks >= 3600)
    {
        hrs = ticks/3600;
        ticks %= 3600;
    }

    if(ticks >= 60)
    {
        mns = ticks/60;
        ticks %= 60;
    }

    scs = ticks;

    days = ds;
    hours = hrs;
    minutes = mns;
    seconds = scs;
}

void display_running_process_state(thstr_vars *th)
{
  char digit[17];
  float current_temperature = count_to_temp(th, th->process_variable_cnts, 12+SHIFT);
  counter = 0;

  Serial.print(current_temperature);

  //Print Temperature
  HWSERIAL.write(0xFE);
  HWSERIAL.write(0x48);
  //delay(10);

  //Format temperature output (sprint with float seems to not work correctly on LC, so conversion to ints is being used)
  HWSERIAL.print("PV: ");
  lcd_print_temperature_F(current_temperature);
  HWSERIAL.println();

  if(toggle_display && temp_control_enabled)
  {
    toggle_display = false;
    //Print Time
     secs2dhms(ticker);
    sprintf(digit, "%02u d, ", (unsigned) days);
    HWSERIAL.print(digit);
    sprintf(digit, "%02u:", (unsigned) hours);
    HWSERIAL.print(digit);
    sprintf(digit, "%02u:", (unsigned) minutes);
    HWSERIAL.print(digit);
    sprintf(digit, "%02u  ", (unsigned) seconds);
    HWSERIAL.print(digit);

    sprintf(digit, " %03d.", (int) (floorf(th->set_point_high_degreesF)) );
    Serial.print(digit);
    sprintf(digit, "%01d", (int) ((th->set_point_high_degreesF - floorf(th->set_point_high_degreesF))*10.0) );
    Serial.print(digit);
    Serial.print("  ");

    sprintf(digit, " %03d.", (int) (floorf(th->set_point_low_degreesF)) );
    Serial.print(digit);
    sprintf(digit, "%01d", (int) ((th->set_point_low_degreesF - floorf(th->set_point_low_degreesF))*10.0) );
    Serial.println(digit);
  } else
  {
    toggle_display = true;
    //Print process set point
    lcd_print_temperature_F(th->set_point_low_degreesF);
    HWSERIAL.print(" ");
    lcd_print_temperature_F(th->set_point_high_degreesF);
  }

}


void run_clock()
{
  if(run_time >= 1000)
  {
    if(ticker > 0)
    {
      ticker--;
    }
    else if(temp_control_enabled )
    {
        temp_control_enabled = false;
        digitalWrite(ledPin, LOW);
        backlight_green();
    }

    secs2dhms(ticker);
   run_time = 0;
  }
}

void setup()
{
  analogReadResolution(12);
  Serial.begin(38400);
  HWSERIAL.begin(9600);
  counter = 0;
  led_state = false;
  temp_control_enabled = false;
  pinMode(ledPin, OUTPUT);

  //Setup for reading input switches
  init_switches();

  sample_period = 1000/SAMPLE_RATE;

  temp_set_high = 182;
  temp_set_low = 178;
  setup_thstr(&temp_sensor2, B, R0, T0, VREF, RS, OFST, 0.99);
  set_process_control_target(&temp_sensor2, temp_set_low, temp_set_high, 12);

  //Clock
  run_time = 0;
  sample_timer = 0;
  ticker = 0;
  seconds = 0;
  minutes = 0;
  hours = 0;
  days = 0;
  toggle_display = false;
  toggle_blue = true;
  toggle_red = true;
  current_mode = 0;

  //Disable auto scroll mode
  HWSERIAL.write(0xFE);
  HWSERIAL.write(0x52);
  //Clear screen
  lcd_clear_home();
  delay(10);
  //Contrast
  HWSERIAL.write(0xFE);
  HWSERIAL.write(0x91);
  HWSERIAL.write(180);
  //RGB
  backlight_green();
  delay(10);
}

void loop()
{
  char digit[17];
  //
  // Scan input switch states
  //
  if(switch_scan_timer >= 1)
  {
    switch_scan_timer = 0;
    run_debounce_switches();
    if(run_edge_rising)
    {
      Serial.println("Run Switch Released");
      run_edge_rising = false;
    }
    if(run_edge_falling)
    {
      float temp_set = 200.0;
      lcd_home();
      if(temp_control_enabled)
      {
        temp_control_enabled = false;
        digitalWrite(ledPin, LOW);
        backlight_green();
      }
      else
      {
        temp_control_enabled = true;
        set_process_control_target(&temp_sensor2, temp_set_low, temp_set_high, 12);
        Serial.println("Temperature control enabled");
        toggle_blue = true;
        toggle_red = true;
      }
      Serial.println("Run Switch Pressed");
      Serial.print("Set Temp to:  ");
      Serial.println(temp_set);
      run_edge_falling = false;
    }
    if(mode_edge_rising)
    {
      Serial.println("Mode Switch Released");
      mode_edge_rising = false;
    }
    if(mode_edge_falling)
    {
      lcd_home();
      Serial.print("Mode Switch Pressed:  ");
      mode_edge_falling = false;
      current_mode++;
      set_process_control_target(&temp_sensor2, temp_set_low, temp_set_high, 12); //update every mode change
      if(current_mode >= MAX_MODES)
      {
        current_mode = 0;
      }
      Serial.println(current_mode);
    }

  }

  //
  // Run the clock
  //
  if(temp_control_enabled)
  {
    run_clock();
  }

  //
  // Sample the temperature sensor
  //
  if(sample_timer >= sample_period)
  {
    sample_timer = 0;

    // Always run process controller on the temperature sensor
    run_process_control(&temp_sensor2, (uint32_t) analogRead(2));
    if(temp_control_enabled)
    {
      apply_process_state(&temp_sensor2);
    }

    switch (current_mode)
    {

      case MODE_SET_MIN_TEMP:
        temp_set_low = 50.0 + analogRead(1)*(300.0/4095.0);
        if(temp_set_high < temp_set_low)
          temp_set_high = temp_set_low + 1.0;
        //Print to display
        if(counter++ > SAMPLE_RATE/4)
        {
          lcd_home();
          HWSERIAL.println("LOW     HIGH");
          lcd_print_temperature_F(temp_set_low);
          HWSERIAL.print(" ");
          lcd_print_temperature_F(temp_set_high);
          counter = 0;
        }
      break;

      case MODE_SET_MAX_TEMP:
        temp_set_high = 50.0 + analogRead(1)*(300.0/4095.0);
        if(temp_set_high < temp_set_low)
          temp_set_low = temp_set_high - 1.0;
        //Print to display
        if(counter++ > SAMPLE_RATE/4)
        {
          lcd_home();
          HWSERIAL.println("LOW     HIGH");
          lcd_print_temperature_F(temp_set_low);
          HWSERIAL.print(" ");
          lcd_print_temperature_F(temp_set_high);
          counter = 0;
        }
      break;

      case MODE_SET_RUN_TIME:
        ticker = 20*analogRead(1);
        secs2dhms(ticker);
        //print to display
        if(counter++ > SAMPLE_RATE/4)
        {
          lcd_home();
          sprintf(digit, "%02u d, ", (unsigned) days);
          HWSERIAL.print(digit);
          sprintf(digit, "%02u:", (unsigned) hours);
          HWSERIAL.print(digit);
          sprintf(digit, "%02u:", (unsigned) minutes);
          HWSERIAL.print(digit);
          sprintf(digit, "%02u", (unsigned) seconds);
          HWSERIAL.print(digit);
          counter = 0;
        }
      break;

      case MODE_RUNNING:
        if(counter++ > SAMPLE_RATE)
        {
          display_running_process_state(&temp_sensor2);
          counter = 0;
        }
      break;

      default:
      break;
    }

  }



}
