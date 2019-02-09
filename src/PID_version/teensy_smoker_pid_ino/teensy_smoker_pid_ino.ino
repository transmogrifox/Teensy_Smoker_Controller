//
//  Teensy temperature controller for an electric smoker
//
#include "thermistor_conversion.hpp"
#include "iir_compensator.hpp"


// Thermistor parameters
#define B               4010        // Thermistor Beta value, cooking chamber
#define Be              3950        // Thermistor Beta value, at element
#define R0              678.6       // Thermistor resistance at T0, cooking chamber
#define R0e             810.6       // R0 for thermistor at element

#define T0              100          // Reference temeperature for thermistor calibration (T0 in ^C), cooking chamber
#define T0e              94         // T0 at element

// ADC and hardware configuration
#define VREF            3.3         // ADC reference voltage
#define RS              1000.0        // Resistor connected in series with the thermistor, cooking chamber
#define RSe             809.0        // RS at the element
#define OFST            0.0         // Degrees F for final output error biasing, cooking chamber thermistor
#define OFSTe            0.0         // OFST for thermistor at element

//Fixed point processing defs
#define SAMPLE_RATE     120
#define ADC_RES         12

// Setup control loop compensator (PID controller)
iir_comp typeIII;
iir_comp* comp;

volatile int32_t counter;
volatile static bool heating_active;
volatile static bool enable_heating;

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

elapsedMillis sample_timer;
elapsedMillis seconds_timer;
static volatile uint32_t sample_period;
static volatile uint8_t minutes_timer;
static volatile uint8_t timer_scanner;

thstr_vars tempsensor;  // placed in vicinity to food being cooked or smoked
thstr_vars element_temp;  // placed near to the heating element for minimum thermal time constant
                          // Thermistor at element_temp is part of the integration loop for the sigma delta converter 
thstr_vars* ptempsensor = &tempsensor;
thstr_vars* pelement_temp = &element_temp;

const int RELAY = 13;
const int RELAY_ = 12;

//LCD Communication
// Serial LCD Output
#define HWSERIAL Serial1
#define LCD_ROWS    2
#define LCD_COLS    16
#define LCD_BUF_SZ  (LCD_ROWS*LCD_COLS+1)

char lcd_buffer[LCD_BUF_SZ];

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
  HWSERIAL.write(80);
  HWSERIAL.write(0);
  HWSERIAL.write(220);
}

void backlight_green()
{
  HWSERIAL.write(0xFE);
  HWSERIAL.write(0xD5);
  HWSERIAL.write(0);
  HWSERIAL.write(220);
  HWSERIAL.write(0);
}

void lcd_home()
{
  //Home
  HWSERIAL.write(0xFE);
  HWSERIAL.write(0x48);
  delay(10);
}

void lcd_clear_home()
{
  HWSERIAL.write(0xFE);
  HWSERIAL.write(0x58);
  delay(10);
  //Home
  lcd_home();
}

void flip_lcd_buffer()
{
  for(int i=0; i<LCD_BUF_SZ - 1; i++)
  {
    HWSERIAL.write(lcd_buffer[i]);  
    delay(10);
  }
}

void lcd_update_status(float setp, float pvar, float tvar, bool hyst)
{

  sprintf(&lcd_buffer[0], "S%03d.", (int) (floorf(setp)) );
  sprintf(&lcd_buffer[5], "%01d ", (int) ((setp - floorf(setp))*10.0) );

  sprintf(&lcd_buffer[7], "P%03d.", (int) (floorf(pvar)) );
  sprintf(&lcd_buffer[7+5], "%01d ", (int) ((pvar - floorf(pvar))*10.0) );

  float hrs = tvar/60.0;
  sprintf(&lcd_buffer[16], "T: %02d:", (int) (floorf(hrs)) ); //hours
  sprintf(&lcd_buffer[16+6], "%02d:", (int) ((hrs - floorf(hrs))*60.0) ); //minutes
  //sprintf(&lcd_buffer[16], "T: %04d:", (int) (floorf(tvar)) );
  sprintf(&lcd_buffer[16+6+3], "%02d ", (int) ((tvar - floorf(tvar))*60.0) ); //seconds

  for(int i = 27; i < LCD_BUF_SZ - 1; i++)
  {
    lcd_buffer[i] = ' ';
  }

  //Hysteretic or PID?
  if(hyst)
  {
    sprintf(&lcd_buffer[LCD_BUF_SZ - 1- 3], "HYS");
  }
  else
  {
    sprintf(&lcd_buffer[LCD_BUF_SZ - 1- 3], "PID");
  }
  
  //Degree symbol followed by F
  lcd_buffer[14] = 0xDF;  //Degree symbol
  lcd_buffer[15] = 'F'; 

  //Finally write it out
  flip_lcd_buffer();

}
  
void setup()
{   

  for(int i=0; i < LCD_BUF_SZ; i++)
  {
    lcd_buffer[i] = ' ';
  }

  lcd_clear_home();
  delay(10);
  flip_lcd_buffer();
  delay(10);
    
  pinMode(RELAY, OUTPUT);
  digitalWrite(RELAY, 0);
  pinMode(RELAY_, OUTPUT);
  digitalWrite(RELAY_, 1);
  
  analogReadResolution(ADC_RES);           
  Serial.begin(38400);
  HWSERIAL.begin(9600);
  counter = 0;

  setup_thstr(ptempsensor, B, R0, T0, VREF, RS, OFST, ADC_RES, SAMPLE_RATE, 1.0);
  setup_thstr(pelement_temp, Be, R0e, T0e, VREF, RSe, OFSTe, ADC_RES, SAMPLE_RATE, 1.0);
  

  set_point = 75.0;
  err = 0.0;
  throttle = 0.0;

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
  float r1 = 3520.0;
  float r2 = 750.0;
  float r3 = 150;
  float c1 = 650*m;
  float c2 = c1/20.0;
  float c3 = 250*m;
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
  heating_active = 0;
  enable_heating = 0;
  time_remaining = 0;
  seconds_timer = 0;
  minutes_timer = 0;
  timer_scanner = 0;

  digitalWrite(RELAY, 0); 
  digitalWrite(RELAY_, 1);

}


void loop()                     
{
   uint32_t val, valel;
  //Clock
  if(seconds_timer >= 999)
  {
    seconds_timer = 0;
    minutes_timer++;
    if(minutes_timer > 60)
    {
      minutes_timer = 0;
      
    }

    time_remaining -= 1.0/60.0;
      if(time_remaining < 1/60.0) time_remaining = 0.0;
  }

  //Process control
  if(sample_timer >= sample_period)
  {
    sample_timer = 0;

   
    //Get value from ADC with thermistor
    val = analogRead(2); //chamber temperature
    valel = analogRead(1); //element sensor temperature

    // Convert it to degrees F, only for use in scope of loop()
    float tempF_pv = count_to_tempF(ptempsensor, val);
    float tempF_el = count_to_tempF(pelement_temp, valel);

    // Run control
    if(hysteresis > 1.0)
    {
      //In this case we just want to cycle up and down and don't care about overshoot, etc.
      set_process_control_target(pelement_temp, set_point, hysteresis);
      run_process_control(pelement_temp, val);
      throttle = run_compensator(comp, 0.0);
      running_hysteretic = 1;
    }
    else
    {
      // Throttle is amplified error, inverted, so adding it to set point creates a set-point for the 
      // inner-loop hysteretic contoller.  In this case the hysteretic controller functions as a delta-sigma modulator
      err = tempF_pv - set_point;
      
      throttle = run_compensator(comp, err);
          //Unclamped throttle is not meaningful, so it is clamped within something meaningful to the capability of the system.
    if(throttle > 350.0) 
      throttle = 350.0;
    else if (throttle < -200.0) 
      throttle = -200.0;
      
      set_process_control_target(pelement_temp, set_point + throttle, 0.001);
      run_process_control(pelement_temp, valel);
      running_hysteretic = 0;
    }
    

    if(enable_heating)
    {
      digitalWrite(RELAY, pelement_temp->throttle); 
      digitalWrite(RELAY_, !(pelement_temp->throttle));
      heating_active = pelement_temp->throttle;
    }

    
    if(++counter >= SAMPLE_RATE/4)
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

      set_point = 50.0 + (pot_set_point/4096.0)*200.0;
      hysteresis = (pot_hysteresis/4096.0)*20.0 - 2.0;
      if(hysteresis < 0.0) hysteresis = 0.0;
      
      // Evaluate if changed significantly
      if(abs(pot_set_point - pot_set_point_l) > 10)
      {
        //set_point = 50.0 + (pot_set_point/4096.0)*200.0;
        Serial.println();
        Serial.print("Set Point:  ");
        Serial.println(set_point);
      }
      if(abs(pot_hysteresis - pot_hysteresis_l) > 10)
      {
//        hysteresis = (pot_hysteresis/4096.0)*20.0;
//        Serial.println();
//        Serial.print("New Hysteresis:  ");
//        Serial.println(hysteresis);
      }
      if(abs(pot_timer - pot_timer_l) > 10)
      {
        
        timer_scanner = 4;
//        Serial.println();
//        Serial.print("New Timer:  ");
//        Serial.println(time_remaining);
      }    
      if(timer_scanner > 0)
      {
        time_remaining = 24.0*60.0*(pot_timer)/4096.0-1.0; //Setting in minutes, let it go up to 24 hours;
        if(time_remaining < 0.0) time_remaining = 0.0;
        timer_scanner--;
      }
        
      //Serial.print("analog 3 is: ");
      Serial.print(set_point);
      Serial.print(" ");
      Serial.print(tempF_pv);
      Serial.print(" "); 
      Serial.print(throttle+set_point);
      Serial.print(" ");
      Serial.println(tempF_el);
      //Serial.print(" ");
      //Serial.println(pelement_temp->throttle);
      //Serial.print(" ");
      //Serial.println(err);
           

      //Output pot readings
//      Serial.print(" SP: ");
//      Serial.print(set_point);
//      Serial.print(" HYST: ");
//      Serial.print(hysteresis);
//      Serial.print(" TIM: ");
//      Serial.print(pot_timer);
//      if(running_hysteretic)
//        Serial.println(" RUNNING HYSTERETIC");
//      else
//        Serial.println(" PID CONTROL");

      //Output to LCD
        //Clear display
      lcd_home();
      
       //Output to LCD
     lcd_update_status(set_point, tempF_pv, time_remaining, running_hysteretic);

      if(time_remaining > 2.0/60.0)
      {
       if(pelement_temp->throttle) 
       {
          backlight_red();
          enable_heating = 1;
       }
       else
       {
         backlight_blue();
         enable_heating = 0;
       }
      }
      else
      {
        backlight_green();
        enable_heating = 0;
      }
      

          
      
    }
    
  }
  

}
