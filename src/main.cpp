#include <Arduino.h>
/*
 * 
 * Cloud Chamber Temperature Control
 */
  
//
// ARDUINO UNO CONNECTIONS
// =======================
//
//     D2   DS18B20 Temperature sensor data (4K7 pullup)
//     D3   CONFIG MODE ON/OFF INTERRUPT (ENCODER BUTTON) (4K7 PULLUP)
//     D4   ENCODER PIN A (consider using debounce circuit)
//     D5   ENCODER PIN B
//     D6   COOLING WATER PUMP RELAY
//     D7   PELTIER COOLER RELAY
//
//     
//     D18  I2C SDA
//     D19  I2C SCL
//
#define PIN_ONE_WIRE_BUS   2      // One or more DS18B20 sensors Dallas Library
//
#define PIN_CONFIG_BUTTON  3      // Config mode on-off (rotary encoder pushbutton)
#define PIN_ENCODER_A      4      // Rotary encoder A 
#define PIN_ENCODER_B      5      // Rotary encoder B
//
#define PIN_PUMP_RELAY     6      // Commpressor pump on/off relay 
#define PIN_COOLER_RELAY   7      // Cooler on/off relay


// LOOP DELAY 
unsigned long MAIN_LOOP_DELAY = 10; // SLEEPING TIME BETWEEN LOOPS 


//----------------------------------------------------------------------------------
// EEPROM Configuration
// 
#include <EEPROM.h>

const unsigned long EEPROM_MAGIC_NUMBER = 100100001; //  

// Limits
const float  MAX_TARGET_TEMP   =   50.0;   // Reject target temperature greater than 1 degree Celsius
const float  MIN_TARGET_TEMP   =  -99.0;   // Reject target pressure less than -31 degrees Celsius
const float  MAX_COOLER_TEMP   =   50.0;   // Stop pump if temperature reaches 50 degrees Celsius on the cold side

const float  DEFAULT_TARGET_TEMP =  -20.0;  // Default target temperature

bool EEPROM_valid = false;

struct config_data_s {  // V4
  unsigned long   magic_number;
  float           target_temp;
};

struct config_data_s GL_config_data;

bool loadEEPROMConfig();    // Load configuration from EEPROM (forward)
bool updateEEPROMConfig();  // Save configuration from EEPRO (forward)


//----------------------------------------------------------------------------------
// Configuration interface
//
// 0-Normal operation 
// 1-Configure target temperature
// 2-Configure max temp
// 3-Configure pressure sensor presure range (max pressure) 
// 4-Configure restart temp ( < max temp  )
// 5-Configure max running time
// 6-Configure rest time
// 
//
// If max temperature is zero, then no temperature check is performed.
// 
volatile int GL_config_mode = 0;  
int GL_old_config_mode = 0;

bool GL_config_requested = false;  // Configuration requested
bool GL_config_active = false;     // Configuration in process
bool GL_parameter_changed = false; // One parameters has been modified during configuration
bool GL_config_changed = false;    // One or more parameters chaged -> update EEPROM

unsigned long GL_button_time = 0UL;         // time ms config button was pressed
int GL_last_state_A = 0;                    // last state of ENCODER_A signal
unsigned long GL_last_encoder_change = 0;   // time the encoder knob was last turned

unsigned int GL_button_counter = 0;

int GL_paused = 0; // 0=normal operation, 1=paused

//
// Display label for each parameter (config mode)
//
const char *GL_MODELABELS[] = {
      "** END CONFIG **",  // NOT USED
      "SET TARGET TEMP ",
      "PAUSE (0/1)     " 
};
char INVALID_MODE_LABEL[] = "(INVALID MODE!)";
const int NUM_MODES = sizeof(GL_MODELABELS) / sizeof(char *);

//
// Handle interrupt from rotary encoder button, select next config mode (0=config ended)
//
void handle_button_int()  {                 
  if ((millis() - GL_button_time) > 300) { 
    if (!GL_config_mode) {
      GL_config_requested = true; // Run->Config
    } else  {
      GL_config_mode = ((GL_config_mode+1) % NUM_MODES); 
    }
    GL_button_time = millis(); 
    GL_button_counter ++;  
  }  
}


// Encoder interface, prompts user depending on the current GL_config_mode 
// value and updates the corresponondig configuration parameter

int GL_value_set, GL_value_max, GL_value_min, GL_value_old; // Encoder values 

const int MAX_KNOB_INCREMENT = 8;
unsigned int GL_knob_increment = 1; 

void prepare_config_interface(int mode); // Prepare interface and values acording to mode
void read_encoder(int mode);             // Update values


//----------------------------------------------------------------------------------
// I2C LCD Module
//
// WARNING, REMEMBER TO ADJUST THE DISPLAY'S CONTRAST POTENTIOMENTER.
//
//  Nano, UNO
//  =========
//  SDA - D18 
//  SCL - D19
//  
#include <Wire.h>                     // I2C
#include <LiquidCrystal_I2C.h>        // set the LCD address to 0x27 for a 16 chars and 2 line display

LiquidCrystal_I2C lcd(0x3F, 16, 2);     // Use i2c scanner example if necessary

// Forward

void LCD_refresh();
void updateLCD();

unsigned long GL_last_display_refresh_time = 0;
const unsigned long DISPLAY_REFRESH_TIME = 3000;

//
//----------------------------------------------------------------------------------
// DS18B20 Temperature sensors library 
// Requires OneWire library
// All data pins are conected in parallel to digital pin 2 and collectivelly pulled
// up with a 4.7K resistor
//

#include <OneWire.h>
#include <DallasTemperature.h>

// Set up a oneWire instance to communicate with temp sensnor
OneWire oneWire(PIN_ONE_WIRE_BUS);  

// Pass oneWire buses reference to DallasTemperature library
DallasTemperature tempSensor(&oneWire);
int GL_temp_sensor_count = 0;
float GL_temperatures[8];  // Up to 8 sensors
float GL_current_temperature = 0;   // Sensor 0 temperature (used to control cooler)

const unsigned long TMP_MEASUREMENT_INTERVAL = 1000;  // 1s 
unsigned long GL_last_tmp_measurement_time = 0; // Use millis() after each read


//
//----------------------------------------------------------------------------------
// PUMP AND COOLER RELAYS
//
bool GL_pump_state = false;      // true=pump on, false=pump off
bool GL_cooler_state = false;    // true=cooler on, false=cooler off

inline void switchPump(bool onoff)      {     // Switch compressor pump on/off
  digitalWrite(PIN_PUMP_RELAY, onoff? HIGH : LOW); GL_pump_state = onoff; 
}

inline void switchCooler(bool onoff)    {     // Switch cooler on/off
  digitalWrite(PIN_COOLER_RELAY, onoff? HIGH : LOW); GL_cooler_state = onoff; 
} 

// Other forward declarations

void displayConfig();



//
//----------------------------------------------------------------------------------
// Initial configuration
//
//
void setDefaultConfig() {
      GL_config_data.magic_number = EEPROM_MAGIC_NUMBER;
      GL_config_data.target_temp = DEFAULT_TARGET_TEMP; // Target temperature
};


static bool LED_CONFIGURED = false;

void configure_led() {
  pinMode(LED_BUILTIN, OUTPUT);
  LED_CONFIGURED = true;
}
void led_on() {
  if (!LED_CONFIGURED) {
    configure_led();
  }
}

void led_off() {
  if (!LED_CONFIGURED) {
    configure_led();
  }
  digitalWrite(LED_BUILTIN, LOW);
}


// **********************************************************************************
// SETUP
// **********************************************************************************
void setup() {
  
  led_on();

  //
  // Serial
  //
  Serial.begin(115200);
  Serial.println("Temperature control unit");
  Serial.println("STARTING");

  //
  // lcd setup
  //
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("STARTING");

  //
  // EEPROM
  //
  loadEEPROMConfig();
  Serial.println("Press CONFIG button to modify configuration");
  
  //
  // DS18B20 TEMPERATURE SENSOR (DALLAs LIBRARY)
  //
  tempSensor.begin();  // Start up the library
  GL_temp_sensor_count = tempSensor.getDeviceCount();
  lcd.setCursor(0,1);
  lcd.print("#OF DS18B20: ");lcd.print(GL_temp_sensor_count);
  Serial.print("FOUND ");Serial.print(GL_temp_sensor_count);Serial.println(" DS18B20 sensors");

  //
  // CONFIG MODE 
  //
  GL_config_mode = 0;
  GL_config_requested = false;
  GL_button_time = millis();
  pinMode(PIN_CONFIG_BUTTON, INPUT_PULLUP); // Remember to pull up the pin w/4K7 resistor  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< CHECK IF INTERNAL PULLUP CAUSES PROBLEMS
  attachInterrupt(digitalPinToInterrupt(PIN_CONFIG_BUTTON), handle_button_int, FALLING);
  
  //  pinMode (PIN_ENCODER_A, INPUT_PULLUP);
  //  pinMode (PIN_ENCODER_B, INPUT_PULLUP);
  pinMode (PIN_ENCODER_A, INPUT); // w/debouncing circuitry
  pinMode (PIN_ENCODER_B, INPUT);
  GL_last_state_A = digitalRead(PIN_ENCODER_A);  // Read the initial state of the ENCODER_A

  //
  // PUMP RELAY 
  // 
  pinMode(PIN_PUMP_RELAY, OUTPUT);
  switchPump(true); // Start with pump on

  // 
  // COOLER RELAY
  //
  pinMode(PIN_COOLER_RELAY, OUTPUT);
  switchCooler(false);  // Start with cooler off, turn on when necessary

  //
  // END SETUP
  // 
  lcd.clear();
  Serial.println("END SETUP. PUMP IS ON, COOLER IS OFF");
  lcd.print("END SETUP");
   delay(600);
  led_off();
}

// **********************************************************************************
// LOOP
// **********************************************************************************

void loop() {
  //
  // Perform configuration if requested
  //
  if (GL_config_requested) {
    noInterrupts();
    GL_config_mode = 1;
    interrupts();
    GL_old_config_mode = 1;
    lcd.clear();
    GL_config_changed = false;
    GL_config_requested = false; 
    GL_config_active = true;
    Serial.println("CONFIGURATION");
    prepare_config_interface(GL_config_mode); 
  }
  if (GL_config_active) {
    GL_parameter_changed = false;  // Set by read_encoder()
    read_encoder(GL_old_config_mode);
    if (GL_parameter_changed) {
          GL_config_changed = true;
    }
    if (GL_config_mode != GL_old_config_mode) {
      if (GL_config_mode == 0) { // configuration ended
        GL_config_active = false;
        if (GL_config_changed) {
          Serial.println("\nNew configuration:\n");
          displayConfig();
          updateEEPROMConfig();
        } else {
          Serial.println("No changes\n");
        }
      }
      GL_old_config_mode = GL_config_mode;
      prepare_config_interface(GL_old_config_mode);       
    }
    return; // keep looping during configuration
  }

  // 
  // Read temperature
  // 
  if ((millis() - GL_last_tmp_measurement_time) >= TMP_MEASUREMENT_INTERVAL) {
    if (GL_temp_sensor_count > 0) 
    {
      tempSensor.requestTemperatures();
      delay(750); // Wait for conversion
      for (int i = 0; i < GL_temp_sensor_count; i++) {
        GL_temperatures[i] = tempSensor.getTempCByIndex(i);
      }
      GL_current_temperature = tempSensor.getTempCByIndex(0);
    }
    GL_last_tmp_measurement_time = millis();
  }



  //
  // Turn cooler on/off according to temperature
  //  
  // TO DO:
  // ========================
  //  * Update pump relay. Pump should be turned off only if a steep raise in temperature is desired
  //  * Implement PID control for temperature
  //
  //

  bool prev_pump_state = GL_pump_state;
  bool prev_cooler_state = GL_cooler_state;

  // --- If paused, turn / keep off pump and cooler
  if (GL_paused) {
    if (GL_pump_state) {
      switchPump(false);
    } 
    if (GL_cooler_state)  {
      switchCooler(false);
      led_off();
    }
  } else {  // Not paused
    if (!GL_pump_state) {
      // Pump always on except when paused
      switchPump(true);
    }
    // --- If no temperature reading, cooler off for security
    if ((GL_temp_sensor_count < 1) ) {
      GL_cooler_state = false;
    } else {
      if (GL_current_temperature > GL_config_data.target_temp) {
        GL_cooler_state = true;
      } else {
        GL_cooler_state = false;
      }
    }
    if (prev_cooler_state != GL_cooler_state) {
      Serial.print("Cooler "); Serial.println(GL_cooler_state? "ON": "OFF");  
      switchCooler(GL_cooler_state);
    }
    if (prev_pump_state != GL_pump_state) {
      Serial.print("Pump   "); Serial.println(GL_pump_state? "ON": "OFF");
      switchPump(GL_pump_state);
    }
  }
  //
  // Light builtin LED acording to cooler state
  //
  if (GL_cooler_state) {
    led_on();
  } else {
    led_off();
  } 

  //
  // Update LCD
  //
  updateLCD();

  //
  // Delay
  //
  delay(MAIN_LOOP_DELAY);

}


// **********************************************************************************
// AUXILIARY 
// **********************************************************************************

void updateLCD() {
  //
  // UPDATE IF NEEDED
  //
  if ((millis() - GL_last_display_refresh_time) >= DISPLAY_REFRESH_TIME) {
    LCD_refresh ();
    GL_last_display_refresh_time = millis();
  }
}

// Update 16x2 LCD 
// ---------------------------------------------------------------------
// 
// Display format:
//
//    Alternate between two pages, 1st page shows current temperature and set target
//    along with the state of the cooler. 2nd page shows the temperature of each sensor
//
//       0123456789012345
//      +----------------+
//    0 |TMP -99.0 COOLER|  (PAGE 1)
//    1 |SET -99.9   OFF |  (or ON or PAUSED)
//      +----------------+
//    0 |0:-99.9  2:-99.9|  (PAGE 2)
//    1 |1:-99.9  3:-99.9|  (or ON)
//      +----------------+
//
//
void LCD_refresh()
{
  char buf[17];      // buffer for max 16 char display
  char fltbuf1[14];  // float buffer for dtostrf() (arduino's snprintf does not handle floats well)
  char fltbuf2[14];

  static bool page1 = true; // alternate between pages, start with 1st page
    
  //lcd.clear();
  const char *notavail =   "  N/A";
  const char *outofrange = " !ERR";
  float setT = GL_config_data.target_temp;
  float curT = GL_current_temperature;
  dtostrf(curT, 5, 1, fltbuf1);  
  dtostrf(setT, 5, 1, fltbuf2);  
  const char *tmpstr = GL_temp_sensor_count < 1?  notavail : fltbuf1;
  const char *setstr = abs(setT) > 99.9?  outofrange : fltbuf2;

  if (page1) {
    // Page 1 
    // first line
    snprintf(buf, sizeof(buf), "TMP:%s  COOLR",tmpstr);     
    lcd.setCursor(0,0);  // char 0, line 0
    lcd.print(buf);
    Serial.println(buf);  
    // second line
    lcd.setCursor(0,1);  // char 0, line 1
    const char *statestr = GL_paused? "PAUSED": GL_cooler_state? "   ON ": "  OFF ";
    snprintf(buf, sizeof(buf), "SET:%s %s ", setstr, statestr);
    lcd.print(buf);
    Serial.println(buf);
    Serial.println();
  } else {
    // Page 2
    // first line
    dtostrf(GL_temperatures[0], 5, 1, fltbuf1);
    dtostrf(GL_temperatures[2], 5, 1, fltbuf2);
    const char *s0 = GL_temp_sensor_count > 0?  fltbuf1 : notavail;
    const char *s2 = GL_temp_sensor_count > 2?  fltbuf2 : notavail;
    snprintf(buf, sizeof(buf), "0:%s  2:%s", s0, s2);
    lcd.setCursor(0,0);  // char 0, line 0
    lcd.print(buf);
    Serial.println(buf);
    // second line
    dtostrf(GL_temperatures[1], 5, 1, fltbuf1);
    dtostrf(GL_temperatures[3], 5, 1, fltbuf2);
    const char *s1 = GL_temp_sensor_count > 1? fltbuf1: notavail;
    const char *s3 = GL_temp_sensor_count > 3? fltbuf2: notavail;
    snprintf(buf, sizeof(buf), "1:%s  3:%s", s1, s3);
    lcd.setCursor(0,1);  // char 0, line 1
    lcd.print(buf);
    Serial.println(buf);
    Serial.println();
  }
  page1 = !page1;
}
// **********************************************************************************

//
// Load configuration from EEPROM
// ---------------------------------------------------------------------
// 
bool loadEEPROMConfig() {
  #if defined(ESP8266) || defined(ESP32)
  #define ESP__GEN
  #endif

  #ifdef ESP__GEN
    Serial.print(" (ESP32 Version)");
    EEPROM.begin(2048);
  #endif
  Serial.print("EEPROM size is "); Serial.print(EEPROM.length()); Serial.println(" bytes\n");
  lcd.setCursor(0,1);
  lcd.print("EEPROM "); lcd.print(EEPROM.length());
  if (EEPROM.length() < sizeof(config_data_s))  {
    lcd.print(" ERR");
    Serial.print("Error. EEPROM too small (need ");Serial.print((int)sizeof(config_data_s));Serial.println(")");
    Serial.println("Setting configuration to default values. Configuration will not be saved.");
    setDefaultConfig();
    EEPROM_valid = false;
  } else {
    EEPROM_valid = true;
    // read config from EEPROM
    EEPROM.get(0, GL_config_data); // EEPROM.get() is a template based on 2nd arg type, passed as reference
    if (GL_config_data.magic_number != EEPROM_MAGIC_NUMBER) {  
      // EEPROM is not initialized
      setDefaultConfig();
      // Not initialized
      EEPROM.put(0, GL_config_data);
      Serial.println("EEPROM not initilized. Initializing with default values");
    } else {
      Serial.println("Valid configuration found:\n");
    }
  }
  displayConfig();
  return EEPROM_valid;
}

// Update EEPROM configuration
// ---------------------------------------------------------------------
// 
bool updateEEPROMConfig() {
  GL_config_data.magic_number = EEPROM_MAGIC_NUMBER;
  struct config_data_s conf2;
  EEPROM.put(0, GL_config_data);
  EEPROM.get(0, conf2);
  EEPROM_valid = ((GL_config_data.magic_number == conf2.magic_number) && (GL_config_data.target_temp == conf2.target_temp));
  return EEPROM_valid;
}

// Display value while configuring
// ---------------------------------------------------------------------
void show_value(int v) {
  lcd.setCursor(0, 1);
  lcd.print("> ");
  lcd.print(v);  
  lcd.print("     ");
}

// Prepare interface and globals for a new configuration mode
// ---------------------------------------------------------------------
// 
void prepare_config_interface(int mode) {
  ///---int GL_value_old = GL_value_set;
  const char *label = ((mode>=0) && (mode < NUM_MODES)) ? GL_MODELABELS[mode]: INVALID_MODE_LABEL;
  switch (mode) {
    case 0:
      break;
    case 1:
      GL_value_set = (int) GL_config_data.target_temp; 
      GL_value_max = MAX_TARGET_TEMP;
      GL_value_min = MIN_TARGET_TEMP;
      break;
    case 2:
      GL_value_set = GL_paused;
      GL_value_max = 1;
      GL_value_min = 0;
      break;
    default:
      GL_config_mode = 0;
  }
  lcd.setCursor(0, 0);
  lcd.print(label);
  show_value(GL_value_set);
  if (mode) { Serial.print("Config: ");Serial.print(mode);Serial.print(" "); }
  Serial.print(label);
  // Serial.print("- ");Serial.println(GL_button_counter);
  Serial.println();
}

// Read encoder, update config
// ---------------------------------------------------------------------
// Read encoder. Sets GL_parameter_changed to true if one or more 
// configuration parameters have changed
// 
// Config mode is changed 0-1-2-3-4-0 using the config button ()
//    0-Normal operation 
//    1-Configure target temperature
//    2-pause on/off (1/0)
//

void read_encoder(int mode) {  

  int state_A = 0;             // state of encoder A pin

  //
  // Interrupts might change config mode to 0 in the middle of
  // the operation, so we don't use the global mode
  
  GL_value_old = GL_value_set;
  byte direction_cw = false; // is encoder rotating clockwise?
  byte old_direction_cw = direction_cw;
  
  // Read the current state of PIN_ENCODER_A
  state_A = digitalRead(PIN_ENCODER_A);
  // If last and current state of PIN_ENCODER_A are different, then pulse occurred.
  // Value is updated according to the direction of rotation, except whein it
  // changes, to avoid spurious pulses
  while (state_A != GL_last_state_A) { 
    // If the PIN_ENCODER_B state is different than the PIN_ENCODER_A state then
    // the encoder is rotating CCW so decrement
    if (digitalRead(PIN_ENCODER_B) != state_A) {
      direction_cw = false;
      //  GL_value_set ++;
      GL_value_set += GL_knob_increment;
      // Serial.print("+ ");Serial.println(GL_knob_increment); 
    } else {
      direction_cw = true;
      // Encoder is rotating CW so increment
      // GL_value_set --;
      GL_value_set -= GL_knob_increment;
      // Serial.print("- ");Serial.println(GL_knob_increment); 
    }
    //->>Serial.println(GL_value_set);
    // Remember last PIN_ENCODER_A state
    GL_last_encoder_change = millis();
    GL_last_state_A = state_A;
    // Put in a slight delay to help debounce the reading
    delay(1);
    // Read again
    state_A = digitalRead(PIN_ENCODER_A);
    if (old_direction_cw != direction_cw) GL_knob_increment = 1;
    old_direction_cw = direction_cw;
  }
  if (GL_value_set != GL_value_old) {
    GL_parameter_changed = true;   // SIGNAL SOMETHING HAS CHANGED
    if (GL_value_set < GL_value_min) { 
      GL_value_set = GL_value_min;
    } else  {
      if (GL_value_set > GL_value_max) { 
        GL_value_set = GL_value_max;
      }
    }
    switch (mode) {
      case 1: GL_config_data.target_temp        =  GL_value_set; break;
      case 2: GL_paused                         =  GL_value_set; break;
      default: break; 
    }
    show_value(GL_value_set);
    // debounce and increment speed if turning fast 
    unsigned long t_knob = millis() < GL_last_encoder_change;
    if (t_knob < 150) {
      if ((t_knob > 5) && (GL_knob_increment < MAX_KNOB_INCREMENT)) {
        GL_knob_increment++;
      }
    } else {
      GL_knob_increment = 1;
    }
  }
}


// Display configuration on terminal
// ---------------------------------------------------------------------
// 

void displayConfig() {
  Serial.print("Magic_number (Version ID).........: "); Serial.println(GL_config_data.magic_number    );
  Serial.print("Target temperature (Celsius)......: "); Serial.println(GL_config_data.target_temp     );
  Serial.print("Paused............................: "); Serial.println(GL_paused);
  return;
}
