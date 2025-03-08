/* ---================================YAORP32 v0.2 ==================================---
Software to drive Wattle Labs YAORP32 (Yet Another Open Reflow Plate) based on ESP32 S3
and commonly available aluminium 400W PTC heating plates from AliExpress. Temperature is
determined using either a K type thermocouple and read using a MAX6675 OR an MLX90614
Infrared thermometer. The PTC is driven by an SSR using PWM and controlled by a PID.

Revision History:

v0.1 - Initial release for testing and circuit board design purposes. Future improvements
may implement selectable reflow curves. For now the reflow temp can be changed in the UI and
the single curve in the code using "temp_preheat" variable.

v0.2 - Implemented and tested MLX90614 infrared thermometer. Testing shows the MLX90614
works better with the PID as it is more responsive to the temperature response of the PTC.
Physical spacing from PTC is important. Found that with a 20mm gap between, the MLX reached
70degC. Spaced to 40mm it only reached 30degC (ambient 25degC).
*/

/* --- To do ---
1. Add confirmation sound(s) when each phase completes. Maybe a pip to represent each phase
2. Add a safety timer that will shutdown the PID/Hotplate if the elapsed time exceeds a value
3. Add a safety check for thermocouple/infrared thermometer to ensure they are connected before
    allowing attempted operation.
4. Add a power bar graph to the OLED display when the SSR is running.
*/

// Native libraries
#include <Arduino.h>
#include <Wire.h>

#include "driver/ledc.h"  // ESP32 library for hardware timers (for LED flashing, and SSR PWM)

// Third party libraries
#include <U8g2lib.h>           // Graphics library for OLED screen - https://github.com/olikraus/u8g2
#include "max6675.h"           // MAX6675 thermocouple library - https://github.com/adafruit/MAX6675-library
#include <DFRobot_MLX90614.h>  // MLX90614 Infrared Thermometer library - https://github.com/DFRobot/DFRobot_MLX90614
#include <AutoPID.h>           // PID libary - https://ryand.io/AutoPID/
#include "avdweb_Switch.h"     // Switch library for debounce and hold function - https://github.com/avdwebLibraries/avdweb_Switch
#include <Rotary.h>            // Rotary encoder library

// UI image resources
#include "logos.h"

// I2C GPIO connections for OLED display (SSD1306)
#define PIN_I2C_SDA 8
#define PIN_I2C_SCL 9

// Ancillary GPIO connections for SSR, Buzzer, indicator LEDs
#define PIN_SSR 17
#define PIN_BUZZER 18
#define PIN_FAN 11 // not used at this point
#define PIN_LED_SAFE 16
#define PIN_LED_WARN 15

// SPI GPIO connections for MAX6675 Thermocouple Converter
#define PIN_THERMO_SDO 12
#define PIN_THERMO_CS 13
#define PIN_THERMO_SCK 14

// Rotary encoder GPIO connections
#define PIN_ROTARY_CLK 47  // rotary encoder CLK pin
#define PIN_ROTARY_SIG 48  // rotary encoder SIG pin
#define PIN_ROTARY_BUT 21  // press and hold button > 1s to "select"

// PID settings and gains
#define OUTPUT_MIN 0
#define OUTPUT_MAX 255
#define KP 2
#define KI .0025
#define KD 9

/* Try these :)
#define KP .12
#define KI .0003
#define KD 0
*/

#define TEMP_READ_DELAY 250    // Delay in ms to take temperature readings. Max6675 allows only every ~250ms
#define SCREEN_DRAW_DELAY 500  // Delay in ms to update the OLED display.

// Variables /////////////////////////////////////////////////////////////////
unsigned long lastTempUpdate;    //tracks time of last temp update
unsigned long lastScreenUpdate;  //tracks time of last screen update
unsigned long lastTimerUpdate;   // tracks timer of last timer update

int timerMode;  // used to track how long to spend in each mode once temperature is reached by the PID

double temperature, setTemp, outputVal;
double temp_preheat = 130;
double temp_reflow = 165;
int mode;  // 0 = Idle, 1 = Preheat, 2 = Soak, 3 = Reflow
int screen;
char *modeDescription = "";

bool boolAtSetPoint = false;
/////////////////////////////////////////////////////////////////////////////

// Instansiate SSD1306 display object
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, PIN_I2C_SCL, PIN_I2C_SDA, U8X8_PIN_NONE);

// Instantiate MAX6675 thermocouple object
MAX6675 thermocouple(PIN_THERMO_SCK, PIN_THERMO_CS, PIN_THERMO_SDO);

// Instantiate MLX90614 InfraRed thermometer object
DFRobot_MLX90614_I2C mlx(0x5A, &Wire);

// Instantiate AutoPID object
// input/output variables passed by reference, so they are updated automatically
AutoPID myPID(&temperature, &setTemp, &outputVal, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);

// Instantiate rotary encoder rotation detection/debounce object - library "Rotary"
Rotary rotary = Rotary(PIN_ROTARY_CLK, PIN_ROTARY_SIG);

// Instantiate rotary encoder switch debounce object - library "AVDWeb_Switch"
//Switch pushButton = Switch(PIN_ROTARY_BUT, INPUT_PULLUP, LOW, 1);
Switch pushButton = Switch(PIN_ROTARY_BUT, INPUT, LOW);

void setup() {
  Serial.begin(115200);
  Serial.println(F("Debug started..."));

  // Setup ledc channels for PWM control of SSR and sound output
  // Note the channel set-up was required to ensure SSR PWM freq. = 1kHz.
  ledcAttachChannel(PIN_SSR, /*PWM_FREQUENCY*/ 1000, /*PWM_RESOUTION */ 8, /*CHANNEL*/ 7);  // 1000Hz needed to drive the plate to full temperatures!
  ledcAttachChannel(PIN_BUZZER, /*PWM_FREQUENCY=*/1000, /*PWM_RESOUTION=*/8, /*CHANNEL*/ 0);

  // Setup temperature warning and safe temperature LEDs
  pinMode(PIN_LED_SAFE, OUTPUT);
  pinMode(PIN_LED_WARN, OUTPUT);

  // Initialize I2C
  //Wire.setPins(PIN_I2C_SCL, PIN_I2C_SDA);
  Wire.setClock(100000);

  // Setup PID
  myPID.setBangBang(30);               // Bang bang set to > 30 below setpoint (0 to turn off)
  myPID.setTimeStep(TEMP_READ_DELAY);  // set PID update freq. to same as temperature sensor

  // Ensure status LEDs reflect startup temperature!
  getTemperature();

  updateLEDs();  // Update LEDs based on plate temperature

  ui_boot();  // Display boot screen

  disableSSR();  // Ensure the SSR is turned off at boot

  mode = 0;    // Start in idle mode
  screen = 0;  // Start in temp monitoring screen
}

void loop() {
  processEncoder();  // Process the rotary encoder

  getTemperature();  // Get temperature from thermocouple/IR thermometer

  updateLEDs();  // Update LEDs based on plate temperature

  updateUI();  // Update OLED display

  processModes();  // Pre-heat, soak, reflow...
}
