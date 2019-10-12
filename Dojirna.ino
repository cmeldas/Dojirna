/*Dojirna

  Ovladani boileru - podle èasu HDO
  Spinani ohrevu trubek - ochrana proti zamrznuti
  Tlacitko pro zapnuti ohrevu boileru mimo stanoveny cas

  LED:
  > 

   Periferie:
   RTC
   dallas DS18..
   Rele 4x


   Pinout:
   Rele:
    K1 - 17
    K2 - 16
    K3 - 4
    K4 - 0

   Dalas - 13

   Tlacitko:
   teleny- 14

   Status LED:
   zluty kabel - 12



*/

// Include the libraries that we need
#include <OneWire.h>
#include <DallasTemperature.h>

//#include <SerialCommand.h>
#include <JC_Button.h>

#define LED_P 14 //27 26
#define DALAS_PIN 13
#define BTN_P 12
#define BOILER_P 16
#define PIPE_HEATER_P 4


Button BTN(BTN_P);

#define PIPE_TEMP_ON 1
#define PIPE_TEMP_OFF 3

// Dallas
#define TEMPERATURE_PRECISION 9
OneWire oneWire(DALAS_PIN);
DallasTemperature sensors(&oneWire);

DeviceAddress boiler =       {0x28, 0xFF, 0x64, 0x1D, 0x8B, 0x8B, 0xEE, 0xBA};
DeviceAddress pipe_ground =  {0x28, 0xFF, 0x77, 0x01, 0x40, 0x17, 0x03, 0xAA};
DeviceAddress pipe_inside =  {0x28, 0xFF, 0x9E, 0x11, 0x40, 0x17, 0x03, 0xBB};
DeviceAddress pipe_outside = {0x28, 0xFF, 0xBC, 0x41, 0x01, 0x70, 0x03, 0x18};

double t_boiler, t_pipe_inside, t_pipe_outside, t_pipe_ground = 0.0;
bool blik_p, slow_p, fast_p = 0; //for blinking

//SerialCommand SCmd; // The SerialCommand object

int boiler_mode = 0;

bool temperature_read(void);
void print_temperatures(void);
void unrecognized(void);
void pulse(void);
double pipe_min_temperature(void);
void pipe_heater_control(void);
void boiler_control(void);
void led(void);

void led(void)
{
  if (!digitalRead(PIPE_HEATER_P))
  {

    digitalWrite(LED_P, fast_p ? LOW : HIGH);
  }
  else if (!digitalRead(BOILER_P))
  {
    digitalWrite(LED_P, slow_p ? LOW : HIGH);
  }
  else
  {
    digitalWrite(LED_P, HIGH);
  }
}

void pipe_heater_control(void)
{

  digitalWrite(PIPE_HEATER_P, HIGH);

  if (pipe_min_temperature() < PIPE_TEMP_ON)
  {
    // ON
  }
  if (pipe_min_temperature() > PIPE_TEMP_OFF)
  {
    // OFF
  }
}

void boiler_control(void)
{
  digitalWrite(BOILER_P, LOW);
}

bool temperature_read(void)
{
  //read all temperatures and store it in global variable
  const int conversion_delay = 2000;
  static unsigned long lastT = 0;

  if ((millis() > lastT + conversion_delay))
  {
    t_boiler = sensors.getTempC(boiler);
    t_pipe_ground = sensors.getTempC(pipe_ground);
    t_pipe_inside = sensors.getTempC(pipe_inside);
    t_pipe_outside = sensors.getTempC(pipe_outside);

    sensors.requestTemperatures();
    lastT = millis();

    return true;
  }
  return false;
}

void pulse(void)
{
// periodically blink, use int LEDs blinking
#define SLOW_PERIOD 700
#define FAST_PERIOD 100
#define BLIK_DELAY 6000
  static unsigned long bl_period = BLIK_DELAY;
  static unsigned long lastT_s = 0;
  static unsigned long lastT_f = 0;
  static unsigned long lastT_b = 0;
  unsigned long now = millis();

  if (now > bl_period + lastT_b)
  {

    blik_p = !blik_p;
    lastT_b = now;
    if (blik_p)
    {
      bl_period = FAST_PERIOD;
    }
    else
    {
      bl_period = BLIK_DELAY;
    }
  }

  if (now > SLOW_PERIOD + lastT_s)
  {
    slow_p = !slow_p;
    lastT_s = now;
  }

  if (now > FAST_PERIOD + lastT_f)
  {
    fast_p = !fast_p;
    lastT_f = now;
  }
}

double pipe_min_temperature(void)
{
  double min_t;
  min_t = min(t_pipe_outside, t_pipe_inside);
  min_t = min(min_t, t_pipe_ground);
}

void print_temperatures(void)
{
  Serial.println();
  Serial.println("Temperature print:");
  Serial.print("> Boiler: ");
  Serial.println(t_boiler, 1);
  Serial.print("> pipe_ground: ");
  Serial.println(t_pipe_ground, 1);
  Serial.print("> pipe_inside: ");
  Serial.println(t_pipe_inside, 1);
  Serial.print("> pipe_outside: ");
  Serial.println(t_pipe_outside, 1);
  Serial.println();
}

void unrecognized(void)
{
  Serial.println("What?");
  Serial.println("> t - print_temperatures");
}

//--------------SETUP--------------

void setup()
{
  // start serial port
  Serial.begin(115200);
  Serial.print("Kotel:  ");
  Serial.println(__DATE__);

  //--------------------- comand register
  //SCmd.addCommand("t", print_temperatures);
  //SCmd.addDefaultHandler(unrecognized);

  // Dalas start
  sensors.begin();
  sensors.setResolution(boiler, TEMPERATURE_PRECISION);
  sensors.setResolution(pipe_ground, TEMPERATURE_PRECISION);
  sensors.setResolution(pipe_inside, TEMPERATURE_PRECISION);
  sensors.setResolution(pipe_outside, TEMPERATURE_PRECISION);
  sensors.setWaitForConversion(true);
  sensors.requestTemperatures();
  temperature_read();
  sensors.setWaitForConversion(false);

  //-------- pin mode
  pinMode(BOILER_P, OUTPUT);
  pinMode(PIPE_HEATER_P, OUTPUT);
  pinMode(LED_P, OUTPUT);

  // Get dalas count
  Serial.print("Found  ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" Dalas devices.");

  while (!temperature_read())
  {
  } //wait until temp sensor updated
  print_temperatures();

  BTN.begin();
}

void loop()
{
  temperature_read();
  BTN.read();
  pulse();
  pipe_heater_control();
  boiler_control();
  led();
}
