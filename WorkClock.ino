#include <Adafruit_EEPROM_I2C.h>
#include <Adafruit_FRAM_I2C.h>

#include <Arduino.h>
#include <DS3231.h>
#include <HT1621.h>
#include <Wire.h>
#include <PinChangeInterrupt.h>

#include <avr/sleep.h>
#include <avr/power.h>

// #define SerialPrintF(fmt, ...)        \
//   do {                                \
//     char buf[64];                     \
//     snprintf(buf, sizeof(buf), fmt, __VA_ARGS__); \
//     Serial.println(buf);              \
//   } while (0)


// void SerialPrintFD(const char *fmt, double d1) {
//   char numBuf[16];
//   dtostrf(d1, 0, 2, numBuf); // width=0, precision=2
//   char buf[64];
//   snprintf(buf, sizeof(buf), fmt, numBuf);
//   Serial.println(buf);
// }

const uint8_t CS_PIN  = 10;
const uint8_t WR_PIN  = 13;
const uint8_t DAT_PIN = 11;
const uint8_t LED_PIN = 12;
const uint8_t SQW_PIN =  2;

const uint8_t START_STOP_BTN_PIN = 5;
const uint8_t CLEAR_BTN_PIN =  4;

const uint8_t STATE_FRAM_ADDRESS = 0x0;

const uint16_t CHECKPOINT_SAVE_SECONDS_INTERVAL = 600;  // 600s = 10 min

HT1621 lcd;
Adafruit_FRAM_I2C fram = Adafruit_FRAM_I2C();
DS3231 rtc;
RTClib rtcLib;

volatile bool wakeFlag = false;
volatile bool startStopPressed = false;
volatile bool clearPressed = false;

typedef struct {
    int hours;
    int minutes;
} TimeHolder;

struct State {
  uint32_t accumulated_stored_seconds = 0;
  uint32_t current_running_seconds = 0;
  uint32_t current_starting_seconds = 0;
  uint32_t last_divisible_second = 0;
  uint32_t last_checkpoint_seconds = 0;
  bool running_indicator_showing = false;
  bool running = false;  
};

State state = {};
State last_saved_state = {};

static void setup_lcd() {
  Serial.println("setup_lcd");
  lcd.begin(CS_PIN, WR_PIN, DAT_PIN, LED_PIN);
  lcd.noBacklight();
  lcd.clear();  
}

static void setup_rtc() {
  Serial.println("setup_rtc");
  Wire.begin();
  rtc.turnOffAlarm(1);
  rtc.turnOffAlarm(2);

  pinMode(SQW_PIN, INPUT_PULLUP);
}

static void setup_buttons() {
  Serial.println("setup_buttons");
  pinMode(START_STOP_BTN_PIN, INPUT_PULLUP);
  pinMode(CLEAR_BTN_PIN, INPUT_PULLUP);
  attachPCINT(digitalPinToPCINT(START_STOP_BTN_PIN), start_stop_button_triggered, FALLING);
  attachPCINT(digitalPinToPCINT(CLEAR_BTN_PIN), clear_button_triggered, FALLING);
}

static void save_state() {
  Serial.println("save_state");
  if (memcmp(&state, &last_saved_state, sizeof(State)) != 0) {
    fram.write(STATE_FRAM_ADDRESS, (uint8_t*)&state, sizeof(State));
    last_saved_state = state;
    Serial.println("State was saved");
  }
}

static void load_state() {
  Serial.println("load_state");
  fram.begin();
  fram.read(STATE_FRAM_ADDRESS, (uint8_t*)&state, sizeof(state));
  last_saved_state = state;
}

static void update_running_time() {
  Serial.println("update_running_time");
  DateTime now = rtcLib.now();
  const uint32_t epoch_now = now.unixtime();
  state.current_running_seconds = epoch_now - state.current_starting_seconds;
}

static uint32_t get_total_seconds() {
  uint32_t ret = state.accumulated_stored_seconds + 
    state.current_running_seconds;

    // SerialPrintF("get_total_seconds:'%d'", ret);
  return ret;
}

static TimeHolder get_total_time() {
  const uint32_t total_seconds = get_total_seconds();
  TimeHolder t;
  static uint32_t seconds_per_hour = 3600;
  t.hours = total_seconds / seconds_per_hour;
  const uint32_t remaining_seconds = total_seconds % seconds_per_hour;
  t.minutes = remaining_seconds / 60;

  // SerialPrintFD("get_total_time, H:'%d'", t.hours);
  // SerialPrintFD("get_total_time, M:'%d'", t.minutes);

  return t;
}

static double convert_time_to_decimal(TimeHolder t) {
  double ret = t.hours + (t.minutes / 100.00);
  // SerialPrintFD("convert_time_to_decimal, '%s'", ret);
  return ret;
}

static double convert_time_to_integer(TimeHolder t) {
  double ret = (t.hours * 100.00) + t.minutes;
  // SerialPrintFD("convert_time_to_integer, '%s'", ret);
  return ret;
}

static void get_time_string(TimeHolder t, const char* buf) {

  if (state.running_indicator_showing) {

  } else {

  }
  double ret = (t.hours * 100.00) + t.minutes;
  // SerialPrintFD("convert_time_to_integer, '%s'", ret);
  return ret;
}

static void update_screen() {
  Serial.println("update_screen");
  TimeHolder t = get_total_time();
  char buf[8];  // Enough for HH.MM\0 or HHMM\0
  if (state.running_indicator_showing) {
    snprintf(buf, sizeof(buf), "%02d-%02d", t.hours, t.minutes);
  } else {
    snprintf(buf, sizeof(buf), "%02d %02d", t.hours, t.minutes);
  }
  lcd.print(buf, true);
}

// static void update_screen() {
//   Serial.println("update_screen");
//   TimeHolder t = get_total_time();
//   if (state.running_indicator_showing) {
//     double decimal_time = convert_time_to_decimal(t);
//     lcd.print(decimal_time, 2);
//   } else {
//     double integer_time = convert_time_to_integer(t);
//     lcd.print(integer_time, 0);
//   }  
// }

static void check_checkpoint_save() {
  Serial.println("check_checkpoint_save");
  uint32_t elapsed_seconds_since_checkpoint = state.current_running_seconds - state.last_checkpoint_seconds;
  if (elapsed_seconds_since_checkpoint >= CHECKPOINT_SAVE_SECONDS_INTERVAL) {
    state.last_checkpoint_seconds = state.current_running_seconds;
    save_state();
  }
}

static void on_minutes_tick() {
  Serial.println("on_minutes_tick");
  if (state.running) {
    update_screen();
    check_checkpoint_save();
  }
}

static void on_seconds_tick() {
  Serial.println("on_seconds_tick");
  if (state.running) {
    state.running_indicator_showing = !state.running_indicator_showing;
    update_screen();
  }
}

static void on_clear_pressed() {
  Serial.println("on_clear_pressed");
  state.running = false;
  state.running_indicator_showing = true;
  state.accumulated_stored_seconds = 0;
  state.current_running_seconds = 0;
  state.current_starting_seconds = 0;
  state.last_divisible_second = 0;
  state.last_checkpoint_seconds = 0;
  tear_down_interupts();
  update_screen();
  save_state();
}

static void stop() {
  Serial.println("stop");
  state.running_indicator_showing = true;
  state.accumulated_stored_seconds += state.current_running_seconds;
  state.current_running_seconds = 0;
  state.last_checkpoint_seconds = 0;
  save_state();
  tear_down_interupts();
}

static void start() {
  Serial.println("start");
  DateTime dt = rtcLib.now();
  state.current_starting_seconds = dt.unixtime();
  state.current_running_seconds = 0;
  state.last_checkpoint_seconds = state.current_running_seconds;
  save_state();
  setup_interupts();
}


static void on_start_stop_pressed() {
  Serial.println("on_start_stop_pressed");
  state.running = !state.running;
  if(!state.running) {
    stop();
  } else {
    start();
  }
  update_running_time();
}

static bool a_minute_has_passed() {
  Serial.println("a_minute_has_passed");
  uint32_t current_second = state.current_running_seconds;
  bool ret = false;
  if(state.last_divisible_second !=current_second &&
    (current_second % 60) == 0) {
    state.last_divisible_second = current_second;
    ret = true;
  }

  // SerialPrintF("a_minute_has_passed'%d'", ret);

  return ret;
}

static void alarm_triggered() {
  Serial.println("alarm_triggered");
  wakeFlag = true;
}

void start_stop_button_triggered() {
  Serial.println("start_stop_button_triggered");
  startStopPressed = true;
  wakeFlag = true;
}

void clear_button_triggered() {
  Serial.println("clear_button_triggered");
  clearPressed = true;
  wakeFlag = true;
}

static void update() {
  Serial.println("update");
  update_running_time();
  if(a_minute_has_passed()) {
    on_minutes_tick();
  } else {
    on_seconds_tick();
  }
}

static void setup_interupts() {
  Serial.println("setup_interupts");
  byte AlarmBits = 0b00001111; // A1M1=1, A1M2=1, A1M3=1, A1M4=1
  rtc.setA1Time(0, 0, 0, 0, AlarmBits, false, false, false);
  rtc.turnOnAlarm(1);
  attachInterrupt(digitalPinToInterrupt(SQW_PIN), alarm_triggered, FALLING);
}

static void tear_down_interupts() {
  Serial.println("tear_down_interupts");
  detachInterrupt(digitalPinToInterrupt(SQW_PIN));
}

void goToSleep() {
  Serial.println("goToSleep");
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_cpu();
  sleep_disable();
}

void setup() {
  Serial.println("setup");
  Serial.begin(115200);
  setup_lcd();
  setup_rtc();
  setup_buttons();
  load_state();
  update_screen();
  if (state.running) {
    setup_interupts();
  }
}

void loop() {
  Serial.println("loop");
  if (wakeFlag) {
    wakeFlag = false;
    Serial.println("wakeFlag passed");
    if (startStopPressed) {
      startStopPressed = false;
      on_start_stop_pressed();
      Serial.println("Start/Stop Pressed");
    }
    if (clearPressed) {
      clearPressed = false;
      on_clear_pressed();
      Serial.println("Clear Pressed");
    }
    update();
    rtc.checkIfAlarm(1);
  }
  goToSleep();
}
