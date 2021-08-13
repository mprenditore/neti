#include <Arduino.h>

// #define CHECK_ALL  //comment this lie to go into CheckAll mode.
                      //  useful to check that all connections work
// #define DEBUG   //comment this line to disable all Print
                //  DPRINT & DPRINTLN lines are defined as blank.

// #define WORKBENCH_TEST  // uncomment this line to have all pin configs
                      // as NO (Normally Open) for workbench testing

// #define ELECTRIC_LOCK  // comment this line if you don't have the control
                  // an external electric lock

// config constants won't change

const long BOUND_RATE = 921600;
const int NO = LOW;
const int NC = HIGH;
const int OPEN = HIGH;
const int CLOSE = LOW;
const int LOCK = HIGH;
const int UNLOCK = LOW;
const int ON = LOW;
const int OFF = HIGH;
const int ON_OPEN = 1;
const int ON_CLOSE = 2;
const int OFF_OPEN = 3;
const int OFF_CLOSE = 4;

// buzzer
const long wait_buzzer_freq = 1000;
const int tune_size = 6;

// def delays
const int def_dir_delay = 250;
const int def_motor_delay = 100;
const int def_lock_delay = 1000;
// custom delays
const int restart_dir_delay = 800;
const int restart_motor_delay = def_motor_delay;
const int wait_dir_delay = def_dir_delay;
const int wait_motor_delay = 800;

// set pin numbers
const int buzzer_pin = D8;
const int safety_pin = RX;
const int stop_pin = D5;
const int run_pin = D3;
const int end_c_pin = D7;
const int end_o_pin = D6;
const int direction_pin = D1;
const int motor_pin = D2;
const int lock_pin = D4;

// input configuration
#ifdef WORKBENCH_TEST
    const int safety_conf = NO;
    const int stop_conf = NO;
    const int run_conf = NO;
    const int end_c_conf = NO;
    const int end_o_conf = NO;
#else
    const int safety_conf = NC;
    const int stop_conf = NC;
    const int run_conf = NO;
    const int end_c_conf = NO;
    const int end_o_conf = NO;
#endif

// timeouts
const unsigned long door_waiting_time = 45000;               // milliseconds of door waiting time
const unsigned long door_timeout_time = 30000;               // milliseconds of door moving time
const unsigned long force_close_time = 3000;                 // milliseconds between double run command to force close
const unsigned long force_stop_time = 3000;                  // milliseconds between double run command to force stop
const unsigned long buzzer_on_time = 50;                     // milliseconds for waiting buzzer ON status
const unsigned long buzzer_off_time = 2000;                  // milliseconds for waiting buzzer OFF status
const unsigned long buzzer_speedup_timeout = 10000;          // milliseconds remaining from when reduce waiting bugger OFF status time
const unsigned long bouncing_timeout = 2000;                 // milliseconds for door bouncing detection

int get_interrupt_mode(int pin_config){
  if(pin_config == NO){
    return FALLING;
  }
  return RISING;
}