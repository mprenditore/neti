/* Garage Door Advanced System
*
*  GDC can control different sensors and motors.
*
*  Functionalities:
*    * Motor control via relays
*    * Motor state based octions
*    * Input configuration (Normal Open/Closed)
*    * Buzzer feedback
*    * Forced close while on waiting with double trigger of run_pin
*    * Stop and reopen in case safety is triggered
*    * Deactivate Lock via relay before open the door
*
*  created 2020
*  by Stefano Stella <mprenditore@gmail.com>
*
*
*/
#include <Arduino.h>
#include "config.h"
#include "pitches.h"

#ifdef CHECK_ALL
  #define LOOP(...) check_loop()     //enable check_loop
#else
  #define LOOP(...) normal_loop()    //enable normal_loop
#endif

#ifdef DEBUG
  #define DPRINT(...)   Serial.print(__VA_ARGS__)     //DPRINT is a macro, debug print
  #define DPRINTLN(...) Serial.println(__VA_ARGS__)   //DPRINTLN is a macro, debug print with new line
#else
  #define DPRINT(...)     //now defines a blank line
  #define DPRINTLN(...)   //now defines a blank line
#endif

// main variables (don't touch)
bool standby = true;                        // is the system fully stopped?
bool waiting = false;                       // is the port waiting before closing?
bool running = false;                       // is the port running?
int direction_state = OPEN;                 // OPEN / CLOSE
int motor_state = OFF;                      // ON / OFF
int buzzer_state = OFF;                      // ON / OFF
int status = OFF_OPEN;
unsigned long elapsed_time = 0;
unsigned long door_waiting_prev_millis = 0; // timeout counter for waiting time
unsigned long door_timeout_prev_millis = 0; // timeout counter for timeout time
unsigned long door_timer = 0;
unsigned long force_close_prev_millis = 0; // timeout counter for force-close time
unsigned long force_stop_prev_millis = 0; // timeout counter for force-stop time
unsigned long buzzer_on_prev_millis = 0; // timeout counter for buzzer-on time
unsigned long buzzer_off_prev_millis = 0; // timeout counter for buzzer-off time
unsigned long buzzer_current_off_time = 0;

// melodies m(elody)_[n(ote)|d(uration)|t(empo)]_<action>
int m_n_alert[] = {NOTE_B6, NOTE_B5, NOTE_G1};
int m_d_alert[] = {16, 16, 8};
int m_t_alert = 200;
int m_n_restart[] = {NOTE_B5, NOTE_B6, NOTE_FS6, NOTE_B4, NOTE_D8};
int m_d_restart[] = {16, 10, 18, 8, 8};
int m_t_restart = 200;
int m_n_stop[] = {NOTE_C6, NOTE_C5, NOTE_C3};
int m_d_stop[] = {8, 16, 4};
int m_t_stop = 120;
int m_n_boot[] = {NOTE_G1, NOTE_B5, NOTE_A7};
int m_d_boot[] = {16, 16, 8};
int m_t_boot = 120;

int get_status(){
  status = ON_OPEN;
  if(motor_state == ON){
    if(direction_state == CLOSE){
      status = ON_CLOSE;
    }
  }else if(direction_state == OPEN){
    status = OFF_OPEN;
  }else{
    status = OFF_CLOSE;
  }
  return status;
}


void play_tune(int notes[], int durations[], int BPM){
  // iterate over the notes of the tune:
  for (int this_note = 0; this_note < tune_size; this_note++) {

    // For details on calculating the note duration using the tempo and the note type,
    // see http://bradthemad.org/guitar/tempo_explanation.php.
    // A quarter note at 60 BPM lasts exactly one second and at 120 BPM - half a second.

    int note_duration = (int)((1000 * (60 * 4 / BPM)) / durations[this_note] + 0.);
    tone(buzzer_pin, notes[this_note],note_duration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 20% seems to work well:
    int pause_between_notes = note_duration * 1.20;
    delay(pause_between_notes);
    // stop the tone playing:
    noTone(buzzer_pin);
  }
}

void buz(int freq, int on_t, int off_t){
  tone(buzzer_pin, freq);
  delay(on_t);
  noTone(buzzer_pin);
  delay(off_t);
}

bool active(int pin_number, bool control_state=NO){
  int state = digitalRead(pin_number);
  delay(15);
  if(state == digitalRead(pin_number) &&
     state == control_state){
    return true;
  }
  return false;
}

bool safety_activated(){
  if (active(safety_pin, safety_conf)){
    return true;
  }
  return false;
}

bool run_activated(){
  if (active(run_pin, run_conf)){
    buz(2000,50,0);
    return true;
  }else{
    return false;
  }
}

void action_on_motor(int motor = ON, int direction = OPEN,
                     int dir_delay = def_dir_delay, int motor_delay = def_motor_delay){
  DPRINT("MOTOR ACTION direction(0=CLOSE / 1=OPEN): ");
  DPRINT(direction);
  DPRINT(" - motor (0=ON / 1=OFF): ");
  DPRINTLN(motor);

  direction_state = direction;
  motor_state = motor;
  digitalWrite(direction_pin, direction_state);
  delay(dir_delay);
  digitalWrite(motor_pin, motor_state);
  delay(motor_delay);
  delay(def_lock_delay);
  digitalWrite(lock_pin, LOCK);
}

void stop_gate(bool full_stop=false, int motor_delay=def_motor_delay){
  DPRINT("Gate: stop - standby: ");
  DPRINTLN(full_stop);
  standby = full_stop;
  running = false;
  waiting = false;
  noTone(buzzer_pin);
  buzzer_state = OFF;
  action_on_motor(OFF, direction_state, 0, motor_delay);
  if(motor_delay < def_dir_delay){
    delay(def_dir_delay - motor_delay);
  }
  action_on_motor(OFF, OPEN, 0, 0);
}

void move_gate(int direction, int dir_delay = def_dir_delay,
               int motor_delay = def_motor_delay){
  if(running == true){
    DPRINTLN("RUNNING...");
    if(millis() - door_timeout_prev_millis >= door_timeout_time){
      DPRINT("Running timeout elapsed: ");
      DPRINTLN(millis() - door_timeout_prev_millis);
      stop_gate(true);
      running = false;
    }else{
      if(run_activated() == true){
        if(force_stop_prev_millis == 0){
          force_stop_prev_millis = millis();
        }else{
          elapsed_time = millis() - force_stop_prev_millis;
          if(elapsed_time >=500 && elapsed_time <= force_stop_time){
            DPRINT("Force stop - double RUN detected in: ");
            DPRINTLN(elapsed_time);
            force_stop_prev_millis = 0;
            waiting = false;
            stop_gate(true);
          }else{
            force_stop_prev_millis = millis();
          }
        }
      }
    }
  }else{
    if(direction == CLOSE && safety_activated() == true){
      DPRINTLN("Gate: move - skipping because security is activated");
      stop_gate(true);
      play_tune(m_n_alert, m_d_alert, m_t_alert);
      return;
    }
    DPRINT("Gate: move - direction: ");
    DPRINTLN(direction);
    DPRINTLN("RESET RUNNING timer");
    standby = false;
    running = true;
    door_timeout_prev_millis = millis();
    action_on_motor(ON, direction, dir_delay, motor_delay);
  }
}

void waiting_buzzer(int buzzer_f, unsigned long buzzer_on_t,
                    unsigned long buzzer_off_t){
  if(buzzer_state == OFF){
    if(millis() - buzzer_off_prev_millis >= buzzer_off_t){
      tone(buzzer_pin, buzzer_f);
      buzzer_on_prev_millis = millis();
      buzzer_state = ON;
    }
  }else{
    if(millis() - buzzer_on_prev_millis >= buzzer_on_t){
      noTone(buzzer_pin);
      buzzer_off_prev_millis = millis();
      buzzer_state = OFF;
    }
  }
}

void wait_gate(){
  if(standby == false){
    if(waiting == true){
      door_timer = ((millis() - door_waiting_prev_millis) - door_waiting_time) * -1;
      if (door_timer <= buzzer_speedup_timeout){
        buzzer_current_off_time = door_timer / 10;
        DPRINT("speed_up buzzer...");
        DPRINTLN(buzzer_current_off_time);
      }else{
        buzzer_current_off_time = buzzer_off_time;
      }
      waiting_buzzer(wait_buzzer_freq, buzzer_on_time, buzzer_current_off_time);
      DPRINTLN("WAITING...");
      if(run_activated() == true){
        if(force_close_prev_millis == 0){
          force_close_prev_millis = millis();
        }else{
          elapsed_time = millis() - force_close_prev_millis;
          if(elapsed_time >=500 && elapsed_time <= force_close_time){
            DPRINT("Force close - double RUN detected in: ");
            DPRINTLN(elapsed_time);
            force_close_prev_millis = 0;
            waiting = false;
            noTone(buzzer_pin);
            buzzer_state = OFF;
            move_gate(CLOSE, def_dir_delay);
          }else{
            force_close_prev_millis = millis();
          }
        }
      }
      if(millis() - door_waiting_prev_millis >= door_waiting_time){
        DPRINT("Waiting timeout elapsed: ");
        DPRINTLN(millis() - door_waiting_prev_millis);
        force_close_prev_millis = 0;
        waiting = false;
        move_gate(CLOSE, def_dir_delay, 100);
      }
    }else{
      DPRINTLN("Gate: wait start");
      buz(2000,50,0);
      stop_gate(false, 0);
      waiting = true;
      door_waiting_prev_millis = millis();
    }
  }
}

void restart_gate(){
  DPRINTLN("Gate: restart");
  stop_gate(false, restart_dir_delay);
  play_tune(m_n_restart, m_d_restart, m_t_restart);
  move_gate(OPEN, def_dir_delay, 100);
}

bool check_end_o(){
  get_status();
  if (active(end_o_pin, end_o_conf)){
    DPRINTLN("PRESSED: end_o");
    switch (status){
      case ON_OPEN:
      case OFF_OPEN:
      case OFF_CLOSE:  // should never happen
        wait_gate();
        break;
      default:
        break;
    }
    return true;
  }else{
    waiting = false;
    return false;
  }
}

bool check_end_c(){
  get_status();
  if (active(end_c_pin, end_c_conf)){
    DPRINTLN("PRESSED: enc_c");
    switch (status){
      case ON_CLOSE:
      case OFF_CLOSE:
        buz(200,50,0);
        stop_gate(true);
        break;
      default:
        break;
    }
    return true;
  }
  return false;
}

bool check_security(){
  get_status();
  if (safety_activated()){
    DPRINTLN("ACTIVATED: photocecell");
    switch (status){
      case ON_CLOSE:
        restart_gate();
        break;
      default:
        break;
    }
    return true;
  }
  return false;
}

bool check_stop(){
  get_status();
  if (active(stop_pin, stop_conf)){
    DPRINTLN("PRESSED: stop");
    stop_gate(true);
    buz(100,50,0);
    return true;
  }
  return false;
}

bool check_run(){
  get_status();
  if (run_activated() == true){
    standby = false;
    DPRINTLN("PRESSED: run");
    switch (status){
      case ON_OPEN:
        move_gate(OPEN);
        break;
      case OFF_OPEN:
        if (check_end_o() == false &&
            check_stop() == false){
          digitalWrite(lock_pin, UNLOCK);
          move_gate(OPEN, def_dir_delay);
          digitalWrite(lock_pin, LOCK);
        }
        break;
      case ON_CLOSE:
        restart_gate();  // to check with dad
        break;
      case OFF_CLOSE:
        if(check_stop() == false){
          restart_gate();
        }
        break;
      default:
        break;
    }
    return true;
  }
  return false;
}

void setup(){

#ifdef DEBUG
  Serial.begin(BOUND_RATE);
#endif
  DPRINT("GDS is starting ");
  // set the digital pin as output:
  pinMode(motor_pin, OUTPUT);
  pinMode(direction_pin, OUTPUT);
  pinMode(lock_pin, OUTPUT);
  DPRINT(".");
  // init relays
  digitalWrite(motor_pin, OFF);
  digitalWrite(direction_pin, OPEN);
  digitalWrite(lock_pin, LOCK);
  DPRINT(".");
  // set input in pullup mode
  pinMode(end_o_pin, INPUT_PULLUP);
  pinMode(end_c_pin, INPUT_PULLUP);
  pinMode(safety_pin, INPUT_PULLUP);
  pinMode(stop_pin, INPUT_PULLUP);
  pinMode(run_pin, INPUT_PULLUP);
  DPRINTLN(". Started!");
  play_tune(m_n_boot, m_d_boot, m_t_boot);
}

void check_loop(){
  if (active(end_o_pin, end_o_conf)){
    DPRINTLN("PRESSED: end_o");
  }
  if (active(end_c_pin, end_c_conf)){
    DPRINTLN("PRESSED: end_c");
  }
  if (active(stop_pin, stop_conf)){
    DPRINTLN("PRESSED: stop");
  }
  if (active(run_pin, run_conf)){
    DPRINTLN("PRESSED: run");
  }
  if (active(safety_pin, safety_conf)){
    DPRINTLN("PRESSED: photo");
  }
  DPRINTLN("____________________");
  delay(2000);

}

void normal_loop(){
  if(running == true){
    move_gate(direction_state);
  }
  if (standby == false){
    check_stop();
    check_end_o();
    check_end_c();
    check_security();
  }
  check_run();
}

void loop(){
  LOOP();
}