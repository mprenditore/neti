/* NETI Gatekeeper
*
*  NETI can control different sensors and motors.
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
#include "pitches.h"
#include "config.h"
#include "debug.h"

// main variables (don't touch)
bool standby = true;                        // is the system fully stopped?
bool waiting = false;                       // is the port waiting before closing?
bool running = false;                       // is the port running?
bool bounced = false;                       // have the port bounced?
bool stop_pressed = false;
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

// interrupt mode variables
int stop_interrupt_mode = get_interrupt_mode(stop_conf);
int safety_interrupt_mode = get_interrupt_mode(safety_conf);
int run_interrupt_mode = get_interrupt_mode(run_conf);
int end_c_interrupt_mode = get_interrupt_mode(end_c_conf);
int end_o_interrupt_mode = get_interrupt_mode(end_o_conf);

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
  DPLN("STATUS: " + String(status));
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

bool stop_activated(){
  if (active(stop_pin, stop_conf)){
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
  DPLN("MOTOR ACTION direction(0=CLOSE / 1=OPEN): " + String(direction) + " - motor (0=ON / 1=OFF): " + String(motor));
  direction_state = direction;
  motor_state = motor;
  digitalWrite(direction_pin, direction_state);
  delay(dir_delay);
  digitalWrite(motor_pin, motor_state);
  delay(motor_delay);
  get_status();
}

void stop_gate(bool full_stop=false, int dir_delay=def_dir_delay){
  DPLN("Gate: stop - standbyL " + String(full_stop));
  standby = full_stop;
  running = false;
  waiting = false;
  noTone(buzzer_pin);
  buzzer_state = OFF;
  action_on_motor(OFF, direction_state, 0, dir_delay);
  if(dir_delay < def_dir_delay){
    delay(def_dir_delay - dir_delay);
  }
  action_on_motor(OFF, OPEN, 0, 0);

  stop_pressed = false;
}

void check_stop(){
  if(stop_pressed or stop_activated()){
    buz(100,def_dir_delay,0);  // at least `def_dir_delay` to prevent a too fast change of direction after turn off
    stop_gate(true);
  }
}

void move_gate(int direction, int dir_delay = def_dir_delay,
               int motor_delay = def_motor_delay){
  if(running == true){
    check_stop();
    DPLN("RUNNING...");
    if(millis() - door_timeout_prev_millis >= door_timeout_time){
      DPLN("Running timeout elapsed: " + String(millis() - door_timeout_prev_millis));
      stop_gate(true);
      running = false;
    }else{
      if(run_activated() == true){
        if(force_stop_prev_millis == 0){
          force_stop_prev_millis = millis();
        }else{
          elapsed_time = millis() - force_stop_prev_millis;
          if(elapsed_time >=500 && elapsed_time <= force_stop_time){
            DPLN("Force stop - double RUN detected in: " + String(elapsed_time));
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
      DPLN("Gate: move - skipping because security is activated");
      stop_gate(true);
      play_tune(m_n_alert, m_d_alert, m_t_alert);
      return;
    }
    DPLN("Gate: move - direction: " + String(direction));
    DPLN("RESET RUNNING timer");
    standby = false;
    running = true;
    door_timeout_prev_millis = millis();
    action_on_motor(ON, direction, dir_delay, motor_delay);
  }
  bounced = false;
}

void waiting_buzzer(int buzzer_f, unsigned long buzzer_on_t,
                    unsigned long buzzer_off_t){
  if(buzzer_state == OFF){
    if(millis() - buzzer_off_prev_millis >= buzzer_off_t){
      tone(buzzer_pin, buzzer_f);
      buzzer_on_prev_millis = millis();
      buzzer_state = ON;
      DPLN("WAITING...");
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
  check_stop();
  if(standby == false){
    if(waiting == true){
      door_timer = ((millis() - door_waiting_prev_millis) - door_waiting_time) * -1;
      if (door_timer <= buzzer_speedup_timeout){
        buzzer_current_off_time = door_timer / 10;
      }else{
        buzzer_current_off_time = buzzer_off_time;
      }
      waiting_buzzer(wait_buzzer_freq, buzzer_on_time, buzzer_current_off_time);
      if(run_activated() == true){
        if(force_close_prev_millis == 0){
          force_close_prev_millis = millis();
        }else{
          elapsed_time = millis() - force_close_prev_millis;
          if(elapsed_time >=500 && elapsed_time <= force_close_time){
            DPLN("Force close - double RUN detected in: " + String(elapsed_time));
            force_close_prev_millis = 0;
            waiting = false;
            noTone(buzzer_pin);
            buzzer_state = OFF;
            move_gate(CLOSE, def_dir_delay, 50);
          }else{
            force_close_prev_millis = millis();
          }
        }
      }
      if(millis() - door_waiting_prev_millis >= door_waiting_time){
        DPLN("Waiting timeout elapsed: " + String(millis() - door_waiting_prev_millis));
        force_close_prev_millis = 0;
        waiting = false;
        move_gate(CLOSE, def_dir_delay, 100);
      }
    }else{
      DPLN("Gate: wait start");
      buz(2000,50,0);
      stop_gate(false, 0);
      waiting = true;
      door_waiting_prev_millis = millis();
    }
  }
}

void restart_gate(){
  DPLN("Gate: restart");
  stop_gate(false, restart_dir_delay);
  play_tune(m_n_restart, m_d_restart, m_t_restart);
  move_gate(OPEN, def_dir_delay, 100);
}

bool check_end_o(){
  if (active(end_o_pin, end_o_conf)){
    DPLN("PRESSED: end_o");
    switch (status){
      case ON_OPEN:
      case OFF_OPEN:
        wait_gate();
        break;
      default:
        break;
    }
    return true;
  }else{
    if(waiting == true){
      if(bounced == false &&
         (millis() - door_waiting_prev_millis) <= bouncing_timeout){
        DPLN("door_waiting_prev_millis at: " + String(door_waiting_prev_millis));
        DPLN("end_o bounced at: " + String(millis()));
        bounced = true;
        return false;
      }
      if(bounced == false){
        buz(200,150,0);
        stop_gate(true, 0);
      }
      else{
        wait_gate();
      }
    }
    return false;
  }
}

bool check_end_c(){
  if (active(end_c_pin, end_c_conf)){
    DPLN("PRESSED: enc_c");
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
  if (safety_activated()){
    DPLN("ACTIVATED: photocecell");
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


ICACHE_RAM_ATTR void isr_stop_pressed() {
  if(stop_pressed == false && standby == false){
    stop_pressed = true;
    DPLN(" ## Interrupt | PRESSED: stop");
    digitalWrite(motor_pin, OFF);
    DPLN(" ## Interrupt | stopped");
  }
}

bool check_run(){
  if (run_activated() == true){
    standby = false;
    DPLN("PRESSED: run");
    switch (status){
      case OFF_OPEN:
        if (stop_activated() ==false &&
            waiting == false){
          #ifdef ELECTRIC_LOCK
            digitalWrite(lock_pin, UNLOCK);
          #endif
          move_gate(OPEN, def_dir_delay, 0);
          #ifdef ELECTRIC_LOCK
            delay(def_lock_delay);
            digitalWrite(lock_pin, LOCK);
          #endif
        }
        break;
      case ON_CLOSE:
        move_gate(CLOSE, def_dir_delay, 0);
        break;
      case OFF_CLOSE:  //should never happen
        restart_gate();
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
  DPLN("NETI is starting ");
  // set the digital pin as output:
  pinMode(motor_pin, OUTPUT);
  pinMode(direction_pin, OUTPUT);
  #ifdef ELECTRIC_LOCK
    pinMode(lock_pin, OUTPUT);
  #endif
  DPRINT(".");
  // init relays
  digitalWrite(motor_pin, OFF);
  digitalWrite(direction_pin, OPEN);
  #ifdef ELECTRIC_LOCK
    digitalWrite(lock_pin, LOCK);
  #endif
  DPRINT(".");
  // set input in pullup mode
  pinMode(end_o_pin, INPUT_PULLUP);
  pinMode(end_c_pin, INPUT_PULLUP);
  pinMode(safety_pin, INPUT_PULLUP);
  pinMode(stop_pin, INPUT_PULLUP);
  pinMode(run_pin, INPUT_PULLUP);
  DPRINT(".");
  // interrupts
  attachInterrupt(digitalPinToInterrupt(stop_pin), isr_stop_pressed, stop_interrupt_mode);
  DPLN("Started!");
  play_tune(m_n_boot, m_d_boot, m_t_boot);
}

void check_loop(){
  if (active(end_o_pin, end_o_conf)){
    DPLN("PRESSED: end_o");
  }
  if (active(end_c_pin, end_c_conf)){
    DPLN("PRESSED: end_c");
  }
  if (active(stop_pin, stop_conf)){
    DPLN("PRESSED: stop");
  }
  if (active(run_pin, run_conf)){
    DPLN("PRESSED: run");
  }
  if (active(safety_pin, safety_conf)){
    DPLN("PRESSED: photo");
  }
  DPRINTLN("____________________");
  delay(2000);

}

void normal_loop(){
  if (standby == false){
    check_stop();
    check_security();
    switch (status){
      case OFF_OPEN:
      case ON_OPEN:
        check_end_o();
        break;
      case ON_CLOSE:
        check_end_c();
        break;
      default:
        break;
    }
  }
  check_run();
}

void loop(){
  LOOP();
}