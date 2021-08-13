#include <Arduino.h>

#ifdef CHECK_ALL
  #define LOOP(...) check_loop()     //enable check_loop
#else
  #define LOOP(...) normal_loop()    //enable normal_loop
#endif

#ifdef DEBUG
  #define DPRINT(...)   Serial.print(__VA_ARGS__)    //DPRINT is a macro, debug print
  #define DPRINTLN(...) Serial.println(__VA_ARGS__)  //DPRINTLN is a macro, debug print with new line
  #define DPLN(...) debug_print(__VA_ARGS__)         //DPLN is a macro, debug print with millis passed as prefix
#else
  #define DPRINT(...)     //defines a blank line if debug is disabled
  #define DPRINTLN(...)   //defines a blank line if debug is disabled
  #define DPLN(...)       //defines a blank line if debug is disabled
#endif

unsigned long log_prev_millis = 0; // time counter for log message diff time

// accept single string as imput
void debug_print(const char* output){
    Serial.println(String(millis()-log_prev_millis) + F("ms - ") + output);
    log_prev_millis=millis();
}

// accept string concatenation as input (demands casting to String for variables)
void debug_print(const StringSumHelper output){
    Serial.println(String(millis()-log_prev_millis) + F("ms - ") + output);
    log_prev_millis=millis();
}