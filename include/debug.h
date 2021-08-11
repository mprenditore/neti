#include <Arduino.h>

#ifdef CHECK_ALL
  #define LOOP(...) check_loop()     //enable check_loop
#else
  #define LOOP(...) normal_loop()    //enable normal_loop
#endif

#ifdef DEBUG
  #define DPRINT(...)   Serial.print(__VA_ARGS__)                  //DPRINT is a macro, debug print
  #define DPRINTLN(...) Serial.println(__VA_ARGS__)                //DPRINTLN is a macro, debug print with new line
  #define DPRINTPRE(...) printpre(__VA_ARGS__)  //DMILLIS is a macro, debug print with millis()
#else
  #define DPRINT(...)     //now defines a blank line
  #define DPRINTLN(...)   //now defines a blank line
  #define DMILLIS(...)    //now defines a blank line
#endif

unsigned long log_prev_millis = 0; // time counter for log message diff time

void printpre(){
    Serial.print(millis()-log_prev_millis);
    log_prev_millis=millis();
    Serial.print(" - ");
}