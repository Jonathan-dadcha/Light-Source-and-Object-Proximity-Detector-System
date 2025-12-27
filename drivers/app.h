#ifndef _app_H_
#define _app_H_


enum FSMstate{state0,state1,state2,state3,state4,state5,state6,state7,state8,state9}; // global variable
enum StatusReceive{Name,Size,Content};
enum SYSmode{mode0,mode1,mode2,mode3,mode4}; // global variable

// expose global FSM vars so api.c can set state
extern enum FSMstate state;
extern enum SYSmode  lpm_mode;


#endif







