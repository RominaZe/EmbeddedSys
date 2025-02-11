 /* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef PWMLIB_H
#define	PWMLIB_H

#include <xc.h> // include processor files - each processor file is guarded.  

#define WHNULL 0
#define WHMOVESTD 400
#define WHMOVEFULL 7200

#define EMERGENCY_STOP 100
#define PRE_EMERGENCY_STOP 50

#define FORWARD 1
#define COUNT_ROTATION 2
#define CLOCKWISE_ROTATION 3
#define BACKWARD 4

void pwmRemap();
void pwmParametrization();
void pwmConfig(); // standard confiuration
void moveForward ( );
void moveLeft();
void moveRight();
void moveBack();
void whstop();

void input_move(int );

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

#ifdef	__cplusplus
}
#endif 

#endif	

