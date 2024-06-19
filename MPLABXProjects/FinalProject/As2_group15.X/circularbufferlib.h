#ifndef CB_H
#define	CB_H

#define DIMFIFOWRITE 200
#define MAX_COMMANDS 11 // (not now: 10 commands + 1 for the head)

typedef struct {
    int head;
    int tail;
    char msg [DIMFIFOWRITE]; 
} fifo;

typedef struct {
    int head;
    int tail;
    int msg [MAX_COMMANDS][2]; 
} fifo_pwm;

void fifo_init();

void fifo_pwm_init();

#endif	/* CB_H */

