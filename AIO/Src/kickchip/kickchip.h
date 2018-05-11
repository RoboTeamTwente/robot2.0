
#ifndef __kickchip_H
#define __kickchip_H

typedef enum{
	kick_Ready,
	kick_Kicking,
	kick_Charging,
	kick_Idle
}kick_states;

void kick_Init();
void kick_DeInit();
void kick_Kick(int percentage);
void kick_Chip(int percentage);
void kick_Callback();
void kick_Stateprint();


#endif /* __kickchip_H */
