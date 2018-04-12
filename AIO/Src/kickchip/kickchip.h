
#ifndef __kickchip_H
#define __kickchip_H

typedef enum{
	Ready,
	Kicking,
	Charging
}kick_states;


void kick_Kick(int percentage);
void kick_Chip(int percentage);
void kick_Callback();
void kick_ChargeUpdate();
void kick_printblock();


#endif /* __kickchip_H */
