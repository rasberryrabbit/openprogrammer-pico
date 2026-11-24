/*
 * OPcontrol.h — Public interface for the Open Programmer application layer.
 * Same declarations as the original PIC18 firmware.
 */
#ifndef OPCONTROL_H
#define OPCONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

void UserInit(void);
void ProcessIO(void);
void timer_isr(void);

#ifdef __cplusplus
}
#endif

#endif // OPCONTROL_H
