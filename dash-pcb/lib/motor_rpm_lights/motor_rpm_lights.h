#ifndef MOTOR_RPM_LIGHTS_H
#define MOTOR_RPM_LIGHTS_H

#include <stdint.h>

void init_motor_rpm_lights();
void loop_motor_rpm_lights();
void update_motor_rpm(uint8_t* msg, uint8_t len);

#endif // MOTOR_RPM_LIGHTS_H