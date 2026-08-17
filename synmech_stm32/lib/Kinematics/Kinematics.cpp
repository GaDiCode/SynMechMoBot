#include "Kinematics.h"

Kinematics::Kinematics(float track_width, float max_vel) {
    this->track_width = track_width;
    this->max_vel = max_vel;
}

void Kinematics::inverseKinematics(float vx, float wz, float &v_left, float &v_right) {
    v_left = vx - (wz * track_width / 2.0f);
    v_right = vx + (wz * track_width / 2.0f);
}

int Kinematics::velocityToPWM(float vel) {
    int pwm = (int)(vel / max_vel * 255.0f);
    if (pwm > 255) pwm = 255;
    if (pwm < -255) pwm = -255;
    return pwm;
}
