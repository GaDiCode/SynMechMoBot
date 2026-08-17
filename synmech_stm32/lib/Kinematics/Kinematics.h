#ifndef KINEMATICS_H
#define KINEMATICS_H

class Kinematics {
private:
    float track_width;
    float max_vel;

public:
    Kinematics(float track_width, float max_vel);
    
    // Convert (vx, wz) to left/right wheel velocities in m/s
    void inverseKinematics(float vx, float wz, float &v_left, float &v_right);
    
    // Convert wheel velocity (m/s) to PWM (-255 to 255)
    int velocityToPWM(float vel);
};

#endif
