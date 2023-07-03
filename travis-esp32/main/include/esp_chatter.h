#ifndef ESP_CHATTER_H
#define ESP_CHATTER_H

#ifdef __cplusplus
extern "C" {
#endif

void rosserialSetup();

void publishOdometry(float vel_x, float vel_z);

// void cmdVelCallback(const geometry_msgs::Twist& cmd_vel);

void getCmdVel(float* vel_x, float* vel_z);

#ifdef __cplusplus
}
#endif

#endif /* ESP_CHATTER_H */
