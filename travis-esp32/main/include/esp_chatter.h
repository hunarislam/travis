#ifndef ESP_CHATTER_H
#define ESP_CHATTER_H

#ifdef __cplusplus
extern "C" {
#endif

void rosserial_setup();

void odom_pub(float vel_x, float vel_z);

#ifdef __cplusplus
}
#endif

#endif /* ESP_CHATTER_H */
