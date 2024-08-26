#ifndef __MQTT__H__
#define __MQTT__H__

void mqtt_start(void);
void mqtt_publish_msg(char *topic, char *msg);

#endif  //!__MQTT__H__