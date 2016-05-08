#ifndef _accel_h_
#define _accel_h_

typedef struct Accel_t {
  unsigned short on;
  unsigned short off;
  unsigned short last_time;
  unsigned short last_pin_state;
  unsigned char pos;
  unsigned short value;
} Accel;

void accel_init(Accel *accel);
void accel_pin_state_changed(Accel *accel, unsigned char pin_state, unsigned short now);
void accel_apply_filter(Accel *accel);

#endif//_accel_h_
