#include <stdio.h>
#include "accel.h"

void accel_init(Accel *accel)
{
  // 静止状態の値
  // TODO:とりあえず固定値だけど、自動で計算できるようにしよう
  accel->value = 4585;
}

void accel_pin_state_changed(Accel *accel, unsigned char pin_state, unsigned short now)
{
  if (accel->last_pin_state != pin_state) {
    accel->last_pin_state = pin_state;
    if (accel->last_pin_state == 0) {
      if (accel->last_time < now)
        accel->on = now - accel->last_time;
      else
        accel->on = 65536 - (accel->last_time - now);
    } else {
      if (accel->last_time < now)
        accel->off = now - accel->last_time;
      else
        accel->off = 65536 - (accel->last_time - now);
    }
    accel->last_time = now;
  }
}

void accel_apply_filter(Accel *accel)
{
  accel->value = (unsigned short)(accel->value * 0.9 + accel->on * 0.1);
}
