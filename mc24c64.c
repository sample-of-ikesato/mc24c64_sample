#include <xc.h>
#include "i2c.h"
#include "mc24c64.h"

void mc24c64_init()
{
  i2c_init_master();
}
