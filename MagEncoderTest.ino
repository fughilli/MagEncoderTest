#include "extsetup.h"
#include "Wire.h"
#include "PIDControl.h"
#include <stdlib.h>
#include "Smoother.h"

void setup()
{
    extsetup();
}

void loop()
{
    extloop();
}
