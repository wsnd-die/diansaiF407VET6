#include <assert.h>
#include "pca9685.h"

int main(void)
{
    assert(PCA9685_PulseUsToTicks(0U) == 102U);
    assert(PCA9685_PulseUsToTicks(500U) == 102U);
    assert(PCA9685_PulseUsToTicks(1500U) == 307U);
    assert(PCA9685_PulseUsToTicks(2500U) == 512U);
    assert(PCA9685_PulseUsToTicks(3000U) == 512U);
    return 0;
}
