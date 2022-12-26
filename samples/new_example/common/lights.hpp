#ifndef INCLUDE_GPRT
#define INCLUDE_GPRT
// public GPRT API
#include <gprt.h>
#endif

struct AmbientLight
{
    float3 intensity;

    void setIntensity(float3 input_intensity)
    {
        intensity = input_intensity;
    }
};