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

struct DirectionalLight
{
    float3 intensity;
    float3 direction;

    void setIntensity(float3 input_intensity)
    {
        intensity = input_intensity;
    }

    void setDirection(float3 input_direction)
    {
        direction = input_direction;
    }
};