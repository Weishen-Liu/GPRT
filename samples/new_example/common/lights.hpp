#ifndef INCLUDE_GPRT
#define INCLUDE_GPRT
// public GPRT API
#include <gprt.h>
#endif

struct AmbientLight
{
    float3 intensity;
};

struct DirectionalLight
{
    float3 intensity;
    float3 direction;
};