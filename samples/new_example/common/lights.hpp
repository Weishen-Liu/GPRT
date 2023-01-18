#ifndef INCLUDE_GPRT
#define INCLUDE_GPRT
// public GPRT API
#include <gprt.h>
#endif

struct AmbientLight
{
    bool choosed;
    const char* name;
    float3 intensity;
};

struct DirectionalLight
{
    bool choosed;
    const char* name;
    float3 intensity;
    float3 direction;
};