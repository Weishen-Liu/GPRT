#pragma once
#ifndef INCLUDE_NEWEXAMPLE_COMMON_LIGHTS
#define INCLUDE_NEWEXAMPLE_COMMON_LIGHTS
#include <gprt.h>

struct AmbientLight
{
    bool choosed;
    std::string name;
    float3 intensity;
};

struct DirectionalLight
{
    bool choosed;
    std::string name;
    float3 intensity;
    float3 direction;
};

#endif
