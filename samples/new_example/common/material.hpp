#pragma once
#ifndef INCLUDE_NEWEXAMPLE_COMMON_MATERIAL
#define INCLUDE_NEWEXAMPLE_COMMON_MATERIAL
#include <gprt.h>
#include "deviceCode.h"

struct Lambertian
{
  float3 albedo;
};

struct Metal
{
  float3 albedo;
  float fuzz;
};

struct Dielectric
{
  float ref_idx;
};
#endif
