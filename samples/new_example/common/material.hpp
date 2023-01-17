#ifndef   INCLUDE_GPRT
#define   INCLUDE_GPRT
// public GPRT API
#include <gprt.h>
#endif

#ifndef   INCLUDE_DEVICE_CODE
#define   INCLUDE_DEVICE_CODE
// our device-side data structures
#include "deviceCode.h"
#endif

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
