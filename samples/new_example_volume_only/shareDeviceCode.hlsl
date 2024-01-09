#ifndef INCLUDE_DEVICE_CODE
#define INCLUDE_DEVICE_CODE
// our device-side data structures
#include "deviceCode.h"
#endif

#ifndef INCLUDE_GPRT
#define INCLUDE_GPRT
// public GPRT API
#include <gprt.h>
#endif

#ifndef M_PI
#define M_PI 3.1415926f
#endif

uint
rand_init(uint val0, uint val1) {
  uint v0 = val0;
  uint v1 = val1;
  uint s0 = 0;

  for (int n = 0; n < 4; n++) {
    s0 += 0x9e3779b9;
    v0 += ((v1 << 4) + 0xa341316c) ^ (v1 + s0) ^ ((v1 >> 5) + 0xc8013ea4);
    v1 += ((v0 << 4) + 0xad90777d) ^ (v0 + s0) ^ ((v0 >> 5) + 0x7e95761e);
  }
  return v0;
}

// Generate random unsigned int in [0, 2^24)
RandSeed
rand_generate(RandSeed rand_seed, int number) {
  const uint32_t LCG_A = 1664525u;
  const uint32_t LCG_C = 1013904223u;
  float new_random_number_1, new_random_number_2, new_random_number_3;
  if (number >= 1) {
    rand_seed.state = (LCG_A * rand_seed.state + LCG_C);
    new_random_number_1 = ldexp(float(rand_seed.state), -32);
    rand_seed.random_number = new_random_number_1;
  }

  if (number >= 2) {
    rand_seed.state = (LCG_A * rand_seed.state + LCG_C);
    new_random_number_2 = ldexp(float(rand_seed.state), -32);
    rand_seed.random_number_2 = float2(new_random_number_1, new_random_number_2);
  }

  if (number >= 3) {
    rand_seed.state = (LCG_A * rand_seed.state + LCG_C);
    new_random_number_3 = ldexp(float(rand_seed.state), -32);
    rand_seed.random_number_3 = float3(new_random_number_1, new_random_number_2, new_random_number_3);
  }
  return rand_seed;
}

float2
rand_2_10(float2 uv) {
  float noiseX = (frac(sin(dot(uv, float2(12.9898, 78.233) * 2.0)) * 43758.5453));
  float noiseY = sqrt(1 - noiseX * noiseX);
  return float2(noiseX, noiseY);
}

float3
rand_3_10(float2 uv) {
  float2 noiseXY = rand_2_10(uv);
  float noiseX = noiseXY.x;
  float noiseY = noiseXY.y;
  float noiseZ = rand_2_10(noiseXY).x;
  return float3(noiseX, noiseY, noiseZ);
}

RandSeed
ramdom_point_in_unit_sphere(RandSeed rand) {
  rand = rand_generate(rand, 2);
  float3 p;
  do {
    p = 2.0f * rand_3_10(rand.random_number_2) - float3(1, 1, 1);
    rand = rand_generate(rand, 2);
  } while (dot(p, p) > 1.0f);
  rand.random_number_3 = p;
  return rand;
}

float3
cartesian(float phi, float sinTheta, float cosTheta) {
  float sinPhi, cosPhi;
  sinPhi = sin(phi);
  cosPhi = cos(phi);

  return float3(cosPhi * sinTheta, sinPhi * sinTheta, cosTheta);
}

float3
uniform_sample_sphere(float radius, float2 s) {
  float phi = 2 * M_PI * s.x;
  float cosTheta = radius * (1.f - 2.f * s.y);
  float sinTheta = 2.f * radius * sqrt(s.y * (1.f - s.y));
  return cartesian(phi, sinTheta, cosTheta);
}

static RandSeed
russian_roulette(int scatter_index, float3 attenuation, RandSeed rand) {
  if (scatter_index > 5) {
    float q = (.05f < attenuation.y) ? 1 - attenuation.y : .05f;

    rand = rand_generate(rand, 2);
    float2 compare = rand.random_number_2;
    if (compare.x > compare.y)
      rand.random_number_3 = float3(0.f, 0.f, 0.f);
    return rand;

    if (1 == q) {
      attenuation = float3(1.f, 1.f, 1.f);
    } else {
      attenuation /= 1 - q;
    }
  }
  rand.random_number_3 = attenuation;
  return rand;
}

float
fmaf(float a, float b, float c) {
  return a * b + c;
}

float
__frcp_rn(float a) {
  return 1 / a;
}

float
reduce_max(float3 v) {
  return max(max(v.x, v.y), v.z);
}

float
reduce_min(float3 v) {
  return min(min(v.x, v.y), v.z);
}

float2
intersect_box(float t0, float t1, float3 ray_org, float3 ray_dir, float3 lower, float3 upper) {
  // Option 1
  float float_large = 3.402823466e+38F;
  float float_small = 1.175494351e-38F;
  int3 is_small = int3(abs(ray_dir.x) < float_small, abs(ray_dir.y) < float_small, abs(ray_dir.z) < float_small);
  float3 rcp_dir = float3(__frcp_rn(ray_dir.x), __frcp_rn(ray_dir.y), __frcp_rn(ray_dir.z));
  float3 t_lo = float3(is_small.x ? float_large : (lower.x - ray_org.x) * rcp_dir.x,   //
                       is_small.y ? float_large : (lower.y - ray_org.y) * rcp_dir.y,   //
                       is_small.z ? float_large : (lower.z - ray_org.z) * rcp_dir.z    //
  );
  float3 t_hi = float3(is_small.x ? -float_large : (upper.x - ray_org.x) * rcp_dir.x,   //
                       is_small.y ? -float_large : (upper.y - ray_org.y) * rcp_dir.y,   //
                       is_small.z ? -float_large : (upper.z - ray_org.z) * rcp_dir.z    //
  );
  t0 = max(t0, reduce_max(min(t_lo, t_hi)));
  t1 = min(t1, reduce_min(max(t_lo, t_hi)));

  // Option 2
  // float3 t_lo = (lower - ray_org) / ray_dir;
  // float3 t_hi = (upper - ray_org) / ray_dir;
  // t0 = max(t0, reduce_max(min(t_lo, t_hi)));
  // t1 = min(t1, reduce_min(max(t_lo, t_hi)));

  return float2(t0, t1);
}
