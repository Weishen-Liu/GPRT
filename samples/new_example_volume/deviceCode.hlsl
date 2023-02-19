// MIT License

// Copyright (c) 2022 Nathan V. Morrical

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef   INCLUDE_DEVICE_CODE
#define   INCLUDE_DEVICE_CODE
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

#define TOTAL_SAMPLE_PER_PIXEL 100
#define RAY_DEPTH 20

uint rand_init(uint val0, uint val1)
{
  uint v0 = val0;
  uint v1 = val1;
  uint s0 = 0;

  for (int n = 0; n < 4; n++) {
    s0 += 0x9e3779b9;
    v0 += ((v1<<4)+0xa341316c)^(v1+s0)^((v1>>5)+0xc8013ea4);
    v1 += ((v0<<4)+0xad90777d)^(v0+s0)^((v0>>5)+0x7e95761e);
  }
  return v0;
}

// Generate random unsigned int in [0, 2^24)
RandSeed rand_generate(RandSeed rand_seed, int number)
{
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

float2 rand_2_10(float2 uv) {
    float noiseX = (frac(sin(dot(uv, float2(12.9898,78.233) * 2.0)) * 43758.5453));
    float noiseY = sqrt(1 - noiseX * noiseX);
    return float2(noiseX, noiseY);
}

float3 rand_3_10(float2 uv) {
    float2 noiseXY = rand_2_10(uv);
    float noiseX = noiseXY.x;
    float noiseY = noiseXY.y;
    float noiseZ = rand_2_10(noiseXY).x;
    return float3(noiseX, noiseY, noiseZ);
}

RandSeed ramdom_point_in_unit_sphere(RandSeed rand) {
  rand = rand_generate(rand, 2);
  float3 p;
  do {
    p = 2.0f*rand_3_10(rand.random_number_2) - float3(1, 1, 1);
    rand = rand_generate(rand, 2);
  } while (dot(p,p) > 1.0f);
  rand.random_number_3 = p;
  return rand;
}

float3 cartesian(float phi, float sinTheta, float cosTheta)
{
  float sinPhi, cosPhi;
  sinPhi = sin(phi);
  cosPhi = cos(phi);

  return float3(cosPhi * sinTheta, sinPhi * sinTheta, cosTheta);
}

float3 uniform_sample_sphere(float radius, float2 s)
{
  float phi = 2 * M_PI * s.x;
  float cosTheta = radius * (1.f - 2.f * s.y);
  float sinTheta = 2.f * radius * sqrt(s.y * (1.f - s.y));
  return cartesian(phi, sinTheta, cosTheta);
}

static RandSeed russian_roulette(int scatter_index, float3 attenuation, RandSeed rand)
{
  if (scatter_index > 5) {
    float q = (.05f < attenuation.y) ? 1 - attenuation.y : .05f;

    rand = rand_generate(rand, 2);
    float2 compare = rand.random_number_2;
    if (compare.x > compare.y)
        rand.random_number_3 = float3(0.f, 0.f, 0.f);
        return rand;
    
    if(1 == q){
      attenuation = float3(1.f, 1.f, 1.f);
    }else{
      attenuation /= 1 - q;
    }
    
  }
  rand.random_number_3 = attenuation;
  return rand;
}

float3 direct_lighting(RaytracingAccelerationStructure world, RayGenData record, ScatterResult lastScatterResult, float3 attenuation) {
  float3 total_lights_color = float3(0.f, 0.f, 0.f);

  for (int each_light = 0; each_light < record.ambient_light_size; each_light++) {
    float3 ambientLightIntensity = gprt::load<float3>(record.ambient_lights_intensity, each_light);
    total_lights_color += ambientLightIntensity * attenuation;
  }

  Payload payload;
  RayDesc rayDesc;
  rayDesc.TMin = 1e-3f;
  rayDesc.TMax = 1e10f;
  for (int each_light = 0; each_light < record.directional_light_size; each_light++) {
    float3 directionalLightIntensity = gprt::load<float3>(record.directional_lights_intensity, each_light);
    float3 directionalLightDir = gprt::load<float3>(record.directional_lights_dir, each_light);
    rayDesc.Origin = lastScatterResult.scatteredOrigin;
    rayDesc.Direction = rayDesc.Origin + directionalLightDir;
    
    // Obj
    if (lastScatterResult.isObj) {
      if (dot(lastScatterResult.normal, rayDesc.Direction) <= 0) {
        continue;
      }
    }
    // Volume
    else {
      TraceRay(
        world, // the tree
        RAY_FLAG_FORCE_OPAQUE, // ray flags
        0xff, // instance inclusion mask
        0, // ray type
        1, // number of ray types
        0, // miss type
        rayDesc, // the ray to trace
        payload // the payload IO
      );
      if (payload.scatterResult.scatterEvent == 0) {
        // attenuation *= payload.scatterResult.attenuation;
        rayDesc.Origin = payload.scatterResult.scatteredOrigin;
        rayDesc.Direction = payload.scatterResult.scatteredDirection;
        payload.rand = payload.scatterResult.rand;
      }else {
        continue;
      }
    }

    TraceRay(
      world, // the tree
      RAY_FLAG_FORCE_OPAQUE, // ray flags
      0xff, // instance inclusion mask
      0, // ray type
      1, // number of ray types
      0, // miss type
      rayDesc, // the ray to trace
      payload // the payload IO
    );
    if (payload.scatterResult.scatterEvent == 2) {
      total_lights_color += directionalLightIntensity * attenuation;
    }
  }

  return total_lights_color;
}

GPRT_CLOSEST_HIT_PROGRAM(TriangleMesh, (TrianglesGeomData, record), (Payload, payload), (Attributes, attributes))
{
  payload.color = float3(0.f, 0.f, 0.f);
  ScatterResult result;
  result.scatterEvent = 2;
  result.isObj = true;
  payload.scatterResult = result;
}

GPRT_RAYGEN_PROGRAM(simpleRayGen, (RayGenData, record))
{
  float3 total_payload_color = float3(0.f, 0.f, 0.f);
  uint2 pixelID = DispatchRaysIndex().xy;
  uint2 fbSize = DispatchRaysDimensions().xy;
  float2 screen = (float2(pixelID) + float2(.5f, .5f)) / float2(fbSize);

  for (int each_sample = 0; each_sample < TOTAL_SAMPLE_PER_PIXEL; each_sample++)
  {
    Payload payload;
    payload.rand.state = rand_init(pixelID.x + 1024 * each_sample, pixelID.y + 1024 * each_sample);
    RayDesc rayDesc;
    payload.rand = rand_generate(payload.rand, 2);
    rayDesc.Origin = record.camera.pos;
    rayDesc.Origin += uniform_sample_sphere(1, payload.rand.random_number_2) / (float2(fbSize).x - 1);
    rayDesc.Direction = 
      normalize(record.camera.dir_00
      + screen.x * record.camera.dir_du
      + screen.y * record.camera.dir_dv
      - rayDesc.Origin
    );
    rayDesc.TMin = 1e-3f;
    rayDesc.TMax = 1e10f;
    RaytracingAccelerationStructure world = gprt::getAccelHandle(record.world);

    float3 attenuation = float3(1.f, 1.f, 1.f);
    ScatterResult lastScatterResult;
    for (int i = 0; i < RAY_DEPTH; i++) {
      TraceRay(
        world, // the tree
        RAY_FLAG_FORCE_OPAQUE, // ray flags
        0xff, // instance inclusion mask
        0, // ray type
        1, // number of ray types
        0, // miss type
        rayDesc, // the ray to trace
        payload // the payload IO
      );

      if (payload.scatterResult.scatterEvent == 2) {
        // Miss
        // total_payload_color += payload.color * attenuation;
        // total_payload_color = float3(1.f, 0.f, 0.f);

        // if (i > 0) {
        //   total_payload_color += float3(1.f, 1.f, 1.f) * attenuation;
        //   // total_payload_color += direct_lighting(world, record, lastScatterResult, attenuation);
        // }
        break;
      } else if (payload.scatterResult.scatterEvent == 0) { // Leave AABB
        if (i > 0) {
          // total_payload_color += float3(1.f, 1.f, 1.f) * attenuation;
          total_payload_color += direct_lighting(world, record, lastScatterResult, attenuation);
        }
        break;
      } else {
        attenuation *= payload.scatterResult.attenuation;
        rayDesc.Origin = payload.scatterResult.scatteredOrigin;
        rayDesc.Direction = payload.scatterResult.scatteredDirection;
        lastScatterResult = payload.scatterResult;
        payload.rand = payload.scatterResult.rand;
      }

      // Possibly terminate the path with Russian roulette
      payload.rand = russian_roulette(i, attenuation, payload.rand);
      attenuation = payload.rand.random_number_3;
      if (attenuation.x == 0.f && attenuation.y == 0.f && attenuation.z == 0.f) break;
    }
  }

  const int fbOfs = pixelID.x + fbSize.x * pixelID.y;
  if (record.accId) {
    total_payload_color = total_payload_color + gprt::load<float3>(record.accBuffer, fbOfs);
  }
  gprt::store(record.accBuffer, fbOfs, total_payload_color);
  // GPRT load color with BGR format, not RGB
  // float3 reverse_total_color = float3(total_payload_color.z, total_payload_color.y, total_payload_color.x);
  gprt::store(record.frameBuffer, fbOfs, gprt::make_rgba(total_payload_color * (float(1) / ((record.accId + 1) * TOTAL_SAMPLE_PER_PIXEL))));
}

float fmaf(float a, float b, float c)
{
  return a * b + c;
}

float __frcp_rn(float a)
{
  return 1 / a;
}

float reduce_max(float3 v) {
  return max(max(v.x, v.y),v.z);
}

float reduce_min(float3 v) {
  return min(min(v.x, v.y),v.z);
}

float2 intersect_box(float t0, float t1, float3 ray_org, float3 ray_dir, float3 lower, float3 upper)
{
  // Option 1
  float float_large = 3.402823466e+38F;
  float float_small = 1.175494351e-38F;
  int3 is_small = int3(abs(ray_dir.x) < float_small, abs(ray_dir.y) < float_small, abs(ray_dir.z) < float_small);
  float3 rcp_dir = float3(__frcp_rn(ray_dir.x), __frcp_rn(ray_dir.y), __frcp_rn(ray_dir.z));
  float3 t_lo = float3(is_small.x ? float_large : (lower.x - ray_org.x) * rcp_dir.x, //
                           is_small.y ? float_large : (lower.y - ray_org.y) * rcp_dir.y, //
                           is_small.z ? float_large : (lower.z - ray_org.z) * rcp_dir.z  //
  );
  float3 t_hi = float3(is_small.x ? -float_large : (upper.x - ray_org.x) * rcp_dir.x, //
                           is_small.y ? -float_large : (upper.y - ray_org.y) * rcp_dir.y, //
                           is_small.z ? -float_large : (upper.z - ray_org.z) * rcp_dir.z  //
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

// This intersection program will be called when rays hit our axis
// aligned bounding boxes. Here, we can fetch per-geometry data and
// process that data, but we do not have access to the ray payload
// structure here.
//
// Instead, we pass data through a customizable Attributes structure
// for further processing by closest hit / any hit programs.
GPRT_INTERSECTION_PROGRAM(AABBIntersection, (VolumesGeomData, record))
{
  Attributes attr;
  attr.color = float3(1000.f, 0.f, 0.f);
  attr.color = float3(0.f, 0.f, 0.f);

  float3 org = ObjectRayOrigin();
  float3 dir = ObjectRayDirection();
  // org = WorldRayOrigin();
  // dir = ObjectRayDirection();
  float3 aabb_lower = gprt::load<float3>(record.aabb_position, 0);
  float3 aabb_upper = gprt::load<float3>(record.aabb_position, 1);
  float3 lower = mul(aabb_lower, (float3x3)WorldToObject4x3());
  float3 upper = mul(aabb_upper, (float3x3)WorldToObject4x3());
  // lower = aabb_lower;
  // upper = aabb_upper;

  float t0 = RayTMin();
  float t1 = RayTCurrent();

  float2 t_result = intersect_box(t0, t1, org, dir, lower, upper);
  if (t_result.y > t_result.x) {
    attr.t_min = t_result.x;
    attr.t_max = t_result.y;
    ReportHit(t_result.x, /*hitKind*/ 0, attr);
  }
}

float sample_volume_object_space(VolumesGeomData record, float3 p)
{
  // calculate with volume
  int3 volume_size = gprt::load<int3>(record.volume_size_buffer, 0);
  int volume_size_long_product = volume_size.x * volume_size.y * volume_size.z;

  // Spaces
  p = mul(p, (float3x3)ObjectToWorld4x3());
  return gprt::load<float>(record.volume, volume_size.x * volume_size.y * int(p.z) + volume_size.x * int(p.y) + int(p.x));

  // World
  // p.x = clamp(p.x, 0.f, volume_size.x - 1);
  // p.y = clamp(p.y, 0.f, volume_size.y - 1);
  // p.z = clamp(p.z, 0.f, volume_size.z - 1);
  // return gprt::load<float>(record.volume, volume_size.x * volume_size.y * int(p.z) + volume_size.x * int(p.y) + int(p.x));
} 

float4 sample_transfer_function(VolumesGeomData record, float sample_point)
{
  float2 value_range = gprt::load<float2>(record.tfn_value_range, 0);
  float scale = 1.f / (value_range.y - value_range.x);
  float v = (clamp(sample_point, value_range.x, value_range.y) - value_range.x) * scale;

  int tfn_color_size = gprt::load<int>(record.tfn_color_size_buffer, 0);
  float index = fmaf(v, float(tfn_color_size - 1), 0.5f);
  float4 rgba = gprt::load<float4>(record.tfn_color, int(index));

  int tfn_opacity_size = gprt::load<int>(record.tfn_opacity_size_buffer, 0);
  index = fmaf(v, float(tfn_opacity_size - 1), 0.5f);
  rgba.w = gprt::load<float>(record.tfn_opacity, int(index)); // followed by the alpha correction

  return rgba;
}

ScatterResult delta_tracking(VolumesGeomData record, Payload payload, float3 rayOrg, float3 rayDir, float t_min, float t_max)
{
  float density_scale = 1.f;
  float max_opacity = 1.f;
  float mu_max = density_scale * max_opacity;

  float3 albedo = float3(0.f, 0.f, 0.f);
  bool found_hit = false;
  ScatterResult result;
  result.rand = rand_generate(payload.rand, 2);
  float t = t_min;
  while (true) {
    float2 xi = result.rand.random_number_2;
    t = t + -log(1.f - xi.x) / mu_max;

    if (t > t_max) {
      found_hit = false;
      break;
    }

    float sample_point = sample_volume_object_space(record, rayOrg + rayDir * t); //Current is world space
    float4 rgba = sample_transfer_function(record, sample_point);
    albedo = float3(rgba.x, rgba.y, rgba.z);
    float mu_t = density_scale * rgba.w;
    if (xi.y < mu_t / mu_max) {
      found_hit = true;
      break;
    }
    result.rand = rand_generate(result.rand, 2);
  }
  result.volume_t = t;
  result.volume_albedo = albedo;
  result.volume_hit = found_hit;
  return result;
}

// This closest hit program will be called when our intersection program
// reports a hit between our ray and our custom primitives.
// Here, we can fetch per-geometry data, process that data, and send
// it back to our ray generation program.
//
// Note, since this is a custom AABB primitive, our intersection program
// above defines what attributes are passed to our closest hit program.
//
// Also note, this program is also called after all ReportHit's have been
// called and we can conclude which reported hit is closest.
GPRT_CLOSEST_HIT_PROGRAM(AABBClosestHit, (VolumesGeomData, record), (Payload, payload), (Attributes, attributes))
{
  ScatterResult result;
  float3 rayOrg = ObjectRayOrigin();
  float3 rayDir = ObjectRayDirection();
  // rayOrg = WorldRayOrigin();
  // rayDir = WorldRayDirection();
  
  // See if delta tracking returns hit
  result = delta_tracking(record, payload, rayOrg, rayDir, attributes.t_min, attributes.t_max);

  // If not hit by delta tracking, return
  if (!result.volume_hit) {
    result.scatterEvent        = 0;
    result.scatteredOrigin     = WorldRayOrigin() + WorldRayDirection() * result.volume_t;
    result.scatteredDirection  = WorldRayDirection();
    result.isObj               = false;
  }
  // If hit by delta tracking, sample random direction
  else {
    result.scatterEvent        = 1;
    result.scatteredOrigin     = WorldRayOrigin() + WorldRayDirection() * result.volume_t;
    result.rand                = rand_generate(result.rand, 2);
    result.scatteredDirection  = mul(uniform_sample_sphere(1, result.rand.random_number_2), (float3x3)ObjectToWorld4x3());
    result.attenuation         = result.volume_albedo;
    result.isObj               = false;
  }
  payload.scatterResult = result;
}

GPRT_MISS_PROGRAM(miss, (MissProgData, record), (Payload, payload))
{
  float3 rayDir = WorldRayDirection();
  const float t = 0.5f * (rayDir.y + 1.0f);
  const float3 c = (1.0f - t) * float3(1.0f, 1.0f, 1.0f) + t * float3(0.5f, 0.7f, 1.0f);
  payload.color = c;
  // payload.color = float3(1.f, 1.f, 1.f);
  payload.color = float3(0.f, 0.f, 0.f);

  ScatterResult result;
  result.scatterEvent   = 2;
  result.isObj          = false;
  payload.scatterResult = result;
}