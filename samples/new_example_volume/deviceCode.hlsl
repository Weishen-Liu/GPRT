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

static float3 russian_roulette(int scatter_index, float3 attenuation)
{
  if (scatter_index > 5) {
    float q = (.05f < attenuation.y) ? 1 - attenuation.y : .05f;

    float2 compare = rand_2_10(float2(attenuation.x, attenuation.y));
    if (compare.x > compare.y)
        return float3(0.f, 0.f, 0.f);
    
    if(1 == q){
      attenuation = float3(1.f, 1.f, 1.f);
    }else{
      attenuation /= 1 - q;
    }
    
  }
  return attenuation;
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
    if (dot(lastScatterResult.normal, rayDesc.Direction) <= 0) {
      continue;
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
    rayDesc.Origin += rand_3_10(payload.rand.random_number_2) / (float2(fbSize).x - 1);
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
        total_payload_color += payload.color * attenuation;
        
        // Calculate Light
        if (i > 0) {
          total_payload_color += direct_lighting(world, record, lastScatterResult, attenuation);
        }
        break;
      } else if (payload.scatterResult.scatterEvent == 0) {
        break;
      } else {
        attenuation *= payload.scatterResult.attenuation;
        rayDesc.Origin = payload.scatterResult.scatteredOrigin;
        rayDesc.Direction = payload.scatterResult.scatteredDirection;
        lastScatterResult = payload.scatterResult;
        payload.rand = payload.scatterResult.rand;
      }

      // Possibly terminate the path with Russian roulette
      attenuation = russian_roulette(i, attenuation);
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

// This intersection program will be called when rays hit our axis
// aligned bounding boxes. Here, we can fetch per-geometry data and
// process that data, but we do not have access to the ray payload
// structure here.
//
// Instead, we pass data through a customizable Attributes structure
// for further processing by closest hit / any hit programs.
GPRT_INTERSECTION_PROGRAM(AABBIntersection, (AABBGeomData, record))
{
  Attributes attr;
  attr.color = float3(1.f, 1.f, 1.f);
  ReportHit(0.1f, /*hitKind*/ 0, attr);
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
GPRT_CLOSEST_HIT_PROGRAM(AABBClosestHit, (AABBGeomData, record), (Payload, payload), (Attributes, attributes))
{
  payload.color = attributes.color;
  ScatterResult result;
  result.scatterEvent = 2;
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
  result.scatterEvent = 2;
  payload.scatterResult = result;
}