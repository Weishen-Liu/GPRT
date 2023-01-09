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

#ifndef   INCLUDE_MATERIAL
#define   INCLUDE_MATERIAL
#include "./materials/material.hpp"
#endif

#ifndef   INCLUDE_LIGHTS
#define   INCLUDE_LIGHTS
#include "./common/lights.hpp"
#endif

#ifndef M_PI
#define M_PI 3.1415926f
#endif

#define BIDIRECTION true
#define TOTAL_SAMPLE_PER_PIXEL 100
#define RAY_DEPTH 20
#define LIGHT_SAMPLE_TIMES 100
#define CAMERA_SAMPLE_TIMES 3

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
  RandSeed new_rand_seed;
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

float3 ramdom_point_in_unit_sphere(float2 rand) {
  float3 p;
  do {
    p = 2.0f*rand_3_10(rand) - float3(1, 1, 1);
  } while (dot(p,p) > 1.0f);
  return p;
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

  // for (int each_light = 0; each_light < record.ambient_light_size; each_light++) {
  //   float3 ambientLightIntensity = gprt::load<float3>(record.ambient_lights_intensity, each_light);
  //   total_lights_color += ambientLightIntensity * attenuation;
  // }

  Payload payload;
  RayDesc rayDesc;
  rayDesc.TMin = 1e-3f;
  rayDesc.TMax = 1e10f;
  for (int each_light = 0; each_light < record.directional_light_size; each_light++) {
    float3 directionalLightIntensity = gprt::load<float3>(record.directional_lights_intensity, each_light);
    float3 directionalLightDir = gprt::load<float3>(record.directional_lights_dir, each_light);
    rayDesc.Origin = lastScatterResult.scatteredOrigin;
    rayDesc.Direction = -directionalLightDir;
    if (dot(lastScatterResult.normal, rayDesc.Direction) < 0) {
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

float compare_vector_distance(float3 a, float3 b) {
  return sqrt((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y) + (b.z - a.z) * (b.z - a.z)) - 0.001f; 
}

float3 connection_with_first_hit(
  RaytracingAccelerationStructure world,
  RayGenData record,
  ScatterResult lastScatterResult,
  ScatterResult camera_hit_scatter_result,
  float3 attenuation,
  float3 intensity
) {
  float3 connect_to_first_hit_color = float3(0.f, 0.f, 0.f);

  Payload payload;
  RayDesc rayDesc;
  rayDesc.Origin = lastScatterResult.scatteredOrigin;
  rayDesc.Direction = camera_hit_scatter_result.scatteredOrigin - lastScatterResult.scatteredOrigin;
  if (dot(lastScatterResult.normal, rayDesc.Direction) < 0) {
    return connect_to_first_hit_color;
  }
  rayDesc.TMin = 1e-3f;
  rayDesc.TMax = compare_vector_distance(camera_hit_scatter_result.scatteredOrigin, lastScatterResult.scatteredOrigin);
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
    connect_to_first_hit_color += camera_hit_scatter_result.attenuation * attenuation;
  }
  return connect_to_first_hit_color;
}

Payload sample_direction_from_ambient_light(ScatterResult camera_hit_scatter_result, Payload payload) {
  // Sample a random point on sphere
  payload.rand = rand_generate(payload.rand, 2);
  float3 sample_point = 100 * ramdom_point_in_unit_sphere(payload.rand.random_number_2);
  payload.rand.rayDescOrg = normalize(sample_point);
  payload.rand.rayDescDir = normalize(-sample_point);
  return payload;
}

Payload sample_direction_from_directional_light(ScatterResult camera_hit_scatter_result, float3 directionalLightDir, Payload payload) {

  // Sample a random point on sphere
  payload.rand = rand_generate(payload.rand, 2);
  float3 sample_point = 100 * ramdom_point_in_unit_sphere(payload.rand.random_number_2);
  float3 sample_point_org = sample_point + -directionalLightDir * 100;
  payload.rand.rayDescOrg = normalize(sample_point_org);
  payload.rand.rayDescDir = normalize(directionalLightDir);
  return payload;
}

Payload bidirectional_calculation(RaytracingAccelerationStructure world, RayGenData record, ScatterResult camera_hit_scatter_result, float3 attenuation, Payload payload, float3 intensity) {
  float3 total_payload_color = float3(0.f, 0.f, 0.f);

  ScatterResult lastScatterResult;
  RayDesc rayDesc;
  rayDesc.Origin = payload.rand.rayDescOrg;
  rayDesc.Direction = payload.rand.rayDescDir;
  rayDesc.TMin = 1e-3f;
  rayDesc.TMax = 1e10f;
  for (int depth = 0; depth < RAY_DEPTH; depth++) {
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
      total_payload_color += payload.color * attenuation;
      if (depth > 0) {
        total_payload_color += connection_with_first_hit(world, record, lastScatterResult, camera_hit_scatter_result, attenuation, intensity);
      }
      break;
    } else if (payload.scatterResult.scatterEvent == 0) {
      break;
    } else {
      attenuation *= payload.scatterResult.attenuation;
      rayDesc.Origin = payload.scatterResult.scatteredOrigin;
      rayDesc.Direction = payload.scatterResult.scatteredDirection;
      lastScatterResult = payload.scatterResult;
    }

    // Possibly terminate the path with Russian roulette
    attenuation = russian_roulette(depth, attenuation);
    if (attenuation.x == 0.f && attenuation.y == 0.f && attenuation.z == 0.f) break;
  }
  payload.rand.bidir_total_color = total_payload_color;
  return payload;
}

float3 calculate_from_light(RaytracingAccelerationStructure world, RayGenData record, ScatterResult camera_hit_scatter_result, float3 attenuation, Payload payload) {
  float3 total_payload_color = float3(0.f, 0.f, 0.f);
  RayDesc rayDesc;
  float3 intensity;
  for (int sampling_times = 0; sampling_times < LIGHT_SAMPLE_TIMES; sampling_times++) {
    // for (int ambient_light_count = 0; ambient_light_count < record.ambient_light_size; ambient_light_count++) {
    //   intensity = gprt::load<float3>(record.ambient_lights_intensity, ambient_light_count);
    //   payload = sample_direction_from_ambient_light(camera_hit_scatter_result, payload);
    //   payload = bidirectional_calculation(world, record, camera_hit_scatter_result, attenuation, payload, intensity);
    //   total_payload_color += payload.rand.bidir_total_color;
    // }

    for (int direct_light_count = 0; direct_light_count < record.directional_light_size; direct_light_count++) {
      float3 directionalLightDir = gprt::load<float3>(record.directional_lights_dir, direct_light_count);

      intensity = gprt::load<float3>(record.directional_lights_intensity, direct_light_count);
      payload = sample_direction_from_directional_light(camera_hit_scatter_result, directionalLightDir, payload);
      payload = bidirectional_calculation(world, record, camera_hit_scatter_result, attenuation, payload, intensity);
      total_payload_color += payload.rand.bidir_total_color;
    }
  }
  return total_payload_color / LIGHT_SAMPLE_TIMES;
}

GPRT_RAYGEN_PROGRAM(simpleRayGen, (RayGenData, record))
{
  float3 total_payload_color = float3(0.f, 0.f, 0.f);
  uint2 pixelID = DispatchRaysIndex().xy;
  float2 screen = (float2(pixelID) + float2(.5f, .5f)) / float2(record.fbSize);

  if (BIDIRECTION) {
    for (int each_sample = 0; each_sample < TOTAL_SAMPLE_PER_PIXEL; each_sample++)
    {
      Payload payload;
      payload.rand.state = rand_init(pixelID.x + 1024 * each_sample, pixelID.y + 1024 * each_sample);
      RayDesc rayDesc;
      payload.rand = rand_generate(payload.rand, 2);
      rayDesc.Origin = record.camera.pos;
      rayDesc.Origin += rand_3_10(payload.rand.random_number_2) / (float2(record.fbSize).x - 1);
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
      ScatterResult camera_hit_scatter_result;
      bool got_hit = false;
      for (int camera_hit = 0; camera_hit < CAMERA_SAMPLE_TIMES; camera_hit++) {
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
          total_payload_color += payload.color * attenuation;
          if (camera_hit > 0) {
            total_payload_color += direct_lighting(world, record, camera_hit_scatter_result, attenuation);
            total_payload_color += calculate_from_light(world, record, camera_hit_scatter_result, attenuation, payload);
          }
          break;
        } else if (payload.scatterResult.scatterEvent == 0) {
          break;
        } else {
          attenuation *= payload.scatterResult.attenuation;
          rayDesc.Origin = payload.scatterResult.scatteredOrigin;
          rayDesc.Direction = payload.scatterResult.scatteredDirection;
          camera_hit_scatter_result = payload.scatterResult;
        }

        // Possibly terminate the path with Russian roulette
        attenuation = russian_roulette(camera_hit, attenuation);
        if (attenuation.x == 0.f && attenuation.y == 0.f && attenuation.z == 0.f) break;
      }
    }

    const int fbOfs = pixelID.x + record.fbSize.x * pixelID.y;
    if (record.accId) {
      total_payload_color = total_payload_color + gprt::load<float3>(record.accBuffer, fbOfs);
    }
    gprt::store(record.accBuffer, fbOfs, total_payload_color);
    gprt::store(record.fbPtr, fbOfs, gprt::make_rgba(total_payload_color * (float(1) / ((record.accId + 1) * TOTAL_SAMPLE_PER_PIXEL))));

  } else {
    for (int each_sample = 0; each_sample < TOTAL_SAMPLE_PER_PIXEL; each_sample++)
    {
      Payload payload;
      payload.rand.state = rand_init(pixelID.x + 1024 * each_sample, pixelID.y + 1024 * each_sample);
      RayDesc rayDesc;
      payload.rand = rand_generate(payload.rand, 2);
      rayDesc.Origin = record.camera.pos;
      rayDesc.Origin += rand_3_10(payload.rand.random_number_2) / (float2(record.fbSize).x - 1);
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
          total_payload_color += payload.color * attenuation;
          if (i > 0) {
            total_payload_color += direct_lighting(world, record, lastScatterResult, attenuation);
          }
          break;
        } else if (payload.scatterResult.scatterEvent == 0) {
          total_payload_color += float3(0.f, 0.f, 0.f);
          break;
        } else {
          attenuation *= payload.scatterResult.attenuation;
          rayDesc.Origin = payload.scatterResult.scatteredOrigin;
          rayDesc.Direction = payload.scatterResult.scatteredDirection;
          lastScatterResult = payload.scatterResult;
        }

        // Possibly terminate the path with Russian roulette
        attenuation = russian_roulette(i, attenuation);
        if (attenuation.x == 0.f && attenuation.y == 0.f && attenuation.z == 0.f) break;
      }
    }

    const int fbOfs = pixelID.x + record.fbSize.x * pixelID.y;
    if (record.accId) {
      total_payload_color = total_payload_color + gprt::load<float3>(record.accBuffer, fbOfs);
    }
    gprt::store(record.accBuffer, fbOfs, total_payload_color);
    gprt::store(record.fbPtr, fbOfs, gprt::make_rgba(total_payload_color * (float(1) / ((record.accId + 1) * TOTAL_SAMPLE_PER_PIXEL))));
  }
}

float3 reflect(float3 v, float3 n)
{
  return v - 2.0f*dot(v, n)*n;
}

ScatterResult scatter(Metal metal, float3 P, float3 N, Payload payload)
{
  float3 org   = ObjectRayOrigin();
  float3 dir   = ObjectRayDirection();

  if (dot(N,dir)  > 0.f)
    N = -N;
  N = normalize(N);
  
  ScatterResult result;
  float3 reflected = reflect(normalize(dir),N);
  result.scatteredOrigin = P;
  payload.rand = rand_generate(payload.rand, 2);
  result.scatteredDirection = (reflected+metal.fuzz*ramdom_point_in_unit_sphere(payload.rand.random_number_2));

  result.attenuation         = metal.albedo;
  result.scatterEvent = int(dot(result.scatteredDirection, N) > 0.f);
  return result;
}

ScatterResult scatter(Lambertian lambertian, float3 P, float3 N, Payload payload)
{
  float3 org   = ObjectRayOrigin();
  float3 dir   = ObjectRayDirection();

  if (dot(N,dir)  > 0.f)
    N = -N;
  N = normalize(N);

  payload.rand = rand_generate(payload.rand, 2);
  float3 target = P + (N + ramdom_point_in_unit_sphere(payload.rand.random_number_2));

  // return scattering event
  ScatterResult result;
  result.scatteredOrigin     = P;
  result.scatteredDirection  = (target-P);
  result.attenuation         = lambertian.albedo;
  result.scatterEvent        = 1;
  result.normal              = N;
  return result;
}

float3 refract(float3 v,
             float3 n,
             float ni_over_nt,
             float3 refracted)
{
  float3 uv = normalize(v);
  float dt = dot(uv, n);
  float discriminant = 1.0f - ni_over_nt * ni_over_nt*(1 - dt * dt);
  if (discriminant > 0.f) {
    return ni_over_nt * (uv - n * dt) - n * sqrt(discriminant);
  }
  else
    return float3(0.f, 0.f, 0.f);
}

float schlick(float cosine, float ref_idx)
{
  float r0 = (1.0f - ref_idx) / (1.0f + ref_idx);
  r0 = r0 * r0;
  return r0 + (1.0f - r0)*pow((1.0f - cosine), 5.0f);
}

ScatterResult scatter(Dielectric dielectric, float3 P, float3 N, Payload payload)
{
  ScatterResult result;

  float3 org   = ObjectRayOrigin();
  float3 dir   = ObjectRayDirection();

  N = normalize(N);
  float3 outward_normal;
  float3 reflected = reflect(dir,N);
  float ni_over_nt;
  result.attenuation = float3(1.f, 1.f, 1.f); 
  float3 refracted;
  float reflect_prob;
  float cosine;
  
  if (dot(dir,N) > 0.f) {
    outward_normal = -N;
    ni_over_nt = dielectric.ref_idx;
    cosine = dot(dir, N);// / vec3f(dir).length();
    cosine = sqrt(1.f - dielectric.ref_idx*dielectric.ref_idx*(1.f-cosine*cosine));
  }
  else {
    outward_normal = N;
    ni_over_nt = 1.0 / dielectric.ref_idx;
    cosine = -dot(dir, N);// / vec3f(dir).length();
  }
  float3 new_refracted = refract(dir, outward_normal, ni_over_nt, refracted);
  if (new_refracted.x != 0.f && new_refracted.y != 0.f && new_refracted.z != 0.f) {
    reflect_prob = schlick(cosine, dielectric.ref_idx);
    refracted = new_refracted;
  }
  else {
    reflect_prob = 1.f;
  }

  result.scatteredOrigin = P;
  if (rand_2_10(float2(org.x, org.y)).x < reflect_prob) 
    result.scatteredDirection = reflected;
  else 
    result.scatteredDirection = refracted;
  
  // return scattering event
  result.scatterEvent        = 1;
  result.normal              = N;
  return result;
}

struct Attributes {
  float2 bc;
};

float3 get_triangle_barycentrics(float3 P, float3 A, float3 B, float3 C)
{
  float3 v0 = B - A;
  float3 v1 = C - A;
  float3 v2 = P - A;
  float d00 = dot(v0, v0);
  float d01 = dot(v0, v1);
  float d11 = dot(v1, v1);
  float d20 = dot(v2, v0);
  float d21 = dot(v2, v1);
  float denom = d00 * d11 - d01 * d01;
  float u = (d11 * d20 - d01 * d21) / denom;
  float v = (d00 * d21 - d01 * d20) / denom;
  float w = 1.0f - u - v;
  return float3(u, v, w);
}

GPRT_CLOSEST_HIT_PROGRAM(TriangleMesh, (TrianglesGeomData, record), (Payload, payload), (Attributes, attributes))
{
  uint   primID = PrimitiveIndex();
  float3 rayOrg = ObjectRayOrigin();
  float3 rayDir = ObjectRayDirection();
  float  tHit   = RayTCurrent();
  float3 targetPoint = rayOrg + rayDir * tHit;

  // compute normal:
  int3   index  = gprt::load<int3>(record.index, primID);
  float3 A      = gprt::load<float3>(record.vertex, index.x);
  float3 B      = gprt::load<float3>(record.vertex, index.y);
  float3 C      = gprt::load<float3>(record.vertex, index.z);
  float3 n_A      = gprt::load<float3>(record.normal, index.x);
  float3 n_B      = gprt::load<float3>(record.normal, index.y);
  float3 n_C      = gprt::load<float3>(record.normal, index.z);

  float3 normal;

  if (
    n_A.x == 0.f && n_A.y == 0.f && n_A.z == 0.f &&
    n_B.x == 0.f && n_B.y == 0.f && n_B.z == 0.f &&
    n_C.x == 0.f && n_C.y == 0.f && n_C.z == 0.f
  ) {
    normal     = normalize(cross(B-A,C-A));
  } else {
    float3 triangle_barycentrics = get_triangle_barycentrics(targetPoint, A, B, C);
    normal     = triangle_barycentrics.z * n_A + 
                 triangle_barycentrics.x * n_B + 
                 triangle_barycentrics.y * n_C;
  }

  int material_type = gprt::load<int>(record.material_type, primID);
  if (material_type == 0) {
    // Lambertian
    Lambertian lambertian = gprt::load<Lambertian>(record.lambertian, primID);
    payload.scatterResult = scatter(lambertian, targetPoint, normal, payload);
  } else if (material_type == 1) {
    // Metal
    Metal metal;
    metal.albedo = gprt::load<float3>(record.metal_albedo, primID);
    metal.fuzz = gprt::load<float>(record.metal_fuzz, primID);
    payload.scatterResult = scatter(metal, targetPoint, normal, payload);
  } else if (material_type == 2) {
    // Dielectric
    Dielectric dielectric = gprt::load<Dielectric>(record.dielectric, primID);
    payload.scatterResult = scatter(dielectric, targetPoint, normal, payload);
  }
}

GPRT_MISS_PROGRAM(miss, (MissProgData, record), (Payload, payload))
{
  float3 rayDir = WorldRayDirection();
  const float t = 0.5f * (rayDir.y + 1.0f);
  const float3 c = (1.0f - t) * float3(1.0f, 1.0f, 1.0f) + t * float3(0.5f, 0.7f, 1.0f);
  payload.color = c;

  ScatterResult result;
  result.scatterEvent = 2;
  payload.scatterResult = result;
}
