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
#include "./common/material.hpp"
#endif

#ifndef   INCLUDE_SHARE_DEVICE_CODE
#define   INCLUDE_SHARE_DEVICE_CODE
#include "shareDeviceCode.hlsl"
#endif

#define TOTAL_SAMPLE_PER_PIXEL 100
#define RAY_DEPTH 20

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
        0b111111111, // instance inclusion mask
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
      } else {
        continue;
      }
    }

    TraceRay(
      world, // the tree
      RAY_FLAG_FORCE_OPAQUE, // ray flags
      0b111111111, // instance inclusion mask
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

ScatterResult scatter(Metal metal, float3 P, float3 N, Payload payload)
{
  float3 org   = ObjectRayOrigin();
  float3 dir   = ObjectRayDirection();

  if (dot(N,dir) > 0.f){
    N = -N;
  }
  N = normalize(N);
  
  ScatterResult result;
  result.scatteredOrigin = P;
  float3 reflected = reflect(normalize(dir),N);
  result.rand = ramdom_point_in_unit_sphere(payload.rand);
  result.scatteredDirection = (reflected + metal.fuzz * result.rand.random_number_3);
  result.attenuation        = metal.albedo;
  result.scatterEvent       = int(dot(result.scatteredDirection, N) > 0.f);
  result.normal             = N;
  return result;
}

ScatterResult scatter(Lambertian lambertian, float3 P, float3 N, Payload payload)
{
  float3 org   = ObjectRayOrigin();
  float3 dir   = ObjectRayDirection();

  if (dot(N,dir) > 0.f){
    N = -N;
  }
  N = normalize(N);

  ScatterResult result;
  result.scatteredOrigin     = P;
  result.rand = ramdom_point_in_unit_sphere(payload.rand);
  float3 target = P + (N + result.rand.random_number_3);
  result.scatteredDirection  = (target-P);
  result.attenuation         = lambertian.albedo;
  result.scatterEvent        = 1;
  result.normal              = N;
  return result;
}

ScatterResult scatter(Dielectric dielectric, float3 P, float3 N, Payload payload)
{
  ScatterResult result;

  float3 org   = ObjectRayOrigin();
  float3 dir   = ObjectRayDirection();

  if (dot(N,dir) > 0.f){
    N = -N;
  }
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
  result.rand = ramdom_point_in_unit_sphere(payload.rand);
  if (result.rand.random_number_3.x < reflect_prob) 
    result.scatteredDirection = reflected;
  else 
    result.scatteredDirection = refracted;
  
  // return scattering event
  result.scatterEvent        = 1;
  result.normal              = N;
  return result;
}

GPRT_CLOSEST_HIT_PROGRAM(TriangleMesh, (TrianglesGeomData, record), (Payload, payload), (Attributes, attributes))
{
  payload.color = float3(0.f, 0.f, 0.f);
  ScatterResult result;
  result.scatterEvent = 2;
  result.isObj = true;
  payload.scatterResult = result;

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

  uint instanceID = 0; //InstanceIndex();
  int material_type = record.material.type;
  // int material_type = gprt::load<int>(record.material_type, instanceID);
  // int material_type = gprt::load<int>(record.material_type, primID);
  if (material_type == 0) {
    // Lambertian
    Lambertian lambertian;
    // lambertian.albedo = float3(record.material.lambertian_albedo);
    lambertian.albedo = gprt::load<float3>(record.lambertian, instanceID);
    // lambertian.albedo = gprt::load<float3>(record.lambertian, primID);

    payload.scatterResult = scatter(lambertian, targetPoint, normal, payload);
  } else if (material_type == 1) {
    // Metal
    Metal metal;
    // metal.albedo = record.material.metal_albedo;
    // metal.fuzz = record.material.metal_fuzz;
    metal.albedo = gprt::load<float3>(record.metal_albedo, instanceID);
    metal.fuzz = gprt::load<float>(record.metal_fuzz, instanceID);
    // metal.albedo = gprt::load<float3>(record.metal_albedo, primID);
    // metal.fuzz = gprt::load<float>(record.metal_fuzz, primID);
    payload.scatterResult = scatter(metal, targetPoint, normal, payload);
  } else if (material_type == 2) {
    // Dielectric
    Dielectric dielectric;
    // dielectric.ref_idx = record.material.dielectric_ref_idx;
    dielectric.ref_idx = gprt::load<float>(record.dielectric, instanceID);
    // dielectric.ref_idx = gprt::load<float>(record.dielectric, primID);
    payload.scatterResult = scatter(dielectric, targetPoint, normal, payload);
  }
  payload.scatterResult.ray_t = tHit;
  payload.scatterResult.isObj = true;
  payload.scatterResult.scatteredOrigin = WorldRayOrigin() + WorldRayDirection() * tHit;
  payload.scatterResult.scatteredDirection = mul(payload.scatterResult.scatteredDirection, (float3x3)ObjectToWorld4x3());
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
    bool atLeastOneHit = false;
    for (int i = 0; i < RAY_DEPTH; i++) {
      TraceRay(
        world, // the tree
        RAY_FLAG_FORCE_OPAQUE, // ray flags
        0b111111111, // instance inclusion mask
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

        if (i > 0) {
          if (atLeastOneHit) {
            // total_payload_color += float3(1.f, 1.f, 1.f) * attenuation;
            total_payload_color += direct_lighting(world, record, lastScatterResult, attenuation);
          }
        }
        break;
      } else if (payload.scatterResult.scatterEvent == 0) {
        if (payload.scatterResult.isObj){
          break;
        }
        float3 leaveAABBOrigin = payload.scatterResult.scatteredOrigin;
        float3 leaveAABBDirection = payload.scatterResult.scatteredDirection;
        payload.rand = payload.scatterResult.rand;

        // Check if Obj in between
        rayDesc.TMax = payload.scatterResult.ray_t;
        TraceRay(
          world, // the tree
          RAY_FLAG_FORCE_OPAQUE, // ray flags
          0b00000001, // instance inclusion mask
          0, // ray type
          1, // number of ray types
          0, // miss type
          rayDesc, // the ray to trace
          payload // the payload IO
        );

        if (payload.scatterResult.scatterEvent == 2) {
          // No hit obj, continue with previous sampling
          rayDesc.Origin = leaveAABBOrigin;
          rayDesc.Direction = leaveAABBDirection;
        }
        else if (payload.scatterResult.scatterEvent == 0) {
          break;
          rayDesc.Origin = leaveAABBOrigin;
          rayDesc.Direction = leaveAABBDirection;
        }
        else {
          // Hit Obj
          attenuation *= payload.scatterResult.attenuation;
          rayDesc.Origin = payload.scatterResult.scatteredOrigin;
          rayDesc.Direction = payload.scatterResult.scatteredDirection;
          lastScatterResult = payload.scatterResult;
        }
        payload.rand = payload.scatterResult.rand;
        rayDesc.TMax = 1e10f;

      } else {
        atLeastOneHit = true;
        lastScatterResult = payload.scatterResult;
        payload.rand = payload.scatterResult.rand;

        if (payload.scatterResult.isObj) {
          attenuation *= payload.scatterResult.attenuation;
          rayDesc.Origin = payload.scatterResult.scatteredOrigin;
          rayDesc.Direction = payload.scatterResult.scatteredDirection;
          continue;
        }

        // Check if Obj in between
        rayDesc.TMax = payload.scatterResult.ray_t;
        TraceRay(
          world, // the tree
          RAY_FLAG_FORCE_OPAQUE, // ray flags
          0b00000001, // instance inclusion mask
          0, // ray type
          1, // number of ray types
          0, // miss type
          rayDesc, // the ray to trace
          payload // the payload IO
        );

        if (payload.scatterResult.scatterEvent == 2) {
          // No hit obj, continue with previous sampling
          attenuation *= lastScatterResult.attenuation;
          rayDesc.Origin = lastScatterResult.scatteredOrigin;
          rayDesc.Direction = lastScatterResult.scatteredDirection;
        }
        else if (payload.scatterResult.scatterEvent == 0) {
          break;
          attenuation *= lastScatterResult.attenuation;
          rayDesc.Origin = lastScatterResult.scatteredOrigin;
          rayDesc.Direction = lastScatterResult.scatteredDirection;
        }
        else {
          // Hit Obj
          attenuation *= payload.scatterResult.attenuation;
          rayDesc.Origin = payload.scatterResult.scatteredOrigin;
          rayDesc.Direction = payload.scatterResult.scatteredDirection;
          lastScatterResult = payload.scatterResult;
        }
        payload.rand = payload.scatterResult.rand;
        rayDesc.TMax = 1e10f;
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
  attr.color = float3(0.f, 0.f, 0.f);

  float3 org = ObjectRayOrigin();
  float3 dir = ObjectRayDirection();

  float3 aabb_lower = gprt::load<float3>(record.aabb_position, 0);
  float3 aabb_upper = gprt::load<float3>(record.aabb_position, 1);
  float3 lower = mul(aabb_lower, (float3x3)WorldToObject4x3());
  float3 upper = mul(aabb_upper, (float3x3)WorldToObject4x3());

  float t0 = RayTMin();
  float t1 = RayTCurrent();

  float2 t_result = intersect_box(t0, t1, org, dir, lower, upper);
  if (t_result.y > t_result.x) {
    attr.t_min = t_result.x;
    attr.t_max = t_result.y;
    ReportHit(t_result.x, /*hitKind*/ 0, attr);
  }
}

float sample_volume_object_space(VolumesGeomData record, float3 p, inout RandSeed rand)
{
  // calculate with volume
  int3 volume_size = gprt::load<int3>(record.volume_size_buffer, 0);
  int volume_size_long_product = volume_size.x * volume_size.y * volume_size.z;

  // Texture
  Texture3D texture = gprt::getTexture3DHandle(record.volume);
  
  // Use Sampler
  // SamplerState sampler = gprt::getSamplerHandle(record.samplers[0]);
  // p.x = clamp(p.x / volume_size.x, 0, 1);
  // p.y = clamp(p.y / volume_size.y, 0, 1);
  // p.z = clamp(p.z / volume_size.z, 0, 1);
  // return texture.SampleLevel(sampler, p, 0);

  // Read Data Directly + Random Shifting
  rand = rand_generate(rand, 3);
  float3 new_gen = rand.random_number_3;
  new_gen.x = clamp(new_gen.x, 0, 1);
  new_gen.y = clamp(new_gen.y, 0, 1);
  new_gen.z = clamp(new_gen.z, 0, 1);
  return texture[p + new_gen];
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

    float sample_point = sample_volume_object_space(record, rayOrg + rayDir * t, result.rand); //Current is world space
    float4 rgba = sample_transfer_function(record, sample_point);
    albedo = float3(rgba.x, rgba.y, rgba.z);
    float mu_t = density_scale * rgba.w;
    if (xi.y < mu_t / mu_max) {
      found_hit = true;
      break;
    }
    result.rand = rand_generate(result.rand, 2);
  }
  result.ray_t = t;
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
    result.scatteredOrigin     = WorldRayOrigin() + WorldRayDirection() * result.ray_t;
    result.scatteredDirection  = WorldRayDirection();
    result.isObj               = false;
  }
  // If hit by delta tracking, sample random direction
  else {
    result.scatterEvent        = 1;
    result.scatteredOrigin     = WorldRayOrigin() + WorldRayDirection() * result.ray_t;
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
