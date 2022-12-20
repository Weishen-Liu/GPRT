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

#include "gprt.h"

// #ifndef   INCLUDE_MATERIAL
// #define   INCLUDE_MATERIAL
// #include "./materials/material.hpp"
// #endif
#include "./materials/material.hpp"

#ifndef M_PI
#define M_PI 3.1415926f
#endif

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

float3 uniform_sample_hemisphere(float2 s) // To +Z axis
{
  float phi = 2 * M_PI * s.x;
  float cosTheta = s.y;
  float sinTheta = sqrt(max(float(0), float(1 - cosTheta * cosTheta)));
  return cartesian(phi, sinTheta, cosTheta);
}

float2 rand_2_10(float2 uv) {
    float noiseX = (frac(sin(dot(uv, float2(12.9898,78.233) * 2.0)) * 43758.5453));
    float noiseY = sqrt(1 - noiseX * noiseX);
    return float2(noiseX, noiseY);
}

float3 transform_to_look_at_direction(float3 lookingAt, float3 sampleOrg, float3 sampleDirection)
{
  float3 pointA = lookingAt - sampleOrg;
  float magnitude;
  float x_degree = 0.f;
  float y_degree = 0.f;
  float z_degree = 0.f;

  if (pointA.x == 0.f && pointA.y == 0.f && pointA.z == 0.f) return float3(0.f, 0.f, 0.f);
  if(pointA.x == 0.f && pointA.y == 0.f)
  {
    magnitude = sqrt(pointA.z * pointA.z + pointA.y * pointA.y);
    x_degree = acos(pointA.y / magnitude) * 180 / M_PI;
    
    magnitude = sqrt(pointA.x * pointA.x + pointA.z * pointA.z);
    y_degree = acos(pointA.z / magnitude) * 180 / M_PI;
  }
  else if(pointA.z == 0.f && pointA.y == 0.f)
  {
    magnitude = sqrt(pointA.x * pointA.x + pointA.z * pointA.z);
    y_degree = acos(pointA.z / magnitude) * 180 / M_PI;

    magnitude = sqrt(pointA.x * pointA.x + pointA.y * pointA.y);
    z_degree = acos(pointA.x / magnitude) * 180 / M_PI;

  }
  else if(pointA.x == 0.f && pointA.z == 0.f)
  {
    magnitude = sqrt(pointA.z * pointA.z + pointA.y * pointA.y);
    x_degree = acos(pointA.y / magnitude) * 180 / M_PI;

    magnitude = sqrt(pointA.x * pointA.x + pointA.y * pointA.y);
    z_degree = acos(pointA.x / magnitude) * 180 / M_PI;
  }
  else
  {
    magnitude = sqrt(pointA.z * pointA.z + pointA.y * pointA.y);
    x_degree = acos(pointA.y / magnitude) * 180 / M_PI;
    
    magnitude = sqrt(pointA.x * pointA.x + pointA.z * pointA.z);
    y_degree = acos(pointA.z / magnitude) * 180 / M_PI;

    magnitude = sqrt(pointA.x * pointA.x + pointA.y * pointA.y);
    z_degree = acos(pointA.x / magnitude) * 180 / M_PI;
  }
  

  // R = [R_x, R_y, R_z] = Rz(z_degree)Ry(y_degree)Rx(x_degree)
  float3 R_x = float3( cos(y_degree) * cos(z_degree),
                    (sin(x_degree) * sin(y_degree) * cos(z_degree) - cos(x_degree) * sin(z_degree)), 
                    (cos(x_degree) * sin(y_degree) * cos(z_degree) + sin(x_degree) * sin(z_degree))
                  );
  float3 R_y = float3( cos(y_degree) * sin(z_degree),
                    (sin(x_degree) * sin(y_degree) * sin(z_degree) + cos(x_degree) * cos(z_degree)), 
                    (cos(x_degree) * sin(y_degree) * sin(z_degree) - sin(x_degree) * cos(z_degree))
                  );
  float3 R_z = float3(
                     -1 * sin(y_degree),
                     sin(x_degree) * cos(z_degree),
                     cos(x_degree) * cos(y_degree)
                  );

  float3 new_ray_direction = float3(0.f, 0.f, 0.f);
  new_ray_direction.x = sampleDirection.x * R_x.x + sampleDirection.y * R_x.y + sampleDirection.z * R_x.z;
  new_ray_direction.y = sampleDirection.x * R_y.x + sampleDirection.y * R_y.y + sampleDirection.z * R_y.z;
  new_ray_direction.z = sampleDirection.x * R_z.x + sampleDirection.y * R_z.y + sampleDirection.z * R_z.z;

  return new_ray_direction;
}

float3 hack_sampling_hemisphere(float radius, float2 s, float3 normal)
{
  float3 sample_dir = uniform_sample_sphere(radius, s);
  if (dot(sample_dir, normal) < 0) {
    sample_dir *= -1;
  } else if (dot(sample_dir, normal) == 0) {
    sample_dir += float3(0.01f, 0.01f, 0.01f);
    if (dot(sample_dir, normal) < 0) {
      sample_dir *= -1;
    }
  }
  return sample_dir;
}

float3 reflect(float3 v, float3 n)
{
  return v - 2.0f*dot(v, n)*n;
}

float3 randomPointInUnitSphere(float2 rand) {
  float3 p;
  do {
    p = 2.0f*uniform_sample_hemisphere(rand) - float3(1, 1, 1);
  } while (dot(p,p) >= 1.0f);
  return p;
}

// ScatterResult scatter(Metal metal, float3 P, float3 N)
// {
//   float3 org   = WorldRayOrigin();
//   float3 dir   = WorldRayDirection();

//   if (dot(N,dir)  > 0.f)
//     N = -N;
//   N = normalize(N);
  
//   ScatterResult result;
//   float3 reflected = reflect(normalize(dir),N);
//   result.scatteredOrigin = P;
//   result.scatteredDirection = (reflected+metal.fuzz*randomPointInUnitSphere(float2(org.x, org.y)));

//   // float2 random = rand_2_10(float2(org.x, org.y));
//   // result.scatteredDirection = (reflected+metal.fuzz*hack_sampling_hemisphere(1, random, N));

//   result.attenuation         = metal.albedo;
//   result.scatterEvent = int(dot(result.scatteredDirection, N) > 0.f);
//   return result;
// }

ScatterResult scatter(Lambertian lambertian, float3 P, float3 N)
{
  float3 org   = WorldRayOrigin();
  float3 dir   = WorldRayDirection();

  if (dot(N,dir)  > 0.f)
    N = -N;
  N = normalize(N);

  float3 target = P + (N + randomPointInUnitSphere(float2(org.x, org.y)));

  // float2 random = rand_2_10(float2(org.x, org.y));
  // target = P + (N + hack_sampling_hemisphere(1, random, N));
  
  // return scattering event
  ScatterResult result;
  result.scatteredOrigin     = P;
  result.scatteredDirection  = (target-P);
  result.attenuation         = lambertian.albedo;
  result.scatterEvent        = true;
  return result;
}

GPRT_RAYGEN_PROGRAM(simpleRayGen, (RayGenData, record))
{
  float3 total_payload_color = float3(0.f, 0.f, 0.f);
  int total_sample_per_pixel = 1;
  int ray_depth = 50;
  uint2 pixelID = DispatchRaysIndex().xy;
  float2 screen = (float2(pixelID) + 
                  float2(.5f, .5f)) / float2(record.fbSize);
  for (int each_sample = 0; each_sample < total_sample_per_pixel; each_sample++)
  {
    Payload payload;
    RayDesc rayDesc;
    rayDesc.Origin = record.camera.pos;
    rayDesc.Direction = 
      normalize(record.camera.dir_00
      + screen.x * record.camera.dir_du
      + screen.y * record.camera.dir_dv
    );
    rayDesc.TMin = 0.01;
    rayDesc.TMax = 10000.0;
    RaytracingAccelerationStructure world = gprt::getAccelHandle(record.world);

    float3 attenuation = float3(1.f, 1.f, 1.f);
    for (int i = 0; i < ray_depth; i++) {
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
        break;
      // } else if (payload.scatterResult.scatterEvent == 0) {
      //   total_payload_color += float3(0.f, 0.f, 0.f);
      //   break;
      } else {
        attenuation *= payload.scatterResult.attenuation;
        rayDesc.Origin = payload.scatterResult.scatteredOrigin;
        rayDesc.Direction = payload.scatterResult.scatteredDirection;
      }
      if (i == ray_depth-1) {
        total_payload_color += float3(1.f, 0.f, 0.f);
      }
    }
    // TraceRay(
    //   world, // the tree
    //   RAY_FLAG_FORCE_OPAQUE, // ray flags
    //   0xff, // instance inclusion mask
    //   0, // ray type
    //   1, // number of ray types
    //   0, // miss type
    //   rayDesc, // the ray to trace
    //   payload // the payload IO
    // );
    // total_payload_color = payload.color;
  }

  const int fbOfs = pixelID.x + record.fbSize.x * pixelID.y;
  if (record.accId) {
    total_payload_color = total_payload_color + gprt::load<float3>(record.accBuffer, fbOfs);
  }
  gprt::store(record.accBuffer, fbOfs, total_payload_color);
  gprt::store(record.fbPtr, fbOfs, gprt::make_rgba(total_payload_color * (float(1) / ((record.accId + 1) * total_sample_per_pixel))));
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
  float3 rayOrg = WorldRayOrigin();
  float3 rayDir = WorldRayDirection();
  float tHit = RayTCurrent();
  float3 targetPoint = rayOrg + rayDir * tHit;

  // compute normal:
  int3   index  = gprt::load<int3>(record.index, primID);
  float3 A      = gprt::load<float3>(record.vertex, index.x);
  float3 B      = gprt::load<float3>(record.vertex, index.y);
  float3 C      = gprt::load<float3>(record.vertex, index.z);

  float3 normal;
  if (
    // gprt::load<float3>(record.normal, index.x).x == 0.f && 
    // gprt::load<float3>(record.normal, index.x).y == 0.f && 
    // gprt::load<float3>(record.normal, index.x).z == 0.f  
    true
  ) {
    normal     = normalize(cross(B-A,C-A));
    payload.color = (.2f + .8f * abs(dot(rayDir,normal))) * gprt::load<float3>(record.color, index.x);
  // } else {
  //   float3 triangle_barycentrics = get_triangle_barycentrics(rayOrg, A, B, C);
  //   normal     = triangle_barycentrics.z * gprt::load<float3>(record.normal, index.x) + 
  //                triangle_barycentrics.x * gprt::load<float3>(record.normal, index.y) + 
  //                triangle_barycentrics.y * gprt::load<float3>(record.normal, index.z);
  //   float3 current_color = gprt::load<float3>(record.color, index.x);
  //   payload.color = (.2f + .8f * abs(dot(rayDir,normal))) * current_color;
  }
  
  // payload.color = abs(normal);
  
  // Metal metal  = gprt::load<Metal>(record.metal, primID);
  // payload.scatterResult = scatter(metal, targetPoint, normal);

  Lambertian lambertian  = gprt::load<Lambertian>(record.lambertian, primID);
  payload.scatterResult = scatter(lambertian, targetPoint, normal);

  // Use hemisphere respect to normal
  // float2 random = rand_2_10(float2(payload.rayOrg.x, payload.rayOrg.y));
  // payload.rayDir = hack_sampling_hemisphere(1, random, normal);
}

GPRT_MISS_PROGRAM(miss, (MissProgData, record), (Payload, payload))
{
  // uint2 pixelID = DispatchRaysIndex().xy;  
  // int pattern = (pixelID.x / 8) ^ (pixelID.y/8);
  // payload.color = record.color1;//(pattern & 1) ? record.color1 : record.color0;

  float3 rayDir = WorldRayDirection();
  const float t = 0.5f * (rayDir.y + 1.0f);
  const float3 c = (1.0f - t) * float3(1.0f, 1.0f, 1.0f) + t * float3(0.5f, 0.7f, 1.0f);
  payload.color = c;

  ScatterResult result;
  result.scatterEvent = 2;
  payload.scatterResult = result;
  // payload.find_hit = 0;
}
