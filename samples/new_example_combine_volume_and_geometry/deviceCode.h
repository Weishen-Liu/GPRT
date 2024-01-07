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
#pragma once
#ifndef INCLUDE_NEWEXAMPLE_DEVICE_CODE
#define INCLUDE_NEWEXAMPLE_DEVICE_CODE
#include <gprt.h>

/* variables available to all programs */

struct VolumesGeomData {
  gprt::Buffer aabb_position;

  /*! base color texture we use for the entire mesh */
  gprt::Texture volume;
  gprt::Sampler samplers[2];

  gprt::Buffer tfn_color;         // float4
  gprt::Buffer tfn_opacity;       // float
  gprt::Buffer tfn_value_range;   // float2

  //   int volume_size;              // int
  //   int tfn_color_size;           // int
  //   int tfn_opacity_size;         // int

  gprt::Buffer volume_size_buffer;
  gprt::Buffer tfn_color_size_buffer;
  gprt::Buffer tfn_opacity_size_buffer;
};

/* variables for the triangle mesh geometry */
struct TrianglesGeomData {
  /*! array/buffer of vertex indices */
  gprt::Buffer index;   // vec3i*
  /*! array/buffer of vertex positions */
  gprt::Buffer vertex;   // vec3f *
  /*! array/buffer of vertex normal */
  gprt::Buffer normal;   // vec3f *

  // /*! base color we use for the entire mesh */
  // float3 color;
  /*! array/buffer of color */
  gprt::Buffer color;   // vec3f *

  gprt::Buffer material_type;   // vec3f *
  gprt::Buffer lambertian;      // vec3f *
  gprt::Buffer metal_albedo;    // vec3f *
  gprt::Buffer metal_fuzz;      // vec3f *
  gprt::Buffer dielectric;      // vec3f *

  struct {
    int type;
    float3 lambertian_albedo;
    float3 metal_albedo;
    float metal_fuzz;
    float dielectric_ref_idx;
  } material;
};

struct RayGenData {
  gprt::Buffer accBuffer;
  int accId;

  gprt::Buffer frameBuffer;

  gprt::Accel world;

  gprt::Buffer ambient_lights_intensity;
  int ambient_light_size;

  gprt::Buffer directional_lights_intensity;
  gprt::Buffer directional_lights_dir;
  int directional_light_size;

  struct {
    float3 pos;
    float3 dir_00;
    float3 dir_du;
    float3 dir_dv;
  } camera;
};

/* variables for the miss program */
struct MissProgData {
  float3 color0;
  float3 color1;
};

struct RandSeed {
  uint state;
  float random_number;
  float2 random_number_2;
  float3 random_number_3;

  float3 rayDescOrg;
  float3 rayDescDir;
  float3 bidir_total_color;
  float3 total_color_from_light;
};

struct ScatterResult {
  RandSeed rand;

  int scatterEvent;
  float3 scatteredOrigin;
  float3 scatteredDirection;
  float3 attenuation;
  bool isObj;
  float3 normal;

  float ray_t;
  bool volume_hit;
  float3 volume_albedo;
};

// struct Payload
// {
//     float3 color;
//     ScatterResult scatterResult;
//     RandSeed rand;
// };

struct Attributes {
  float2 bc;
  float3 color;

  // ray t
  float t_min;
  float t_max;
};
#endif