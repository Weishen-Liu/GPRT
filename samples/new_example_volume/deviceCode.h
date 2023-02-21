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
  alignas(16) gprt::Buffer aabb_position;
  alignas(16) gprt::Buffer volume;          // float3
  /*! base color texture we use for the entire mesh */
  alignas(16) gprt::Texture volume_in_texture;
  alignas(16) gprt::Buffer tfn_color;       // float4
  alignas(16) gprt::Buffer tfn_opacity;     // float
  alignas(16) gprt::Buffer tfn_value_range; // float2

//   alignas(16) int volume_size;              // int
//   alignas(16) int tfn_color_size;           // int
//   alignas(16) int tfn_opacity_size;         // int

  alignas(16) gprt::Buffer volume_size_buffer;
  alignas(16) gprt::Buffer tfn_color_size_buffer;
  alignas(16) gprt::Buffer tfn_opacity_size_buffer;
};

/* variables for the triangle mesh geometry */
struct TrianglesGeomData
{
    /*! array/buffer of vertex indices */
    alignas(16) gprt::Buffer index; // vec3i*
    /*! array/buffer of vertex positions */
    alignas(16) gprt::Buffer vertex; // vec3f *
    /*! array/buffer of vertex normal */
    alignas(16) gprt::Buffer normal; // vec3f *

    // /*! base color we use for the entire mesh */
    // alignas(16) float3 color;
    /*! array/buffer of color */
    alignas(16) gprt::Buffer color; // vec3f *

    alignas(16) gprt::Buffer material_type; // vec3f *
    alignas(16) gprt::Buffer lambertian;    // vec3f *
    alignas(16) gprt::Buffer metal_albedo;  // vec3f *
    alignas(16) gprt::Buffer metal_fuzz;    // vec3f *
    alignas(16) gprt::Buffer dielectric;    // vec3f *

    struct
    {
        alignas(16) int    type;
        alignas(16) float3 lambertian_albedo;
        alignas(16) float3 metal_albedo;
        alignas(16) float  metal_fuzz;
        alignas(16) float  dielectric_ref_idx;
    } material;
};

struct RayGenData
{
    alignas(16) gprt::Buffer accBuffer;
    alignas(16) int accId;

    alignas(16) gprt::Buffer frameBuffer;

    alignas(16) gprt::Accel triangleWorld;
    alignas(16) gprt::Accel world;

    alignas(16) gprt::Buffer ambient_lights_intensity;
    alignas(16) int ambient_light_size;

    alignas(16) gprt::Buffer directional_lights_intensity;
    alignas(16) gprt::Buffer directional_lights_dir;
    alignas(16) int directional_light_size;

    struct
    {
        alignas(16) float3 pos;
        alignas(16) float3 dir_00;
        alignas(16) float3 dir_du;
        alignas(16) float3 dir_dv;
    } camera;
};

/* variables for the miss program */
struct MissProgData
{
    alignas(16) float3 color0;
    alignas(16) float3 color1;
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

struct ScatterResult
{
    RandSeed rand;

    int scatterEvent;
    float3 scatteredOrigin;
    float3 scatteredDirection;
    float3 attenuation;
    bool isObj;
    float3 normal;
    
    bool volume_hit;
    float volume_t;
    float3 volume_albedo;
};

struct Payload
{
    float3 color;
    ScatterResult scatterResult;
    RandSeed rand;
};

struct Attributes {
  float2 bc;
  float3 color;

  // ray t
  float t_min;
  float t_max;
};
#endif