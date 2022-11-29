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

#include "gprt.h"

/* variables available to all programs */


/* variables for the triangle mesh geometry */
struct TrianglesGeomData
{
  /*! array/buffer of vertex indices */
  alignas(16) gprt::Buffer index; // vec3i*
  /*! array/buffer of vertex positions */
  alignas(16) gprt::Buffer vertex; // vec3f *  
  // /*! base color we use for the entire mesh */
  // alignas(16) float3 color;
  /*! array/buffer of vertex positions */
  alignas(16) gprt::Buffer color; // vec3f *  
};

struct RayGenData
{
  alignas(16) gprt::Buffer fbPtr;

  alignas(8) int2 fbSize;
  alignas(16) gprt::Accel world;

  struct { 
    alignas(16) float3 pos;   
    alignas(16) float3 dir_00;
    alignas(16) float3 dir_du;
    alignas(16) float3 dir_dv;
  } camera;
};

/* variables for the miss program */
struct MissProgData
{
  alignas(16) float3  color0;
  alignas(16) float3  color1;
};
