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

#include "deviceCode.h"
#include "gprt.h"

struct Payload
{
  float3 color;
};

GPRT_RAYGEN_PROGRAM(simpleRayGen, (RayGenData, record))
{
  Payload payload;
  uint2 pixelID = DispatchRaysIndex().xy;
  float2 screen = (float2(pixelID) + 
                  float2(.5f, .5f)) / float2(record.fbSize);

  RayDesc rayDesc;
  rayDesc.Origin = record.camera.pos;
  rayDesc.Direction = 
    normalize(record.camera.dir_00
    + screen.x * record.camera.dir_du
    + screen.y * record.camera.dir_dv
  );
  rayDesc.TMin = 0.001;
  rayDesc.TMax = 10000.0;
  RaytracingAccelerationStructure world = gprt::getAccelHandle(record.world);
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

  const int fbOfs = pixelID.x + record.fbSize.x * pixelID.y;
  gprt::store(record.fbPtr, fbOfs, gprt::make_rgba(payload.color));
}

struct Attributes {
  float2 bc;
};

GPRT_CLOSEST_HIT_PROGRAM(TriangleMesh, (TrianglesGeomData, record), (Payload, payload), (Attributes, attributes))
{
  // compute normal:
  uint   primID = PrimitiveIndex();
  int3   index  = gprt::load<int3>(record.index, primID);
  float3 A      = gprt::load<float3>(record.vertex, index.x);
  float3 B      = gprt::load<float3>(record.vertex, index.y);
  float3 C      = gprt::load<float3>(record.vertex, index.z);
  float3 Ng     = normalize(cross(B-A,C-A));
  float3 rayDir = WorldRayDirection();
  payload.color = (.2f + .8f * abs(dot(rayDir,Ng))) * record.color;
}

GPRT_MISS_PROGRAM(miss, (MissProgData, record), (Payload, payload))
{
  uint2 pixelID = DispatchRaysIndex().xy;  
  int pattern = (pixelID.x / 8) ^ (pixelID.y/8);
  payload.color = (pattern & 1) ? record.color1 : record.color0;
}
