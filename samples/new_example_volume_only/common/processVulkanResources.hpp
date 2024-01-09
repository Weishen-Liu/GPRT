#pragma once
#ifndef INCLUDE_NEWEXAMPLE_COMMON_PROCESS_VULKAN_RESOURCES
#define INCLUDE_NEWEXAMPLE_COMMON_PROCESS_VULKAN_RESOURCES

#include <gprt.h>
// #include <gdt/math/mat.h>
#include "deviceCode.h"
#include <GLFW/glfw3.h>

#include "configureImgui.hpp"

struct VulkanResources {
  ConfigureImgui configureImgui;

  // Volume
  struct GeometryVolume {
    GPRTGeomOf<VolumesGeomData> aabbGeom;
    GPRTAccel aabbBLAS;
    VolumesGeomData *volumeData;

    // gdt::affine3f matrix;
    float2 volume_value_range;
    float volume_scale;

    // tfn
    std::vector<float4> tfn_colors_data;
    std::vector<float> tfn_opacity_data;
    float2 tfn_value_range;
    float tfn_range_rcp_norm;

    GPRTTextureOf<float> textureBuffer;
    std::vector<GPRTSampler> samplers;

    GPRTBufferOf<float3> aabbPositionsBuffer;
    GPRTBufferOf<float> volumeBuffer;
    GPRTBufferOf<float4> tfnColorBuffer;
    GPRTBufferOf<float> tfnOpacityBuffer;
    GPRTBufferOf<float2> tfnValueRangeBuffer;

    GPRTBufferOf<int3> volumeSizeBuffer;
    GPRTBufferOf<int> tfnColorSizeBuffer;
    GPRTBufferOf<int> tfnOpacitySizeBuffer;
  };
  std::vector<GeometryVolume> listOfGeometryVolume;
  float3 emptyAABB[2] = {{0.f, 0.f, 0.f}, {0.f, 0.f, 0.f}};
  GeometryVolume trashGeometryVolume;

  // General
  GPRTContext context;
  GPRTModule module;

  // Accel
  std::vector<float4x4> transformsVolume;
  GPRTBufferOf<float4x4> transformVolumeBuffer;
  std::vector<GPRTAccel> listOfVolumesBLAS;
  GPRTAccel volumeTLAS;

  // DataType
  MissProgData *missData;
  RayGenData *rayGenData;

  // Generate
  GPRTGeomTypeOf<VolumesGeomData> aabbGeomType;
  GPRTMissOf<MissProgData> miss;
  GPRTRayGenOf<RayGenData> rayGen;

  GPRTBufferOf<float3> accBuffer;
  GPRTBufferOf<uint32_t> frameBuffer;

  // Ray Gen Buffers
  GPRTBufferOf<float3> ambientLightIntensityBuffer;
  GPRTBufferOf<float3> directionalLightIntensityBuffer;
  GPRTBufferOf<float3> directionalLightDirBuffer;

  // Lights
  std::vector<float3> list_of_ambient_lights_intensity = {};
  std::vector<float3> list_of_directional_lights_intensity = {};
  std::vector<float3> list_of_directional_lights_direction = {};

  void initialVulkanResources(GPRTProgram new_example_deviceCode);
  // void resetVulkanGeometryResources(GPRTProgram new_example_deviceCode);
  void refreshVolume();
  void refreshLights();
  void updateVulkanResources();
  void updateLights();
  void buildSBT();
  void createVolume(Volume &volume);
  void createVolumeAndGeometryAccel();
  void createMiss();
  void createRayGen();
  void destoryVulkanResources();

  void loadLights(std::vector<float3> &list_of_ambient_lights_intensity,
                  std::vector<float3> &list_of_directional_lights_intensity,
                  std::vector<float3> &list_of_directional_lights_dir);

  void loadVolume(Volume &volume);
};

#endif
