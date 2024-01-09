#include "processVulkanResources.hpp"

#include "jsonLoader.hpp"
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

struct Vertex {
  float3 pos;
  float3 normal;
  float3 color;
  float2 texCoord;

  static VkVertexInputBindingDescription getBindingDescription() {
    VkVertexInputBindingDescription bindingDescription{};
    bindingDescription.binding = 0;
    bindingDescription.stride = sizeof(Vertex);
    bindingDescription.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

    return bindingDescription;
  }

  static std::array<VkVertexInputAttributeDescription, 3> getAttributeDescriptions() {
    std::array<VkVertexInputAttributeDescription, 3> attributeDescriptions{};
    attributeDescriptions[0].binding = 0;
    attributeDescriptions[0].location = 0;
    attributeDescriptions[0].format = VK_FORMAT_R32G32B32_SFLOAT;
    attributeDescriptions[0].offset = offsetof(Vertex, pos);

    attributeDescriptions[1].binding = 0;
    attributeDescriptions[1].location = 1;
    attributeDescriptions[1].format = VK_FORMAT_R32G32B32_SFLOAT;
    attributeDescriptions[1].offset = offsetof(Vertex, color);

    attributeDescriptions[2].binding = 0;
    attributeDescriptions[2].location = 2;
    attributeDescriptions[2].format = VK_FORMAT_R32G32_SFLOAT;
    attributeDescriptions[2].offset = offsetof(Vertex, texCoord);

    return attributeDescriptions;
  }

  bool operator==(const Vertex &other) const {
    return pos == other.pos && color == other.color && texCoord == other.texCoord;
  }
};

namespace std {
template <> struct hash<Vertex> {
  size_t operator()(Vertex const &vertex) const {
    return ((hash<float3>()(vertex.pos) ^ (hash<float3>()(vertex.color) << 1)) >> 1) ^
           (hash<float2>()(vertex.texCoord) << 1);
  }
};
}   // namespace std

void
VulkanResources::createVolume(Volume &volume) {
  loadVolume(volume);

  GeometryVolume newGeometryVolume;
  float2 value_range = get_volume_value_range(volume.data);

  // Create our AABB geometry. Every AABB is defined using two float3's. The
  // first float3 defines the bottom lower left near corner, and the second
  // float3 defines the upper far right corner.
  newGeometryVolume.aabbPositionsBuffer = gprtDeviceBufferCreate<float3>(context, 2, volume.aabbPositions);
  newGeometryVolume.aabbGeom = gprtGeomCreate<VolumesGeomData>(context, aabbGeomType);
  gprtAABBsSetPositions(newGeometryVolume.aabbGeom, newGeometryVolume.aabbPositionsBuffer, 1 /* just one aabb */);

  int volume_size = volume.data->dims.long_product();
  int tfn_color_size = int(volume.transferFunction.color->size() / 4);
  int tfn_opacity_size = int(volume.transferFunction.opacity->size() / 2);

  newGeometryVolume.textureBuffer = gprtDeviceTextureCreate<float>(
      context, GPRT_IMAGE_TYPE_3D, GPRT_FORMAT_D32_SFLOAT, volume.data->dims.x, volume.data->dims.y,
      volume.data->dims.z, /* generate mipmaps */ true, (float *) (volume.data->data()));

  newGeometryVolume.samplers.push_back(gprtSamplerCreate(context, GPRT_FILTER_LINEAR));

  newGeometryVolume.volume_scale = 1.f / (value_range.y - value_range.x);
  newGeometryVolume.tfn_value_range.x = value_range.x;
  newGeometryVolume.tfn_value_range.y = value_range.y;
  newGeometryVolume.tfn_range_rcp_norm =
      1.f / (newGeometryVolume.tfn_value_range.y - newGeometryVolume.tfn_value_range.x);

  newGeometryVolume.tfn_opacity_data.resize(tfn_opacity_size);
  for (int i = 0; i < newGeometryVolume.tfn_opacity_data.size(); ++i) {
    newGeometryVolume.tfn_opacity_data[i] = ((float *) volume.transferFunction.opacity->data())[2 * i + 1];
  }

  newGeometryVolume.tfnColorBuffer =
      gprtDeviceBufferCreate<float4>(context, tfn_color_size, (float4 *) volume.transferFunction.color->data());
  newGeometryVolume.tfnOpacityBuffer =
      gprtDeviceBufferCreate<float>(context, tfn_opacity_size, newGeometryVolume.tfn_opacity_data.data());
  newGeometryVolume.tfnValueRangeBuffer =
      gprtDeviceBufferCreate<float2>(context, 1, &newGeometryVolume.tfn_value_range);

  newGeometryVolume.volumeData = gprtGeomGetParameters(newGeometryVolume.aabbGeom);
  newGeometryVolume.volumeData->aabb_position = gprtBufferGetHandle(newGeometryVolume.aabbPositionsBuffer);
  newGeometryVolume.volumeData->tfn_color = gprtBufferGetHandle(newGeometryVolume.tfnColorBuffer);
  newGeometryVolume.volumeData->tfn_opacity = gprtBufferGetHandle(newGeometryVolume.tfnOpacityBuffer);
  newGeometryVolume.volumeData->tfn_value_range = gprtBufferGetHandle(newGeometryVolume.tfnValueRangeBuffer);

  newGeometryVolume.volumeData->volume = gprtTextureGetHandle(newGeometryVolume.textureBuffer);
  for (uint32_t i = 0; i < newGeometryVolume.samplers.size(); ++i) {
    newGeometryVolume.volumeData->samplers[i] = gprtSamplerGetHandle(newGeometryVolume.samplers[i]);
  }

  newGeometryVolume.volumeSizeBuffer = gprtDeviceBufferCreate<int3>(context, 1, (int3 *) &volume.data->dims);
  newGeometryVolume.tfnColorSizeBuffer = gprtDeviceBufferCreate<int>(context, 1, &tfn_color_size);
  newGeometryVolume.tfnOpacitySizeBuffer = gprtDeviceBufferCreate<int>(context, 1, &tfn_opacity_size);
  newGeometryVolume.volumeData->volume_size_buffer = gprtBufferGetHandle(newGeometryVolume.volumeSizeBuffer);
  newGeometryVolume.volumeData->tfn_color_size_buffer = gprtBufferGetHandle(newGeometryVolume.tfnColorSizeBuffer);
  newGeometryVolume.volumeData->tfn_opacity_size_buffer = gprtBufferGetHandle(newGeometryVolume.tfnOpacitySizeBuffer);

  // Note, we must create an "AABB" accel rather than a triangles accel.
  newGeometryVolume.aabbBLAS = gprtAABBAccelCreate(context, 1, &newGeometryVolume.aabbGeom);
  gprtAccelBuild(context, newGeometryVolume.aabbBLAS, GPRT_BUILD_MODE_FAST_TRACE_NO_UPDATE);
  listOfGeometryVolume.push_back(newGeometryVolume);
}

void
VulkanResources::buildSBT() {
  // ##################################################################
  // build *SBT* required to trace the groups
  // ##################################################################
  // gprtBuildPipeline(context);
  gprtBuildShaderBindingTable(context, GPRT_SBT_ALL);
}

void
VulkanResources::refreshVolume() {
  listOfVolumesBLAS = {};
  transformsVolume = {};

  std::cout << "Accel" << std::endl;
  createVolumeAndGeometryAccel();
  rayGenData->world = gprtAccelGetHandle(volumeTLAS);
  rayGenData->accId = (uint64_t) configureImgui.accId;
  buildSBT();
}

void
VulkanResources::refreshLights() {
  list_of_ambient_lights_intensity = {};
  list_of_directional_lights_intensity = {};
  list_of_directional_lights_direction = {};

  createRayGen();

  buildSBT();
}

void
VulkanResources::updateVulkanResources() {
  std::cout << "Volume" << std::endl;
  for (auto &each_volume : configureImgui.LIST_OF_VOLUMES) {
    createVolume(each_volume);
  }

  // Create Trash Volume Geometry for 0 TLAS Buffer
  trashGeometryVolume = listOfGeometryVolume[0];
  trashGeometryVolume.aabbPositionsBuffer = gprtDeviceBufferCreate<float3>(context, 2, emptyAABB);
  gprtAABBsSetPositions(trashGeometryVolume.aabbGeom, trashGeometryVolume.aabbPositionsBuffer, 1 /* just one aabb */);
  trashGeometryVolume.aabbBLAS = gprtAABBAccelCreate(context, 1, &trashGeometryVolume.aabbGeom);
  gprtAccelBuild(context, trashGeometryVolume.aabbBLAS, GPRT_BUILD_MODE_FAST_TRACE_NO_UPDATE);

  // ------------------------------------------------------------------
  // the group/accel for that mesh
  // ------------------------------------------------------------------
  std::cout << "Accel" << std::endl;
  createVolumeAndGeometryAccel();

  // ##################################################################
  // set miss and raygen program required for SBT
  // ##################################################################
  std::cout << "Miss" << std::endl;
  createMiss();

  std::cout << "RayGen" << std::endl;
  createRayGen();

  buildSBT();
}

void
VulkanResources::initialVulkanResources(GPRTProgram new_example_deviceCode) {
  // gprtRequestWindow(configureImgui.fbSize.x, configureImgui.fbSize.y, "New Example");
  context = gprtContextCreate(nullptr, 1);
  module = gprtModuleCreate(context, new_example_deviceCode);
  // gprtRequestRayTypeCount(2);

  aabbGeomType = gprtGeomTypeCreate<VolumesGeomData>(context, GPRT_AABBS /* <- This is new! */);
  gprtGeomTypeSetClosestHitProg(aabbGeomType, 0, module, "AABBClosestHit");
  gprtGeomTypeSetIntersectionProg(aabbGeomType, 0, module, "AABBIntersection");

  miss = gprtMissCreate<MissProgData>(context, module, "miss");
  rayGen = gprtRayGenCreate<RayGenData>(context, module, "simpleRayGen");

  accBuffer = gprtDeviceBufferCreate<float3>(context, configureImgui.fbSize.x * configureImgui.fbSize.y);
  frameBuffer = gprtHostBufferCreate<uint32_t>(context, configureImgui.fbSize.x * configureImgui.fbSize.y);

  updateVulkanResources();

  gprtBuildShaderBindingTable(context);
}

void
VulkanResources::createVolumeAndGeometryAccel() {
  for (int i = 0; i < configureImgui.LIST_OF_VOLUMES.size(); i++) {
    for (auto &eachInstance : configureImgui.LIST_OF_VOLUMES[i].instances) {
      if (eachInstance.choosed) {
        listOfVolumesBLAS.push_back(listOfGeometryVolume[i].aabbBLAS);
        float4x4 translateMatrix = translation_matrix(eachInstance.translate);
        float3 axisX = float3(1.f, 0.f, 0.f);
        float3 axisY = float3(0.f, 1.f, 0.f);
        float3 axisZ = float3(0.f, 0.f, 1.f);
        float4x4 rotateX = rotation_matrix(rotation_quat(axisX, eachInstance.rotate.x));
        float4x4 rotateY = rotation_matrix(rotation_quat(axisY, eachInstance.rotate.y));
        float4x4 rotateZ = rotation_matrix(rotation_quat(axisZ, eachInstance.rotate.z));
        float4x4 rotateMatrix = mul(rotateZ, mul(rotateY, rotateX));
        float4x4 scaleMatrix = scaling_matrix(eachInstance.scale);
        transformsVolume.push_back(mul(scaleMatrix, mul(rotateMatrix, transpose(translateMatrix))));
      }
    }
  }

  if (transformsVolume.size() > 0) {
    for (auto &eachVolumeBLAS : listOfVolumesBLAS) {
      listOfVolumesBLAS.push_back(eachVolumeBLAS);
    }
    for (auto &eachVolumeTransform : transformsVolume) {
      transformsVolume.push_back(eachVolumeTransform);
    }
  }

  if (listOfVolumesBLAS.size() > 0) {
    volumeTLAS = gprtInstanceAccelCreate(context, listOfVolumesBLAS.size(), listOfVolumesBLAS.data());
    transformVolumeBuffer = gprtDeviceBufferCreate<float4x4>(context, transformsVolume.size(), transformsVolume.data());
    gprtInstanceAccelSet4x4Transforms(volumeTLAS, transformVolumeBuffer);
  } else {
    volumeTLAS = gprtInstanceAccelCreate(context, 1, &trashGeometryVolume.aabbBLAS);
  }
  gprtAccelBuild(context, volumeTLAS, GPRT_BUILD_MODE_FAST_TRACE_NO_UPDATE);
}

void
VulkanResources::createMiss() {
  // ----------- set variables  ----------------------------
  missData = gprtMissGetParameters(miss);
  missData->color0 = float3(1.0f, 1.0f, 1.0f);
  missData->color1 = float3(0.0f, 0.0f, 0.0f);
}

void
VulkanResources::updateLights() {
  rayGenData = gprtRayGenGetParameters(rayGen);
  // ----------- create lights  ----------------------------
  loadLights(list_of_ambient_lights_intensity, list_of_directional_lights_intensity,
             list_of_directional_lights_direction);

  if (list_of_ambient_lights_intensity.size() > 0) {
    ambientLightIntensityBuffer = gprtDeviceBufferCreate<float3>(context, list_of_ambient_lights_intensity.size(),
                                                                 list_of_ambient_lights_intensity.data());

    rayGenData->ambient_lights_intensity = gprtBufferGetHandle(ambientLightIntensityBuffer);
  }

  if (list_of_directional_lights_intensity.size() > 0) {
    directionalLightIntensityBuffer = gprtDeviceBufferCreate<float3>(
        context, list_of_directional_lights_intensity.size(), list_of_directional_lights_intensity.data());
    directionalLightDirBuffer = gprtDeviceBufferCreate<float3>(context, list_of_directional_lights_direction.size(),
                                                               list_of_directional_lights_direction.data());

    rayGenData->directional_lights_intensity = gprtBufferGetHandle(directionalLightIntensityBuffer);
    rayGenData->directional_lights_dir = gprtBufferGetHandle(directionalLightDirBuffer);
  }
  rayGenData->ambient_light_size = (uint64_t) list_of_ambient_lights_intensity.size();
  rayGenData->directional_light_size = (uint64_t) list_of_directional_lights_intensity.size();
}

void
VulkanResources::createRayGen() {
  updateLights();

  // ----------- set variables  ----------------------------
  rayGenData->world = gprtAccelGetHandle(volumeTLAS);
  rayGenData->frameBuffer = gprtBufferGetHandle(frameBuffer);
  rayGenData->accBuffer = gprtBufferGetHandle(accBuffer);
  rayGenData->accId = (uint64_t) configureImgui.accId;
}

void
VulkanResources::destoryVulkanResources() {
  if (list_of_ambient_lights_intensity.size()) {
    gprtBufferDestroy(ambientLightIntensityBuffer);
  }
  if (list_of_directional_lights_intensity.size()) {
    gprtBufferDestroy(directionalLightIntensityBuffer);
    gprtBufferDestroy(directionalLightDirBuffer);
  }

  gprtBufferDestroy(frameBuffer);
  gprtBufferDestroy(accBuffer);

  if (transformsVolume.size()) {
    gprtBufferDestroy(transformVolumeBuffer);
  }

  for (auto &eachGeo : listOfGeometryVolume) {
    gprtBufferDestroy(eachGeo.aabbPositionsBuffer);
    gprtBufferDestroy(eachGeo.tfnColorBuffer);
    gprtBufferDestroy(eachGeo.tfnOpacityBuffer);
    gprtBufferDestroy(eachGeo.tfnValueRangeBuffer);
    gprtTextureDestroy(eachGeo.textureBuffer);
    for (auto &sampler : eachGeo.samplers) {
      gprtSamplerDestroy(sampler);
    }
    gprtAccelDestroy(eachGeo.aabbBLAS);
    gprtGeomDestroy(eachGeo.aabbGeom);
  }

  gprtRayGenDestroy(rayGen);
  gprtMissDestroy(miss);
  gprtAccelDestroy(volumeTLAS);
  gprtGeomTypeDestroy(aabbGeomType);
  gprtModuleDestroy(module);
  gprtContextDestroy(context);
}

void
VulkanResources::loadLights(std::vector<float3> &list_of_ambient_lights_intensity,
                            std::vector<float3> &list_of_directional_lights_intensity,
                            std::vector<float3> &list_of_directional_lights_dir) {
  for (int each_ambient_light; each_ambient_light < configureImgui.LIST_OF_AMBIENT_LIGHTS.size();
       each_ambient_light++) {
    if (configureImgui.LIST_OF_AMBIENT_LIGHTS[each_ambient_light].choosed == false) {
      continue;
    }
    list_of_ambient_lights_intensity.push_back(configureImgui.LIST_OF_AMBIENT_LIGHTS[each_ambient_light].intensity);
  }

  for (int each_directional_light; each_directional_light < configureImgui.LIST_OF_DIRECTIONAL_LIGHTS.size();
       each_directional_light++) {
    if (configureImgui.LIST_OF_DIRECTIONAL_LIGHTS[each_directional_light].choosed == false) {
      continue;
    }
    list_of_directional_lights_intensity.push_back(
        configureImgui.LIST_OF_DIRECTIONAL_LIGHTS[each_directional_light].intensity);
    list_of_directional_lights_dir.push_back(
        configureImgui.LIST_OF_DIRECTIONAL_LIGHTS[each_directional_light].direction);
  }
}

void
VulkanResources::loadVolume(Volume &volume) {
  load_volume_data_from_file(volume);
}
