#pragma once
#ifndef INCLUDE_NEWEXAMPLE_COMMON_PROCESS_VULKAN_RESOURCES
#define INCLUDE_NEWEXAMPLE_COMMON_PROCESS_VULKAN_RESOURCES

#include <gprt.h>
#include "deviceCode.h"
#include <GLFW/glfw3.h>

#include "configureImgui.hpp"
#include "material.hpp"

struct VulkanResources {
    ConfigureImgui configureImgui;

    // General
    GPRTContext context;
    GPRTModule module;

    // Accel
    GPRTAccel trianglesAccel;
    GPRTAccel world;

    // DataType
    TrianglesGeomData *geomData;
    MissProgData *missData;
    RayGenData *rayGenData;

    // Generate
    GPRTGeomTypeOf<TrianglesGeomData> trianglesGeomType;
    GPRTGeomOf<TrianglesGeomData> trianglesGeom;
    GPRTMissOf<MissProgData> miss;
    GPRTRayGenOf<RayGenData> rayGen;

    // Geom Buffers
    GPRTBufferOf<float3> vertexBuffer;
    GPRTBufferOf<int3> indexBuffer;
    GPRTBufferOf<float3> normalBuffer;
    GPRTBufferOf<float3> colorBuffer;
    GPRTBufferOf<int> materialTypeBuffer;
    GPRTBufferOf<float3> lambertianBuffer;
    GPRTBufferOf<float3> metalAlbedoBuffer;
    GPRTBufferOf<float> metalFuzzBuffer;
    GPRTBufferOf<float> dielectricBuffer;
    GPRTBufferOf<float3> accBuffer;
    GPRTBufferOf<uint32_t> frameBuffer;

    // Ray Gen Buffers
    GPRTBufferOf<float3> ambientLightIntensityBuffer;
    GPRTBufferOf<float3> directionalLightIntensityBuffer;
    GPRTBufferOf<float3> directionalLightDirBuffer;

    // Geometry
    std::vector<int3> list_of_indices = {};
    std::vector<float3> list_of_vertices = {};
    std::vector<float3> list_of_colors = {};
    std::vector<float3> list_of_vertex_normals= {};

    // Lights
    std::vector<float3> list_of_ambient_lights_intensity = {};
    std::vector<float3> list_of_directional_lights_intensity = {};
    std::vector<float3> list_of_directional_lights_direction = {};

    // Materials
    std::vector<int> list_of_material_type = {};
    std::vector<float3> list_of_lambertians = {};
    std::vector<float3> list_of_metals_albedo = {};
    std::vector<float> list_of_metals_fuzz = {};
    std::vector<float> list_of_dielectrics = {};

    void initialVulkanResources(GPRTProgram new_example_deviceCode);
    void resetVulkanGeometryResources(GPRTProgram new_example_deviceCode);
    void createGeometry();
    void createAccel();
    void createMiss();
    void createRayGen();
    void destoryVulkanResources();

    void loadModel(
        Obj& obj,
        std::vector<float3>& list_of_vertices,
        std::vector<int3>& list_of_indices,
        std::vector<float3>& list_of_colors,
        std::vector<float3>&list_of_vertex_normals,
        std::vector<int>&list_of_material_type,
        std::vector<float3>&list_of_lambertians,
        std::vector<float3>&list_of_metals_albedo,
        std::vector<float>&list_of_metals_fuzz,
        std::vector<float>&list_of_dielectrics);

    void loadLights(
        std::vector<float3>& list_of_ambient_lights_intensity,
        std::vector<float3>& list_of_directional_lights_intensity,
        std::vector<float3>& list_of_directional_lights_dir);
};

#endif
