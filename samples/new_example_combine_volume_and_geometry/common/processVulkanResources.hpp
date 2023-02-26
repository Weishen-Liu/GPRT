#pragma once
#ifndef INCLUDE_NEWEXAMPLE_COMMON_PROCESS_VULKAN_RESOURCES
#define INCLUDE_NEWEXAMPLE_COMMON_PROCESS_VULKAN_RESOURCES

#include <gprt.h>
// #include <gdt/math/mat.h>
#include "deviceCode.h"
#include <GLFW/glfw3.h>

#include "configureImgui.hpp"
#include "material.hpp"

struct VulkanResources {
    ConfigureImgui configureImgui;

    std::vector<uint32_t> maskTypes = {
        0b11111111, // Obj & Volume
        0b11111110  // Only Obj -> assign to Volume to hide Volume in Ray Trace
    };

    struct Geometry{
        GPRTGeomOf<TrianglesGeomData> trianglesGeom;
        TrianglesGeomData *geomData;
        GPRTAccel trianglesBLAS;

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

        // Geometry
        std::vector<int3> list_of_indices          = {};
        std::vector<float3> list_of_vertices       = {};
        std::vector<float3> list_of_colors         = {};
        std::vector<float3> list_of_vertex_normals = {};

        // Materials
        int obj_material_type;
        float3 lambertian_albedo;
        float3 metal_albedo;
        float metal_fuzz;
        float dielectric_ref_idx;

        std::vector<int> list_of_material_type = {};
        std::vector<float3> list_of_lambertians = {};
        std::vector<float3> list_of_metals_albedo = {};
        std::vector<float> list_of_metals_fuzz = {};
        std::vector<float> list_of_dielectrics = {};
    };
    std::vector<Geometry> listOfGeometry;
    Geometry trashGeometry; // Used for when no Obj wants to be rendered

    // Volume
    struct GeometryVolume{
        GPRTGeomOf<VolumesGeomData> aabbGeom;
        GPRTAccel aabbBLAS;
        VolumesGeomData *volumeData;

        // gdt::affine3f matrix;
        float2 volume_value_range;
        float volume_scale;

        // tfn
        std::vector<float4> tfn_colors_data;
        std::vector<float>  tfn_opacity_data;
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
    std::vector<float4x4> transformsObj;
    std::vector<GPRTAccel> listOfTrianglesBLAS;

    std::vector<float4x4> transformsVolume;
    std::vector<GPRTAccel> listOfVolumesBLAS;

    std::vector<float4x4> transformsObjAndVolume;
    GPRTBufferOf<float4x4> transformObjAndVolumeBuffer;
    std::vector<GPRTAccel> listOfObjAndVolumesBLAS;
    GPRTAccel objAndVolumeTLAS;
    std::vector<uint32_t> masksForObjAndVolume;
    GPRTBufferOf<uint32_t> masksBuffer;

    // DataType
    MissProgData *missData;
    RayGenData *rayGenData;

    // Generate
    GPRTGeomTypeOf<TrianglesGeomData> trianglesGeomType;
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
    void resetVulkanGeometryResources(GPRTProgram new_example_deviceCode);
    void refreshObjMaterial();
    void refreshObjAndVolume();
    void refreshLights();
    void updateGeometryMaterial(Geometry &geometry, Obj &obj);
    void updateVulkanResources();
    void updateLights();
    void buildSBT();
    void createGeometry(Obj &obj);
    void createVolume(Volume &volume);
    void createVolumeAndGeometryAccel();
    void createMiss();
    void createRayGen();
    void destoryVulkanResources();

    void loadModel(
        Obj& obj,
        std::vector<float3>& list_of_vertices,
        std::vector<int3>& list_of_indices,
        std::vector<float3>& list_of_colors,
        std::vector<float3>&list_of_vertex_normals
    );

    void loadMaterials(
        Obj& obj,
        int& obj_material_type,
        float3& lambertian_albedo,
        float3& metal_albedo,
        float& metal_fuzz,
        float& dielectric_ref_idx
    );

    void loadMaterials(
        Obj& obj,
        int list_of_indices_size,
        std::vector<int>&list_of_material_type,
        std::vector<float3>&list_of_lambertians,
        std::vector<float3>&list_of_metals_albedo,
        std::vector<float>&list_of_metals_fuzz,
        std::vector<float>&list_of_dielectrics
    );

    void loadLights(
        std::vector<float3>& list_of_ambient_lights_intensity,
        std::vector<float3>& list_of_directional_lights_intensity,
        std::vector<float3>& list_of_directional_lights_dir
    );

    void loadVolume(
        Volume &volume
    );
};

#endif
