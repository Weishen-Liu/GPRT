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
    Geometry trashGeometry;

    // Volume
    GPRTGeomTypeOf<AABBGeomData> aabbGeomType;
    GPRTBufferOf<float3> aabbPositionsBuffer;
    GPRTGeomOf<AABBGeomData> aabbGeom;

    // General
    GPRTContext context;
    GPRTModule module;

    // Accel
    std::vector<float4x4> transforms;
    GPRTBufferOf<float4x4> transformBuffer;
    std::vector<GPRTAccel> listOfTrianglesBLAS;
    GPRTAccel trianglesTLAS;
    GPRTAccel aabbBLAS;
    GPRTAccel aabbTLAS;

    // DataType
    MissProgData *missData;
    RayGenData *rayGenData;

    // Generate
    GPRTGeomTypeOf<TrianglesGeomData> trianglesGeomType;
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
    void refreshObj();
    void refreshObjMaterial();
    void refreshLights();
    void updateGeometryMaterial(Geometry &geometry, Obj &obj);
    void updateVulkanResources();
    void updateLights();
    void buildSBT();
    void createGeometry(int each_path);
    void createAccel();
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
};

#endif
