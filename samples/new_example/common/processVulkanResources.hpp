#ifndef INCLUDE_GPRT
#define INCLUDE_GPRT
// public GPRT API
#include <gprt.h>
#endif

#ifndef INCLUDE_DEVICE_CODE
#define INCLUDE_DEVICE_CODE
// our device-side data structures
#include "deviceCode.h"
#endif

#ifndef INCLUDE_GLFW
#define INCLUDE_GLFW
// library for windowing
#include <GLFW/glfw3.h>
#endif

#ifndef INCLUDE_CONTANTS
#define INCLUDE_CONTANTS
#include "./constants.hpp"
#endif

#ifndef INCLUDE_LOADMODEL
#define INCLUDE_LOADMODEL
#include "./loadModel.hpp"
#endif

#include <iostream>

struct VulkanResources {
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
    GPRTBufferOf<Lambertian> lambertianBuffer;
    GPRTBufferOf<float3> metalAlbedoBuffer;
    GPRTBufferOf<float> metalFuzzBuffer;
    GPRTBufferOf<Dielectric> dielectricBuffer;
    GPRTBufferOf<float3> accBuffer;
    GPRTBufferOf<uint32_t> frameBuffer;

    // Ray Gen Buffers
    GPRTBufferOf<float3> ambientLightIntensityBuffer;
    GPRTBufferOf<float3> directionalLightIntensityBuffer;
    GPRTBufferOf<float3> directionalLightDirBuffer;

    void initialVulkanResources(GPRTProgram new_example_deviceCode);
    void resetVulkanGeometryResources(GPRTProgram new_example_deviceCode);
    void createGeometry();
    void createAccel();
    void createMiss();
    void createRayGen();
    void destoryVulkanResources();
};

void VulkanResources::createGeometry() {
    trianglesGeomType = gprtGeomTypeCreate<TrianglesGeomData>(
        context, GPRT_TRIANGLES);
    gprtGeomTypeSetClosestHitProg(trianglesGeomType, 0, module, "TriangleMesh");

    // ------------------------------------------------------------------
    // triangle mesh
    // ------------------------------------------------------------------

    int each_material_type = -1;
    for (int each_path = 0; each_path < LIST_OF_OBJS.size(); each_path++)
    {
        if (LIST_OF_OBJS[each_path].choosed == false)
        {
            continue;
        }

        each_material_type = material_types[each_path];
        loadModel(
            LIST_OF_OBJS[each_path].path,
            list_of_vertices,
            list_of_indices,
            list_of_colors,
            list_of_vertex_normals,
            each_material_type,
            list_of_material_type,
            list_of_lambertians,
            list_of_metals_albedo,
            list_of_metals_fuzz,
            list_of_dielectrics,
            translation_matrix(LIST_OF_OBJS[each_path].transform));
    }
    vertexBuffer = gprtDeviceBufferCreate<float3>(context, list_of_vertices.size(), static_cast<const void *>(list_of_vertices.data()));
    indexBuffer = gprtDeviceBufferCreate<int3>(context, list_of_indices.size(), static_cast<const void *>(list_of_indices.data()));
    normalBuffer = gprtDeviceBufferCreate<float3>(context, list_of_vertex_normals.size(), static_cast<const void *>(list_of_vertex_normals.data()));
    colorBuffer = gprtDeviceBufferCreate<float3>(context, list_of_colors.size(), static_cast<const void *>(list_of_colors.data()));
    materialTypeBuffer = gprtDeviceBufferCreate<int>(context, list_of_material_type.size(), static_cast<const void *>(list_of_material_type.data()));
    lambertianBuffer = gprtDeviceBufferCreate<Lambertian>(context, list_of_lambertians.size(), static_cast<const void *>(list_of_lambertians.data()));
    metalAlbedoBuffer = gprtDeviceBufferCreate<float3>(context, list_of_metals_albedo.size(), static_cast<const void *>(list_of_metals_albedo.data()));
    metalFuzzBuffer = gprtDeviceBufferCreate<float>(context, list_of_metals_fuzz.size(), static_cast<const void *>(list_of_metals_fuzz.data()));
    dielectricBuffer = gprtDeviceBufferCreate<Dielectric>(context, list_of_dielectrics.size(), static_cast<const void *>(list_of_dielectrics.data()));
    accBuffer = gprtDeviceBufferCreate<float3>(context, fbSize.x * fbSize.y);
    frameBuffer = gprtHostBufferCreate<uint32_t>(context, fbSize.x * fbSize.y);

    trianglesGeom = gprtGeomCreate<TrianglesGeomData>(context, trianglesGeomType);
    gprtTrianglesSetVertices(trianglesGeom, vertexBuffer,
                             list_of_vertices.size());
    gprtTrianglesSetIndices(trianglesGeom, indexBuffer,
                            list_of_indices.size());

    geomData = gprtGeomGetPointer(trianglesGeom);
    geomData->vertex = gprtBufferGetHandle(vertexBuffer);
    geomData->normal = gprtBufferGetHandle(normalBuffer);
    geomData->index = gprtBufferGetHandle(indexBuffer);
    geomData->color = gprtBufferGetHandle(colorBuffer);
    geomData->material_type = gprtBufferGetHandle(materialTypeBuffer);
    geomData->metal_albedo = gprtBufferGetHandle(metalAlbedoBuffer);
    geomData->metal_fuzz = gprtBufferGetHandle(metalFuzzBuffer);
    geomData->lambertian = gprtBufferGetHandle(lambertianBuffer);
    geomData->dielectric = gprtBufferGetHandle(dielectricBuffer);
}

void VulkanResources::resetVulkanGeometryResources(GPRTProgram new_example_deviceCode)
{
    // Reset Geo Buffers
    list_of_indices = {};
    list_of_vertices = {};
    list_of_colors = {};
    list_of_vertex_normals= {};
    list_of_material_type = {};
    list_of_lambertians = {};
    list_of_metals = {};
    list_of_metals_albedo = {};
    list_of_metals_fuzz = {};
    list_of_dielectrics = {};

    list_of_ambient_lights_intensity = {};
    list_of_directional_lights_intensity = {};
    list_of_directional_lights_direction = {};

    initialVulkanResources(new_example_deviceCode);
}

void VulkanResources::initialVulkanResources(GPRTProgram new_example_deviceCode) {
    // gprtRequestWindow(fbSize.x, fbSize.y, "New Example");
    context = gprtContextCreate(nullptr, 1);
    module = gprtModuleCreate(context, new_example_deviceCode);

    // ##################################################################
    // set up all the *GEOMETRY* graph we want to render
    // ##################################################################
    std::cout<<"Geo"<<std::endl;
    createGeometry();

    // ------------------------------------------------------------------
    // the group/accel for that mesh
    // ------------------------------------------------------------------
    std::cout<<"Accel"<<std::endl;
    createAccel();

    // ##################################################################
    // set miss and raygen program required for SBT
    // ##################################################################
    std::cout<<"Miss"<<std::endl;
    createMiss();
    std::cout<<"RayGen"<<std::endl;
    createRayGen();
    
    // ##################################################################
    // build *SBT* required to trace the groups
    // ##################################################################
    gprtBuildPipeline(context);
    gprtBuildShaderBindingTable(context, GPRT_SBT_ALL);
}

void VulkanResources::createAccel() {
    trianglesAccel = gprtTrianglesAccelCreate(context, 1, &trianglesGeom);
    gprtAccelBuild(context, trianglesAccel);

    world = gprtInstanceAccelCreate(context, 1, &trianglesAccel);
    gprtAccelBuild(context, world);
}

void VulkanResources::createMiss() {
    miss = gprtMissCreate<MissProgData>(context, module, "miss");

    // ----------- set variables  ----------------------------
    missData = gprtMissGetPointer(miss);
    missData->color0 = float3(1.0f, 1.0f, 1.0f);
    missData->color1 = float3(0.0f, 0.0f, 0.0f);

}

void VulkanResources::createRayGen() {
    rayGen = gprtRayGenCreate<RayGenData>(context, module, "simpleRayGen");

    // ----------- create lights  ----------------------------
    loadLights(
        list_of_ambient_lights_intensity,
        list_of_directional_lights_intensity,
        list_of_directional_lights_direction);

    ambientLightIntensityBuffer = gprtDeviceBufferCreate<float3>(
        context, list_of_ambient_lights_intensity.size(), static_cast<const void *>(list_of_ambient_lights_intensity.data()));
    directionalLightIntensityBuffer = gprtDeviceBufferCreate<float3>(
        context, list_of_directional_lights_intensity.size(), static_cast<const void *>(list_of_directional_lights_intensity.data()));
    directionalLightDirBuffer = gprtDeviceBufferCreate<float3>(
        context, list_of_directional_lights_direction.size(), static_cast<const void *>(list_of_directional_lights_direction.data()));

    // ----------- set variables  ----------------------------
    rayGenData = gprtRayGenGetPointer(rayGen);
    rayGenData->frameBuffer = gprtBufferGetHandle(frameBuffer);
    rayGenData->world = gprtAccelGetHandle(world);
    rayGenData->accBuffer = gprtBufferGetHandle(accBuffer);
    rayGenData->accId = (uint64_t)accId;
    rayGenData->ambient_lights_intensity = gprtBufferGetHandle(ambientLightIntensityBuffer);
    rayGenData->ambient_light_size = (uint64_t)list_of_ambient_lights_intensity.size();
    rayGenData->directional_lights_intensity = gprtBufferGetHandle(directionalLightIntensityBuffer);
    rayGenData->directional_lights_dir = gprtBufferGetHandle(directionalLightDirBuffer);
    rayGenData->directional_light_size = (uint64_t)list_of_directional_lights_intensity.size();
}

void VulkanResources::destoryVulkanResources() {
    gprtBufferDestroy(ambientLightIntensityBuffer);
    gprtBufferDestroy(directionalLightIntensityBuffer);
    gprtBufferDestroy(directionalLightDirBuffer);
    gprtBufferDestroy(vertexBuffer);
    gprtBufferDestroy(normalBuffer);
    gprtBufferDestroy(indexBuffer);
    gprtBufferDestroy(materialTypeBuffer);
    gprtBufferDestroy(metalAlbedoBuffer);
    gprtBufferDestroy(metalFuzzBuffer);
    gprtBufferDestroy(lambertianBuffer);
    gprtBufferDestroy(dielectricBuffer);
    gprtBufferDestroy(colorBuffer);
    gprtBufferDestroy(frameBuffer);
    gprtBufferDestroy(accBuffer);
    gprtRayGenDestroy(rayGen);
    gprtMissDestroy(miss);
    gprtAccelDestroy(trianglesAccel);
    gprtAccelDestroy(world);
    gprtGeomDestroy(trianglesGeom);
    gprtGeomTypeDestroy(trianglesGeomType);
    gprtModuleDestroy(module);
    gprtContextDestroy(context);
}