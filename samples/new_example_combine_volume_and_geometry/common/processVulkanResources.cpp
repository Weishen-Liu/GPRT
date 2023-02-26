#include "processVulkanResources.hpp"

#include <iostream>
#include <iostream>
#include <vector>
#include <string>
#include <unordered_map>
#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader.h>
#include "jsonLoader.hpp"

struct Vertex
{
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

    bool operator==(const Vertex& other) const {
        return pos == other.pos && color == other.color && texCoord == other.texCoord;
    }
};

namespace std {
    template<> struct hash<Vertex> {
        size_t operator()(Vertex const& vertex) const {
            return ((hash<float3>()(vertex.pos) ^
                    (hash<float3>()(vertex.color) << 1)) >> 1) ^
                    (hash<float2>()(vertex.texCoord) << 1);
        }
    };
}

void VulkanResources::createGeometry(Obj &obj) {
    // ------------------------------------------------------------------
    // triangle mesh
    // ------------------------------------------------------------------
    Geometry geometry;
    geometry.trianglesGeom = gprtGeomCreate<TrianglesGeomData>(context, trianglesGeomType);

    loadModel(
        obj,
        geometry.list_of_vertices,
        geometry.list_of_indices,
        geometry.list_of_colors,
        geometry.list_of_vertex_normals
    );
    geometry.vertexBuffer = gprtDeviceBufferCreate<float3>(
        context, geometry.list_of_vertices.size(), geometry.list_of_vertices.data());
    geometry.indexBuffer = gprtDeviceBufferCreate<int3>(
        context, geometry.list_of_indices.size(), geometry.list_of_indices.data());
    geometry.normalBuffer = gprtDeviceBufferCreate<float3>(
        context, geometry.list_of_vertex_normals.size(), geometry.list_of_vertex_normals.data());
    geometry.colorBuffer = gprtDeviceBufferCreate<float3>(
        context, geometry.list_of_colors.size(), geometry.list_of_colors.data());

    geometry.geomData = gprtGeomGetParameters(geometry.trianglesGeom);
    geometry.geomData->vertex = gprtBufferGetHandle(geometry.vertexBuffer);
    geometry.geomData->index = gprtBufferGetHandle(geometry.indexBuffer);
    geometry.geomData->normal = gprtBufferGetHandle(geometry.normalBuffer);
    geometry.geomData->color = gprtBufferGetHandle(geometry.colorBuffer);

    gprtTrianglesSetVertices(geometry.trianglesGeom, geometry.vertexBuffer, geometry.list_of_vertices.size());
    gprtTrianglesSetIndices(geometry.trianglesGeom, geometry.indexBuffer, geometry.list_of_indices.size());

    geometry.trianglesBLAS = gprtTrianglesAccelCreate(context, 1, &geometry.trianglesGeom);
    gprtAccelBuild(context, geometry.trianglesBLAS);
    listOfGeometry.push_back(geometry);
}

void VulkanResources::createVolume(Volume &volume) {
    loadVolume(volume);

    GeometryVolume newGeometryVolume;
    float2 value_range = get_volume_value_range(volume.data);

    // Create our AABB geometry. Every AABB is defined using two float3's. The
    // first float3 defines the bottom lower left near corner, and the second
    // float3 defines the upper far right corner.
    newGeometryVolume.aabbPositionsBuffer = gprtDeviceBufferCreate<float3>(context, 2, volume.aabbPositions);
    newGeometryVolume.aabbGeom = gprtGeomCreate<VolumesGeomData>(context, aabbGeomType);
    gprtAABBsSetPositions(newGeometryVolume.aabbGeom, newGeometryVolume.aabbPositionsBuffer, 1 /* just one aabb */);

    // float3 scale = volume.grid_spacing * float3(volume.data->dims.x, volume.data->dims.y, volume.data->dims.z);
    // gdt::vec3f scale_gdt = gdt::vec3f(scale.x, scale.y, scale.z);
    // float3 translate = volume.grid_origin;
    // gdt::vec3f translate_gdt = gdt::vec3f(translate.x, translate.y, translate.z);
    // newGeometryVolume.matrix = gdt::affine3f::translate(translate_gdt) * gdt::affine3f::scale(scale_gdt);

    int volume_size = volume.data->dims.long_product();
    int tfn_color_size = int(volume.transferFunction.color->size() / 4);
    int tfn_opacity_size = int(volume.transferFunction.opacity->size() / 2);

    newGeometryVolume.textureBuffer = gprtDeviceTextureCreate<float>(context, GPRT_IMAGE_TYPE_3D, GPRT_FORMAT_D32_SFLOAT, volume.data->dims.x, volume.data->dims.y, volume.data->dims.z, /* generate mipmaps */ true, (float*)(volume.data->data()));

    newGeometryVolume.samplers.push_back(gprtSamplerCreate(context, GPRT_FILTER_LINEAR));

    newGeometryVolume.volume_scale = 1.f / (value_range.y - value_range.x);
    newGeometryVolume.tfn_value_range.x = value_range.x;
    newGeometryVolume.tfn_value_range.y = value_range.y;
    newGeometryVolume.tfn_range_rcp_norm = 1.f / (newGeometryVolume.tfn_value_range.y - newGeometryVolume.tfn_value_range.x);

    newGeometryVolume.tfn_opacity_data.resize(tfn_opacity_size);
    for (int i = 0; i < newGeometryVolume.tfn_opacity_data.size(); ++i) {
        newGeometryVolume.tfn_opacity_data[i] = ((float*)volume.transferFunction.opacity->data())[2 * i + 1];
    }

    newGeometryVolume.tfnColorBuffer = gprtDeviceBufferCreate<float4>(context, tfn_color_size, (float4*)volume.transferFunction.color->data());
    newGeometryVolume.tfnOpacityBuffer = gprtDeviceBufferCreate<float>(context, tfn_opacity_size, newGeometryVolume.tfn_opacity_data.data());
    newGeometryVolume.tfnValueRangeBuffer = gprtDeviceBufferCreate<float2>(context, 1, &newGeometryVolume.tfn_value_range);

    newGeometryVolume.volumeData = gprtGeomGetParameters(newGeometryVolume.aabbGeom);
    newGeometryVolume.volumeData->aabb_position = gprtBufferGetHandle(newGeometryVolume.aabbPositionsBuffer);
    newGeometryVolume.volumeData->tfn_color = gprtBufferGetHandle(newGeometryVolume.tfnColorBuffer);
    newGeometryVolume.volumeData->tfn_opacity = gprtBufferGetHandle(newGeometryVolume.tfnOpacityBuffer);
    newGeometryVolume.volumeData->tfn_value_range = gprtBufferGetHandle(newGeometryVolume.tfnValueRangeBuffer);

    newGeometryVolume.volumeData->volume = gprtTextureGetHandle(newGeometryVolume.textureBuffer);
    for (uint32_t i = 0; i < newGeometryVolume.samplers.size(); ++i) {
        newGeometryVolume.volumeData->samplers[i] = gprtSamplerGetHandle(newGeometryVolume.samplers[i]);
    }

    // newGeometryVolume.volumeData->volume_size = volume_size;
    // newGeometryVolume.volumeData->tfn_color_size = tfn_color_size;
    // newGeometryVolume.volumeData->tfn_opacity_size = tfn_opacity_size;

    newGeometryVolume.volumeSizeBuffer = gprtDeviceBufferCreate<int3>(context, 1, (int3*)&volume.data->dims);
    newGeometryVolume.tfnColorSizeBuffer = gprtDeviceBufferCreate<int>(context, 1, &tfn_color_size);
    newGeometryVolume.tfnOpacitySizeBuffer = gprtDeviceBufferCreate<int>(context, 1, &tfn_opacity_size);
    newGeometryVolume.volumeData->volume_size_buffer = gprtBufferGetHandle(newGeometryVolume.volumeSizeBuffer);
    newGeometryVolume.volumeData->tfn_color_size_buffer = gprtBufferGetHandle(newGeometryVolume.tfnColorSizeBuffer);
    newGeometryVolume.volumeData->tfn_opacity_size_buffer = gprtBufferGetHandle(newGeometryVolume.tfnOpacitySizeBuffer);

    // Note, we must create an "AABB" accel rather than a triangles accel.
    newGeometryVolume.aabbBLAS = gprtAABBAccelCreate(context, 1, &newGeometryVolume.aabbGeom);
    gprtAccelBuild(context, newGeometryVolume.aabbBLAS);
    listOfGeometryVolume.push_back(newGeometryVolume);
}

void VulkanResources::updateGeometryMaterial(Geometry &geometry, Obj &obj)
{
    loadMaterials(
        obj,
        geometry.obj_material_type,
        geometry.lambertian_albedo,
        geometry.metal_albedo,
        geometry.metal_fuzz,
        geometry.dielectric_ref_idx
    );

    geometry.materialTypeBuffer = gprtDeviceBufferCreate<int>(
        context, 1, &geometry.obj_material_type);
    geometry.lambertianBuffer = gprtDeviceBufferCreate<float3>(
        context, 1, &geometry.lambertian_albedo);
    geometry.metalAlbedoBuffer = gprtDeviceBufferCreate<float3>(
        context, 1, &geometry.metal_albedo);
    geometry.metalFuzzBuffer = gprtDeviceBufferCreate<float>(
        context, 1, &geometry.metal_fuzz);
    geometry.dielectricBuffer = gprtDeviceBufferCreate<float>(
        context, 1, &geometry.dielectric_ref_idx);

    geometry.geomData->material.type               = (uint64_t)geometry.obj_material_type;
    geometry.geomData->material.metal_albedo       = geometry.metal_albedo;
    geometry.geomData->material.metal_fuzz         = geometry.metal_fuzz;
    geometry.geomData->material.lambertian_albedo  = geometry.lambertian_albedo;
    geometry.geomData->material.dielectric_ref_idx = geometry.dielectric_ref_idx;

    // loadMaterials(
    //     obj,
    //     geometry.list_of_indices.size(),
    //     geometry.list_of_material_type,
    //     geometry.list_of_lambertians,
    //     geometry.list_of_metals_albedo,
    //     geometry.list_of_metals_fuzz,
    //     geometry.list_of_dielectrics
    // );

    // geometry.materialTypeBuffer = gprtDeviceBufferCreate<int>(
    //     context, geometry.list_of_material_type.size(), geometry.list_of_material_type.data());
    // geometry.lambertianBuffer = gprtDeviceBufferCreate<float3>(
    //     context, geometry.list_of_lambertians.size(), geometry.list_of_lambertians.data());
    // geometry.metalAlbedoBuffer = gprtDeviceBufferCreate<float3>(
    //     context, geometry.list_of_metals_albedo.size(), geometry.list_of_metals_albedo.data());
    // geometry.metalFuzzBuffer = gprtDeviceBufferCreate<float>(
    //     context, geometry.list_of_metals_fuzz.size(), geometry.list_of_metals_fuzz.data());
    // geometry.dielectricBuffer = gprtDeviceBufferCreate<float>(
    //     context, geometry.list_of_dielectrics.size(), geometry.list_of_dielectrics.data());

    geometry.geomData->material_type = gprtBufferGetHandle(geometry.materialTypeBuffer);
    geometry.geomData->metal_albedo  = gprtBufferGetHandle(geometry.metalAlbedoBuffer);
    geometry.geomData->metal_fuzz    = gprtBufferGetHandle(geometry.metalFuzzBuffer);
    geometry.geomData->lambertian    = gprtBufferGetHandle(geometry.lambertianBuffer);
    geometry.geomData->dielectric    = gprtBufferGetHandle(geometry.dielectricBuffer);
}

void VulkanResources::resetVulkanGeometryResources(GPRTProgram deviceCode)
{
    list_of_ambient_lights_intensity = {};
    list_of_directional_lights_intensity = {};
    list_of_directional_lights_direction = {};

    listOfGeometry = {};
    listOfTrianglesBLAS = {};
    transformsObj = {};
    transformsVolume = {};
    initialVulkanResources(deviceCode);
}

void VulkanResources::buildSBT()
{
    // ##################################################################
    // build *SBT* required to trace the groups
    // ##################################################################
    // gprtBuildPipeline(context);
    gprtBuildShaderBindingTable(context, GPRT_SBT_ALL);
}

void VulkanResources::refreshObjMaterial()
{
    for (int each_path = 0; each_path < configureImgui.LIST_OF_OBJS.size(); each_path++)
    {
        updateGeometryMaterial(listOfGeometry[each_path], configureImgui.LIST_OF_OBJS[each_path]);
    }

    refreshObjAndVolume();
}

void VulkanResources::refreshObjAndVolume()
{
    listOfVolumesBLAS = {};
    transformsVolume = {};
    listOfTrianglesBLAS = {};
    transformsObj = {};
    listOfObjAndVolumesBLAS = {};
    transformsObjAndVolume = {};
    masksForObjAndVolume = {};

    std::cout<<"Accel"<<std::endl;
    createVolumeAndGeometryAccel();
    rayGenData->world = gprtAccelGetHandle(objAndVolumeTLAS);
    rayGenData->accId = (uint64_t)configureImgui.accId;
    buildSBT();
}

void VulkanResources::refreshLights()
{
    list_of_ambient_lights_intensity = {};
    list_of_directional_lights_intensity = {};
    list_of_directional_lights_direction = {};

    createRayGen();
    
    buildSBT();
}

void VulkanResources::updateVulkanResources() {
    // ##################################################################
    // set up all the *GEOMETRY* graph we want to render
    // ##################################################################
    std::cout<<"Geo"<<std::endl;
    for (auto &each_obj : configureImgui.LIST_OF_OBJS)
    {
        createGeometry(each_obj);
    }

    // Create Trash Geometry for 0 TLAS Buffer
    trashGeometry = listOfGeometry[0];
    gprtTrianglesSetIndices(trashGeometry.trianglesGeom, trashGeometry.indexBuffer, 0);
    trashGeometry.trianglesBLAS = gprtTrianglesAccelCreate(context, 1, &trashGeometry.trianglesGeom);
    gprtAccelBuild(context, trashGeometry.trianglesBLAS);

    for (int each_path = 0; each_path < configureImgui.LIST_OF_OBJS.size(); each_path++)
    {
        updateGeometryMaterial(listOfGeometry[each_path], configureImgui.LIST_OF_OBJS[each_path]);
    }

    std::cout<<"Volume"<<std::endl;
    for (auto &each_volume : configureImgui.LIST_OF_VOLUMES)
    {
        createVolume(each_volume);
    }

    // Create Trash Volume Geometry for 0 TLAS Buffer
    trashGeometryVolume = listOfGeometryVolume[0];
    trashGeometryVolume.aabbPositionsBuffer = gprtDeviceBufferCreate<float3>(context, 2, emptyAABB);
    gprtAABBsSetPositions(trashGeometryVolume.aabbGeom, trashGeometryVolume.aabbPositionsBuffer, 1 /* just one aabb */);
    trashGeometryVolume.aabbBLAS = gprtAABBAccelCreate(context, 1, &trashGeometryVolume.aabbGeom);
    gprtAccelBuild(context, trashGeometryVolume.aabbBLAS);

    // ------------------------------------------------------------------
    // the group/accel for that mesh
    // ------------------------------------------------------------------
    std::cout<<"Accel"<<std::endl;
    createVolumeAndGeometryAccel();

    // ##################################################################
    // set miss and raygen program required for SBT
    // ##################################################################
    std::cout<<"Miss"<<std::endl;
    createMiss();

    std::cout<<"RayGen"<<std::endl;
    createRayGen();
    
    buildSBT();
}

void VulkanResources::initialVulkanResources(GPRTProgram new_example_deviceCode) {
    // gprtRequestWindow(configureImgui.fbSize.x, configureImgui.fbSize.y, "New Example");
    context = gprtContextCreate(nullptr, 1);
    module = gprtModuleCreate(context, new_example_deviceCode);
    // gprtRequestRayTypeCount(2);

    trianglesGeomType = gprtGeomTypeCreate<TrianglesGeomData>(context, GPRT_TRIANGLES);
    gprtGeomTypeSetClosestHitProg(trianglesGeomType, 0, module, "TriangleMesh");

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

void VulkanResources::createVolumeAndGeometryAccel() {
    for (int i = 0; i < configureImgui.LIST_OF_OBJS.size(); i++)
    {
        for (auto &eachInstance: configureImgui.LIST_OF_OBJS[i].instances)
        {
            if (eachInstance.choosed)
            {
                listOfTrianglesBLAS.push_back(listOfGeometry[i].trianglesBLAS);
                transformsObj.push_back(transpose(translation_matrix(eachInstance.transform)));
            }
        }
    }

    for (int i = 0; i < configureImgui.LIST_OF_VOLUMES.size(); i++)
    {
        for (auto &eachInstance: configureImgui.LIST_OF_VOLUMES[i].instances)
        {
            if (eachInstance.choosed)
            {
                listOfVolumesBLAS.push_back(listOfGeometryVolume[i].aabbBLAS);
                transformsVolume.push_back(transpose(translation_matrix(eachInstance.transform)));
            }
        }
    }

    if (transformsObj.size() > 0) {
        for (auto& eachTriangleBLAS: listOfTrianglesBLAS) {
            listOfObjAndVolumesBLAS.push_back(eachTriangleBLAS);
            masksForObjAndVolume.push_back(maskTypes[0]);
        }
        for (auto& eachTriangleTransform: transformsObj) {
            transformsObjAndVolume.push_back(eachTriangleTransform);
        }
    }

    if (transformsVolume.size() > 0) {
        for (auto& eachVolumeBLAS: listOfVolumesBLAS) {
            listOfObjAndVolumesBLAS.push_back(eachVolumeBLAS);
            masksForObjAndVolume.push_back(maskTypes[1]);
        }
        for (auto& eachVolumeTransform: transformsVolume) {
            transformsObjAndVolume.push_back(eachVolumeTransform);
        }
    }

    if (listOfObjAndVolumesBLAS.size() > 0) {
        objAndVolumeTLAS = gprtInstanceAccelCreate(context, listOfObjAndVolumesBLAS.size(), listOfObjAndVolumesBLAS.data());
        transformObjAndVolumeBuffer = gprtDeviceBufferCreate<float4x4>(context, transformsObjAndVolume.size(), transformsObjAndVolume.data());
        gprtInstanceAccelSet4x4Transforms(objAndVolumeTLAS, transformObjAndVolumeBuffer);
        masksBuffer = gprtDeviceBufferCreate<uint32_t>(context, masksForObjAndVolume.size(), masksForObjAndVolume.data());
        gprtInstanceAccelSetVisibilityMasks(objAndVolumeTLAS, masksBuffer);
    } else {
        objAndVolumeTLAS = gprtInstanceAccelCreate(context, 1, &trashGeometry.trianglesBLAS);
    }
    gprtAccelBuild(context, objAndVolumeTLAS);
}

void VulkanResources::createMiss() {
    // ----------- set variables  ----------------------------
    missData = gprtMissGetParameters(miss);
    missData->color0 = float3(1.0f, 1.0f, 1.0f);
    missData->color1 = float3(0.0f, 0.0f, 0.0f);
}

void VulkanResources::updateLights() {
    rayGenData = gprtRayGenGetParameters(rayGen);
    // ----------- create lights  ----------------------------
    loadLights(
        list_of_ambient_lights_intensity,
        list_of_directional_lights_intensity,
        list_of_directional_lights_direction);

    if (list_of_ambient_lights_intensity.size() > 0) {
        ambientLightIntensityBuffer = gprtDeviceBufferCreate<float3>(
        context, list_of_ambient_lights_intensity.size(), list_of_ambient_lights_intensity.data());

        rayGenData->ambient_lights_intensity = gprtBufferGetHandle(ambientLightIntensityBuffer);
    }

    if (list_of_directional_lights_intensity.size() > 0) {
        directionalLightIntensityBuffer = gprtDeviceBufferCreate<float3>(
            context, list_of_directional_lights_intensity.size(), list_of_directional_lights_intensity.data());
        directionalLightDirBuffer = gprtDeviceBufferCreate<float3>(
            context, list_of_directional_lights_direction.size(), list_of_directional_lights_direction.data());

        rayGenData->directional_lights_intensity = gprtBufferGetHandle(directionalLightIntensityBuffer);
        rayGenData->directional_lights_dir = gprtBufferGetHandle(directionalLightDirBuffer);
    }
    rayGenData->ambient_light_size = (uint64_t)list_of_ambient_lights_intensity.size();
    rayGenData->directional_light_size = (uint64_t)list_of_directional_lights_intensity.size();
}

void VulkanResources::createRayGen() {
    updateLights();

    // ----------- set variables  ----------------------------
    rayGenData->world = gprtAccelGetHandle(objAndVolumeTLAS);
    rayGenData->frameBuffer = gprtBufferGetHandle(frameBuffer);
    rayGenData->accBuffer = gprtBufferGetHandle(accBuffer);
    rayGenData->accId = (uint64_t)configureImgui.accId;
}

void VulkanResources::destoryVulkanResources() {
    if (list_of_ambient_lights_intensity.size())
    {
        gprtBufferDestroy(ambientLightIntensityBuffer);
    }
    if (list_of_directional_lights_intensity.size())
    {
        gprtBufferDestroy(directionalLightIntensityBuffer);
        gprtBufferDestroy(directionalLightDirBuffer);
    }
    
    for (auto &eachGeo: listOfGeometry)
    {
        gprtBufferDestroy(eachGeo.vertexBuffer);
        gprtBufferDestroy(eachGeo.normalBuffer);
        gprtBufferDestroy(eachGeo.indexBuffer);
        gprtBufferDestroy(eachGeo.colorBuffer);
        gprtBufferDestroy(eachGeo.materialTypeBuffer);
        gprtBufferDestroy(eachGeo.metalAlbedoBuffer);
        gprtBufferDestroy(eachGeo.metalFuzzBuffer);
        gprtBufferDestroy(eachGeo.lambertianBuffer);
        gprtBufferDestroy(eachGeo.dielectricBuffer);
        gprtAccelDestroy(eachGeo.trianglesBLAS);
        gprtGeomDestroy(eachGeo.trianglesGeom);
    }

    gprtBufferDestroy(frameBuffer);
    gprtBufferDestroy(accBuffer);

    if (transformsObjAndVolume.size()) {
        gprtBufferDestroy(transformObjAndVolumeBuffer);
    }

    if (masksForObjAndVolume.size()) {
        gprtBufferDestroy(masksBuffer);
    }

    for (auto &eachGeo: listOfGeometryVolume) 
    {
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
    gprtAccelDestroy(objAndVolumeTLAS);
    gprtGeomTypeDestroy(trianglesGeomType);
    gprtGeomTypeDestroy(aabbGeomType);
    gprtModuleDestroy(module);
    gprtContextDestroy(context);
}

void VulkanResources::loadModel(
    Obj& obj,
    std::vector<float3>& list_of_vertices,
    std::vector<int3>& list_of_indices,
    std::vector<float3>& list_of_colors,
    std::vector<float3>&list_of_vertex_normals
){
    std::string obj_path = obj.path;

    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string warn, err;

    if (!tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, obj_path.c_str())) {
        throw std::runtime_error(warn + err);
    }

    std::unordered_map<Vertex, uint32_t> uniqueVertices{};
    std::vector<int> indices;
    for (const auto& shape : shapes) {
        for (const auto& index : shape.mesh.indices) {
            Vertex vertex{};

            vertex.pos = {
                attrib.vertices[3 * index.vertex_index + 0],
                attrib.vertices[3 * index.vertex_index + 1],
                attrib.vertices[3 * index.vertex_index + 2]
            };

            if (attrib.normals.size()) {
                vertex.normal = {
                    attrib.normals[3 * index.normal_index + 0],
                    attrib.normals[3 * index.normal_index + 1],
                    attrib.normals[3 * index.normal_index + 2]
                };
            } else {
                vertex.normal = float3(0.f, 0.f, 0.f);
            }
            vertex.normal = float3(0.f, 0.f, 0.f);

            // vertex.texCoord = {
            //     attrib.texcoords[2 * index.texcoord_index + 0],
            //     1.0f - attrib.texcoords[2 * index.texcoord_index + 1]
            // };

            vertex.color = {1.0f, 0.0f, 0.0f};

            if (uniqueVertices.count(vertex) == 0) {
                uniqueVertices[vertex] = static_cast<uint32_t>(list_of_vertices.size());
                list_of_vertices.push_back(vertex.pos);
                list_of_colors.push_back(vertex.color);
                list_of_vertex_normals.push_back(vertex.normal);
            }

            indices.push_back(uniqueVertices[vertex]);
        }
    }
    
    for (int i = 0; i < indices.size(); i+=3) {
        int3 each_indices = {indices[i], indices[i+1], indices[i+2]};
        list_of_indices.push_back(each_indices);
    }
}

void VulkanResources::loadMaterials(
    Obj& obj,
    int& obj_material_type,
    float3& lambertians_albedo,
    float3& metals_albedo,
    float& metals_fuzz,
    float& dielectrics_ref_idx
)
{
    Material obj_material = obj.material;
    std::string material_type = obj_material.type;

    lambertians_albedo = obj_material.lambertian.albedo;
    metals_albedo = obj_material.metal.albedo;
    metals_fuzz = obj_material.metal.fuzz;
    dielectrics_ref_idx = obj_material.dielectric.ref_idx;

    for (int each_material = 0; each_material < configureImgui.ALL_MATERIALS.size(); each_material++)
    {
        if (material_type == configureImgui.ALL_MATERIALS[each_material])
        {
            obj_material_type = each_material;
            break;
        }
    }
}

void VulkanResources::loadMaterials(
    Obj& obj,
    int list_of_indices_size,
    std::vector<int>&list_of_material_type,
    std::vector<float3>&list_of_lambertians,
    std::vector<float3>&list_of_metals_albedo,
    std::vector<float>&list_of_metals_fuzz,
    std::vector<float>&list_of_dielectrics
)
{
    Material obj_material = obj.material;
    std::string material_type = obj_material.type;
    for (int i = 0; i < list_of_indices_size; i++) {

        list_of_lambertians.push_back(obj_material.lambertian.albedo);
        list_of_metals_albedo.push_back(obj_material.metal.albedo);
        list_of_metals_fuzz.push_back(obj_material.metal.fuzz);
        list_of_dielectrics.push_back(obj_material.dielectric.ref_idx);

        for (int each_material = 0; each_material < configureImgui.ALL_MATERIALS.size(); each_material++)
        {
            if (material_type == configureImgui.ALL_MATERIALS[each_material])
            {
                list_of_material_type.push_back(each_material);
                break;
            }
        }
    }
}

void VulkanResources::loadLights(
    std::vector<float3>& list_of_ambient_lights_intensity,
    std::vector<float3>& list_of_directional_lights_intensity,
    std::vector<float3>& list_of_directional_lights_dir
)
{
    for (int each_ambient_light; each_ambient_light < configureImgui.LIST_OF_AMBIENT_LIGHTS.size(); each_ambient_light++) {
        if (configureImgui.LIST_OF_AMBIENT_LIGHTS[each_ambient_light].choosed == false)
        {
            continue;
        }
        list_of_ambient_lights_intensity.push_back(configureImgui.LIST_OF_AMBIENT_LIGHTS[each_ambient_light].intensity);
    }

    for (int each_directional_light; each_directional_light < configureImgui.LIST_OF_DIRECTIONAL_LIGHTS.size(); each_directional_light++) {
        if (configureImgui.LIST_OF_DIRECTIONAL_LIGHTS[each_directional_light].choosed == false)
        {
            continue;
        }
        list_of_directional_lights_intensity.push_back(configureImgui.LIST_OF_DIRECTIONAL_LIGHTS[each_directional_light].intensity);
        list_of_directional_lights_dir.push_back(configureImgui.LIST_OF_DIRECTIONAL_LIGHTS[each_directional_light].direction);
    }
}

void VulkanResources::loadVolume(Volume &volume)
{
    load_volume_data_from_file(volume);
}
