#include "processVulkanResources.hpp"

#include <iostream>
#include <iostream>
#include <vector>
#include <string>
#include <unordered_map>
#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader.h>


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

void VulkanResources::createGeometry() {
    trianglesGeomType = gprtGeomTypeCreate<TrianglesGeomData>(context, GPRT_TRIANGLES);
    gprtGeomTypeSetClosestHitProg(trianglesGeomType, 0, module, "TriangleMesh");
    trianglesGeom = gprtGeomCreate<TrianglesGeomData>(context, trianglesGeomType);

    // ------------------------------------------------------------------
    // triangle mesh
    // ------------------------------------------------------------------

    for (int each_path = 0; each_path < configureImgui.LIST_OF_OBJS.size(); each_path++)
    {
        if (configureImgui.LIST_OF_OBJS[each_path].choosed == false)
        {
            continue;
        }

        loadModel(
            configureImgui.LIST_OF_OBJS[each_path],
            list_of_vertices,
            list_of_indices,
            list_of_colors,
            list_of_vertex_normals,
            list_of_material_type,
            list_of_lambertians,
            list_of_metals_albedo,
            list_of_metals_fuzz,
            list_of_dielectrics);
    }

    if (list_of_vertices.size() > 0) {
        vertexBuffer = gprtDeviceBufferCreate<float3>(context, list_of_vertices.size(), static_cast<const void *>(list_of_vertices.data()));
        indexBuffer = gprtDeviceBufferCreate<int3>(context, list_of_indices.size(), static_cast<const void *>(list_of_indices.data()));
        normalBuffer = gprtDeviceBufferCreate<float3>(context, list_of_vertex_normals.size(), static_cast<const void *>(list_of_vertex_normals.data()));
        colorBuffer = gprtDeviceBufferCreate<float3>(context, list_of_colors.size(), static_cast<const void *>(list_of_colors.data()));
        materialTypeBuffer = gprtDeviceBufferCreate<int>(context, list_of_material_type.size(), static_cast<const void *>(list_of_material_type.data()));
        lambertianBuffer = gprtDeviceBufferCreate<float3>(context, list_of_lambertians.size(), static_cast<const void *>(list_of_lambertians.data()));
        metalAlbedoBuffer = gprtDeviceBufferCreate<float3>(context, list_of_metals_albedo.size(), static_cast<const void *>(list_of_metals_albedo.data()));
        metalFuzzBuffer = gprtDeviceBufferCreate<float>(context, list_of_metals_fuzz.size(), static_cast<const void *>(list_of_metals_fuzz.data()));
        dielectricBuffer = gprtDeviceBufferCreate<float>(context, list_of_dielectrics.size(), static_cast<const void *>(list_of_dielectrics.data()));
        accBuffer = gprtDeviceBufferCreate<float3>(context, configureImgui.fbSize.x * configureImgui.fbSize.y);
        frameBuffer = gprtHostBufferCreate<uint32_t>(context, configureImgui.fbSize.x * configureImgui.fbSize.y);
    }

    gprtTrianglesSetVertices(trianglesGeom, vertexBuffer, list_of_vertices.size());
    gprtTrianglesSetIndices(trianglesGeom, indexBuffer, list_of_indices.size());

    geomData = gprtGeomGetPointer(trianglesGeom);
    geomData->vertex = gprtBufferGetHandle(vertexBuffer);
    geomData->index = gprtBufferGetHandle(indexBuffer);

    geomData->normal = gprtBufferGetHandle(normalBuffer);
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
    list_of_metals_albedo = {};
    list_of_metals_fuzz = {};
    list_of_dielectrics = {};

    list_of_ambient_lights_intensity = {};
    list_of_directional_lights_intensity = {};
    list_of_directional_lights_direction = {};

    initialVulkanResources(new_example_deviceCode);
}

void VulkanResources::initialVulkanResources(GPRTProgram new_example_deviceCode) {
    // gprtRequestWindow(configureImgui.fbSize.x, configureImgui.fbSize.y, "New Example");
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
    rayGenData = gprtRayGenGetPointer(rayGen);
    // ----------- create lights  ----------------------------
    loadLights(
        list_of_ambient_lights_intensity,
        list_of_directional_lights_intensity,
        list_of_directional_lights_direction);

    if (list_of_ambient_lights_intensity.size() > 0) {
        ambientLightIntensityBuffer = gprtDeviceBufferCreate<float3>(
        context, list_of_ambient_lights_intensity.size(), static_cast<const void *>(list_of_ambient_lights_intensity.data()));
    }

    if (list_of_directional_lights_intensity.size() > 0) {
        directionalLightIntensityBuffer = gprtDeviceBufferCreate<float3>(
            context, list_of_directional_lights_intensity.size(), static_cast<const void *>(list_of_directional_lights_intensity.data()));
        directionalLightDirBuffer = gprtDeviceBufferCreate<float3>(
            context, list_of_directional_lights_direction.size(), static_cast<const void *>(list_of_directional_lights_direction.data()));
    }

    // ----------- set variables  ----------------------------
    rayGenData->frameBuffer = gprtBufferGetHandle(frameBuffer);
    rayGenData->world = gprtAccelGetHandle(world);
    rayGenData->accBuffer = gprtBufferGetHandle(accBuffer);
    rayGenData->accId = (uint64_t)configureImgui.accId;
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

void VulkanResources::loadModel(
    Obj& obj,
    std::vector<float3>& list_of_vertices,
    std::vector<int3>& list_of_indices,
    std::vector<float3>& list_of_colors,
    std::vector<float3>&list_of_vertex_normals,
    std::vector<int>&list_of_material_type,
    std::vector<float3>&list_of_lambertians,
    std::vector<float3>&list_of_metals_albedo,
    std::vector<float>&list_of_metals_fuzz,
    std::vector<float>&list_of_dielectrics
){
    std::string obj_path = obj.path;
    Material obj_material = obj.material;
    std::string material_type = obj_material.type;
    float4x4 transform = translation_matrix(obj.transform);

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

            // vertex.texCoord = {
            //     attrib.texcoords[2 * index.texcoord_index + 0],
            //     1.0f - attrib.texcoords[2 * index.texcoord_index + 1]
            // };

            vertex.color = {1.0f, 0.0f, 0.0f};

            if (uniqueVertices.count(vertex) == 0) {
                uniqueVertices[vertex] = static_cast<uint32_t>(list_of_vertices.size());
                float4 p = mul(
                    transform, float4(vertex.pos[0], vertex.pos[1], vertex.pos[2], 1.0)
                );
                list_of_vertices.push_back(p.xyz());
                list_of_colors.push_back(vertex.color);
                list_of_vertex_normals.push_back(vertex.normal);
            }

            indices.push_back(uniqueVertices[vertex]);
        }
    }
    
    for (int i = 0; i < indices.size(); i+=3) {
        int3 each_indices = {indices[i], indices[i+1], indices[i+2]};
        list_of_indices.push_back(each_indices);

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

