#ifndef   INCLUDE_GPRT
#define   INCLUDE_GPRT
// public GPRT API
#include <gprt.h>
#endif

#ifndef   INCLUDE_DEVICE_CODE
#define   INCLUDE_DEVICE_CODE
// our device-side data structures
#include "../deviceCode.h"
#endif

#include <iostream>
#include <vector>
#include <string>
#include <unordered_map>

#ifndef TINYOBJLOADER_IMPLEMENTATION
#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader.h>
#endif

#ifndef   INCLUDE_LIGHTS
#define   INCLUDE_LIGHTS
#include "./common/lights.hpp"
#endif

#ifndef   INCLUDE_MATERIAL
#define   INCLUDE_MATERIAL
#include "./materials/material.hpp"
#endif

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

        for (int each_material = 0; each_material < ALL_MATERIALS.size(); each_material++)
        {
            if (material_type == ALL_MATERIALS[each_material])
            {
                list_of_material_type.push_back(each_material);
                break;
            }
        }
    }
}

void loadLights(
    std::vector<float3>& list_of_ambient_lights_intensity,
    std::vector<float3>& list_of_directional_lights_intensity,
    std::vector<float3>& list_of_directional_lights_dir
)
{
    for (int each_ambient_light; each_ambient_light < LIST_OF_AMBIENT_LIGHTS.size(); each_ambient_light++) {
        if (LIST_OF_AMBIENT_LIGHTS[each_ambient_light].choosed == false)
        {
            continue;
        }
        list_of_ambient_lights_intensity.push_back(LIST_OF_AMBIENT_LIGHTS[each_ambient_light].intensity);
    }

    // Check If Ambient Light Disable
    if (list_of_ambient_lights_intensity.size() == 0)
    {
        list_of_ambient_lights_intensity.push_back(float3(0.f, 0.f, 0.f));
    }

    for (int each_directional_light; each_directional_light < LIST_OF_DIRECTIONAL_LIGHTS.size(); each_directional_light++) {
        if (LIST_OF_DIRECTIONAL_LIGHTS[each_directional_light].choosed == false)
        {
            continue;
        }
        list_of_directional_lights_intensity.push_back(LIST_OF_DIRECTIONAL_LIGHTS[each_directional_light].intensity);
        list_of_directional_lights_dir.push_back(LIST_OF_DIRECTIONAL_LIGHTS[each_directional_light].direction);
    }

    // Check If Directional Light Disable
    if (list_of_directional_lights_intensity.size() == 0)
    {
        list_of_directional_lights_intensity.push_back(float3(0.f, 0.f, 0.f));
        list_of_directional_lights_dir.push_back(float3(1.f, 0.f, 0.f));
    }
}
