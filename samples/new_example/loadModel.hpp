#ifndef   INCLUDE_GPRT
#define   INCLUDE_GPRT
// public GPRT API
#include <gprt.h>
#endif

#ifndef   INCLUDE_DEVICE_CODE
#define   INCLUDE_DEVICE_CODE
// our device-side data structures
#include "deviceCode.h"
#endif

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

void loadModel(const std::string MODEL_PATH,
               std::vector<float3>& list_of_vertices,
               std::vector<int3>& list_of_indices,
               std::vector<float3>& list_of_colors,
               std::vector<float3>&list_of_vertex_normals)
{
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string warn, err;

    if (!tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, MODEL_PATH.c_str())) {
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

            vertex.normal = {
                attrib.normals[3 * index.normal_index + 0],
                attrib.normals[3 * index.normal_index + 1],
                attrib.normals[3 * index.normal_index + 2]
            };

            // vertex.texCoord = {
            //     attrib.texcoords[2 * index.texcoord_index + 0],
            //     1.0f - attrib.texcoords[2 * index.texcoord_index + 1]
            // };

            vertex.color = {1.0f, 1.0f, 1.0f};

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