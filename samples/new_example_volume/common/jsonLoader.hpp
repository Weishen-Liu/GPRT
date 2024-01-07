#pragma once
#ifndef INCLUDE_NEWEXAMPLE_VOLUME_COMMON_JSON_LOADER
#define INCLUDE_NEWEXAMPLE_VOLUME_COMMON_JSON_LOADER
#include "array.hpp"
#include "configureImgui.hpp"
#include <colormap.h>
#include <filesystem>   // C++17 is required by this project
#include <fstream>
#include <string>

namespace tfn {
typedef float2 vec2f;
typedef int2 vec2i;
typedef float3 vec3f;
typedef int3 vec3i;
typedef float4 vec4f;
typedef int4 vec4i;
}   // namespace tfn
#define TFN_MODULE_EXTERNAL_VECTOR_TYPES
#include "tfn/core.h"
#include "tfn/json.h"
using json = nlohmann::json;

array_1d_float4_t CreateArray1DFloat4(const std::vector<float4> &input, bool shared = false);
array_1d_float4_t CreateArray1DFloat4(const float4 *input, size_t len, bool shared = false);
array_1d_float4_t CreateColorMap(const std::string &name);
array_3d_scalar_t CreateArray3DScalarFromFile(const std::string &filename, int3 dims, ValueType type, size_t offset,
                                              bool is_big_endian);

std::string check_valid_extension(std::string filename);
void extract_root_and_workdir(std::string &file_path, json &root, std::string &workdir);
void reverse_byte_order(char *data, size_t elemCount, size_t elemSize);
void create_scene_volume(json &jsdata, std::string &workdir, Volume &volume);
void create_scene_camera(const json &jsview, Volume &volume);
float2 range_from_json(json jsrange);
void create_scene_tfn(const json &jsview, Volume &volume);
void load_volume_data_from_file(Volume &volume);
float2 get_volume_value_range(array_3d_scalar_t data);
#endif
