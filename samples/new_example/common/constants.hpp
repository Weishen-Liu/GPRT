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

#ifndef INCLUDE_LIGHTS
#define INCLUDE_LIGHTS
#include "./lights.hpp"
#endif

#ifndef INCLUDE_MATERIAL
#define INCLUDE_MATERIAL
#include "./material.hpp"
#endif

const std::string TEXTURE_PATH = "/media/storage0/weishen/GPRT-1/samples/new_example/textures/viking_room.png";

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
std::vector<Metal> list_of_metals = {};
std::vector<float3> list_of_metals_albedo = {};
std::vector<float> list_of_metals_fuzz = {};
std::vector<float> list_of_dielectrics = {};

// initial image resolution
const int2 fbSize = {800, 600};
uint64_t accId = 0;
GLuint fbTexture{0};

// Camera
float3 lookFrom = {0.f, 10.f, 10.f};
float3 lookAt = {0.f, 0.f, 0.f};
float3 lookUp = {0.f, 1.f, 0.f};
float cosFovy = 20.f;

// ImGUI
float imgui_test_input = 0.f;
static const char* current_item = NULL;
static const char* current_item_material = NULL;
int current_item_index = -1;

struct CurrentLight
{
    const char* name = NULL;
    const char* type = NULL;
    int index = -1;
}current_light;

const std::vector<std::string> ALL_MODEL_NAME = {
    "Viking Room",
    "Cube",
    "Mario",
    "Bunny",
    "Sphere",
    "Sponza",
    "Horse"
};

const std::vector<std::string> ALL_MODEL_PATH = {
    "/media/storage0/weishen/GPRT-1/samples/new_example/models/viking_room.obj",
    "/media/storage0/weishen/GPRT-1/samples/new_example/models/Cube.obj",
    "/media/storage0/weishen/GPRT-1/samples/new_example/models/Mario.obj",
    "/media/storage0/weishen/GPRT-1/samples/new_example/models/bunny.obj",
    "/media/storage0/weishen/GPRT-1/samples/new_example/models/sphere.obj",
    "/media/storage0/weishen/GPRT-1/samples/new_example/models/sponza.obj",
    "/media/storage0/weishen/GPRT-1/samples/new_example/models/horse.obj"
};

std::vector<float3> INITIAL_TRANSFORM = {
    float3(0.0f, 0.0f, 0.0f),
    float3(0.0f, 0.0f, 1.1f),
    float3(0.344626f, 12.9949f, -0.114619f),
    float3(2 * sin(2 * M_PI * .33), 2 * cos(2 * M_PI * .33), 1.5f),
    float3(2 * sin(2 * M_PI * .66), 2 * cos(2 * M_PI * .66), 1.5f),
    float3(2 * sin(2 * M_PI * 1.0), 2 * cos(2 * M_PI * 1.0), 1.5f),
    float3(0.0f, 0.0f, -4.0f)
};

const std::vector<std::string> ALL_MATERIALS = {
    "Lambertian",
    "Metal",
    "Dielectric"
};

struct Material
{
  const char* type;
  Lambertian lambertian;
  Metal metal;
  Dielectric dielectric;
};

struct Obj {
    std::string name;
    std::string path;
    bool choosed = false;
    float3 transform;
    Material material;
};

std::vector<Obj> LIST_OF_OBJS;
int SELECTED_OBJS = 1;

// Lights
const std::vector<std::string> ALL_LIGHTS = {
    "Ambient",
    "Directional"
};

std::vector<AmbientLight> LIST_OF_AMBIENT_LIGHTS;
std::vector<DirectionalLight> LIST_OF_DIRECTIONAL_LIGHTS;
int SELECTED_LIGHTS = 0;
