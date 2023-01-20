#pragma once
#ifndef INCLUDE_NEWEXAMPLE_COMMON_CONFIGURE_IMGUI
#define INCLUDE_NEWEXAMPLE_COMMON_CONFIGURE_IMGUI
#include <gprt.h>

#include <GLFW/glfw3.h>

#include "./math/random.h"
#include "lights.hpp"
#include "material.hpp"

#include <cstring>
#include <string>

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

struct ConfigureImgui {
    const char* current_item = NULL;
    const char* current_item_material = NULL;
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

    std::vector<Obj> LIST_OF_OBJS;
    int SELECTED_OBJS = 1;

    const std::vector<std::string> ALL_MATERIALS = {
        "Lambertian",
        "Metal",
        "Dielectric"
    };

    // Lights
    const std::vector<std::string> ALL_LIGHTS = {
        "Ambient",
        "Directional"
    };

    std::vector<AmbientLight> LIST_OF_AMBIENT_LIGHTS;
    std::vector<DirectionalLight> LIST_OF_DIRECTIONAL_LIGHTS;
    int SELECTED_LIGHTS = 0;

    const int2 fbSize = {800, 600};
    uint64_t accId = 0;

    bool showImgui = true;
    bool showLightControlPanel = false;
    bool showObjControlPanel = false;
    bool addNewObj = false;
    bool updateSelectedObj = false;
    bool objCountWarning = false;
    bool addNewLight = false;

    void render();
    void renderObjCP();
    void renderLightCP();

    void initObj();
    void addObj();
    void showSelectedObjCountWarning();
    void updateTransform();
    void updateMaterial();
    void updateMaterialDetail();

    void initLight();
    void addLight();
    void updateLight();

    void inputAndSlider(float3& source, float min_v, float max_v, const char *title, const char *inputLabel, const char *sliderLabel);
};

#endif
