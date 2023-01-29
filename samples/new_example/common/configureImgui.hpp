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

struct LoadInObj {
    std::string name;
    std::string path;

    LoadInObj(std::string inputName, std::string inputPath) : name(inputName), path(inputPath) {}
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

    std::vector<LoadInObj*> INITIAL_OBJ = {
        new LoadInObj("Viking Room", "/media/storage0/weishen/GPRT-1/samples/new_example/models/viking_room.obj"),
        new LoadInObj("Cube",        "/media/storage0/weishen/GPRT-1/samples/new_example/models/Cube.obj"),
        new LoadInObj("Mario",       "/media/storage0/weishen/GPRT-1/samples/new_example/models/Mario.obj"),
        new LoadInObj("Bunny",       "/media/storage0/weishen/GPRT-1/samples/new_example/models/bunny.obj"),
        new LoadInObj("Sphere",      "/media/storage0/weishen/GPRT-1/samples/new_example/models/sphere.obj"),
        new LoadInObj("Sponza",      "/media/storage0/weishen/GPRT-1/samples/new_example/models/sponza.obj"),
        new LoadInObj("Horse",       "/media/storage0/weishen/GPRT-1/samples/new_example/models/horse.obj"),
        new LoadInObj("New Cube",    "/media/storage0/weishen/GPRT-1/samples/new_example/models/newCube.obj"),
        new LoadInObj("Cottage",     "/media/storage0/weishen/GPRT-1/samples/new_example/models/cottage.obj"),
    };

    std::vector<Obj> LIST_OF_OBJS;
    int SELECTED_OBJS = 0;

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
    bool addNewLight = false;

    bool updateObjSelection = false;
    bool updateObjTransform = false;
    bool updateObjMaterials = false;    
    bool updateLights = false;

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

    void inputAndSlider(float3& source, float min_v, float max_v, const char *title, const char *inputLabel, const char *sliderLabel, bool& trigger);
};

#endif
