#pragma once
#ifndef INCLUDE_NEWEXAMPLE_COMMON_CONFIGURE_IMGUI
#define INCLUDE_NEWEXAMPLE_COMMON_CONFIGURE_IMGUI
#include <gprt.h>

#include <GLFW/glfw3.h>

#include "./math/random.h"
#include "lights.hpp"
#include "material.hpp"
#include "array.hpp"

#include <cstring>
#include <string>

struct Material
{
  const char* type;
  Lambertian lambertian;
  Metal metal;
  Dielectric dielectric;
};

struct Instance {
    std::string name;
    bool choosed = false;
    float3 transform;
};

struct Obj {
    std::string name;
    std::string path;
    Material material;

    bool openInstanceWindow = false;
    bool addInstanceWindow = false;
    std::vector<Instance> instances;
    int SELECTED_INSTANCE = 0;
    const char* current_instance = NULL;
    const char* current_instance_material = NULL;
    int current_instance_index = -1;
    int instanceUniqueName = 0;
    float3 defaultTransform = float3(0.f, 0.f, 0.f);
    int generateInstance = 1;
};

struct TransferFunction {
  array_1d_float4_t color;
  array_1d_scalar_t opacity;
  float2 value_range;
};

struct Volume {
    struct Camera {
        float3 from = float3(0.f, 0.f, 0.f);

        float3 position{0, -1, 0};
        float3 at;

        float3 upVector{0, 1, 0};
        float3 up;
        float  perspective_fovy;
    } camera;

    std::string name;
    std::string path;
    float3 grid_origin  = float3(0.f, 0.f, 0.f);
    float3 grid_spacing = float3(1.f, 1.f, 1.f);
    array_3d_scalar_t data;
    TransferFunction transferFunction;
    float3 aabbPositions[2] = {{-1.0f, -1.0f, -1.0f}, {1.0f, 1.0f, 1.0f}};

    bool openInstanceWindow = false;
    bool addInstanceWindow = false;
    std::vector<Instance> instances;
    int SELECTED_INSTANCE = 0;
    const char* current_instance = NULL;
    int current_instance_index = -1;
    int instanceUniqueName = 0;
    float3 defaultTransform = float3(0.f, 0.f, 0.f);
    int generateInstance = 1;
};

struct LoadInObj {
    std::string name;
    std::string path;

    LoadInObj(std::string inputName, std::string inputPath) : name(inputName), path(inputPath) {}
};

struct LoadInVolume {
    std::string name;
    std::string path;

    LoadInVolume(std::string inputName, std::string inputPath) : name(inputName), path(inputPath) {}
};

struct ConfigureImgui {
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
    struct CurrentLight
    {
        const char* name = NULL;
        const char* type = NULL;
        int index = -1;
    } current_light;
    const std::vector<std::string> ALL_LIGHTS = {
        "Ambient",
        "Directional"
    };
    int ambientLightUniqueName = 0;
    int directionalLightUniqueName = 0;
    std::vector<AmbientLight> LIST_OF_AMBIENT_LIGHTS;
    std::vector<DirectionalLight> LIST_OF_DIRECTIONAL_LIGHTS;
    int SELECTED_LIGHTS = 0;

    // The extents of our bounding box
    std::vector<LoadInVolume*> INITIAL_VOLUME = {
        new LoadInVolume("Mechanic Hand", "/media/storage0/weishen/GPRT-1/samples/new_example_volume/volumes/scene_mechhand.json")
        // ,new LoadInVolume("Heatrelease", "/media/storage0/weishen/GPRT-1/samples/new_example_volume/volumes/generated_heatrelease_1atm_camera_adjusted.json")
    };
    std::vector<Volume> LIST_OF_VOLUMES;
    int SELECTED_VOLUMES = 0;

    const int2 fbSize = {800, 600};
    uint64_t accId = 0;

    bool showImgui = true;
    
    bool showObjControlPanel = false;
    bool addNewObj = false;

    bool showVolumeControlPanel = false;
    bool addNewVolume = false;

    bool showLightControlPanel = false;
    bool addNewLight = false;

    bool updateObjSelection = false;
    bool updateObjTransform = false;
    bool updateObjMaterials = false;  

    bool updateVolumeSelection = false;
    bool updateVolumeTransform = false;
    bool updateLights = false;

    void render();
    void renderObjCP();
    void renderVolumeCP();
    void renderLightCP();

    void initObj();
    // void createDefaultInstance(Obj& obj);
    void objInstances();
    void addObjInstance(Obj& obj);
    void showSelectedObjCountWarning();
    void updateTransform(Obj& obj);
    void updateMaterial(Obj& obj);
    void updateMaterialDetail(Obj& obj);

    void initLight();
    void addLight();
    void updateLight();

    void initVolume();
    // void createDefaultInstance(Volume& volume);
    void volumeInstances();
    void addVolumeInstance(Volume& volume);
    void updateTransform(Volume& volume);

    template <typename T>
    void createDefaultInstance(T& target);

    void inputAndSlider(float3& source, float min_v, float max_v, const char *title, const char *inputLabel, const char *sliderLabel, bool& trigger);
};

#endif
