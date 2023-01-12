// MIT License

// Copyright (c) 2022 Nathan V. Morrical

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// This program sets up a single geometric object, a mesh for a cube, and
// its acceleration structure, then ray traces it.

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

#ifndef INCLUDE_VIEWER
#define INCLUDE_VIEWER
#include "./common/viewer.cpp"
#endif

#define LOG(message)                                             \
    std::cout << GPRT_TERMINAL_BLUE;                             \
    std::cout << "#gprt.sample(main): " << message << std::endl; \
    std::cout << GPRT_TERMINAL_DEFAULT;
#define LOG_OK(message)                                          \
    std::cout << GPRT_TERMINAL_LIGHT_BLUE;                       \
    std::cout << "#gprt.sample(main): " << message << std::endl; \
    std::cout << GPRT_TERMINAL_DEFAULT;

#include "./common/loadModel.hpp"
#include "./common/loadTexture.hpp"

#ifndef   INCLUDE_LIGHTS
#define   INCLUDE_LIGHTS
#include "./common/lights.hpp"
#endif

#ifndef INCLUDE_MATERIAL
#define INCLUDE_MATERIAL
#include "./materials/material.hpp"
#endif
#include <iostream>

extern GPRTProgram new_example_deviceCode;


const std::vector<std::string> MODEL_PATH = {
    "/media/storage0/weishen/GPRT-1/samples/new_example/models/viking_room.obj",
    // "/media/storage0/weishen/GPRT-1/samples/new_example/models/Cube.obj",
    // "/media/storage0/weishen/GPRT-1/samples/new_example/models/Mario.obj",
    // "/media/storage0/weishen/GPRT-1/samples/new_example/models/bunny.obj",
    // "/media/storage0/weishen/GPRT-1/samples/new_example/models/sphere.obj",
    // "/media/storage0/weishen/GPRT-1/samples/new_example/models/sponza.obj",
    // "/media/storage0/weishen/GPRT-1/samples/new_example/models/horse.obj"
};
const std::string TEXTURE_PATH = "/media/storage0/weishen/GPRT-1/samples/new_example/textures/viking_room.png";

// Geometry
std::vector<int3> list_of_indices;
std::vector<float3> list_of_vertices;
std::vector<float3> list_of_colors;
std::vector<float3> list_of_vertex_normals;
std::vector<float4x4> list_of_transform = {
    // translation_matrix(float3(0.0f, 0.0f, 1.1f)),
    translation_matrix(float3(0.0f, 0.0f, 0.0f)),
    translation_matrix(float3(0.344626f,12.9949f,-0.114619f)),
    translation_matrix(float3(2 * sin(2 * M_PI * .33), 2 * cos(2 * M_PI * .33), 1.5f)),
    translation_matrix(float3(2 * sin(2 * M_PI * .66), 2 * cos(2 * M_PI * .66), 1.5f)),
    translation_matrix(float3(2 * sin(2 * M_PI * 1.0), 2 * cos(2 * M_PI * 1.0), 1.5f)),
    translation_matrix(float3(0.0f, 0.0f, -4.0f))
};

// Lights
std::vector<float3> list_of_ambient_lights_intensity;
std::vector<float3> list_of_directional_lights_intensity;
std::vector<float3> list_of_directional_lights_direction;

// Materials
std::vector<int> material_types {
    0, // viking_room
    // 2, // Cube
    // 1, // Mario
    // 0, // bunny
    // 0, // sphere
    // 0, // sponza
    // 0, // horse

    // -1, // viking_room
    // -1, // Cube
    // -1, // Mario
    // -1, // bunny
    // -1, // sphere
    // -1, // sponza
    // -1 // horse
};
std::vector<int> list_of_material_type;
std::vector<Lambertian> list_of_lambertians;
std::vector<Metal> list_of_metals;
std::vector<float3> list_of_metals_albedo;
std::vector<float> list_of_metals_fuzz;
std::vector<Dielectric> list_of_dielectrics;

// initial image resolution
const int2 fbSize = {800, 600};
uint64_t accId = 0;
GLuint fbTexture{0};

float3 lookFrom = {0.f, 10.f, 10.f};
float3 lookAt = {0.f, 0.f, 0.f};
float3 lookUp = {0.f, 1.f, 0.f};
float cosFovy = 20.f;

int main(int ac, char **av)
{
    LOG("gprt example '" << av[0] << "' starting up");

    // create a context on the first device:
    gprtRequestWindow(fbSize.x, fbSize.y, "New Example");
    GPRTContext context = gprtContextCreate(nullptr, 1);
    GPRTModule module = gprtModuleCreate(context, new_example_deviceCode);

    // ##################################################################
    // set up all the *GEOMETRY* graph we want to render
    // ##################################################################

    // -------------------------------------------------------
    // declare geometry type
    // -------------------------------------------------------
    GPRTGeomTypeOf<TrianglesGeomData> trianglesGeomType = gprtGeomTypeCreate<TrianglesGeomData>(
        context, GPRT_TRIANGLES
    );
    gprtGeomTypeSetClosestHitProg(trianglesGeomType, 0, module, "TriangleMesh");

    // ##################################################################
    // set up all the *GEOMS* we want to run that code on
    // ##################################################################

    LOG("building geometries ...");

    // ------------------------------------------------------------------
    // triangle mesh
    // ------------------------------------------------------------------
    int each_material_type = -1;
    for (int each_path = 0; each_path < MODEL_PATH.size(); each_path++) {
        each_material_type = material_types[each_path];
        loadModel(
            MODEL_PATH[each_path],
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
            list_of_transform[each_path]
        );
    }

    GPRTBufferOf<float3> vertexBuffer = gprtDeviceBufferCreate<float3>(context, list_of_vertices.size(), static_cast<const void *>(list_of_vertices.data()));
    GPRTBufferOf<int3> indexBuffer = gprtDeviceBufferCreate<int3>(context, list_of_indices.size(), static_cast<const void *>(list_of_indices.data()));
    GPRTBufferOf<float3> normalBuffer = gprtDeviceBufferCreate<float3>(context, list_of_vertex_normals.size(), static_cast<const void *>(list_of_vertex_normals.data()));
    GPRTBufferOf<float3> colorBuffer = gprtDeviceBufferCreate<float3>(context, list_of_colors.size(), static_cast<const void *>(list_of_colors.data()));
    GPRTBufferOf<int> materialTypeBuffer = gprtDeviceBufferCreate<int>(context, list_of_material_type.size(), static_cast<const void *>(list_of_material_type.data()));
    GPRTBufferOf<Lambertian> lambertianBuffer = gprtDeviceBufferCreate<Lambertian>(context, list_of_lambertians.size(), static_cast<const void *>(list_of_lambertians.data()));
    GPRTBufferOf<float3> metalAlbedoBuffer = gprtDeviceBufferCreate<float3>(context, list_of_metals_albedo.size(), static_cast<const void *>(list_of_metals_albedo.data()));
    GPRTBufferOf<float> metalFuzzBuffer = gprtDeviceBufferCreate<float>(context, list_of_metals_fuzz.size(), static_cast<const void *>(list_of_metals_fuzz.data()));
    GPRTBufferOf<Dielectric> dielectricBuffer = gprtDeviceBufferCreate<Dielectric>(context,list_of_dielectrics.size(), static_cast<const void *>(list_of_dielectrics.data()));
    GPRTBufferOf<float3> accBuffer = gprtDeviceBufferCreate<float3>(context, fbSize.x * fbSize.y);
    GPRTBufferOf<uint32_t> frameBuffer = gprtDeviceBufferCreate<uint32_t>(context, fbSize.x * fbSize.y);

    GPRTGeomOf<TrianglesGeomData> trianglesGeom = gprtGeomCreate<TrianglesGeomData>(context, trianglesGeomType);
    gprtTrianglesSetVertices(trianglesGeom, vertexBuffer,
                             list_of_vertices.size());
    gprtTrianglesSetIndices(trianglesGeom, indexBuffer,
                            list_of_indices.size());

    TrianglesGeomData *geomData = gprtGeomGetPointer(trianglesGeom);
    geomData->vertex = gprtBufferGetHandle(vertexBuffer);
    geomData->normal = gprtBufferGetHandle(normalBuffer);
    geomData->index = gprtBufferGetHandle(indexBuffer);
    geomData->color = gprtBufferGetHandle(colorBuffer);
    geomData->material_type = gprtBufferGetHandle(materialTypeBuffer);
    geomData->metal_albedo = gprtBufferGetHandle(metalAlbedoBuffer);
    geomData->metal_fuzz = gprtBufferGetHandle(metalFuzzBuffer);
    geomData->lambertian = gprtBufferGetHandle(lambertianBuffer);
    geomData->dielectric = gprtBufferGetHandle(dielectricBuffer);

    // ------------------------------------------------------------------
    // the group/accel for that mesh
    // ------------------------------------------------------------------
    GPRTAccel trianglesAccel = gprtTrianglesAccelCreate(context, 1, &trianglesGeom);
    gprtAccelBuild(context, trianglesAccel);

    GPRTAccel world = gprtInstanceAccelCreate(context, 1, &trianglesAccel);
    gprtAccelBuild(context, world);

    // ##################################################################
    // set miss and raygen program required for SBT
    // ##################################################################

    // -------------------------------------------------------
    // set up miss prog
    // -------------------------------------------------------

    // ----------- create object  ----------------------------
    GPRTMissOf<MissProgData> miss = gprtMissCreate<MissProgData>(context, module, "miss");

    // ----------- set variables  ----------------------------
    MissProgData *missData = gprtMissGetPointer(miss);
    missData->color0 = float3(1.0f, 1.0f, 1.0f);
    missData->color1 = float3(0.0f, 0.0f, 0.0f);

    // -------------------------------------------------------
    // set up ray gen program
    // -------------------------------------------------------

    // ----------- create object  ----------------------------
    GPRTRayGenOf<RayGenData> rayGen = gprtRayGenCreate<RayGenData>(context, module, "simpleRayGen");

    // ----------- create camera  ----------------------------
    Viewer viewer;
    viewer.camera.setOrientation(lookFrom,
                                 lookAt,
                                 lookUp,
                                 cosFovy);

    // ----------- create lights  ----------------------------
    loadLights(
        list_of_ambient_lights_intensity,
        list_of_directional_lights_intensity,
        list_of_directional_lights_direction
    );

    GPRTBufferOf<float3> ambientLightIntensityBuffer = gprtDeviceBufferCreate<float3>(
        context, list_of_ambient_lights_intensity.size(), static_cast<const void *>(list_of_ambient_lights_intensity.data())
    );
    GPRTBufferOf<float3> directionalLightIntensityBuffer = gprtDeviceBufferCreate<float3>(
        context, list_of_directional_lights_intensity.size(), static_cast<const void *>(list_of_directional_lights_intensity.data())
    );
    GPRTBufferOf<float3> directionalLightDirBuffer = gprtDeviceBufferCreate<float3>(
        context, list_of_directional_lights_direction.size(), static_cast<const void *>(list_of_directional_lights_direction.data())
    );

    // ----------- set variables  ----------------------------
    RayGenData *rayGenData = gprtRayGenGetPointer(rayGen);
    rayGenData->frameBuffer = gprtBufferGetHandle(frameBuffer);
    rayGenData->world = gprtAccelGetHandle(world);
    rayGenData->accBuffer = gprtBufferGetHandle(accBuffer);
    rayGenData->accId = (uint64_t)accId;
    rayGenData->ambient_lights_intensity = gprtBufferGetHandle(ambientLightIntensityBuffer);
    rayGenData->ambient_light_size = (uint64_t)list_of_ambient_lights_intensity.size();
    rayGenData->directional_lights_intensity = gprtBufferGetHandle(directionalLightIntensityBuffer);
    rayGenData->directional_lights_dir = gprtBufferGetHandle(directionalLightDirBuffer);
    rayGenData->directional_light_size = (uint64_t)list_of_directional_lights_intensity.size();

    // ##################################################################
    // build *SBT* required to trace the groups
    // ##################################################################
    gprtBuildPipeline(context);
    gprtBuildShaderBindingTable(context, GPRT_SBT_ALL);

    // ##################################################################
    // now that everything is ready: launch it ....
    // ##################################################################

    LOG("launching ...");
    GLFWwindow *window = glfwCreateWindow(fbSize.x, fbSize.y, "New Example", NULL, NULL);
    glfwMakeContextCurrent(window);
    viewer.addCallBack(window);

    bool firstFrame = true;
    do {
        // If we click the mouse, we should rotate the camera
        if (viewer.camera.lastModified || firstFrame)
        {
            viewer.camera.lastModified = false;
            accId = 0;
            firstFrame = false;
#ifndef M_PI
#define M_PI 3.1415926f
#endif

            const float3 lookFrom = viewer.camera.getFrom();
            const float3 lookAt = viewer.camera.getAt();
            const float3 lookUp = viewer.camera.getUp();
            const float cosFovy = viewer.camera.getCosFovy();
            const float vfov = toDegrees(acosf(cosFovy));
            viewer.camera.toString();
            // ........... compute variable values  ..................
            const float3 vup = lookUp;
            const float aspect = fbSize.x / float(fbSize.y);
            const float theta = vfov * ((float)M_PI) / 180.0f;
            const float half_height = tanf(theta / 2.0f);
            const float half_width = aspect * half_height;
            const float focusDist = 10.f;
            const float3 origin = lookFrom;
            const float3 w = normalize(lookFrom - lookAt);
            const float3 u = normalize(cross(vup, w));
            const float3 v = cross(w, u);
            const float3 lower_left_corner = origin - half_width * focusDist * u - half_height * focusDist * v - focusDist * w;
            const float3 horizontal = 2.0f * half_width * focusDist * u;
            const float3 vertical = 2.0f * half_height * focusDist * v;

            // ----------- set variables  ----------------------------
            RayGenData *raygenData = gprtRayGenGetPointer(rayGen);
            raygenData->camera.pos = origin;
            raygenData->camera.dir_00 = lower_left_corner;
            raygenData->camera.dir_du = horizontal;
            raygenData->camera.dir_dv = vertical;
        }

        rayGenData->accId = (uint64_t)accId;
        accId++;
        // Now, trace rays
        gprtBuildShaderBindingTable(context, GPRT_SBT_RAYGEN);
        gprtRayGenLaunch2D(context, rayGen, fbSize.x, fbSize.y);

        // If a window exists, presents the framebuffer here to that window
        gprtBufferPresent(context, frameBuffer);

        // Render results to screen
        void *pixels = gprtBufferGetPointer(frameBuffer);
        if (fbTexture == 0)
            glGenTextures(1, &fbTexture);

        glBindTexture(GL_TEXTURE_2D, fbTexture);
        GLenum texFormat = GL_RGBA;
        GLenum texelType = GL_UNSIGNED_BYTE;
        glTexImage2D(GL_TEXTURE_2D, 0, texFormat, fbSize.x, fbSize.y, 0, GL_RGBA,
                     texelType, pixels);

        glDisable(GL_LIGHTING);
        glColor3f(1, 1, 1);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, fbTexture);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        glDisable(GL_DEPTH_TEST);

        glViewport(0, 0, fbSize.x, fbSize.y);

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(0.f, (float)fbSize.x, (float)fbSize.y, 0.f, -1.f, 1.f);

        glBegin(GL_QUADS);
        {
            glTexCoord2f(0.f, 0.f);
            glVertex3f(0.f, 0.f, 0.f);

            glTexCoord2f(0.f, 1.f);
            glVertex3f(0.f, (float)fbSize.y, 0.f);

            glTexCoord2f(1.f, 1.f);
            glVertex3f((float)fbSize.x, (float)fbSize.y, 0.f);

            glTexCoord2f(1.f, 0.f);
            glVertex3f((float)fbSize.x, 0.f, 0.f);
        }
        glEnd();

        glfwSwapBuffers(window);
        glfwPollEvents();

    } while (!glfwWindowShouldClose(window));//!gprtWindowShouldClose(context));

    // ##################################################################
    // and finally, clean up
    // ##################################################################

    LOG("cleaning up ...");
    glfwDestroyWindow(window);
    glfwTerminate();

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

    LOG_OK("seems all went OK; app is done, this should be the last output ...");
}
