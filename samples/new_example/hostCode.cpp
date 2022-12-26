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

#ifndef INCLUDE_MATERIAL
#define INCLUDE_MATERIAL
#include "./materials/material.hpp"
#endif
#include <iostream>

extern GPRTProgram new_example_deviceCode;


const std::vector<std::string> MODEL_PATH = {
    // "/media/storage0/weishen/GPRT-1/samples/new_example/models/viking_room.obj",
    "/media/storage0/weishen/GPRT-1/samples/new_example/models/Cube.obj",
    // "/media/storage0/weishen/GPRT-1/samples/new_example/models/Mario.obj",
    // "/media/storage0/weishen/GPRT-1/samples/new_example/models/bunny.obj",
    // "/media/storage0/weishen/GPRT-1/samples/new_example/models/sphere.obj",
    // "/media/storage0/weishen/GPRT-1/samples/new_example/models/sponza.obj",
    "/media/storage0/weishen/GPRT-1/samples/new_example/models/horse.obj"
};
const std::string TEXTURE_PATH = "/media/storage0/weishen/GPRT-1/samples/new_example/textures/viking_room.png";

const int NUM_VERTICES = 8;
float3 vertices[NUM_VERTICES] =
    {
        {-1.f, -1.f, -1.f},
        {+1.f, -1.f, -1.f},
        {-1.f, +1.f, -1.f},
        {+1.f, +1.f, -1.f},
        {-1.f, -1.f, +1.f},
        {+1.f, -1.f, +1.f},
        {-1.f, +1.f, +1.f},
        {+1.f, +1.f, +1.f}};

const int NUM_INDICES = 12;
int3 indices[NUM_INDICES] =
    {
        {0, 1, 3}, {2, 3, 0}, {5, 7, 6}, {5, 6, 4}, {0, 4, 5}, {0, 5, 1}, {2, 3, 7}, {2, 7, 6}, {1, 5, 7}, {1, 7, 3}, {4, 0, 2}, {4, 2, 6}};

std::vector<float3> list_of_vertices;
std::vector<int3> list_of_indices;
std::vector<Lambertian> list_of_lambertians;
std::vector<Metal> list_of_metals;
std::vector<float3> list_of_colors;
std::vector<float3> list_of_vertex_normals;
std::vector<float4x4> list_of_transform = {
    translation_matrix(float3(2 * sin(2 * M_PI * .33), 2 * cos(2 * M_PI * .33), 1.5f)),
    translation_matrix(float3(2 * sin(2 * M_PI * .66), 2 * cos(2 * M_PI * .66), 1.5f)),
    translation_matrix(float3(2 * sin(2 * M_PI * 1.0), 2 * cos(2 * M_PI * 1.0), 1.5f)),
    translation_matrix(float3(0.0f, 0.0f, -4.0f))
};

float transform[3][4] =
    {
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f};

// initial image resolution
const int2 fbSize = {800, 600};
uint64_t accId = 0;
GLuint fbTexture{0};

float3 lookFrom = {13.f, 2.f, 3.f};
float3 lookAt = {0.f, 0.f, 0.f};
float3 lookUp = {0.f, 1.f, 0.f};
float cosFovy = 20.f;

int main(int ac, char **av)
{
    LOG("gprt example '" << av[0] << "' starting up");

    // create a context on the first device:
    GPRTContext context = gprtContextCreate(nullptr, 1);
    GPRTModule module = gprtModuleCreate(context, new_example_deviceCode);

    // ##################################################################
    // set up all the *GEOMETRY* graph we want to render
    // ##################################################################

    // -------------------------------------------------------
    // declare geometry type
    // -------------------------------------------------------
    GPRTVarDecl trianglesGeomVars[] = {
        {"index", GPRT_BUFFER, GPRT_OFFSETOF(TrianglesGeomData, index)},
        {"vertex", GPRT_BUFFER, GPRT_OFFSETOF(TrianglesGeomData, vertex)},
        // { "color",  GPRT_FLOAT3, GPRT_OFFSETOF(TrianglesGeomData,color)},
        {"color", GPRT_BUFFER, GPRT_OFFSETOF(TrianglesGeomData, color)},
        {"normal", GPRT_BUFFER, GPRT_OFFSETOF(TrianglesGeomData, normal)},
        {"metal", GPRT_BUFFER, GPRT_OFFSETOF(TrianglesGeomData, metal)},
        {"lambertian", GPRT_BUFFER, GPRT_OFFSETOF(TrianglesGeomData, lambertian)},
        {/* sentinel to mark end of list */}};
    GPRTGeomType trianglesGeomType = gprtGeomTypeCreate(context,
                                                        GPRT_TRIANGLES,
                                                        sizeof(TrianglesGeomData),
                                                        trianglesGeomVars, -1);
    gprtGeomTypeSetClosestHitProg(trianglesGeomType, 0,
                                  module, "TriangleMesh");

    // ##################################################################
    // set up all the *GEOMS* we want to run that code on
    // ##################################################################

    LOG("building geometries ...");

    // ------------------------------------------------------------------
    // triangle mesh
    // ------------------------------------------------------------------

    for (int each_path = 0; each_path < MODEL_PATH.size(); each_path++) {
        loadModel(
            MODEL_PATH[each_path],
            list_of_vertices,
            list_of_indices,
            list_of_colors,
            list_of_vertex_normals,
            list_of_lambertians,
            list_of_metals,
            list_of_transform[each_path]
        );
    }
    GPRTBuffer vertexBuffer = gprtHostBufferCreate(context, GPRT_FLOAT3, list_of_vertices.size(), static_cast<const void *>(list_of_vertices.data()));
    GPRTBuffer normalBuffer = gprtDeviceBufferCreate(context, GPRT_FLOAT3, list_of_vertex_normals.size(), static_cast<const void *>(list_of_vertex_normals.data()));
    GPRTBuffer indexBuffer = gprtDeviceBufferCreate(context, GPRT_INT3, list_of_indices.size(), static_cast<const void *>(list_of_indices.data()));
    GPRTBuffer colorBuffer = gprtDeviceBufferCreate(context, GPRT_FLOAT3, list_of_colors.size(), static_cast<const void *>(list_of_colors.data()));
    GPRTBuffer lambertianBuffer = gprtDeviceBufferCreate(context, GPRT_USER_TYPE(list_of_lambertians[0]), list_of_lambertians.size(), static_cast<const void *>(list_of_lambertians.data()));
    GPRTBuffer metalBuffer = gprtDeviceBufferCreate(context, GPRT_USER_TYPE(list_of_metals[0]), list_of_metals.size(), static_cast<const void *>(list_of_metals.data()));

    GPRTBuffer frameBuffer = gprtHostBufferCreate(context, GPRT_INT, fbSize.x * fbSize.y);

    GPRTBuffer accBuffer = gprtDeviceBufferCreate(context, GPRT_FLOAT3, fbSize.x * fbSize.y, nullptr);

    GPRTGeom trianglesGeom = gprtGeomCreate(context, trianglesGeomType);

    gprtTrianglesSetVertices(trianglesGeom, vertexBuffer,
                             list_of_vertices.size(), sizeof(float3), 0);
    gprtTrianglesSetVertexNormal(trianglesGeom, normalBuffer,
                                 list_of_vertex_normals.size(), sizeof(float3), 0);
    gprtTrianglesSetIndices(trianglesGeom, indexBuffer,
                            list_of_indices.size(), sizeof(int3), 0);
    gprtTrianglesSetVertexColor(trianglesGeom, colorBuffer,
                                list_of_colors.size(), sizeof(float3), 0);
    gprtTrianglesSetLambertian(trianglesGeom, lambertianBuffer,
                               list_of_lambertians.size(), sizeof(Lambertian), 0);
    gprtTrianglesSetMetal(trianglesGeom, metalBuffer,
                          list_of_metals.size(), sizeof(Metal), 0);

    gprtGeomSetBuffer(trianglesGeom, "vertex", vertexBuffer);
    gprtGeomSetBuffer(trianglesGeom, "normal", normalBuffer);
    gprtGeomSetBuffer(trianglesGeom, "index", indexBuffer);
    // gprtGeomSet3f(trianglesGeom,"color",0,1,0);
    gprtGeomSetBuffer(trianglesGeom, "color", colorBuffer);
    gprtGeomSetBuffer(trianglesGeom, "metal", metalBuffer);
    gprtGeomSetBuffer(trianglesGeom, "lambertian", lambertianBuffer);

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
    GPRTVarDecl missProgVars[] = {
        {"color0", GPRT_FLOAT3, GPRT_OFFSETOF(MissProgData, color0)},
        {"color1", GPRT_FLOAT3, GPRT_OFFSETOF(MissProgData, color1)},
        {/* sentinel to mark end of list */}};
    // ----------- create object  ----------------------------
    GPRTMiss miss = gprtMissCreate(context, module, "miss", sizeof(MissProgData),
                                   missProgVars, -1);

    // ----------- set variables  ----------------------------
    gprtMissSet3f(miss, "color0", 1.f, 1.f, 1.f);
    // gprtMissSet3f(miss,"color0",0.1f,0.1f,0.1f);
    gprtMissSet3f(miss, "color1", .0f, .0f, .0f);

    // -------------------------------------------------------
    // set up ray gen program
    // -------------------------------------------------------
    GPRTVarDecl rayGenVars[] = {
        {"fbSize", GPRT_INT2, GPRT_OFFSETOF(RayGenData, fbSize)},
        {"fbPtr", GPRT_BUFFER, GPRT_OFFSETOF(RayGenData, fbPtr)},
        {"accBuffer", GPRT_BUFFER, GPRT_OFFSETOF(RayGenData, accBuffer)},
        {"accId", GPRT_INT, GPRT_OFFSETOF(RayGenData, accId)},
        {"world", GPRT_ACCEL, GPRT_OFFSETOF(RayGenData, world)},
        {"camera.pos", GPRT_FLOAT3, GPRT_OFFSETOF(RayGenData, camera.pos)},
        {"camera.dir_00", GPRT_FLOAT3, GPRT_OFFSETOF(RayGenData, camera.dir_00)},
        {"camera.dir_du", GPRT_FLOAT3, GPRT_OFFSETOF(RayGenData, camera.dir_du)},
        {"camera.dir_dv", GPRT_FLOAT3, GPRT_OFFSETOF(RayGenData, camera.dir_dv)},
        {/* sentinel to mark end of list */}};

    // ----------- create camera  ----------------------------
    Viewer viewer;
    viewer.camera.setOrientation(lookFrom,
                                 lookAt,
                                 lookUp,
                                 cosFovy);

    // ----------- create object  ----------------------------
    GPRTRayGen rayGen = gprtRayGenCreate(context, module, "simpleRayGen",
                                         sizeof(RayGenData),
                                         rayGenVars, -1);

    // ----------- set variables  ----------------------------
    gprtRayGenSetBuffer(rayGen, "accBuffer", accBuffer);
    gprtRayGenSet1i(rayGen, "accId", (uint64_t)accId);
    gprtRayGenSetBuffer(rayGen, "fbPtr", frameBuffer);
    gprtRayGenSet2iv(rayGen, "fbSize", (int32_t *)&fbSize);
    gprtRayGenSetAccel(rayGen, "world", world);

    // ##################################################################
    // build *SBT* required to trace the groups
    // ##################################################################
    gprtBuildPipeline(context);
    gprtBuildShaderBindingTable(context);

    // ##################################################################
    // create a window we can use to display and interact with the image
    // ##################################################################
    if (!glfwInit())
        // Initialization failed
        throw std::runtime_error("Can't initialize GLFW");

    auto error_callback = [](int error, const char *description)
    {
        fprintf(stderr, "Error: %s\n", description);
    };
    glfwSetErrorCallback(error_callback);

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
    GLFWwindow *window = glfwCreateWindow(fbSize.x, fbSize.y,
                                          "New Example", NULL, NULL);
    if (!window)
        throw std::runtime_error("Window or OpenGL context creation failed");
    glfwMakeContextCurrent(window);

    // ##################################################################
    // now that everything is ready: launch it ....
    // ##################################################################

    LOG("launching ...");
    viewer.addCallBack(window);

    bool firstFrame = true;
    double xpos = 0.f, ypos = 0.f;
    double lastxpos, lastypos;
    while (!glfwWindowShouldClose(window))
    {
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
            gprtRayGenSet3fv(rayGen, "camera.pos", (float *)&origin);
            gprtRayGenSet3fv(rayGen, "camera.dir_00", (float *)&lower_left_corner);
            gprtRayGenSet3fv(rayGen, "camera.dir_du", (float *)&horizontal);
            gprtRayGenSet3fv(rayGen, "camera.dir_dv", (float *)&vertical);
        }

        gprtRayGenSet1i(rayGen, "accId", (uint64_t)accId);
        accId++;
        // Now, trace rays
        gprtBuildShaderBindingTable(context);
        gprtRayGenLaunch2D(context, rayGen, fbSize.x, fbSize.y);

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
    }

    // ##################################################################
    // and finally, clean up
    // ##################################################################

    LOG("cleaning up ...");

    glfwDestroyWindow(window);
    glfwTerminate();

    gprtBufferDestroy(vertexBuffer);
    gprtBufferDestroy(normalBuffer);
    gprtBufferDestroy(indexBuffer);
    gprtBufferDestroy(metalBuffer);
    gprtBufferDestroy(lambertianBuffer);
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
