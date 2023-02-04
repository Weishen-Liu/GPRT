#pragma once
#ifndef INCLUDE_NEWEXAMPLE_COMMON_VIEWER
#define INCLUDE_NEWEXAMPLE_COMMON_VIEWER

#include <gprt.h>

#include <GLFW/glfw3.h>

#include "arcball.hpp"
#include "camera.hpp"
#include "processVulkanResources.hpp"

#include "./math/Quaternion.h"
#include "./math/LinearSpace.h"

struct Viewer
{
    ArcBall arcball;
    // Camera
    float3 lookFrom = {0.f, 10.f, 10.f};
    float3 lookAt = {0.f, 0.f, 0.f};
    float3 lookUp = {0.f, 1.f, 0.f};
    float cosFovy = 20.f;
    Camera camera;

    GLFWwindow *handle{nullptr};
    VulkanResources vulkanResources;
    GPRTProgram deviceCode;

    struct ButtonState
    {
        bool isPressed{false};
    };

    ButtonState leftButton;
    ButtonState rightButton;
    int2 lastMousePosition{-1, -1};
    int2 lastMousePos = {-1, -1};

    GLuint fbTexture{0};

    const float degrees_per_drag_fraction = 150;
    const float pixels_per_move = 10.f;
    bool pressed = false;
    float maxDistance = std::numeric_limits<float>::infinity();
    float minDistance = 1e-3f;

    bool firstFrame = true;

    void init(GPRTProgram new_example_deviceCode);
    void initWindow();
    void destory();
    void destoryWindow();
    void addWindowCallBack();
    void run();

    int arg_min(float3 &v);

    /*! return quaternion for given rotation matrix */
    inline owl::common::Quaternion3f rotation(const owl::common::linear3f &a);

    /*! return matrix for rotation around arbitrary axis */
    inline owl::common::linear3f ling_rotate(const float3 &_u, const float &r);

    /*! re-computes the 'camera' from the 'cameracontrol', and notify
      app that the camera got changed */
    void recalculateCamera();
    void rotate(const float deg_u, const float deg_v);
    float3 arcballProject(const int2 &where);
    void move(const float step);
    void key(int key, const int2 &where);

    void mouseButtonLeft(const int2 &where, bool pressed);
    void mouseButtonRight(const int2 &where, bool pressed);
    void mouseButton(GLFWwindow *window, int button, int action, int mods);
    void mouseDragLeft(const int2 &where, const int2 &delta);

    /*! mouse got dragged with left button pressedn, by 'delta'
      pixels, at last position where */
    void mouseDragRight(const int2 &where, const int2 &delta);

    /*! this gets called when the window determines that the mouse got
      _moved_ to the given position */
    void mouseMotion(const int2 &newMousePosition);
};

#endif
