#ifndef INCLUDE_GPRT
#define INCLUDE_GPRT
// public GPRT API
#include <gprt.h>
#endif

#ifndef INCLUDE_GLFW
#define INCLUDE_GLFW
// library for windowing
#include <GLFW/glfw3.h>
#endif

#ifndef INCLUDE_IMGUI
#define INCLUDE_IMGUI
#include <imgui/imgui.h>
#include <imgui/imgui_impl_glfw.h>
#include <imgui/imgui_impl_opengl2.h>
#endif

#ifndef INCLUDE_PROCESSVULKANRESOURCES
#define INCLUDE_PROCESSVULKANRESOURCES
#include "./processVulkanResources.hpp"
#endif

#ifndef INCLUDE_ARCBALL
#define INCLUDE_ARCBALL
#include "./arcball.hpp"
#endif

#ifndef INCLUDE_CAMERA
#define INCLUDE_CAMERA
#include "./camera.hpp"
#endif

#ifndef INCLUDE_CONFIGUREIMGUI
#define INCLUDE_CONFIGUREIMGUI
#include "./configureImgui.cpp"
#endif

#include "./math/Quaternion.h"
#include "./math/LinearSpace.h"

#define Arcball true

#include <iostream>

struct Viewer
{

    ArcBall arcball;
    Camera camera;
    GLFWwindow *handle{nullptr};
    VulkanResources vulkanResources;
    ConfigureImgui configureImgui;
    GPRTProgram deviceCode;

    struct ButtonState
    {
        bool isPressed{false};
    };

    ButtonState leftButton;
    ButtonState rightButton;
    int2 lastMousePosition{-1, -1};
    int2 lastMousePos = {-1, -1};

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
