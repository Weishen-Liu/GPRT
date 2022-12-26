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

#ifndef INCLUDE_ARCBALL
#define INCLUDE_ARCBALL
#include "./arcball.hpp"
#endif

#ifndef INCLUDE_CAMERA
#define INCLUDE_CAMERA
#include "./camera.hpp"
#endif

#define Arcball true

#include "./math/Quaternion.h"
#include "./math/LinearSpace.h"

#include <iostream>

struct Viewer
{

    ArcBall arcball;
    Camera camera;
    GLFWwindow *handle{nullptr};
    int2 windowSize{800, 600};

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

    int2 getMousePos() const
    {
        double x, y;
        std::cout << "Collect Cursor Position" << std::endl;
        glfwGetCursorPos(handle, &x, &y);
        std::cout << "Cursor Position: " << (int)x << " " << (int)y << std::endl;
        return int2((int)x, (int)y);
    }

    static int arg_min(float3 &v)
    {
        std::vector<float> input = {};
        input.push_back(v.x);
        input.push_back(v.y);
        input.push_back(v.z);

        int biggestDim = 0;
        for (int i = 1; i < 3; i++)
        {
            if (input[i] < input[biggestDim])
            {
                biggestDim = i;
            }
        }
        return biggestDim;
    }

    /*! return quaternion for given rotation matrix */
    static inline owl::common::Quaternion3f rotation(const owl::common::linear3f &a)
    {
        float tr = a.vx.x + a.vy.y + a.vz.z + 1;
        float3 diag = float3(a.vx.x, a.vy.y, a.vz.z);
        if (tr > 1)
        {
            float s = sqrtf(tr) * 2;
            return owl::common::Quaternion3f(.25f * s,
                              (a.vz.y - a.vy.z) / s,
                              (a.vx.z - a.vz.x) / s,
                              (a.vy.x - a.vx.y) / s);
        }
        else if (arg_min(diag) == 0)
        {
            float s = sqrtf(1.f + diag.x - diag.y - diag.z) * 2.f;
            return owl::common::Quaternion3f((a.vz.y - a.vy.z) / s,
                              .25f * s,
                              (a.vx.y - a.vy.x) / s,
                              (a.vx.z - a.vz.x) / s);
        }
        else if (arg_min(diag) == 1)
        {
            float s = sqrtf(1.f + diag.y - diag.x - diag.z) * 2.f;
            return owl::common::Quaternion3f((a.vx.z - a.vz.x) / s,
                              (a.vx.y - a.vy.x) / s,
                              .25f * s,
                              (a.vy.z - a.vz.y) / s);
        }
        else
        {
            float s = sqrtf(1.f + diag.z - diag.x - diag.y) * 2.f;
            return owl::common::Quaternion3f((a.vy.x - a.vx.y) / s,
                              (a.vx.z - a.vz.x) / s,
                              (a.vy.z - a.vz.y) / s,
                              .25f * s);
        }
    }

    void mouseButtonLeft(const int2 &where, bool pressed)
    {
        std::cout << "Left Click" << std::endl;
        if (Arcball && pressed)
        {
            if (abs(arcball.rotation) < 1e-8f)
            {
                Camera &fc = this->camera;
                arcball.rotation = rotation(fc.frame);
            }
            arcball.down_pos = arcballProject(where);
            arcball.down_rotation = arcball.rotation;
        }
        lastMousePosition = where;
    }

    void mouseButtonRight(const int2 &where, bool pressed)
    {
        std::cout << "Right Click" << std::endl;
        lastMousePosition = where;
    }

    void mouseButton(int button, int action, int mods)
    {
        std::cout << "Button Click" << std::endl;
        pressed = (action == GLFW_PRESS);
        lastMousePos = getMousePos();
        switch (button)
        {
        case GLFW_MOUSE_BUTTON_LEFT:
            leftButton.isPressed = pressed;
            mouseButtonLeft(lastMousePos, pressed);
            break;
        case GLFW_MOUSE_BUTTON_RIGHT:
            rightButton.isPressed = pressed;
            mouseButtonRight(lastMousePos, pressed);
            break;
        }
    }

    /*! callback for pressing _or_ releasing a mouse button*/
    static void glfwindow_mouseButton_cb(GLFWwindow *window,
                                         int button,
                                         int action,
                                         int mods)
    {
        Viewer *gw = static_cast<Viewer *>(glfwGetWindowUserPointer(window));
        assert(gw);
        gw->mouseButton(button, action, mods);
    }

    /*! re-computes the 'camera' from the 'cameracontrol', and notify
      app that the camera got changed */
    void updateCamera()
    {
        // camera.digestInto(simpleCamera);
        // if (isActive)
        camera.lastModified = true;
    }

    /*! return matrix for rotation around arbitrary axis */
    static inline owl::common::linear3f ling_rotate(const float3 &_u, const float &r)
    {
        float3 u = normalize(_u);
        float s = sin(r);
        float c = cos(r);
        return owl::common::linear3f(
            u.x * u.x + (1 - u.x * u.x) * c, u.x * u.y * (1 - c) - u.z * s, u.x * u.z * (1 - c) + u.y * s,
            u.x * u.y * (1 - c) + u.z * s, u.y * u.y + (1 - u.y * u.y) * c, u.y * u.z * (1 - c) - u.x * s,
            u.x * u.z * (1 - c) - u.y * s, u.y * u.z * (1 - c) + u.x * s, u.z * u.z + (1 - u.z * u.z) * c
        );
    }

    void rotate(const float deg_u, const float deg_v)
    {
        float rad_u = -(float)M_PI / 180.f * deg_u;
        float rad_v = -(float)M_PI / 180.f * deg_v;

        Camera &fc = this->camera;
        const float3 poi = fc.getPOI();
        fc.frame = owl::common::linear3f::rotate(fc.frame.vy, rad_u) * owl::common::linear3f::rotate(fc.frame.vx, rad_v) * fc.frame;

        if (fc.forceUp)
        {
            fc.forceUpFrame();
        }

        fc.position = poi + fc.poiDistance * fc.frame.vz;

        updateCamera();
    }

    float3 arcballProject(const int2 &where)
    {
        int width = windowSize.x;
        int height = windowSize.y;

        float3 v(0.f, 0.f, 0.f);

        v.x = (where.x - .5f * width) / (.5f * width);
        v.y = -(where.y - .5f * height) / (.5f * height);

        float d = v.x * v.x + v.y * v.y;

        if (d > 1.f)
        {
            float length = sqrtf(d);

            v.x /= length;
            v.y /= length;
        }
        else
            v.z = sqrtf(1.f - d);

        return v;
    }

    void mouseDragLeft(const int2 &where, const int2 &delta)
    {
        if (Arcball)
        {
            float3 curr_pos = arcballProject(where + delta);
            float3 from = normalize(arcball.down_pos);
            float3 to = normalize(curr_pos);
            arcball.rotation = owl::common::Quaternion3f(dot(from, to), cross(from, to)) * arcball.down_rotation;

            owl::common::LinearSpace3f rotation_matrix(conj(arcball.rotation));

            Camera &fc = this->camera;

            float3  eye(0.f, 0.f, length(fc.getPOI() - fc.position));
            eye = rotation_matrix * eye;
            eye += fc.getPOI();

            float3 up = rotation_matrix.vy;

            fc.setOrientation(eye, fc.getPOI(), up, fc.fovyInDegrees, fc.focalDistance);

            if (fc.forceUp)
            {
                fc.forceUpFrame();
            }

            updateCamera();
        }
        else
        {
            const float2 fraction = float2(delta) / float2(windowSize);
            rotate(fraction.x * degrees_per_drag_fraction,
                   fraction.y * degrees_per_drag_fraction);
        }
    }

    void move(const float step)
    {
        Camera &fc = this->camera;

        const float3 poi = fc.getPOI();
        fc.poiDistance   = std::min(maxDistance, std::max(minDistance,fc.poiDistance-step*fc.motionSpeed));
        fc.position = poi + fc.poiDistance * fc.frame.vz;

        updateCamera();
    }

    /*! mouse got dragged with left button pressedn, by 'delta'
      pixels, at last position where */
    void mouseDragRight(const int2 &where, const int2 &delta)
    {
        const float2 fraction = float2(delta) / float2(windowSize);
        move(fraction.y * pixels_per_move);
    }

    /*! this gets called when the window determines that the mouse got
      _moved_ to the given position */
    void mouseMotion(const int2 &newMousePosition)
    {
        // std::cout<< " Current Cursor X Position: "<< newMousePosition.x << std::endl;
        // std::cout<< " Current Cursor Y Position: "<< newMousePosition.y << std::endl;
        // std::cout<< " Previous Cursor X Position: "<< lastMousePosition.x << std::endl;
        // std::cout<< " Previous Cursor Y Position: "<< lastMousePosition.y << std::endl;
        if (lastMousePosition != int2(-1))
        {
            if (leftButton.isPressed)
                mouseDragLeft(newMousePosition, newMousePosition - lastMousePosition);
            if (rightButton.isPressed)
                mouseDragRight(newMousePosition, newMousePosition - lastMousePosition);
        }
        lastMousePosition = newMousePosition;
    }

    /*! callback for _moving_ the mouse to a new position */
    static void glfwindow_mouseMotion_cb(GLFWwindow *window, double x, double y)
    {
        Viewer *gw = static_cast<Viewer *>(glfwGetWindowUserPointer(window));
        assert(gw);
        gw->mouseMotion(int2((int)x, (int)y));
    }

    void addCallBack(GLFWwindow *window)
    {
        handle = window;
        glfwSetWindowUserPointer(handle, this);
        glfwSetMouseButtonCallback(handle, glfwindow_mouseButton_cb);
        glfwSetCursorPosCallback(handle, glfwindow_mouseMotion_cb);
    }
};
