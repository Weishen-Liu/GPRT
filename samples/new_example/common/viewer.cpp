#ifndef   INCLUDE_GPRT
#define   INCLUDE_GPRT
// public GPRT API
#include <gprt.h>
#endif

#ifndef   INCLUDE_GLFW
#define   INCLUDE_GLFW
// library for windowing
#include <GLFW/glfw3.h>
#endif

#ifndef   INCLUDE_ARCBALL
#define   INCLUDE_ARCBALL
#include "./arcball.cpp"
#endif

#include <iostream>

struct Viewer {

    struct Camera
    {
        float3      from;    // camera position - *from* where we are looking
        float3      at;      // which point we are looking *at*
        float3      up;      // up direction of the camera
        float       cosFovy = 0.66f;

        void setupCamera (float3 from, float3 at, float3 up)
        {
            this->from = from;
            this->at = at;
            this->up = up;
        }
    };

    ArcBall arcball;

    struct ButtonState
    {
        bool  isPressed        { false };
        int2 posFirstPressed  { -1 };
        int2 posLastSeen      { -1 };
        bool  shiftWhenPressed { false };
    };

    ButtonState leftButton;
    ButtonState rightButton;
    int2        lastMousePosition { -1,-1 };
    int2        lastMousePos = { -1,-1 };
    Camera      camera;
    GLFWwindow *handle { nullptr };

    int2 getMousePos() const
    {
        double x,y;
        std::cout<< "Collect Cursor Position" <<std::endl;
        glfwGetCursorPos(handle,&x,&y);
        std::cout<<"Cursor Position: " << (int)x << " " << (int)y <<std::endl;
        return int2((int)x, (int)y);
    }

    void mouseButtonLeft (const int2 &where, bool pressed)
    {
        std::cout<<"Left Click"<<std::endl;
        this->lastMousePosition = where;
    }

    void mouseButtonRight (const int2 &where, bool pressed)
    {
        std::cout<<"Right Click"<<std::endl;
        this->lastMousePosition = where;
    }

    void mouseButton (int button, int action, int mods)
    {
        std::cout<<"Button Click"<<std::endl;
        const bool pressed = (action == GLFW_PRESS);
        lastMousePos = getMousePos();
        switch(button) {
        case GLFW_MOUSE_BUTTON_LEFT:
        std::cout<<"Left Button Click"<<std::endl;
        this->leftButton.isPressed        = pressed;
        this->leftButton.shiftWhenPressed = (mods & GLFW_MOD_SHIFT);
        mouseButtonLeft(lastMousePos, pressed);
        break;
        case GLFW_MOUSE_BUTTON_RIGHT:
        this->rightButton.isPressed = pressed;
        this->rightButton.shiftWhenPressed = (mods & GLFW_MOD_SHIFT);
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
        Viewer *gw = static_cast<Viewer*>(glfwGetWindowUserPointer(window));
        assert(gw);
        gw->mouseButton(button, action, mods);
    }

    void addCallBack(GLFWwindow* window)
    {
        this->handle = window;
        glfwSetMouseButtonCallback(handle, glfwindow_mouseButton_cb);
        // glfwSetCursorPosCallback(window, glfwindow_mouseMotion_cb);
    }
};
