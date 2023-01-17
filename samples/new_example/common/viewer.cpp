#ifndef INCLUDE_VIEWER_H
#define INCLUDE_VIEWER_H
#include "./viewer.hpp"
#endif

#ifndef INCLUDE_CONTANTS
#define INCLUDE_CONTANTS
#include "./constants.hpp"
#endif

#ifndef INCLUDE_CONFIGUREIMGUI
#define INCLUDE_CONFIGUREIMGUI
#include "./configureImgui.cpp"
#endif

#ifndef M_PI
#define M_PI 3.1415926f
#endif

#include <cstring>
#include <string>

int2 getMousePos(GLFWwindow *window)
{
    double x, y;
    std::cout << "Collect Cursor Position" << std::endl;
    glfwGetCursorPos(window, &x, &y);
    std::cout << "Cursor Position: " << (int)x << " " << (int)y << std::endl;
    return int2((int)x, (int)y);
}

/*! callback for pressing _or_ releasing a mouse button*/
static void glfwindow_mouseButton_cb(GLFWwindow *window,
                                     int button,
                                     int action,
                                     int mods)
{
    /* action handled by ImGui */
    if (ImGui::GetIO().WantCaptureMouse)
    {
        if (action == GLFW_PRESS){
            std::cout << "Mouse Inside Imgui" << std::endl;
        }
        return;
    }
    std::cout << "Mouse Outside Imgui" << std::endl;
    Viewer *gw = static_cast<Viewer *>(glfwGetWindowUserPointer(window));
    assert(gw);
    gw->mouseButton(window, button, action, mods);
}

/*! callback for a key press */
static void glfwindow_key_cb(GLFWwindow *window,
                             int key,
                             int scancode,
                             int action,
                             int mods)
{
    /* action handled by ImGui */
    if (ImGui::GetIO().WantCaptureKeyboard)
    {
        if (action == GLFW_PRESS){
            std::cout << "Keyboard Inside Imgui" << std::endl;
        }
        return;
    }

    Viewer *gw = static_cast<Viewer *>(glfwGetWindowUserPointer(window));
    assert(gw);
    if (action == GLFW_PRESS)
    {
        gw->key(key, getMousePos(window));
    }
}

/*! callback for _moving_ the mouse to a new position */
static void glfwindow_mouseMotion_cb(GLFWwindow *window, double x, double y)
{
    Viewer *gw = static_cast<Viewer *>(glfwGetWindowUserPointer(window));
    assert(gw);
    gw->mouseMotion(int2((int)x, (int)y));
}

int Viewer::arg_min(float3 &v)
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
inline owl::common::Quaternion3f Viewer::rotation(const owl::common::linear3f &a)
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

void Viewer::mouseButtonLeft(const int2 &where, bool pressed)
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

void Viewer::mouseButtonRight(const int2 &where, bool pressed)
{
    std::cout << "Right Click" << std::endl;
    lastMousePosition = where;
}

void Viewer::mouseButton(GLFWwindow *window, int button, int action, int mods)
{
    std::cout << "Button Click" << std::endl;
    pressed = (action == GLFW_PRESS);
    lastMousePos = getMousePos(window);
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

/*! return matrix for rotation around arbitrary axis */
inline owl::common::linear3f Viewer::ling_rotate(const float3 &_u, const float &r)
{
    float3 u = normalize(_u);
    float s = sin(r);
    float c = cos(r);
    return owl::common::linear3f(
        u.x * u.x + (1 - u.x * u.x) * c, u.x * u.y * (1 - c) - u.z * s, u.x * u.z * (1 - c) + u.y * s,
        u.x * u.y * (1 - c) + u.z * s, u.y * u.y + (1 - u.y * u.y) * c, u.y * u.z * (1 - c) - u.x * s,
        u.x * u.z * (1 - c) - u.y * s, u.y * u.z * (1 - c) + u.x * s, u.z * u.z + (1 - u.z * u.z) * c);
}

void Viewer::rotate(const float deg_u, const float deg_v)
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

    camera.lastModified = true;
}

float3 Viewer::arcballProject(const int2 &where)
{
    int width = fbSize.x;
    int height = fbSize.y;

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

void Viewer::mouseDragLeft(const int2 &where, const int2 &delta)
{
    if (Arcball)
    {
        float3 curr_pos = arcballProject(where + delta);
        float3 from = normalize(arcball.down_pos);
        float3 to = normalize(curr_pos);
        arcball.rotation = owl::common::Quaternion3f(dot(from, to), cross(from, to)) * arcball.down_rotation;

        owl::common::LinearSpace3f rotation_matrix(conj(arcball.rotation));

        Camera &fc = this->camera;

        float3 eye(0.f, 0.f, length(fc.getPOI() - fc.position));
        eye = rotation_matrix * eye;
        eye += fc.getPOI();

        float3 up = rotation_matrix.vy;

        fc.setOrientation(eye, fc.getPOI(), up, fc.fovyInDegrees, fc.focalDistance);

        if (fc.forceUp)
        {
            fc.forceUpFrame();
        }

        camera.lastModified = true;
    }
    else
    {
        const float2 fraction = float2(delta) / float2(fbSize);
        rotate(fraction.x * degrees_per_drag_fraction,
               fraction.y * degrees_per_drag_fraction);
    }
}

void Viewer::move(const float step)
{
    Camera &fc = this->camera;

    const float3 poi = fc.getPOI();
    fc.poiDistance = std::min(maxDistance, std::max(minDistance, fc.poiDistance - step * fc.motionSpeed));
    fc.position = poi + fc.poiDistance * fc.frame.vz;

    camera.lastModified = true;
}

/*! mouse got dragged with left button pressedn, by 'delta'
  pixels, at last position where */
void Viewer::mouseDragRight(const int2 &where, const int2 &delta)
{
    const float2 fraction = float2(delta) / float2(fbSize);
    move(fraction.y * pixels_per_move);
}

/*! this gets called when the window determines that the mouse got
  _moved_ to the given position */
void Viewer::mouseMotion(const int2 &newMousePosition)
{
    if (lastMousePosition != int2(-1))
    {
        if (leftButton.isPressed)
            mouseDragLeft(newMousePosition, newMousePosition - lastMousePosition);
        if (rightButton.isPressed)
            mouseDragRight(newMousePosition, newMousePosition - lastMousePosition);
    }
    lastMousePosition = newMousePosition;
}

void Viewer::key(int key, const int2 &where)
{
    switch (key)
    {
    // case 'p':
    //     std::cout<< "Switch Imgui and Camera Control"<<std::endl;
    //     break;
    case GLFW_KEY_ESCAPE:
        glfwSetWindowShouldClose(handle, GLFW_TRUE);
        break;
    case GLFW_KEY_G:
        configureImgui.showImgui = !configureImgui.showImgui;
        break;
    case GLFW_KEY_UP:
        std::cout << "Up Key Pressed" << std::endl;
        break;
    }
}

void Viewer::init(GPRTProgram new_example_deviceCode)
{
    initWindow();
    camera.setOrientation(lookFrom,
                            lookAt,
                            lookUp,
                            cosFovy);
    configureImgui.initObj();
    deviceCode = new_example_deviceCode;
    vulkanResources.initialVulkanResources(new_example_deviceCode);
}

void Viewer::initWindow()
{
    if (!glfwInit())
        exit(EXIT_FAILURE);

    // todo, allow the window to resize and recreate swapchain
    glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    GLFWwindow *window = glfwCreateWindow(fbSize.x, fbSize.y, "New Example", NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        exit(EXIT_FAILURE);
    }
    handle = window;
    glfwSetWindowUserPointer(handle, this);
    glfwMakeContextCurrent(handle);
    glfwSwapInterval(1);

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    // Setup Dear ImGui style
    ImGui::StyleColorsDark();

    // Custom CallBack
    addWindowCallBack();
    
    // Setup Platform/Renderer bindings
    ImGui_ImplGlfw_InitForOpenGL(handle, true);
    ImGui_ImplOpenGL2_Init();
}

void Viewer::destory() {
    destoryWindow();
    vulkanResources.destoryVulkanResources();
}

void Viewer::destoryWindow()
{
    ImGui_ImplOpenGL2_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(handle);
    glfwTerminate();
}

void Viewer::addWindowCallBack()
{
    glfwSetMouseButtonCallback(handle, glfwindow_mouseButton_cb);
    glfwSetKeyCallback(handle, glfwindow_key_cb);
    glfwSetCursorPosCallback(handle, glfwindow_mouseMotion_cb);
}

void Viewer::recalculateCamera()
{
    const float3 newlookFrom = camera.getFrom();
    const float3 newlookAt = camera.getAt();
    const float3 newlookUp = camera.getUp();
    const float cosFovy = camera.getCosFovy();
    const float vfov = toDegrees(acosf(cosFovy));
    camera.toString();
    // ........... compute variable values  ..................
    const float3 vup = newlookUp;
    const float aspect = fbSize.x / float(fbSize.y);
    const float theta = vfov * ((float)M_PI) / 180.0f;
    const float half_height = tanf(theta / 2.0f);
    const float half_width = aspect * half_height;
    const float focusDist = 10.f;
    const float3 origin = newlookFrom;
    const float3 w = normalize(newlookFrom - newlookAt);
    const float3 u = normalize(cross(vup, w));
    const float3 v = cross(w, u);
    const float3 lower_left_corner = origin - half_width * focusDist * u - half_height * focusDist * v - focusDist * w;
    const float3 horizontal = 2.0f * half_width * focusDist * u;
    const float3 vertical = 2.0f * half_height * focusDist * v;

    // ----------- set variables  ----------------------------
    vulkanResources.rayGenData->camera.pos = origin;
    vulkanResources.rayGenData->camera.dir_00 = lower_left_corner;
    vulkanResources.rayGenData->camera.dir_du = horizontal;
    vulkanResources.rayGenData->camera.dir_dv = vertical;
}

void Viewer::run()
{
    while (!glfwWindowShouldClose(handle)) {
        // If we click the mouse, we should rotate the camera
        if (camera.lastModified || firstFrame)
        {
            camera.lastModified = false;
            accId = 0;
            firstFrame = false;
            recalculateCamera();
        }

        vulkanResources.rayGenData->accId = (uint64_t)accId;
        accId++;
        // Now, trace rays
        gprtBuildShaderBindingTable(vulkanResources.context, GPRT_SBT_RAYGEN);
        gprtRayGenLaunch2D(vulkanResources.context, vulkanResources.rayGen, fbSize.x, fbSize.y);

        // If a window exists, presents the framebuffer here to that window
        // gprtBufferPresent(context, frameBuffer);

        // Render results to screen
        void *pixels = gprtBufferGetPointer(vulkanResources.frameBuffer);
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

        configureImgui.render();

        glfwSwapBuffers(handle);
        glfwPollEvents();

        if (configureImgui.updateSelectedObj) {
            firstFrame = true;
            accId = 0;
            vulkanResources.resetVulkanGeometryResources(deviceCode);
            configureImgui.updateSelectedObj = false;
        }
    }
}
