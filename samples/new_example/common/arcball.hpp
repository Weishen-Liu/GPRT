#pragma once
#ifndef INCLUDE_NEWEXAMPLE_COMMON_ARCBALL
#define INCLUDE_NEWEXAMPLE_COMMON_ARCBALL

#include <gprt.h>
#include <GLFW/glfw3.h>

#include "./math/Quaternion.h"

struct ArcBall
{
    /*! the current rotation
        zero-length signifies that the arcball wasn't
        initialized yet */
    owl::common::Quaternion3f rotation = owl::common::Quaternion3f(0.f);

    /*! the rotation when the mouse button was pressed */
    owl::common::Quaternion3f down_rotation = owl::common::Quaternion3f(1.f);

    /*! the position projected to the unit arcball when
        the dragging mouse button was pressed */
    float3 down_pos = float3(0.f);
};

#endif
