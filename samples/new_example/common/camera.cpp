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
#include "./arcball.cpp"
#endif
#include "./math/Quaternion.h"
#include "./math/LinearSpace.h"
#include <iostream>

// struct Space3f
// {
//     float3 vx, vy, vz;

//     Space3f(){};
//     Space3f(float input)
//     {
//         vx = float3(input, 0, 0);
//         vy = float3(0, input, 0);
//         vz = float3(0, 0, input);
//     };
//     Space3f(float3 x, float3 y, float3 z)
//     {
//         vx = x;
//         vy = y;
//         vz = z;
//     };
//     Space3f(const owl::common::Quaternion3f &q)
//     {
//         vx = float3((q.r * q.r + q.i * q.i - q.j * q.j - q.k * q.k), 2.0f * (q.i * q.j + q.r * q.k), 2.0f * (q.i * q.k - q.r * q.j));
//         vy = float3(2.0f * (q.i * q.j - q.r * q.k), (q.r * q.r - q.i * q.i + q.j * q.j - q.k * q.k), 2.0f * (q.j * q.k + q.r * q.i));
//         vz = float3(2.0f * (q.i * q.k + q.r * q.j), 2.0f * (q.j * q.k - q.r * q.i), (q.r * q.r - q.i * q.i - q.j * q.j + q.k * q.k));
//     }
//     Space3f(const float &m00, const float &m01, const float &m02,
//             const float &m10, const float &m11, const float &m12,
//             const float &m20, const float &m21, const float &m22)
//     {
//         vx = float3(m00, m10, m20);
//         vy = float3(m01, m11, m21);
//         vz = float3(m02, m12, m22);
//     }

//     float3 multiplyBy(const float3 &a)
//     {
//         return a.x * vx + a.y * vy + a.z * vz;
//     }

//     Space3f multiplyBy(const Space3f &a)
//     {
//         return Space3f(multiplyBy(a.vx), multiplyBy(a.vy), multiplyBy(a.vz));
//     }
// };

inline float toRadian(float deg) { return deg * float(M_PI / 180.f); }
inline float toDegrees(float rad) { return rad / float(M_PI / 180.f); }

struct Camera
{
    owl::common::linear3f frame{1.f};
    float3 position{0, -1, 0};
    /*! distance to the 'point of interst' (poi); e.g., the point we
        will rotate around */
    float poiDistance{1.f};
    float focalDistance{1.f};
    float3 upVector{0, 1, 0};
    /* if set to true, any change to the frame will always use to
        upVector to 'force' the frame back upwards; if set to false,
        the upVector will be ignored */
    bool forceUp{true};

    /*! multiplier how fast the camera should move in world space
        for each unit of "user specifeid motion" (ie, pixel
        count). Initial value typically should depend on the world
        size, but can also be adjusted. This is actually something
        that should be more part of the manipulator viewer(s), but
        since that same value is shared by multiple such viewers
        it's easiest to attach it to the camera here ...*/
    float motionSpeed{1.f};
    float aspect{1.f};
    float fovyInDegrees{60.f};
    owl::common::Quaternion3f rotation;
    bool lastModified = true;

    float3 getPOI() const
    {
        return position - poiDistance * frame.vz;
    }
    float getFovyInDegrees() const { return fovyInDegrees; }
    float getCosFovy() const { return cosf(toRadian(fovyInDegrees)); }
    
    float3 getFrom() const
    {
        return position;
    }

    float3 getAt() const
    {
        return position - frame.vz;
    }

    float3 getUp() const
    {
        return frame.vy;
    }

    void setUpVector(const float3 &up)
    {
        upVector = up;
        forceUpFrame();
    }

    void setFovy(const float fovy)
    {
        this->fovyInDegrees = fovy;
    }

    void setAspect(const float aspect)
    {
        this->aspect = aspect;
    }

    void setFocalDistance(float focalDistance)
    {
        this->focalDistance = focalDistance;
    }

    /*! tilt the frame around the z axis such that the y axis is "facing upwards" */
    void forceUpFrame()
    {
        // frame.vz remains unchanged
        if (fabsf(dot(frame.vz, upVector)) < 1e-6f)
            // looking along upvector; not much we can do here ...
            return;
        frame.vx = normalize(cross(upVector, frame.vz));
        frame.vy = normalize(cross(frame.vz, frame.vx));
    }

    void setOrientation(/* camera origin    : */ const float3 &origin,
                        /* point of interest: */ const float3 &interest,
                        /* up-vector        : */ const float3 &up,
                        /* fovy, in degrees : */ float inputFovyInDegrees,
                        /* set focal dist?  : */ bool setFocalDistance=true)
    {
        fovyInDegrees = inputFovyInDegrees;
        position = origin;
        upVector = up;
        frame.vz = (interest == origin)
                       ? float3(0, 0, 1)
                       : /* negative because we use NEGATIZE z axis */ -normalize(interest - origin);
        frame.vx = cross(up, frame.vz);
        if (dot(frame.vx, frame.vx) < 1e-8f)
            frame.vx = float3(0, 1, 0);
        else
            frame.vx = normalize(frame.vx);
        frame.vy = normalize(cross(frame.vz, frame.vx));
        poiDistance = length(interest - origin);
        if (setFocalDistance)
            focalDistance = poiDistance;
        forceUpFrame();
    }

    void toString() {
        std::cout<< "Camera Position: " << getFrom() << std::endl;
        std::cout<< "Camera At: " << getAt() << std::endl;
        std::cout<< "Camera Up: " << getUp() << std::endl;
    }
};