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

#include "./math/Quaternion.h"

// struct Quaternion
// {
//     float r, i, j, k;

//     Quaternion(){};
//     Quaternion(float input)
//     {
//         r = input;
//         i = 0.f;
//         j = 0.f;
//         k = 0.f;
//     };
//     Quaternion(float w, float3 v)
//     {
//         r = w;
//         i = v.x;
//         j = v.y;
//         k = v.z;
//     };
//     Quaternion(float x, float y, float z, float w)
//     {
//         r = x;
//         i = y;
//         j = w;
//         k = z;
//     };
//     Quaternion(float3& vx, float3& vy, float3& vz )
//     {
//       if ( vx.x + vy.y + vz.z >= 0 )
//         {
//           const float t = 1.f + (vx.x + vy.y + vz.z);
//           const float s = 1/sqrtf(t)*float(0.5f);
//           r = t*s;
//           i = (vy.z - vz.y)*s;
//           j = (vz.x - vx.z)*s;
//           k = (vx.y - vy.x)*s;
//         }
//       else if ( vx.x >= std::max(vy.y, vz.z) )
//         {
//           const float t = (1.f + vx.x) - (vy.y + vz.z);
//           const float s = 1/sqrtf(t)*float(0.5f);
//           r = (vy.z - vz.y)*s;
//           i = t*s;
//           j = (vx.y + vy.x)*s;
//           k = (vz.x + vx.z)*s;
//         }
//       else if ( vy.y >= vz.z ) // if ( vy.y >= max(vz.z, vx.x) )
//         {
//           const float t = (1.f + vy.y) - (vz.z + vx.x);
//           const float s = 1/sqrtf(t)*float(0.5f);
//           r = (vz.x - vx.z)*s;
//           i = (vx.y + vy.x)*s;
//           j = t*s;
//           k = (vy.z + vz.y)*s;
//         }
//       else //if ( vz.z >= max(vy.y, vx.x) )
//         {
//           const float t = (1.f + vz.z) - (vx.x + vy.y);
//           const float s = 1/sqrtf(t)*float(0.5f);
//           r = (vx.y - vy.x)*s;
//           i = (vz.x + vx.z)*s;
//           j = (vy.z + vz.y)*s;
//           k = t*s;
//         }
//     }

//     float abs()
//     {
//         return sqrtf(r*r + i*i + j*j + k*k);
//     }

//     Quaternion conj()
//     {
//         return Quaternion(r, -i, -j, -k);
//     }

//     Quaternion multiplyBy(const Quaternion& b)
//     {
//         return Quaternion(r*b.r - i*b.i - j*b.j - k*b.k,
//                         r*b.i + i*b.r + j*b.k - k*b.j,
//                         r*b.j - i*b.k + j*b.r + k*b.i,
//                         r*b.k + i*b.j - j*b.i + k*b.r);
//     }

//     Quaternion multiplyBy(const float& a)
//     {
//         return Quaternion(r * a, i * a, j * a, k * a);
//     }

// };

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