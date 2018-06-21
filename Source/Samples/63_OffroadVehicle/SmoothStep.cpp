//
// Copyright (c) 2008-2016 the Urho3D project.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//

#include "SmoothStep.h"

//=============================================================================
// CatmullRom smooth fn (alternative to simple lerp)
//=============================================================================
//template <typename T> Variant CalculateCatmullRom(const T& p0, const T& p1, const T& p2, const T& p3, float t, float t2, float t3)
//{
//    return Variant(0.5f * ((2.0f * p1) + (-p0 + p2) * t +
//        (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3) * t2 +
//        (-p0 + 3.0f * p1 - 3.0f * p2 + p3) * t3));
//}
//
// *** substitue p0 = p1 and p3 = p2 ***
//Variant(0.5f * ((2.0f * p1) + (-p1 + p2) * t +
//        (2.0f * p1 - 5.0f * p1 + 4.0f * p2 - p2) * t2 +
//        (-p1 + 3.0f * p1 - 3.0f * p2 + p2) * t3));
//
//Variant(0.5f * ( 2.0f * p1 + (-p1 + p2) * t + ( -3.0f * p1 + 3.0f * p2) * t2 + ( 2.0f * p1 - 2.0f * p2) * t3));
//
//Variant(0.5f * ( 2.0f*p1 + (-p1 + p2) * t + (-p1 + p2) * 3.0f*t2 + (-p1 + p2) * -2.0f*t3));
//
//Variant(p1 + (-p1 + p2) * 0.5f*t + (-p1 + p2) * 1.5f*t2 + (-p1 + p2) * -t3);
//
//Variant(p1 + (p2-p1)*(0.5f*t + 1.5f*t2 - t3));

Vector3 SmoothStep(const Vector3 &p1, const Vector3 &p2, float t, float tolerance)
{
    if (t < M_EPSILON) 
        return p1;
    if (t > 1.0f - M_EPSILON) 
        return p2;
    if ((p1-p2).LengthSquared() < tolerance*tolerance) 
        return p2;

    float t2 = t * t;
    float t3 = t2 * t;
    return p1.Lerp(p2, 0.5f * t + 1.5f * t2 - t3);
}

Quaternion SmoothStepAngle(const Quaternion &p1, const Quaternion &p2, float t, float tolerance)
{
    if (t < M_EPSILON) 
        return p1;
    if (t > 1.0f - M_EPSILON) 
        return p2;
    if ((p1-p2).LengthSquared() < tolerance*tolerance) 
        return p2;

    float t2 = t * t;
    float t3 = t2 * t;
    return p1.Slerp(p2, 0.5f * t + 1.5f * t2 - t3);
}

//=============================================================================
// spring implicit euler fn.
// https://github.com/TheAllenChou/numeric-springing 
// MIT license
// x     -   curDist     - value             (input/output)
// xt    -   desiredDist - target value      (input)
// v     -   velocity    - velocity          (input/output)
// zeta  -   damping     - damping ratio     (input)
// omega -   maxVel      - angular frequency (input)
// h     -   timestep    - time step         (input)
//=============================================================================
float SpringDamping(float curDist, float desiredDist, float &velocity, float damping, float maxVel, float timestep)
{
    const float tstep = 0.5f * timestep + 1.5f * timestep*timestep - timestep*timestep*timestep;
    const float f = 1.0f + 2.0f * tstep * damping * maxVel;
    const float oo = maxVel * maxVel;
    const float hoo = tstep * oo;
    const float hhoo = tstep * hoo;
    const float detInv = 1.0f / (f + hhoo);
    const float detX = f * curDist + tstep * velocity + hhoo * desiredDist;
    const float detV = velocity + hoo * (desiredDist - curDist);

    velocity = detV * detInv;
    return detX * detInv;
} 
 



