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
#pragma once
#include <Urho3D/Math/Vector3.h>
#include <Urho3D/Math/Quaternion.h>

using namespace Urho3D;

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

Vector3 SmoothStep(const Vector3 &p1, const Vector3 &p2, float t, float tolerance=0.001f);
Quaternion SmoothStepAngle(const Quaternion &p1, const Quaternion &p2, float t, float tolerance=0.001f);
float SpringDamping(float curDist, float desiredDist, float &velocity, float damping, float maxVel, float timestep);







