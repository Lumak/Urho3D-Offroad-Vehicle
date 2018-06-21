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

#include <Urho3D/Physics/RigidBody.h>
#include <Bullet/BulletDynamics/Vehicle/btRaycastVehicle.h>

using namespace Urho3D;
namespace Urho3D
{
extern const char* PHYSICS_CATEGORY;
}

//=============================================================================
//=============================================================================
class RaycastVehicle : public RigidBody
{
    // **NOTE** the typename is specified as RigidBody due to many existing physics classes 
    // that calls GetComponent<RigidBody>() func. and not aware of RigidBody derived classes!
    URHO3D_OBJECT(RigidBody, RigidBody);
public:
	RaycastVehicle(Context *context); 
	virtual ~RaycastVehicle();

    static void RegisterObject(Context* context);

    void ResetSuspension();

    void SetVehicleCenterOfMass(const Vector3 &centerOfMass);
    const Vector3& GetVehicleCenterOfMass() const;

    float GetSteeringValue(int wheel) const;

    void SetSteeringValue(float steering, int wheel);

    void ApplyEngineForce(float force, int wheel);
	void SetBrake(float brake, int wheel);

    Vector3 GetWheelPositionWS(int wheel) const;
    Vector3 GetWheelPositionLS(int wheel) const;
    Quaternion GetWheelRotation(int wheel) const;

    const btTransform& GetWheelTransformWS(int wheel) const;

    void UpdateWheelTransform(int wheel, bool interpolatedTransform=true);

    btWheelInfo& AddWheel(const Vector3& connectionPointCS0, const Vector3& wheelDirectionCS0, const Vector3& wheelAxleCS, 
                          float suspensionRestLength, float wheelRadius, bool isFrontWheel);

    int GetNumWheels() const;

    const btWheelInfo& GetWheelInfo(int wheel) const;
    btWheelInfo& GetWheelInfo(int wheel);

	Vector3 GetForwardVector() const;

	float GetCurrentSpeedKmHour() const
	{
		return raycastVehicle_->getCurrentSpeedKmHour();
	}

    Vector3 GetCompoundLocalExtents() const;
    Vector3 GetCompooundLocalExtentsCenter() const;
    Vector3 GetCompoundLocalAabbMin() const;
    Vector3 GetCompoundLocalAabbMax() const;

    void CompoundScaleLocalAabbMin(const Vector3& scale);
    void CompoundScaleLocalAabbMax(const Vector3& scale);

protected:
    /// Create the rigid body, or re-add to the physics world with changed flags. Calls UpdateMass().
    virtual void AddBodyToWorld();

protected:
    btRaycastVehicle::btVehicleTuning   vehicleTuning_;
    btVehicleRaycaster                  *vehicleRaycaster_;
    btRaycastVehicle                    *raycastVehicle_;

    btCollisionShape                    *sphShape_;
    Vector3                             vehicleCenterOfMass_;
    int                                 indexAddedShape_;
};







