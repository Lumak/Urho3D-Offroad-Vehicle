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
#include <Urho3D/Core/Context.h>
#include <Urho3D/Physics/PhysicsWorld.h>
#include <Urho3D/Physics/PhysicsUtils.h>
#include <Bullet/BulletDynamics/Vehicle/btRaycastVehicle.h>
#include <Bullet/BulletDynamics/Dynamics/btDynamicsWorld.h>
#include <Bullet/BulletCollision/CollisionShapes/btCompoundShape.h>
#include <Bullet/BulletCollision/CollisionShapes/btSphereShape.h>
#include <SDL/SDL_log.h>

#include "RaycastVehicle.h"

//=============================================================================
//=============================================================================
RaycastVehicle::RaycastVehicle(Context *context) 
    : RigidBody(context)
    , vehicleRaycaster_(NULL)
    , raycastVehicle_(NULL)
    , sphShape_(NULL)
    , vehicleCenterOfMass_(Vector3::ZERO)
    , indexAddedShape_(-1)
{
}

RaycastVehicle::~RaycastVehicle()
{
    if (sphShape_)
    {
        delete sphShape_;
        sphShape_ = NULL;
    }
    if (vehicleRaycaster_)
    {
        delete vehicleRaycaster_;
        vehicleRaycaster_ = NULL;
    }
    if (raycastVehicle_)
    {
        if (GetPhysicsWorld())
        {
            btDynamicsWorld *pbtDynWorld = (btDynamicsWorld *)GetPhysicsWorld()->GetWorld();
            pbtDynWorld->removeVehicle(raycastVehicle_);

        }
        delete raycastVehicle_;
        raycastVehicle_ = NULL;
    }
}

void RaycastVehicle::RegisterObject(Context* context)
{
    context->RegisterFactory<RaycastVehicle>(PHYSICS_CATEGORY);
}

void RaycastVehicle::AddBodyToWorld()
{
    RigidBody::AddBodyToWorld();

    if ( GetBody() && vehicleRaycaster_ == NULL )
    {
        const int rightIndex = 0;
        const int upIndex = 1;
        const int forwardIndex = 2;

        btDynamicsWorld *pbtDynWorld = (btDynamicsWorld*)GetPhysicsWorld()->GetWorld();
        vehicleRaycaster_ = new btDefaultVehicleRaycaster(pbtDynWorld);
        raycastVehicle_ = new btRaycastVehicle(vehicleTuning_, GetBody(), vehicleRaycaster_);

        pbtDynWorld->addVehicle(raycastVehicle_);
        raycastVehicle_->setCoordinateSystem(rightIndex, upIndex, forwardIndex);
    }
}

void RaycastVehicle::SetVehicleCenterOfMass(const Vector3 &centerOfMass)
{
    if (sphShape_ == NULL)
    {
        // compound shape
        btTransform transf;
        transf.setIdentity();
        sphShape_ = new btSphereShape(0.2f);
        indexAddedShape_ = GetCompoundShape()->getNumChildShapes() ;

        // rigid body's center of mass = avg(sum of all compound positions) // size and mass are irrelevant
        vehicleCenterOfMass_ = centerOfMass;
        btVector3 pos = ToBtVector3(vehicleCenterOfMass_ * 2.0f);
        transf.setOrigin(pos);
        GetCompoundShape()->addChildShape(transf, sphShape_);
    }
    else
    {
        btTransform transf;
        transf.setIdentity();
        vehicleCenterOfMass_ = centerOfMass;
        btVector3 pos = ToBtVector3(vehicleCenterOfMass_ * 2.0f);
        transf.setOrigin(pos);

        GetCompoundShape()->updateChildTransform(indexAddedShape_, transf);
    }

    // updatemass to have the compound shape added to the rigid body and recalculate CoM
    UpdateMass();
}

const Vector3& RaycastVehicle::GetVehicleCenterOfMass() const
{
    return vehicleCenterOfMass_;
}

void RaycastVehicle::ResetSuspension()
{
    raycastVehicle_->resetSuspension();
}

float RaycastVehicle::GetSteeringValue(int wheel) const
{
    return raycastVehicle_->getSteeringValue(wheel);
}

void RaycastVehicle::SetSteeringValue(float steering, int wheel)
{
    raycastVehicle_->setSteeringValue(steering, wheel);
}

void RaycastVehicle::ApplyEngineForce(float force, int wheel)
{
    raycastVehicle_->applyEngineForce(force, wheel);
}

void  RaycastVehicle::SetBrake(float brake, int wheel)
{
    raycastVehicle_->setBrake(brake, wheel);
}

Vector3 RaycastVehicle::GetWheelPositionWS(int wheel) const
{
    const btTransform &transform = raycastVehicle_->getWheelTransformWS(wheel);
    return ToVector3(transform.getOrigin());
}

Vector3 RaycastVehicle::GetWheelPositionLS(int wheel) const
{
    btWheelInfo &whInfo = raycastVehicle_->getWheelInfo(wheel);
    return ToVector3(whInfo.m_chassisConnectionPointCS);
}

Quaternion RaycastVehicle::GetWheelRotation(int wheel) const
{
    const btTransform &transform = raycastVehicle_->getWheelTransformWS(wheel);
    return ToQuaternion(transform.getRotation());
}

const btTransform& RaycastVehicle::GetWheelTransformWS(int wheel) const
{
    return raycastVehicle_->getWheelTransformWS(wheel);
}

void RaycastVehicle::UpdateWheelTransform(int wheel, bool interpolatedTransform)
{
    raycastVehicle_->updateWheelTransform(wheel, interpolatedTransform);
}

btWheelInfo& RaycastVehicle::AddWheel(const Vector3& connectionPointCS0, const Vector3& wheelDirectionCS0, const Vector3& wheelAxleCS, 
                                      float suspensionRestLength, float wheelRadius, bool isFrontWheel)
{
    return raycastVehicle_->addWheel(ToBtVector3(connectionPointCS0), ToBtVector3(wheelDirectionCS0), ToBtVector3(wheelAxleCS),
                                     suspensionRestLength, wheelRadius, vehicleTuning_, isFrontWheel);
}   

int RaycastVehicle::GetNumWheels() const
{
    return raycastVehicle_->getNumWheels();
}

const btWheelInfo& RaycastVehicle::GetWheelInfo(int wheel) const
{
    return raycastVehicle_->getWheelInfo(wheel);
}

btWheelInfo& RaycastVehicle::GetWheelInfo(int wheel)
{
    return raycastVehicle_->getWheelInfo(wheel);
}

Vector3 RaycastVehicle::GetForwardVector() const
{
    return ToVector3(raycastVehicle_->getForwardVector());
}

Vector3 RaycastVehicle::GetCompoundLocalExtents() const
{
    const btVector3& minAabb = GetCompoundShape()->getLocalAabbMin();
    const btVector3& maxAabb = GetCompoundShape()->getLocalAabbMax();

    return ToVector3(btScalar(0.5)*(maxAabb-minAabb));
}

Vector3 RaycastVehicle::GetCompooundLocalExtentsCenter() const
{
    const btVector3& minAabb = GetCompoundShape()->getLocalAabbMin();
    const btVector3& maxAabb = GetCompoundShape()->getLocalAabbMax();

    return ToVector3(btScalar(0.5)*(maxAabb+minAabb));
}

Vector3 RaycastVehicle::GetCompoundLocalAabbMin() const
{
    const btVector3& minAabb = GetCompoundShape()->getLocalAabbMin();
    return ToVector3(minAabb);
}

Vector3 RaycastVehicle::GetCompoundLocalAabbMax() const
{
    const btVector3& maxAabb = GetCompoundShape()->getLocalAabbMax();
    return ToVector3(maxAabb);
}

void RaycastVehicle::CompoundScaleLocalAabbMin(const Vector3& scale)
{
    btVector3& minAabb = GetCompoundShape()->getLocalAabbMin();
    Vector3 aabb = ToVector3(minAabb);
    aabb.x_ *= scale.x_;
    aabb.y_ *= scale.y_;
    aabb.z_ *= scale.z_;
    minAabb = ToBtVector3(aabb);
}

void RaycastVehicle::CompoundScaleLocalAabbMax(const Vector3& scale)
{
    btVector3& maxAabb = GetCompoundShape()->getLocalAabbMax();
    Vector3 aabb = ToVector3(maxAabb);
    aabb.x_ *= scale.x_;
    aabb.y_ *= scale.y_;
    aabb.z_ *= scale.z_;
    maxAabb = ToBtVector3(aabb);
}



