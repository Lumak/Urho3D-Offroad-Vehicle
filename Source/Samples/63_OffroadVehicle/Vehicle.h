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

#include <Urho3D/Input/Controls.h>
#include <Urho3D/Scene/LogicComponent.h>
#include "RaycastVehicle.h"

namespace Urho3D
{
class Constraint;
class Node;
class RigidBody;
class SoundSource3D;
}

using namespace Urho3D;

class WheelTrackModel;
class RaycastVehicle;

//=============================================================================
//=============================================================================
const int CTRL_FORWARD = 1;
const int CTRL_BACK = 2;
const int CTRL_LEFT = 4;
const int CTRL_RIGHT = 8;
const int CTRL_SPACE = 16;

const float YAW_SENSITIVITY = 0.1f;
const float ENGINE_POWER = 10.0f;
const float DOWN_FORCE = 10.0f;
const float MAX_WHEEL_ANGLE = 22.5f;

//=============================================================================
//=============================================================================
#define KMH_TO_MPH              (1.0f/1.60934f)

//=============================================================================
// Vehicle component, responsible for physical movement according to controls.
//=============================================================================
class Vehicle : public LogicComponent
{
    URHO3D_OBJECT(Vehicle, LogicComponent)

public:
    /// Construct.
    Vehicle(Context* context);
    ~Vehicle();
   
    /// Register object factory and attributes.
    static void RegisterObject(Context* context);
   
    /// Perform post-load after deserialization. Acquire the components from the scene nodes.
    virtual void ApplyAttributes();

    /// Initialize the vehicle. Create rendering and physics components. Called by the application.
    void Init();

    /// Handle physics world update. Called by LogicComponent base class.
    virtual void FixedUpdate(float timeStep);
    virtual void FixedPostUpdate(float timeStep);
    virtual void PostUpdate(float timeStep);
   
    void ResetForces()
    {
        raycastVehicle_->ResetForces(); 
        raycastVehicle_->SetAngularVelocity( Vector3::ZERO );
    }

    float GetSpeedKmH() const       { return raycastVehicle_->GetCurrentSpeedKmHour(); }
    float GetSpeedMPH() const       { return raycastVehicle_->GetCurrentSpeedKmHour()*KMH_TO_MPH; }
    void SetDbgRender(bool enable)  { dbgRender_ = enable; }
    int GetCurrentGear() const      { return curGearIdx_; }
    float GetCurrentRPM() const     { return curRPM_; }

    void DebugDraw(const Color &color);

    /// Movement controls.
    Controls controls_;
   
protected:
    void UpdateSteering(float newSteering);
    void ApplyEngineForces(float accelerator, bool braking);
    bool ApplyStiction(float steering, float acceleration, bool braking);
    void ApplyDownwardForce();
    void AutoCorrectPitchRoll();
    void UpdateGear();
    void UpdateDrift();
    void LimitLinearAndAngularVelocity();
    void PostUpdateSound(float timeStep);
    void PostUpdateWheelEffects();

protected:
    WeakPtr<RaycastVehicle> raycastVehicle_;
   
    /// Current left/right steering amount (-1 to 1.)
    float steering_;

    // IDs of the wheel scene nodes for serialization.
    Vector<Node*>           m_vpNodeWheel;

    float   m_fVehicleMass;
    float   m_fEngineForce;
    float   m_fBreakingForce;

    float   m_fmaxEngineForce;
    float   m_fmaxBreakingForce;

    float   m_fVehicleSteering;
    float   m_fsteeringIncrement;
    float   m_fsteeringClamp;
    float   m_fwheelRadius;
    float   m_fwheelWidth;
    float   m_fwheelFriction;
    float   m_fsuspensionStiffness;
    float   m_fsuspensionDamping;
    float   m_fsuspensionCompression;
    float   m_frollInfluence;
    float   m_fsuspensionRestLength;

    // slip vars
    float   m_fMaxSteering;
    float   m_fsideFrictionStiffness;
    float   m_fRearSlip;

    Vector3 centerOfMassOffset_;

    // acceleration
    float currentAcceleration_;

    // ang velocity limiter
    float   m_fYAngularVelocity;

    // wheel contacts
    int numWheels_;
    int numWheelContacts_;
    int prevWheelContacts_;
    bool isBraking_;
    PODVector<float> gearShiftSpeed_;
    PODVector<bool>  prevWheelInContact_;

    // gears
    float downShiftRPM_;
    float upShiftRPM_;
    int numGears_;
    int curGearIdx_;
    float curRPM_;
    float minIdleRPM_;

    // sound
    SharedPtr<Sound>         engineSnd_;
    SharedPtr<Sound>         skidSnd_;
    SharedPtr<Sound>         shockSnd_;
    SharedPtr<SoundSource3D> engineSoundSrc_;
    SharedPtr<SoundSource3D> skidSoundSrc_;
    SharedPtr<SoundSource3D> shockSoundSrc_;
    bool                     playAccelerationSoundInAir_;

    // wheel effects - skid track and particles
    SharedPtr<WheelTrackModel> wheelTrackList_[4];
    Vector<Node*>              particleEmitterNodeList_;
    
    // dbg render
    bool dbgRender_;

};

