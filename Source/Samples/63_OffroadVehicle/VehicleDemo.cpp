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

#include <Urho3D/Core/CoreEvents.h>
#include <Urho3D/Core/ProcessUtils.h>
#include <Urho3D/Engine/Engine.h>
#include <Urho3D/Graphics/Camera.h>
#include <Urho3D/Graphics/Light.h>
#include <Urho3D/Graphics/Material.h>
#include <Urho3D/Graphics/Model.h>
#include <Urho3D/Graphics/Octree.h>
#include <Urho3D/Graphics/Renderer.h>
#include <Urho3D/Graphics/StaticModel.h>
#include <Urho3D/Graphics/Terrain.h>
#include <Urho3D/Graphics/Zone.h>
#include <Urho3D/Input/Input.h>
#include <Urho3D/IO/FileSystem.h>
#include <Urho3D/Physics/CollisionShape.h>
#include <Urho3D/Physics/Constraint.h>
#include <Urho3D/Physics/PhysicsWorld.h>
#include <Urho3D/Physics/RigidBody.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/UI/Font.h>
#include <Urho3D/UI/Text.h>
#include <Urho3D/UI/UI.h>
#include <Urho3D/Graphics/DebugRenderer.h>
#include <Urho3D/Audio/Audio.h>
#include <Urho3D/Audio/SoundListener.h>

#include "VehicleDemo.h"
#include "Vehicle.h"
#include "RaycastVehicle.h"
#include "WheelTrackModel.h"
#include "SmoothStep.h"

#include <Urho3D/DebugNew.h>

//=============================================================================
//=============================================================================
const float CAMERA_DISTANCE = 10.0f;
#define ONE_SEC_DURATION 1000

//=============================================================================
//=============================================================================
URHO3D_DEFINE_APPLICATION_MAIN(VehicleDemo)

//=============================================================================
//=============================================================================
VehicleDemo::VehicleDemo(Context* context) :
    Sample(context)
    , drawDebug_(false)
    , springVelocity_(0.0f)
{
    // Register factory and attributes for the Vehicle component so it can be created via CreateComponent, and loaded / saved
    Vehicle::RegisterObject(context);
    RaycastVehicle::RegisterObject(context);
    WheelTrackModel::RegisterObject(context);
}

void VehicleDemo::Setup()
{
    engineParameters_["WindowTitle"]  = GetTypeName();
    engineParameters_["LogName"]      = GetSubsystem<FileSystem>()->GetAppPreferencesDir("urho3d", "logs") + GetTypeName() + ".log";
    engineParameters_["FullScreen"]   = false;
    engineParameters_["Headless"]     = false;
    engineParameters_["WindowWidth"]  = 1280; 
    engineParameters_["WindowHeight"] = 720;
}

void VehicleDemo::Start()
{
    // Execute base class startup
    Sample::Start();

    // Create static scene content
    CreateScene();

    // Create the controllable vehicle
    CreateVehicle();

    InitAudio();

    // Create the UI content
    CreateInstructions();

    // Subscribe to necessary events
    SubscribeToEvents();

    // Set the mouse mode to use in the sample
    Sample::InitMouseMode(MM_RELATIVE);
}

void VehicleDemo::CreateScene()
{
    ResourceCache* cache = GetSubsystem<ResourceCache>();

    scene_ = new Scene(context_);

    // Create scene subsystem components
    scene_->CreateComponent<Octree>();
    PhysicsWorld *pPhysicsWorld = scene_->CreateComponent<PhysicsWorld>();
    DebugRenderer *dbgRenderer = scene_->CreateComponent<DebugRenderer>();
    pPhysicsWorld->SetDebugRenderer( dbgRenderer );

    // Create camera and define viewport. We will be doing load / save, so it's convenient to create the camera outside the scene,
    // so that it won't be destroyed and recreated, and we don't have to redefine the viewport on load
    cameraNode_ = new Node(context_);
    Camera* camera = cameraNode_->CreateComponent<Camera>();
    camera->SetFarClip(500.0f);
    GetSubsystem<Renderer>()->SetViewport(0, new Viewport(context_, scene_, camera));

    // Create static scene content. First create a zone for ambient lighting and fog control
    Node* zoneNode = scene_->CreateChild("Zone");
    Zone* zone = zoneNode->CreateComponent<Zone>();
    zone->SetAmbientColor(Color(0.15f, 0.15f, 0.15f));
    zone->SetFogColor(Color(0.5f, 0.5f, 0.7f));
    zone->SetFogStart(300.0f);
    zone->SetFogEnd(500.0f);
    zone->SetBoundingBox(BoundingBox(-2000.0f, 2000.0f));

    // Create a directional light with cascaded shadow mapping
    Node* lightNode = scene_->CreateChild("DirectionalLight");
    lightNode->SetDirection(Vector3(0.3f, -0.5f, 0.425f));
    Light* light = lightNode->CreateComponent<Light>();
    light->SetLightType(LIGHT_DIRECTIONAL);
    light->SetCastShadows(true);
    light->SetShadowBias(BiasParameters(0.00025f, 0.5f));
    light->SetShadowCascade(CascadeParameters(10.0f, 50.0f, 200.0f, 0.0f, 0.8f));
    light->SetSpecularIntensity(0.5f);

    // Create heightmap terrain with collision
    Node* terrainNode = scene_->CreateChild("Terrain");
    terrainNode->SetPosition(Vector3::ZERO);
    Terrain* terrain = terrainNode->CreateComponent<Terrain>();
    terrain->SetPatchSize(64);
    terrain->SetSpacing(Vector3(2.8f, 0.12f, 2.8f)); 
    terrain->SetHeightMap(cache->GetResource<Image>("Offroad/Terrain/HeightMapRace-257.png"));
    terrain->SetMaterial(cache->GetResource<Material>("Offroad/Terrain/TerrainRace-256.xml"));
    terrain->SetOccluder(true);

    RigidBody* body = terrainNode->CreateComponent<RigidBody>();
    body->SetCollisionLayer(2); // Use layer bitmask 2 for static geometry
    CollisionShape* shape = terrainNode->CreateComponent<CollisionShape>();
    shape->SetTerrain();
}

void VehicleDemo::CreateVehicle()
{
    Node* vehicleNode = scene_->CreateChild("Vehicle");
    vehicleNode->SetPosition(Vector3(273.0f, 7.0f, 77.0f));

    // Create the vehicle logic component
    vehicle_ = vehicleNode->CreateComponent<Vehicle>();
    vehicle_->Init();

    // smooth step
    vehicleRot_ = vehicleNode->GetRotation();
    Quaternion dir(vehicleRot_.YawAngle(), Vector3::UP);
    dir = dir * Quaternion(vehicle_->controls_.yaw_, Vector3::UP);
    dir = dir * Quaternion(vehicle_->controls_.pitch_, Vector3::RIGHT);
    targetCameraPos_ = vehicleNode->GetPosition() - dir * Vector3(0.0f, 0.0f, CAMERA_DISTANCE);
}

void VehicleDemo::InitAudio()
{
    Audio* audio = GetSubsystem<Audio>();
    audio->SetListener(cameraNode_->CreateComponent<SoundListener>());

}
void VehicleDemo::CreateInstructions()
{
    ResourceCache* cache = GetSubsystem<ResourceCache>();
    UI* ui = GetSubsystem<UI>();

    // Construct new Text object, set string to display and font to use
    Text* instructionText = ui->GetRoot()->CreateChild<Text>();
    instructionText->SetText(
        "Use WASD keys to drive\nSpacebar to brake\nBackspace to flip vehicle\nF5 to show debug"
    );
    instructionText->SetFont(cache->GetResource<Font>("Fonts/Anonymous Pro.ttf"), 12);
    instructionText->SetPosition(5, 5);

    textKmH_ = ui->GetRoot()->CreateChild<Text>();
    textKmH_->SetFont(cache->GetResource<Font>("Fonts/Anonymous Pro.ttf"), 15);
    textKmH_->SetColor(Color::GREEN);
    textKmH_->SetTextAlignment(HA_CENTER);
    textKmH_->SetHorizontalAlignment(HA_CENTER);
    textKmH_->SetPosition(0, ui->GetRoot()->GetHeight() - 140);

    textStatus_ = ui->GetRoot()->CreateChild<Text>();
    textStatus_->SetFont(cache->GetResource<Font>("Fonts/Anonymous Pro.ttf"), 12);
    textStatus_->SetColor(Color::YELLOW);
    textStatus_->SetPosition(5, 5);
    fpsTimer_.Reset();
    framesCount_ = 0;
}

void VehicleDemo::SubscribeToEvents()
{
    // Subscribe to Update event for setting the vehicle controls before physics simulation
    SubscribeToEvent(E_UPDATE, URHO3D_HANDLER(VehicleDemo, HandleUpdate));

    // Subscribe to PostUpdate event for updating the camera position after physics simulation
    SubscribeToEvent(E_POSTUPDATE, URHO3D_HANDLER(VehicleDemo, HandlePostUpdate));

    // Unsubscribe the SceneUpdate event from base class as the camera node is being controlled in HandlePostUpdate() in this sample
    UnsubscribeFromEvent(E_SCENEUPDATE);
}

void VehicleDemo::HandleUpdate(StringHash eventType, VariantMap& eventData)
{
    using namespace Update;

    Input* input = GetSubsystem<Input>();

    if (vehicle_)
    {
        UI* ui = GetSubsystem<UI>();

        // Get movement controls and assign them to the vehicle component. If UI has a focused element, clear controls
        if (!ui->GetFocusElement())
        {
            vehicle_->controls_.Set(CTRL_FORWARD, input->GetKeyDown(KEY_W));
            vehicle_->controls_.Set(CTRL_BACK, input->GetKeyDown(KEY_S));
            vehicle_->controls_.Set(CTRL_LEFT, input->GetKeyDown(KEY_A));
            vehicle_->controls_.Set(CTRL_RIGHT, input->GetKeyDown(KEY_D));
            vehicle_->controls_.Set(CTRL_SPACE, input->GetKeyDown(KEY_SPACE));

            // Add yaw & pitch from the mouse motion or touch input. Used only for the camera, does not affect motion
            if (touchEnabled_)
            {
                for (unsigned i = 0; i < input->GetNumTouches(); ++i)
                {
                    TouchState* state = input->GetTouch(i);
                    if (!state->touchedElement_)    // Touch on empty space
                    {
                        Camera* camera = cameraNode_->GetComponent<Camera>();
                        if (!camera)
                            return;

                        Graphics* graphics = GetSubsystem<Graphics>();
                        vehicle_->controls_.yaw_ += TOUCH_SENSITIVITY * camera->GetFov() / graphics->GetHeight() * state->delta_.x_;
                        vehicle_->controls_.pitch_ += TOUCH_SENSITIVITY * camera->GetFov() / graphics->GetHeight() * state->delta_.y_;
                    }
                }
            }
            else
            {
                vehicle_->controls_.yaw_ += (float)input->GetMouseMoveX() * YAW_SENSITIVITY;
                vehicle_->controls_.pitch_ += (float)input->GetMouseMoveY() * YAW_SENSITIVITY;
            }
            // Limit pitch
            vehicle_->controls_.pitch_ = Clamp(vehicle_->controls_.pitch_, 0.0f, 80.0f);

        }
        else
            vehicle_->controls_.Set(CTRL_FORWARD | CTRL_BACK | CTRL_LEFT | CTRL_RIGHT, false);

        // speed
        char buff[20];

        if (1)
        {
            float spd = vehicle_->GetSpeedKmH();
            if (spd<0.0f) spd = 0.0f;
            sprintf(buff, "%.0f KmH", spd);
        }
        else
        {
            float spd = vehicle_->GetSpeedMPH();
            if (spd<0.0f) spd = 0.0f;
            sprintf(buff, "%.0f MPH", spd);
        }
        String data(buff);
        int gear = vehicle_->GetCurrentGear() + 1;
        data += String("\ngear: ") + String(gear);
        float rpm = vehicle_->GetCurrentRPM();
        sprintf(buff, ", %.0f RPM", rpm);
        data += String(buff);
        textKmH_->SetText( data );

        // up right
        if (input->GetKeyPress(KEY_BACKSPACE))
        {
            Node* vehicleNode = vehicle_->GetNode();

            // qualify vehicle orientation
            Vector3 v3Up = vehicleNode->GetWorldUp();
            float fUp = v3Up.DotProduct( Vector3::UP );

            if ( v3Up.y_ < 0.1f )
            {
                // maintain its orientation
                Vector3 vPos = vehicleNode->GetWorldPosition();
                Vector3 vForward = vehicle_->GetNode()->GetDirection();
                Quaternion qRot;
                qRot.FromLookRotation( vForward );

                vPos += Vector3::UP * 3.0f;
                vehicleNode->SetTransform( vPos, qRot );
                vehicle_->ResetForces();
            }
        }

    }

    // Toggle physics debug geometry with space
    if ( input->GetKeyPress(KEY_F5))
    {
        drawDebug_ = !drawDebug_;
    }

    // stat
    #ifdef SHOW_STATS
    framesCount_++;
    if ( fpsTimer_.GetMSec(false) >= ONE_SEC_DURATION )
    {
        Renderer *renderer = GetSubsystem<Renderer>();
        String stat;

        stat.AppendWithFormat( "tris: %d fps: %d", 
                               renderer->GetNumPrimitives(),
                               framesCount_);

        #ifdef SHOW_CAM_POS
        String x, y, z;
        char buff[20];
        sprintf(buff, ", cam: %.1f, ", cameraNode_->GetPosition().x_);
        x = String(buff);
        sprintf(buff, "%.1f, ", cameraNode_->GetPosition().y_);
        y = String(buff);
        sprintf(buff, "%.1f", cameraNode_->GetPosition().z_);
        z = String(buff);
        stat += x + y + z;
        #endif

        textStatus_->SetText(stat);
        framesCount_ = 0;
        fpsTimer_.Reset();
    }
    #endif
}

void VehicleDemo::HandlePostUpdate(StringHash eventType, VariantMap& eventData)
{
    if (!vehicle_)
        return;

    using namespace Update;
    float timeStep = eventData[P_TIMESTEP].GetFloat();

    Node* vehicleNode = vehicle_->GetNode();

    // smooth step
    const float rotLerpRate = 10.0f;
    const float maxVel = 50.0f;
    const float damping = 0.2f;

    // Physics update has completed. Position camera behind vehicle
    vehicleRot_ = SmoothStepAngle(vehicleRot_, vehicleNode->GetRotation(), timeStep * rotLerpRate);
    Quaternion dir(vehicleRot_.YawAngle(), Vector3::UP);
    dir = dir * Quaternion(vehicle_->controls_.yaw_, Vector3::UP);
    dir = dir * Quaternion(vehicle_->controls_.pitch_, Vector3::RIGHT);

    Vector3 vehiclePos = vehicleNode->GetPosition();
    float curDist = (vehiclePos - targetCameraPos_).Length();

    curDist = SpringDamping(curDist, CAMERA_DISTANCE, springVelocity_, damping, maxVel, timeStep);
    targetCameraPos_ = vehiclePos - dir * Vector3(0.0f, 0.0f, curDist);

    Vector3 cameraTargetPos = targetCameraPos_;
    Vector3 cameraStartPos = vehiclePos;

    // Raycast camera against static objects (physics collision mask 2)
    // and move it closer to the vehicle if something in between
    Ray cameraRay(cameraStartPos, cameraTargetPos - cameraStartPos);
    float cameraRayLength = (cameraTargetPos - cameraStartPos).Length();
    PhysicsRaycastResult result;
    scene_->GetComponent<PhysicsWorld>()->RaycastSingle(result, cameraRay, cameraRayLength, 2);
    if (result.body_)
        cameraTargetPos = cameraStartPos + cameraRay.direction_ * (result.distance_ - 0.5f);

    cameraNode_->SetPosition(cameraTargetPos);
    cameraNode_->SetRotation(dir);

    if ( drawDebug_ )
    {
        scene_->GetComponent<PhysicsWorld>()->DrawDebugGeometry(true);

        vehicle_->DebugDraw(Color::MAGENTA);
    }

}
