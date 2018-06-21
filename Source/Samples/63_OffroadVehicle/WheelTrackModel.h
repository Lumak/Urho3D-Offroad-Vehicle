//
// Copyright (c) 2008-2018 Lumak.
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

#include <Urho3D/Urho3D.h>
#include <Urho3D/Core/Object.h>

using namespace Urho3D;
namespace Urho3D
{
}

//=============================================================================
//=============================================================================
//#define DBG_SKID_STRIPS

//=============================================================================
//=============================================================================
struct SkidStrip
{
    SkidStrip() : vertsArrayValid(false), lastPosCnt(0), valid(false){}

    Vector3     pos;
    Vector3     normal;

    Vector3     v[2];

    bool        vertsArrayValid;
    int         lastPosCnt;
    bool        valid;
};

struct GeomData
{
    Vector3     pos;
    Vector3     normal;
    unsigned    color;
    Vector2     uv;
};

//=============================================================================
//=============================================================================
class WheelTrackModel : public StaticModel
{
    URHO3D_OBJECT(WheelTrackModel, StaticModel);

public:
    static void RegisterObject(Context* context);

    WheelTrackModel(Context *context) : StaticModel(context)
    {
        m_fWidth = 1.0f;
        m_fHalfWidth = 0.5f;
    }
    virtual ~WheelTrackModel()
    {
        m_pParentNode = NULL;
    }

    // world pos methods
    virtual void OnWorldBoundingBoxUpdate();
    void SetParentNode(Node *pParentNode);
    void UpdateWorldPos();

    // validation and set methods
    bool CreateVBuffer(const Color &color, float width);
    bool ValidateBufferElements() const;
    void SetWidth(float width)          
    { 
        m_fWidth = width; 
        m_fHalfWidth = width * 0.5f;
    }
    void SetColor(const Color &color)   
    { 
        m_Color = color; 
        m_unsignedColor = color.ToUInt();
    }

    // skid strip
    void AddStrip(const Vector3 &pos, const Vector3 &normal);
    void ClearStrip();
    bool InSkidState() const;

    const PODVector<GeomData>& GetGeomVector() 
    {
        return m_vSkidGeom;
    }

    void DebugRender(DebugRenderer *dbgRender, const Color &color);

protected:
    void CopyToBuffer();

protected:
    WeakPtr<Node> m_pParentNode;

    // strip
    PODVector<GeomData>         m_vSkidGeom;
    PODVector<unsigned short>   m_vSkidIndex;

    SkidStrip   m_firstStripPoint;
    Color       m_Color;
    unsigned    m_unsignedColor;
    float       m_fWidth;
    float       m_fHalfWidth;
};








