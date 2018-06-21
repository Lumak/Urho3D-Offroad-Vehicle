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

#include <Urho3D/Urho3D.h>
#include <Urho3D/Core/Context.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/Graphics/StaticModel.h>
#include <Urho3D/Graphics/Model.h>
#include <Urho3D/Graphics/Geometry.h>
#include <Urho3D/Graphics/VertexBuffer.h>
#include <Urho3D/Graphics/IndexBuffer.h>
#include <Urho3D/Graphics/DebugRenderer.h>
#include <SDL/SDL_log.h>

#include "WheelTrackModel.h"

#include <Urho3D/DebugNew.h>

//=============================================================================
//=============================================================================
#define MAX_SKID_STRIPS         120
#define VERTS_PER_STRIP         4
#define INDECES_PER_STRIP       6
#define STRIP_NORMAL_OFFSET     0.05f
#define MIN_STRIP_LEN           0.4f
#define MAX_STRIP_LEN           4.0f
#define MIN_LASTPOS_CNT         8
#define SMALL_BIT               0.99f

//=============================================================================
//=============================================================================
void WheelTrackModel::RegisterObject(Context* context)
{
    context->RegisterFactory<WheelTrackModel>();
}

void WheelTrackModel::OnWorldBoundingBoxUpdate()
{
    if ( m_pParentNode )
    {
        worldBoundingBox_ = boundingBox_.Transformed(m_pParentNode->GetWorldTransform());
    }
    else
    {
        StaticModel::OnWorldBoundingBoxUpdate();
    }
}

void WheelTrackModel::SetParentNode(Node *pParentNode)
{
    m_pParentNode = pParentNode;
}

void WheelTrackModel::UpdateWorldPos()
{
    OnMarkedDirty(node_);
}

// **not used. instead, just use a model with (MASK_POSITION | MASK_NORMAL | MASK_COLOR | MASK_TEXCOORD1) elements
bool WheelTrackModel::CreateVBuffer(const Color &color, float width)
{
    assert(false && "just use a model with (MASK_POSITION | MASK_NORMAL | MASK_COLOR | MASK_TEXCOORD1) elements");
    return false;
}

bool WheelTrackModel::ValidateBufferElements() const
{
    Geometry *pGeometry = GetModel()->GetGeometry(0,0);
    VertexBuffer *pVbuffer = pGeometry->GetVertexBuffer(0);
    const unsigned uElementMask = pVbuffer->GetElementMask();
    const unsigned uRequiredMask = MASK_POSITION | MASK_NORMAL | MASK_COLOR | MASK_TEXCOORD1;
    unsigned zeroMask = ( uElementMask & ~uRequiredMask );
    return ( uElementMask & uRequiredMask ) == uRequiredMask && zeroMask == 0;
}

void WheelTrackModel::AddStrip(const Vector3 &cpos, const Vector3 &normal)
{
    Vector3 pos = cpos + normal * STRIP_NORMAL_OFFSET; // lift the position away from the ground by NORMAL_OFFSET

    // the 1st entry of the strip
    if ( !m_firstStripPoint.valid )
    {
        m_firstStripPoint.pos    = pos;
        m_firstStripPoint.normal = normal;
        m_firstStripPoint.valid  = true;
        m_firstStripPoint.lastPosCnt = MIN_LASTPOS_CNT;
    }
    else
    {
        // calculate direction and right vectors to the previous position
        Vector3 dir = ( m_firstStripPoint.pos - pos );
        m_firstStripPoint.lastPosCnt = MIN_LASTPOS_CNT;

        // avoid creating tiny strips
        if ( dir.Length() < MIN_STRIP_LEN )
        {
            return;
        }

        // and large strips
        if ( dir.Length() > MAX_STRIP_LEN )
        {
            m_firstStripPoint.valid = false;
            m_firstStripPoint.vertsArrayValid  = false;
            return;
        }

        dir.Normalize();
        Vector3 right = normal.CrossProduct(dir).Normalized();

        GeomData geomData[ VERTS_PER_STRIP ];

        geomData[0].pos = pos - right * m_fHalfWidth;
        geomData[1].pos = pos + right * m_fHalfWidth;
        geomData[2].pos = m_firstStripPoint.pos - right * m_fHalfWidth;
        geomData[3].pos = m_firstStripPoint.pos + right * m_fHalfWidth;

        // copy the last vert positions if present (don't exist on the very first strip)
        if ( m_firstStripPoint.vertsArrayValid )
        {
            geomData[2].pos = m_firstStripPoint.v[0];
            geomData[3].pos = m_firstStripPoint.v[1];
        }

        geomData[0].normal = normal;
        geomData[1].normal = normal;
        geomData[2].normal = m_firstStripPoint.normal;
        geomData[3].normal = m_firstStripPoint.normal;

        geomData[0].color = m_unsignedColor;
        geomData[1].color = m_unsignedColor;
        geomData[2].color = m_unsignedColor;
        geomData[3].color = m_unsignedColor;

        geomData[0].uv = Vector2(0,0);
        geomData[1].uv = Vector2(1,0);
        geomData[2].uv = Vector2(0,1);
        geomData[3].uv = Vector2(1,1);

        // 4 verts, 2 tris, vertex draw order - clockwise dir
        unsigned short triIdx[6] = { 0, 2, 1, 1, 2, 3 };

        // update the first strip (previous) data
        m_firstStripPoint.pos    = pos;
        m_firstStripPoint.normal = normal;
        m_firstStripPoint.vertsArrayValid = true;
        m_firstStripPoint.v[0] = geomData[0].pos;
        m_firstStripPoint.v[1] = geomData[1].pos;

        // shift vbuff elements to the right by 4
        if ( m_vSkidGeom.Size() < VERTS_PER_STRIP * MAX_SKID_STRIPS )
        {
            m_vSkidGeom.Resize(m_vSkidGeom.Size() + VERTS_PER_STRIP);
        }
        for ( int i = (int)m_vSkidGeom.Size() - 1; i >= 0; --i )
        {
            if ( i - VERTS_PER_STRIP >= 0 )
            {
                // shift
                memcpy( &m_vSkidGeom[i], &m_vSkidGeom[i - VERTS_PER_STRIP], sizeof(GeomData) );

                //GeomData &geData = m_vSkidGeom[i];

                // fade alpha by a small bit every shift
                // Color::ToUInt() = (a << 24) | (b << 16) | (g << 8) | r;
                //unsigned a = (m_vSkidGeom[i].color >> 24);
                //a = ((unsigned)Clamp((int)( (float)a * SMALL_BIT ), 0, 255)) << 24;
                //a = 0xff << 24;
                //m_vSkidGeom[i].color = (m_vSkidGeom[i].color & 0x00ffffff) | a;
            }
        }

        // copy new geom
        memcpy( &m_vSkidGeom[0], geomData, sizeof(geomData) );

        // shift indexbuff to the right by 6
        if ( m_vSkidIndex.Size() < INDECES_PER_STRIP * MAX_SKID_STRIPS )
        {
            m_vSkidIndex.Resize(m_vSkidIndex.Size() + INDECES_PER_STRIP);
        }
        for ( int i = (int)m_vSkidIndex.Size() - 1; i >= 0; --i )
        {
            if ( i - INDECES_PER_STRIP >= 0 )
            {
                // need to add +4 offset(for newly added verts) to indeces being shifted
                m_vSkidIndex[i] = m_vSkidIndex[i - INDECES_PER_STRIP] + VERTS_PER_STRIP;
            }
        }

        // copy new indeces
        memcpy( &m_vSkidIndex[0], triIdx, sizeof(triIdx) );

        //=================================
        // copy to vertex/index buffers
        CopyToBuffer();
    }
}

void WheelTrackModel::CopyToBuffer()
{
    Geometry *pGeometry = GetModel()->GetGeometry(0,0);
    VertexBuffer *pVbuffer = pGeometry->GetVertexBuffer(0);
    IndexBuffer *pIbuffer = pGeometry->GetIndexBuffer();
    const unsigned uElementMask = pVbuffer->GetElementMask();
    bool brangeChanged = false;

    if ( pVbuffer->GetVertexCount() != m_vSkidGeom.Size() )
    {
        pVbuffer->SetSize(m_vSkidGeom.Size(), uElementMask);
        brangeChanged = true;
    }
    if ( pIbuffer->GetIndexCount() != m_vSkidIndex.Size() )
    {
        pIbuffer->SetSize(m_vSkidIndex.Size(), false);
        brangeChanged = true;
    }

    void *pVertexData = (void*)pVbuffer->Lock(0, pVbuffer->GetVertexCount());
    void *pIndexData = (void*)pIbuffer->Lock(0, pIbuffer->GetIndexCount());

    if ( pVertexData && pIndexData )
    {
        memcpy( pVertexData, &m_vSkidGeom[0], sizeof(GeomData) * m_vSkidGeom.Size() );
        memcpy( pIndexData, &m_vSkidIndex[0], sizeof(unsigned short) * m_vSkidIndex.Size() );

        pVbuffer->Unlock();
        pIbuffer->Unlock();

        // update draw range
        if ( brangeChanged )
        {
            pGeometry->SetDrawRange(TRIANGLE_LIST, 0, m_vSkidIndex.Size());
        }
    }
}

void WheelTrackModel::ClearStrip()
{
    if ( --m_firstStripPoint.lastPosCnt <= 0 )
    {
        m_firstStripPoint.valid = false;
        m_firstStripPoint.vertsArrayValid  = false;
    }
}

bool WheelTrackModel::InSkidState() const
{
    return m_firstStripPoint.valid;
}

void WheelTrackModel::DebugRender(DebugRenderer *dbgRender, const Color &color)
{
    for ( unsigned i = 0; i < m_vSkidGeom.Size(); i += VERTS_PER_STRIP )
    {
        dbgRender->AddLine( m_vSkidGeom[i+0].pos, m_vSkidGeom[i+1].pos, color );
        dbgRender->AddLine( m_vSkidGeom[i+0].pos, m_vSkidGeom[i+2].pos, color );
        dbgRender->AddLine( m_vSkidGeom[i+1].pos, m_vSkidGeom[i+2].pos, color );
        dbgRender->AddLine( m_vSkidGeom[i+1].pos, m_vSkidGeom[i+3].pos, color );
        dbgRender->AddLine( m_vSkidGeom[i+2].pos, m_vSkidGeom[i+3].pos, color );
    }
}





