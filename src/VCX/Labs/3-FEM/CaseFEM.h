#pragma once

#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"
#include "Labs/Common/OrbitCameraManager.h"
#include "Labs/3-FEM/FEMSystem.h"
#include "Labs/3-FEM/Objects.h"

namespace VCX::Labs::FEM {
    class CaseFEM : public Common::ICase {
    public:
        CaseFEM();

        virtual std::string_view const GetName() override { return "FEM"; }

        virtual void                     OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void                     OnProcessInput(ImVec2 const & pos) override;
        
        void OnProcessMouseControl(glm::vec3 mouseDelta);
    
    private:
        Engine::GL::UniqueProgram           _program;
        Engine::GL::UniqueRenderFrame       _frame;
        Engine::Camera                      _camera { .Eye = glm::vec3(0.f, 0.f, 25.f) };
        Common::OrbitCameraManager          _cameraManager;
        glm::vec3                           _center { 0.f, 0.f, 0.f };
        Engine::GL::UniqueIndexedRenderItem _item;
        Engine::GL::UniqueIndexedRenderItem _lineItem;
        Engine::GL::UniqueIndexedRenderItem _groundItem;
        Engine::GL::UniqueIndexedRenderItem _groundLineItem;
        Engine::GL::UniqueRenderItem        _vertexItem;
        FEMSystem                           _femSystem;
        
        void resetScene(Object object);

        float                               _timeStep = 0.001f;
        glm::vec3                           _color { .8f, .6f, .4f };
        bool                                _pause = false;
        int                                 _activeVertex = -1;
        bool                                _isDraggingVertex = false;
        float                               _pickRadius = .25f;
        float                               _vertexPointSize = 10.f;
        glm::vec3                           _dragPlaneNormal { 0.f, 0.f, 1.f };
        float                               _dragPlaneDistance = 0.f;
        glm::vec3                           _dragOffset { 0.f };
        float                               _E = 20000.f;
        float                               _upsilon = .2f;
        Method                              _method = StVK;
        Object                              _object = block;
        Object                              _previousObject = block;
        float                               _dropHeight = 0.f;
        float                               _stretchRate = 1.2f;
        bool                                _usePlastic = false;

        bool RayIntersectsSphere(glm::vec3 const & rayOrigin, glm::vec3 const & rayDir, glm::vec3 const & center, float radius, float & t) const;
        void GetMouseRay(ImVec2 const & mousePos, glm::vec3 & rayOrigin, glm::vec3 & rayDir) const;
        int  PickVertex(glm::vec3 const & rayOrigin, glm::vec3 const & rayDir, float & t) const;
        void HandleVertexDragging(ImVec2 const & mousePos);
    };
}