#pragma once

#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"
#include "Labs/Common/OrbitCameraManager.h"
#include "Labs/1-RigidBody/Simulator.h"

namespace VCX::Labs::RigidBody {

    class CaseTwoRigids : public Common::ICase {
    public:
        CaseTwoRigids();

        virtual std::string_view const GetName() override { return "Two Rigid Bodies"; }

        virtual void                     OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void                     OnProcessInput(ImVec2 const & pos) override;

        void OnProcessMouseControl(glm::vec3 mouseDelta);

    private:
        Engine::GL::UniqueProgram           _program;
        Engine::GL::UniqueRenderFrame       _frame;
        Engine::Camera                      _camera { .Eye = glm::vec3(0.f, 0.f, 25.f) };
        Common::OrbitCameraManager          _cameraManager;
        Engine::GL::UniqueIndexedRenderItem _boxItem1;  // render the box
        Engine::GL::UniqueIndexedRenderItem _boxItem2;  // render the box
        Engine::GL::UniqueIndexedRenderItem _lineItem1; // render line on box
        Engine::GL::UniqueIndexedRenderItem _lineItem2; // render line on box
        Rigid                               _rigid1;
        Rigid                               _rigid2;
        glm::vec3                           _center { 0.f, 0.f, 0.f };
        glm::vec3                           _dim1 { 2.f, 2.f, 2.f };
        glm::vec3                           _dim2 { 2.f, 2.f, 2.f };
        glm::vec3                           _boxColor1 { 121.0f / 255, 207.0f / 255, 171.0f / 255 };
        glm::vec3                           _boxColor2 { 206.0f / 255, 41.0f / 255, 151.0f / 255 };
        bool                                _pause = false;
        int                                 _controlling = 1;
        float                               _restitution = 1.f;
        int                                 _case = 0;
        void reset_simulation();
    };
} // namespace VCX::Labs::RigidBody 