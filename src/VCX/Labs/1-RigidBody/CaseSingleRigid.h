#pragma once

#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"
#include "Labs/Common/OrbitCameraManager.h"
#include "Labs/1-RigidBody/Simulator.h"

namespace VCX::Labs::RigidBody {

    class CaseSingleRigid : public Common::ICase {
    public:
        CaseSingleRigid();

        virtual std::string_view const GetName() override { return "Single Rigid Body"; }

        virtual void                     OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void                     OnProcessInput(ImVec2 const & pos) override;

        void OnProcessMouseControl(glm::vec3 mouseDelta);

    private:
        Engine::GL::UniqueProgram           _program;
        Engine::GL::UniqueRenderFrame       _frame;
        Engine::Camera                      _camera { .Eye = glm::vec3(0.f, 0.f, 25.f) };
        Common::OrbitCameraManager          _cameraManager;
        Engine::GL::UniqueIndexedRenderItem _boxItem;  // render the box
        Engine::GL::UniqueIndexedRenderItem _lineItem; // render line on box
        Rigid                               _rigid { 1.f, glm::vec3(0.f), glm::vec3(0.f), glm::quat(), glm::vec3(0.f) };
        glm::vec3                           _center { 0.f, 0.f, 0.f };
        glm::vec3                           _dim { 2.f, 2.f, 2.f };
        glm::vec3                           _boxColor { 121.0f / 255, 207.0f / 255, 171.0f / 255 };
        bool                                _pause = false;
    };
} // namespace VCX::Labs::RigidBody 