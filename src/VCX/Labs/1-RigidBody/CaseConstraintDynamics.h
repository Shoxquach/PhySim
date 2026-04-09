#pragma once

#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"
#include "Labs/Common/OrbitCameraManager.h"
#include "Labs/1-RigidBody/Simulator.h"

namespace VCX::Labs::RigidBody {

    class CaseConstraintDynamics : public Common::ICase {
    public:
        CaseConstraintDynamics();

        virtual std::string_view const GetName() override { return "Constraint Dynamics"; }

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
        int                                 _numRigids = 4;
        Engine::GL::UniqueIndexedRenderItem _boxItem;
        Engine::GL::UniqueIndexedRenderItem _lineItem;
        Engine::GL::UniqueIndexedRenderItem _selectedboxItem;
        Engine::GL::UniqueIndexedRenderItem _selectedlineItem;
        Engine::GL::UniqueIndexedRenderItem _groundboxItem;
        Engine::GL::UniqueIndexedRenderItem _groundlineItem;
        std::vector<Rigid>                  _rigids;
        Rigid                               _ground, _walls[4];
        std::vector<glm::vec3>              _dims;
        glm::vec3                           _boxColor { 121.0f / 255, 207.0f / 255, 171.0f / 255 };
        glm::vec3                           _groundColor { .2f, .3f, .1f };
        glm::vec3                           _selectedColor { .5f, .5f, .5f };
        bool                                _pause = false;
        float                               _random_offset = .1f;
        float                               _restitution = .3f;
        float                               _mu = .2f;
        float                               _damping = 0.999f;
        float                               _correction_factor = .1f;
        float                               _timestep = 0.001f;
        glm::vec3                           _gravity { 0.f, -9.8f, 0.f };
        void                                reset_simulation();
        const float                         _ground_size = 6.f;
        const float                         _groundY = -6.f;
        const float                         _wall_height = 6.f;
        const float                         _offset = 0.05f;
        int                                 _selected = 0;
    };
} // namespace VCX::Labs::RigidBody 