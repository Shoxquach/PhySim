#pragma once

#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/UniformBlock.hpp"
#include "Engine/Sphere.h"
#include "Labs/2-FluidSimulation/FluidSimulator.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"
#include "Labs/Common/OrbitCameraManager.h"
#include "Labs/Scene/Content.h"
#include "Labs/Scene/SceneObject.h"


namespace VCX::Labs::FluidSimulation {

    class CaseFluid : public Common::ICase {
    public:
        CaseFluid(std::initializer_list<Assets::ExampleScene> && scenes);

        virtual std::string_view const GetName() override { return "Fluid Simulation"; }

        virtual void                     OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void                     OnProcessInput(ImVec2 const & pos) override;

    private:
        std::vector<Assets::ExampleScene> const _scenes;

        Engine::GL::UniqueProgram         _program;
        Engine::GL::UniqueProgram         _lineprogram;
        Engine::GL::UniqueRenderFrame     _frame;
        VCX::Labs::Rendering::SceneObject _sceneObject;
        std::size_t                       _sceneIdx { 0 };
        bool                              _recompute { true };
        bool                              _uniformDirty { true };
        int                               _msaa { 2 };
        int                               _useBlinn { 0 };
        float                             _shininess { 32 };
        float                             _ambientScale { 1 };
        bool                              _useGammaCorrection { true };
        int                               _attenuationOrder { 2 };
        int                               _bumpMappingPercent { 20 };


        Engine::GL::UniqueIndexedRenderItem _BoundaryItem;
        Common::OrbitCameraManager          _cameraManager;
        float                               _BndWidth { 2.0 };
        bool                                _stopped { false };
        Engine::Model                       _sphere;
        int                                 _res = 32;
        float                               _r;
        Fluid::Simulator                    _simulation;
        float                               _timestep = .01f;
        glm::vec3                           _color {.2f, .4f, .5f};
        int                                 _color_mode = 0;
        Engine::Model                       _sphere_obstacle;
        bool                                _useCGSolver = false;

        // Mouse interaction for dragging sphere obstacle
        bool                                _isDraggingSphere { false };
        glm::vec3                           _dragPlaneNormal { 0, 1, 0 };
        float                               _dragPlaneDistance { 0 };
        glm::vec3                           _dragOffset { 0 };
        glm::vec3                           _dragLastPos { 0 };
        float                               _dragLastTime { 0 };

        char const *          GetSceneName(std::size_t const i) const { return VCX::Labs::Rendering::Content::SceneNames[std::size_t(_scenes[i])].c_str(); }
        Engine::Scene const & GetScene(std::size_t const i) const { return VCX::Labs::Rendering::Content::Scenes[std::size_t(_scenes[i])]; }
        void                  ResetSystem();

        // Ray-sphere intersection test
        bool RayIntersectsSphere(const glm::vec3& rayOrigin, const glm::vec3& rayDir, const glm::vec3& sphereCenter, float sphereRadius, float& t);
        // Screen to world ray
        void GetMouseRay(const ImVec2& mousePos, glm::vec3& rayOrigin, glm::vec3& rayDir);
        // Check if mouse is over sphere and handle dragging
        void HandleSphereDragging(const ImVec2& mousePos);
    };
} // namespace VCX::Labs::GettingStarted
