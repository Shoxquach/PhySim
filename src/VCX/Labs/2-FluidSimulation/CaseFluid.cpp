#include <spdlog/spdlog.h>
#include "Engine/app.h"
#include "Labs/2-FluidSimulation/CaseFluid.h"
#include "Labs/Common/ImGuiHelper.h"
#include <iostream>
#include <imgui.h>
#include <imgui_internal.h>

namespace VCX::Labs::FluidSimulation {
    const std::vector<glm::vec3> vertex_pos = {
            glm::vec3(-0.5f, -0.5f, -0.5f),
            glm::vec3(0.5f, -0.5f, -0.5f),  
            glm::vec3(0.5f, 0.5f, -0.5f),
            glm::vec3(-0.5f, 0.5f, -0.5f), 
            glm::vec3(-0.5f, -0.5f, 0.5f),  
            glm::vec3(0.5f, -0.5f, 0.5f),   
            glm::vec3(0.5f, 0.5f, 0.5f),   
            glm::vec3(-0.5f, 0.5f, 0.5f)
    };
    const std::vector<std::uint32_t> line_index = { 0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6, 6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7 }; // line index

    CaseFluid::CaseFluid(std::initializer_list<Assets::ExampleScene> && scenes) :
        _scenes(scenes),
        _program(
            Engine::GL::UniqueProgram({
                // Engine::GL::SharedShader("assets/shaders/sphere_phong.vert"),
                // Engine::GL::SharedShader("assets/shaders/phong.frag") })),
                Engine::GL::SharedShader("assets/shaders/fluid.vert"),
                Engine::GL::SharedShader("assets/shaders/fluid.frag") })),
        _lineprogram(
            Engine::GL::UniqueProgram({
                Engine::GL::SharedShader("assets/shaders/flat.vert"),
                Engine::GL::SharedShader("assets/shaders/flat.frag") })),
        _sceneObject(1),
        _BoundaryItem(Engine::GL::VertexLayout()
            .Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream , 0), Engine::GL::PrimitiveType::Lines){ 
        _cameraManager.AutoRotate = false;
        _program.BindUniformBlock("PassConstants", 1);
        _program.GetUniforms().SetByName("u_DiffuseMap" , 0);
        _program.GetUniforms().SetByName("u_SpecularMap", 1);
        _program.GetUniforms().SetByName("u_HeightMap"  , 2);
        _lineprogram.GetUniforms().SetByName("u_Color",  glm::vec3(1.0f));
        _BoundaryItem.UpdateElementBuffer(line_index);
        ResetSystem();
        _r = .007f;
        _sphere = Engine::Model{Engine::Sphere(6, _r), 0};
        _simulation.sphere_radius = .15f;
        _sphere_obstacle = Engine::Model{Engine::Sphere(25, _simulation.sphere_radius), 0};
    }

    void CaseFluid::OnSetupPropsUI() {
        if(ImGui::Button("Reset System")) 
            ResetSystem();
        ImGui::SameLine();
        if(ImGui::Button(_stopped ? "Start Simulation":"Stop Simulation"))
            _stopped = ! _stopped;
        ImGui::SliderFloat("Timestep", &_timestep, 0.001f, 0.02f, "%.6f");
        ImGui::SliderFloat("Flip Ratio", &_simulation.m_fRatio, 0.0f, 1.0f, "%.2f");
        ImGui::SliderInt("Color Mode", &_color_mode, 0, 4);
        switch (_color_mode) {
            case 0: ImGui::Text("(Velocity)"); break;
            case 1: ImGui::Text("(Pressure)"); break;
            case 2: ImGui::Text("(Density)"); break;
            case 3: ImGui::Text("(Position)"); break;
            case 4: ImGui::Text("(Direction)"); break;
        }
        if (ImGui::Button(_useCGSolver ? "Use Gauss-Seidel Solver" : "Use CG Solver")) {
            _useCGSolver = !_useCGSolver;
            _uniformDirty = true;
        }
        // ImGui::Selectable("Use Conjugate Gradient Solver", &_useCGSolver);
        
        // Show current sphere position
        ImGui::Text("Sphere Position: (%.2f, %.2f, %.2f)", 
                    _simulation.sphere_center.x, 
                    _simulation.sphere_center.y, 
                    _simulation.sphere_center.z);
        ImGui::Text("Sphere Velocity: (%.2f, %.2f, %.2f)", 
                    _simulation.sphere_velocity.x, 
                    _simulation.sphere_velocity.y, 
                    _simulation.sphere_velocity.z);
    }


    Common::CaseRenderResult CaseFluid::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        if (_recompute) {
            _recompute = false;
            _sceneObject.ReplaceScene(GetScene(_sceneIdx));
            _cameraManager.Save(_sceneObject.Camera);
        }

        if (! _stopped) _simulation.SimulateTimestep(_timestep, _color_mode, _useCGSolver);
        
        _BoundaryItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(vertex_pos));
        _frame.Resize(desiredSize);

        _cameraManager.Update(_sceneObject.Camera);
        _sceneObject.PassConstantsBlock.Update(
            &VCX::Labs::Rendering::SceneObject::PassConstants::Projection,
            _sceneObject.Camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second))
        );
        _sceneObject.PassConstantsBlock.Update(
            &VCX::Labs::Rendering::SceneObject::PassConstants::View,
            _sceneObject.Camera.GetViewMatrix()
        );
        _sceneObject.PassConstantsBlock.Update(
            &VCX::Labs::Rendering::SceneObject::PassConstants::ViewPosition,
            _sceneObject.Camera.Eye
        );
        _lineprogram.GetUniforms().SetByName(
            "u_Projection", _sceneObject.Camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
        _lineprogram.GetUniforms().SetByName(
            "u_View"      , _sceneObject.Camera.GetViewMatrix());
        
        if (_uniformDirty) {
            _uniformDirty = false;
            _program.GetUniforms().SetByName("u_AmbientScale"      , _ambientScale);
            _program.GetUniforms().SetByName("u_UseBlinn"          , _useBlinn);
            _program.GetUniforms().SetByName("u_Shininess"         , _shininess);
            _program.GetUniforms().SetByName("u_UseGammaCorrection", int(_useGammaCorrection));
            _program.GetUniforms().SetByName("u_AttenuationOrder"  , _attenuationOrder);            
            _program.GetUniforms().SetByName("u_BumpMappingBlend"  , _bumpMappingPercent * .01f);            
        }
        
        gl_using(_frame);

        glEnable(GL_DEPTH_TEST);
        glLineWidth(_BndWidth);
        _BoundaryItem.Draw({ _lineprogram.Use() });
        glLineWidth(1.f);

        Rendering::ModelObject m(_sphere, _simulation.m_particlePos, _simulation.m_particleColor);
        auto const & material = _sceneObject.Materials[0];
        m.Mesh.Draw({ material.Albedo.Use(), material.MetaSpec.Use(), material.Height.Use(), _program.Use() },
            _sphere.Mesh.Indices.size(), 0, _simulation.m_iNumSpheres
        );

        std::vector<glm::vec3> obstaclePos{_simulation.sphere_center};
        std::vector<glm::vec3> obstacleColor{glm::vec3(.2f, .2f, .2f)};
        Rendering::ModelObject mm(_sphere_obstacle, obstaclePos, obstacleColor);
        mm.Mesh.Draw({ material.Albedo.Use(), material.MetaSpec.Use(), material.Height.Use(), _program.Use() },
            _sphere_obstacle.Mesh.Indices.size()
        );
        
        glDepthFunc(GL_LEQUAL);
        glDepthFunc(GL_LESS);
        glDisable(GL_DEPTH_TEST);

        return Common::CaseRenderResult {
            .Fixed     = false,
            .Flipped   = true,
            .Image     = _frame.GetColorAttachment(),
            .ImageSize = desiredSize,
        };
    }

    void CaseFluid::OnProcessInput(ImVec2 const& pos) {
        // Handle sphere dragging first (before camera controls)
        HandleSphereDragging(pos);
        
        // Only process camera input if not dragging sphere
        if (!_isDraggingSphere) {
            _cameraManager.ProcessInput(_sceneObject.Camera, pos);
        }
    }

    void CaseFluid::ResetSystem(){
        _simulation.setupScene(_res);
    }

    bool CaseFluid::RayIntersectsSphere(const glm::vec3& rayOrigin, const glm::vec3& rayDir, const glm::vec3& sphereCenter, float sphereRadius, float& t) {
        glm::vec3 oc = rayOrigin - sphereCenter;
        float a = glm::dot(rayDir, rayDir);
        float b = 2.0f * glm::dot(oc, rayDir);
        float c = glm::dot(oc, oc) - sphereRadius * sphereRadius;
        float discriminant = b * b - 4.0f * a * c;
        
        if (discriminant < 0) return false;
        
        float sqrtDisc = glm::sqrt(discriminant);
        float t1 = (-b - sqrtDisc) / (2.0f * a);
        float t2 = (-b + sqrtDisc) / (2.0f * a);
        
        if (t1 > 0) {
            t = t1;
            return true;
        }
        if (t2 > 0) {
            t = t2;
            return true;
        }
        return false;
    }

    void CaseFluid::GetMouseRay(const ImVec2& mousePos, glm::vec3& rayOrigin, glm::vec3& rayDir) {
        auto window = ImGui::GetCurrentWindow();
        ImVec2 windowSize = window->Rect().GetSize();
        ImVec2 windowPos = window->Rect().Min;
        
        // Convert mouse position to normalized device coordinates (-1 to 1)
        float x = (2.0f * mousePos.x) / windowSize.x - 1.0f;
        float y = 1.0f - (2.0f * mousePos.y) / windowSize.y;
        
        // Get camera matrices
        float aspect = windowSize.x / windowSize.y;
        glm::mat4 proj = _sceneObject.Camera.GetProjectionMatrix(aspect);
        glm::mat4 view = _sceneObject.Camera.GetViewMatrix();
        glm::mat4 invProjView = glm::inverse(proj * view);
        
        // Near and far points in clip space
        glm::vec4 nearPoint = invProjView * glm::vec4(x, y, -1.0f, 1.0f);
        glm::vec4 farPoint = invProjView * glm::vec4(x, y, 1.0f, 1.0f);
        
        // Convert from homogeneous to world space
        nearPoint /= nearPoint.w;
        farPoint /= farPoint.w;
        
        rayOrigin = glm::vec3(nearPoint);
        rayDir = glm::normalize(glm::vec3(farPoint - nearPoint));
    }

    void CaseFluid::HandleSphereDragging(const ImVec2& mousePos) {
        auto window = ImGui::GetCurrentWindow();
        ImVec2 windowSize = window->Rect().GetSize();
        ImGuiIO const & io = ImGui::GetIO();
        
        // Check for hover
        bool hover = false;
        bool anyHeld = false;
        ImGui::ButtonBehavior(window->Rect(), window->GetID("##io"), &hover, &anyHeld);
        
        if (!hover) {
            if (_isDraggingSphere) {
                _isDraggingSphere = false;
                _simulation.isSphereHeld = false;
            }
        }
        
        glm::vec3 rayOrigin, rayDir;
        GetMouseRay(mousePos, rayOrigin, rayDir);
        
        // Check if mouse is clicked
        bool leftClicked = ImGui::IsMouseClicked(ImGuiMouseButton_Left);
        bool leftReleased = ImGui::IsMouseReleased(ImGuiMouseButton_Left);
        bool leftHeld = ImGui::IsMouseDown(ImGuiMouseButton_Left);

        if (leftReleased) {
            _isDraggingSphere = false;
            _simulation.isSphereHeld = false;
            // Reset velocity when releasing
            _simulation.sphere_velocity = glm::vec3(0.0f);
        }
        
        if (leftClicked && !ImGui::IsKeyDown(ImGuiKey_LeftCtrl) && !ImGui::IsKeyDown(ImGuiKey_LeftShift)) {
            // Check if clicking on sphere
            float t;
            if (RayIntersectsSphere(rayOrigin, rayDir, _simulation.sphere_center, _simulation.sphere_radius, t)) {
                _isDraggingSphere = true;
                _simulation.isSphereHeld = true;
                // Calculate intersection point
                glm::vec3 hitPoint = rayOrigin + t * rayDir;
                _dragOffset = hitPoint - _simulation.sphere_center;
                // Store the plane for dragging (perpendicular to view direction at sphere center)
                _dragPlaneNormal = -rayDir;
                _dragPlaneDistance = glm::dot(_simulation.sphere_center + _dragOffset, _dragPlaneNormal);
                // Initialize last position for velocity calculation
                _dragLastPos = _simulation.sphere_center;
                _dragLastTime = io.DeltaTime;
            }
        }
        
        if (_isDraggingSphere && leftHeld && hover) {
            // Ray-plane intersection to find new sphere position
            float denom = glm::dot(rayDir, _dragPlaneNormal);
            if (glm::abs(denom) > 1e-6f) {
                float t = (_dragPlaneDistance - glm::dot(rayOrigin, _dragPlaneNormal)) / denom;
                glm::vec3 newPos = rayOrigin + t * rayDir - _dragOffset;
                
                // Clamp to boundary
                float boundary = _simulation.boundary_size - _simulation.sphere_radius;
                glm::vec3 clampedPos = glm::clamp(newPos, glm::vec3(-boundary), glm::vec3(boundary));
                
                // Calculate velocity based on ACTUAL displacement (not desired displacement)
                // This ensures zero velocity when ball hits boundary and stops
                float dt = _timestep;
                
                // If ball is clamped to boundary, actual displacement is reduced
                // Use clampedPos for velocity calculation to reflect real motion
                glm::vec3 actualDisplacement = clampedPos - _simulation.sphere_center;
                _simulation.sphere_velocity = actualDisplacement / dt;
                
                _simulation.sphere_center = clampedPos;
            }
        } else {
            _simulation.sphere_velocity = glm::vec3(0.0f);
        }
        
        // // If mouse moves outside window while dragging, stop tracking velocity
        // // to prevent "infinite speed" when ball is stuck at boundary
        // if (_isDraggingSphere && leftHeld && !hover) {
        //     // Mouse moved outside window, ball is likely clamped at boundary
        //     // Zero out velocity to prevent phantom forces on fluid
        // }
    }
}