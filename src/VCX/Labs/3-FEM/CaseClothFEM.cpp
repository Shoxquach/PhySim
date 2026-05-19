#include <spdlog/spdlog.h>
#include "Engine/app.h"
#include "Labs/3-FEM/CaseClothFEM.h"
#include "Labs/Common/ImGuiHelper.h"
#include <iostream>
#include <imgui.h>
#include <imgui_internal.h>
#include <limits>

namespace VCX::Labs::FEM {
    
    std::vector<std::uint32_t> generateLineIndices(const std::vector<int>& triIndices) {
        std::vector<std::uint32_t> lineIndices;
        lineIndices.reserve(triIndices.size() * 2);
        
        for (size_t i = 0; i < triIndices.size(); i += 3) {
            std::uint32_t i0 = static_cast<std::uint32_t>(triIndices[i]);
            std::uint32_t i1 = static_cast<std::uint32_t>(triIndices[i + 1]);
            std::uint32_t i2 = static_cast<std::uint32_t>(triIndices[i + 2]);
            
            lineIndices.push_back(i0);
            lineIndices.push_back(i1);
            
            lineIndices.push_back(i1);
            lineIndices.push_back(i2);
            
            lineIndices.push_back(i2);
            lineIndices.push_back(i0);
        }
        
        return lineIndices;
    }

    CaseClothFEM::CaseClothFEM():
        _program(
            Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/flat.vert"),
                                        Engine::GL::SharedShader("assets/shaders/flat.frag") })),
        _item(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Triangles),
        _lineItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines),
        _vertexItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Points),
        _resolution(25),
        _femSystem2d(25) {
        _cameraManager.AutoRotate = false;
        _cameraManager.EnableDamping = false;
        _cameraManager.EnablePan = false;
        _cameraManager.Save(_camera);
        
        std::vector<std::uint32_t> triIndices;
        triIndices.reserve(_femSystem2d.Indices.size());
        for (int idx : _femSystem2d.Indices) {
            triIndices.push_back(static_cast<std::uint32_t>(idx));
        }
        
        _item.UpdateElementBuffer(triIndices);
        
        auto lineIndices = generateLineIndices(_femSystem2d.Indices);
        _lineItem.UpdateElementBuffer(lineIndices);
    }

    void CaseClothFEM::OnSetupPropsUI() {
        ImGui::Button("Reset Simulation");
        if (ImGui::IsItemClicked()) {
            _femSystem2d.reset();
            _activeVertex = -1;
            _isDraggingVertex = false;
        }
        ImGui::SameLine();
        ImGui::Button(_pause ? "Resume Simulation" : "Pause Simulation");
        if (ImGui::IsItemClicked()) {
            _pause = !_pause;
        }
        ImGui::Checkbox("Draw Triangles", &_drawTriagles);
        ImGui::SliderFloat("Time Step", &_timeStep, 0.0001f, 0.005f, "%.6f");
        ImGui::SliderFloat("Damping", &_femSystem2d.damping, 0.99f, 1.f, "%.6f");
        ImGui::SliderFloat("E", &_E, 25.f, 250.f);
        ImGui::SliderFloat("Upsilon", &_upsilon, 0.f, .5f);
        ImGui::SliderFloat("Pick Radius", &_pickRadius, 0.05f, 1.f, "%.2f");
        ImGui::InputInt("Resolution", &_resolution);
        if (ImGui::IsItemEdited()) {
            if (_resolution < 5) _resolution = 5;
            if (_resolution > 50) _resolution = 50;
            _femSystem2d.resetRectangularCloth(10.f, 10.f, _resolution, _resolution);
            std::vector<std::uint32_t> triIndices;
            triIndices.reserve(_femSystem2d.Indices.size());
            for (int idx : _femSystem2d.Indices) {
                triIndices.push_back(static_cast<std::uint32_t>(idx));
            }
            
            _item.UpdateElementBuffer(triIndices);
            
            auto lineIndices = generateLineIndices(_femSystem2d.Indices);
            _lineItem.UpdateElementBuffer(lineIndices);
        }
        if (_activeVertex >= 0) {
            ImGui::Text("Selected vertex: %d", _activeVertex);
        } else {
            ImGui::Text("Click a FEM vertex to drag it.");
        }
    }

    Common::CaseRenderResult CaseClothFEM::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        if (ImGui::IsKeyPressed(ImGuiKey_Space)) {
            _pause = !_pause;
        }
        OnProcessMouseControl(_cameraManager.getMouseMove());

        if (!_pause) {
            _femSystem2d.mu = _E / (2 * (1 + _upsilon));
            _femSystem2d.lambda = _E * _upsilon / (1 + _upsilon) / (1 - 2 * _upsilon);
            _femSystem2d.simulateTimestep(_timeStep);
        }

        _frame.Resize(desiredSize);
        _cameraManager.Update(_camera);
        _program.GetUniforms().SetByName("u_Projection", _camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
        _program.GetUniforms().SetByName("u_View", _camera.GetViewMatrix());

        gl_using(_frame);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_LINE_SMOOTH);
        glLineWidth(.5f);

        // for (int i = 0; i < _femSystem.n_Vertices; i ++) {
        //     printf("%f, %f, %f\n", _femSystem.Positions[i].x, _femSystem.Positions[i].y, _femSystem.Positions[i].z);
        // }
        // printf("=========\n");

        auto span_bytes = Engine::make_span_bytes<glm::vec3>(_femSystem2d.Positions);

        _program.GetUniforms().SetByName("u_Color", _color);
        _item.UpdateVertexBuffer("position", span_bytes);
        _item.Draw({ _program.Use() });

        if (_drawTriagles) {
            _program.GetUniforms().SetByName("u_Color", glm::vec3(1.f, 1.f, 1.f));
            _lineItem.UpdateVertexBuffer("position", span_bytes);
            _lineItem.Draw({ _program.Use() });
        }

        if (_activeVertex >= 0 && _activeVertex < _femSystem2d.n_Vertices && _isDraggingVertex) {
            glDisable(GL_DEPTH_TEST);
            glPointSize(_vertexPointSize);
            _program.GetUniforms().SetByName("u_Color", glm::vec3(1.f, .9f, .1f));
            
            std::vector<glm::vec3> selectedVertex { _femSystem2d.Positions[_activeVertex] };
            std::vector<std::uint32_t> selectedIndex { 0 };
            
            _vertexItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(selectedVertex));
            _vertexItem.Draw({ _program.Use() });
            glEnable(GL_DEPTH_TEST);
        }

        glLineWidth(1.f);
        glPointSize(1.f);
        glDisable(GL_LINE_SMOOTH);

        return Common::CaseRenderResult {
            .Fixed     = false,
            .Flipped   = true,
            .Image     = _frame.GetColorAttachment(),
            .ImageSize = desiredSize,
        };
    }
    
    void CaseClothFEM::OnProcessInput(ImVec2 const & pos) {
        HandleVertexDragging(pos);
        if (!_isDraggingVertex) {
            _cameraManager.ProcessInput(_camera, pos);
        }
    }

    void CaseClothFEM::OnProcessMouseControl(glm::vec3 mouseDelta) {
        float movingScale = 0.1f;
        _center += mouseDelta * movingScale;
    }

    bool CaseClothFEM::RayIntersectsSphere(glm::vec3 const & rayOrigin, glm::vec3 const & rayDir, glm::vec3 const & center, float radius, float & t) const {
        glm::vec3 const oc = rayOrigin - center;
        float const     a  = glm::dot(rayDir, rayDir);
        float const     b  = 2.f * glm::dot(oc, rayDir);
        float const     c  = glm::dot(oc, oc) - radius * radius;
        float const     discriminant = b * b - 4.f * a * c;
        if (discriminant < 0.f) return false;

        float const sqrtDisc = glm::sqrt(discriminant);
        float const t0       = (-b - sqrtDisc) / (2.f * a);
        float const t1       = (-b + sqrtDisc) / (2.f * a);
        if (t0 > 0.f) {
            t = t0;
            return true;
        }
        if (t1 > 0.f) {
            t = t1;
            return true;
        }
        return false;
    }

    void CaseClothFEM::GetMouseRay(ImVec2 const & mousePos, glm::vec3 & rayOrigin, glm::vec3 & rayDir) const {
        auto * window = ImGui::GetCurrentWindow();
        ImVec2 const windowSize = window->Rect().GetSize();
        float const x = (2.f * mousePos.x) / windowSize.x - 1.f;
        float const y = 1.f - (2.f * mousePos.y) / windowSize.y;

        float const aspect = windowSize.x / windowSize.y;
        glm::mat4 const invProjView = glm::inverse(_camera.GetProjectionMatrix(aspect) * _camera.GetViewMatrix());
        glm::vec4 nearPoint = invProjView * glm::vec4(x, y, -1.f, 1.f);
        glm::vec4 farPoint  = invProjView * glm::vec4(x, y, 1.f, 1.f);
        nearPoint /= nearPoint.w;
        farPoint /= farPoint.w;

        rayOrigin = glm::vec3(nearPoint);
        rayDir = glm::normalize(glm::vec3(farPoint - nearPoint));
    }

    int CaseClothFEM::PickVertex(glm::vec3 const & rayOrigin, glm::vec3 const & rayDir, float & t) const {
        int   picked = -1;
        float closestT = std::numeric_limits<float>::max();
        for (int i = 0; i < _femSystem2d.n_Vertices; ++i) {
            float vertexT = 0.f;
            if (RayIntersectsSphere(rayOrigin, rayDir, _femSystem2d.Positions[i], _pickRadius, vertexT) && vertexT < closestT) {
                picked = i;
                closestT = vertexT;
            }
        }
        t = closestT;
        return picked;
    }

    void CaseClothFEM::HandleVertexDragging(ImVec2 const & mousePos) {
        auto * window = ImGui::GetCurrentWindow();
        ImGuiIO const & io = ImGui::GetIO();

        bool hover = false;
        bool anyHeld = false;
        ImGui::ButtonBehavior(window->Rect(), window->GetID("##io"), &hover, &anyHeld);

        bool const leftClicked  = ImGui::IsMouseClicked(ImGuiMouseButton_Left);
        bool const leftHeld     = ImGui::IsMouseDown(ImGuiMouseButton_Left);
        bool const leftReleased = ImGui::IsMouseReleased(ImGuiMouseButton_Left);

        if (leftReleased) {
            _isDraggingVertex = false;
            _femSystem2d.clearKinematicVertex();
        }

        glm::vec3 rayOrigin, rayDir;
        GetMouseRay(mousePos, rayOrigin, rayDir);

        if (!_isDraggingVertex && hover && leftClicked && !io.KeyCtrl && !io.KeyShift && !io.KeyAlt) {
            float hitT = 0.f;
            int const pickedVertex = PickVertex(rayOrigin, rayDir, hitT);
            if (pickedVertex >= 0) {
                _activeVertex = pickedVertex;
                _isDraggingVertex = true;
                glm::vec3 const hitPoint = rayOrigin + hitT * rayDir;
                _dragOffset = hitPoint - _femSystem2d.Positions[pickedVertex];
                _dragPlaneNormal = -rayDir;
                _dragPlaneDistance = glm::dot(hitPoint, _dragPlaneNormal);
                _femSystem2d.setKinematicVertex(_activeVertex, _femSystem2d.Positions[_activeVertex]);
            }
        }
        if (_isDraggingVertex && leftHeld) {
            float const denom = glm::dot(rayDir, _dragPlaneNormal);
            if (glm::abs(denom) > 1e-6f) {
                float const t = (_dragPlaneDistance - glm::dot(rayOrigin, _dragPlaneNormal)) / denom;
                glm::vec3 const targetPosition = rayOrigin + t * rayDir - _dragOffset;
                _femSystem2d.setKinematicVertex(_activeVertex, targetPosition);
            }
        }
    }
}
