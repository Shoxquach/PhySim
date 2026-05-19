#include <spdlog/spdlog.h>
#include "Engine/app.h"
#include "Labs/3-FEM/CaseFEM.h"
#include "Labs/Common/ImGuiHelper.h"
#include <iostream>
#include <imgui.h>
#include <imgui_internal.h>
#include <limits>

namespace VCX::Labs::FEM {
    const std::vector<std::uint32_t> ground_tri_index = {
        0, 1, 2,
        0, 2, 3
    };
    const std::vector<std::uint32_t> ground_line_index = {
        0, 1, 1, 2, 2, 3, 3, 0
    };
    const std::vector<glm::vec3> ground_vertex_positions = {
        glm::vec3(-10.f, -5.f, -10.f),
        glm::vec3(10.f, -5.f, -10.f),
        glm::vec3(10.f, -5.f, 10.f),
        glm::vec3(-10.f, -5.f, 10.f)
    };

    CaseFEM::CaseFEM():
        _program(
            Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/flat.vert"),
                                        Engine::GL::SharedShader("assets/shaders/flat.frag") })),
        _item(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Triangles),
        _lineItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines),
        _groundItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Triangles),
        _groundLineItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines),
        _vertexItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Points) {
        resetScene(_object);
        _groundItem.UpdateElementBuffer(ground_tri_index);
        _groundLineItem.UpdateElementBuffer(ground_line_index);
        _cameraManager.AutoRotate = false;
        _cameraManager.EnableDamping = false;
        _cameraManager.EnablePan = false;
        _cameraManager.Save(_camera);
    }

    void CaseFEM::resetScene(Object object) {
        switch (object) {
            case tet:
                _item.UpdateElementBuffer(Tet::tri_index);
                _lineItem.UpdateElementBuffer(Tet::line_index);
                _femSystem.reset(Tet::tet_index, Tet::tet_vertex_positions, _dropHeight);
                break;
            case cube:
                _item.UpdateElementBuffer(Cube::tri_index);
                _lineItem.UpdateElementBuffer(Cube::line_index);
                _femSystem.reset(Cube::tet_index, Cube::tet_vertex_positions, _dropHeight);
                break;
            case object1:
                _item.UpdateElementBuffer(Object1::tri_index);
                _lineItem.UpdateElementBuffer(Object1::line_index);
                _femSystem.reset(Object1::tet_index, Object1::tet_vertex_positions, _dropHeight);
                break;
            case icosahedron:
                _item.UpdateElementBuffer(Icosahedron::tri_index);
                _lineItem.UpdateElementBuffer(Icosahedron::line_index);
                _femSystem.reset(Icosahedron::tet_index, Icosahedron::tet_vertex_positions, _dropHeight);
                break;
            case object2:
                _item.UpdateElementBuffer(Object2::tri_index);
                _lineItem.UpdateElementBuffer(Object2::line_index);
                _femSystem.reset(Object2::tet_index, Object2::tet_vertex_positions, _dropHeight);
                break;
            case block:
                Block _block(2, 2, 8);
                _item.UpdateElementBuffer(_block.tri_index);
                _lineItem.UpdateElementBuffer(_block.line_index);
                _femSystem.reset(_block.tet_index, _block.tet_vertex_positions, _dropHeight);
        }
    }

    void CaseFEM::OnSetupPropsUI() {
        ImGui::Button("Reset");
        if (ImGui::IsItemClicked()) {
            _femSystem.reset(_dropHeight);
            _activeVertex = -1;
            _isDraggingVertex = false;
        }
        ImGui::SameLine();
        ImGui::Button(_pause ? "Resume" : "Pause");
        if (ImGui::IsItemClicked()) {
            _pause = !_pause;
        }
        ImGui::SameLine();
        ImGui::Checkbox("Plastic", &_usePlastic);
        ImGui::Combo("Method", (int *)&_method, "StVK\0Neo-Hookean\0Corotated\0");
        ImGui::Combo("Object", (int *)&_object, "Tet\0Cube\0Object1\0Icosahedron\0Object2\0Block\0");
        ImGui::SliderFloat("Drop Height", &_dropHeight, 0.f, 10.f);
        ImGui::SliderFloat("Time Step", &_timeStep, 0.0005f, 0.005f, "%.6f");
        ImGui::SliderFloat("Damping", &_femSystem.damping, 0.99f, 1.f, "%.6f");
        ImGui::SliderFloat("E", &_E, 1000.f, 20000.f);
        ImGui::SliderFloat("Upsilon", &_upsilon, 0.f, .5f);
        if (_usePlastic) {
            ImGui::SliderFloat("Eta", &_femSystem.eta, 0.f, 10.f, "%.2f");
            ImGui::SliderFloat("Stretch Rate", &_stretchRate, 1.1, 2.f, "%.2f");
        }
        ImGui::SliderFloat("Pick Radius", &_pickRadius, 0.05f, 1.f, "%.2f");
        if (_activeVertex >= 0) {
            ImGui::Text("Selected vertex: %d", _activeVertex);
        } else {
            ImGui::Text("Click a FEM vertex to drag it.");
        }
    }

    Common::CaseRenderResult CaseFEM::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        if (ImGui::IsKeyPressed(ImGuiKey_Space)) {
            _pause = !_pause;
        }
        OnProcessMouseControl(_cameraManager.getMouseMove());

        if (_previousObject != _object) {
            resetScene(_object);
            _previousObject = _object;
        }

        if (!_pause) {
            _femSystem.sigma_max = _stretchRate;
            _femSystem.sigma_min = 1.f / _stretchRate;
            _femSystem.mu = _E / (2 * (1 + _upsilon));
            _femSystem.lambda = _E * _upsilon / (1 + _upsilon) / (1 - 2 * _upsilon);
            _femSystem.simulateTimestep(_timeStep, _method, _usePlastic);
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

        auto span_bytes = Engine::make_span_bytes<glm::vec3>(_femSystem.Positions);

        _program.GetUniforms().SetByName("u_Color", _color);
        _item.UpdateVertexBuffer("position", span_bytes);
        _item.Draw({ _program.Use() });

        _program.GetUniforms().SetByName("u_Color", glm::vec3(1.f, 1.f, 1.f));
        _lineItem.UpdateVertexBuffer("position", span_bytes);
        _lineItem.Draw({ _program.Use() });

        if (_activeVertex >= 0 && _activeVertex < _femSystem.n_Vertices && _isDraggingVertex) {
            std::vector<glm::vec3> const selectedVertex { _femSystem.Positions[_activeVertex] };
            glDisable(GL_DEPTH_TEST);
            glPointSize(_vertexPointSize);
            _program.GetUniforms().SetByName("u_Color", glm::vec3(1.f, .9f, .1f));
            _vertexItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(selectedVertex));
            _vertexItem.Draw({ _program.Use() });
            glEnable(GL_DEPTH_TEST);
        }

        span_bytes = Engine::make_span_bytes<glm::vec3>(ground_vertex_positions);
        _program.GetUniforms().SetByName("u_Color", glm::vec3(0.2f, 0.2f, 0.2f));
        _groundItem.UpdateVertexBuffer("position", span_bytes);
        _groundItem.Draw({ _program.Use() });

        _program.GetUniforms().SetByName("u_Color", glm::vec3(1.f, 1.f, 1.f));
        _groundLineItem.UpdateVertexBuffer("position", span_bytes);
        _groundLineItem.Draw({ _program.Use() });

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
    
    void CaseFEM::OnProcessInput(ImVec2 const & pos) {
        HandleVertexDragging(pos);
        if (!_isDraggingVertex) {
            _cameraManager.ProcessInput(_camera, pos);
        }
    }

    void CaseFEM::OnProcessMouseControl(glm::vec3 mouseDelta) {
        float movingScale = 0.1f;
        _center += mouseDelta * movingScale;
    }

    bool CaseFEM::RayIntersectsSphere(glm::vec3 const & rayOrigin, glm::vec3 const & rayDir, glm::vec3 const & center, float radius, float & t) const {
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

    void CaseFEM::GetMouseRay(ImVec2 const & mousePos, glm::vec3 & rayOrigin, glm::vec3 & rayDir) const {
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

    int CaseFEM::PickVertex(glm::vec3 const & rayOrigin, glm::vec3 const & rayDir, float & t) const {
        int   picked = -1;
        float closestT = std::numeric_limits<float>::max();
        for (int i = 0; i < _femSystem.n_Vertices; ++i) {
            float vertexT = 0.f;
            if (RayIntersectsSphere(rayOrigin, rayDir, _femSystem.Positions[i], _pickRadius, vertexT) && vertexT < closestT) {
                picked = i;
                closestT = vertexT;
            }
        }
        t = closestT;
        return picked;
    }

    void CaseFEM::HandleVertexDragging(ImVec2 const & mousePos) {
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
            _femSystem.clearKinematicVertex();
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
                _dragOffset = hitPoint - _femSystem.Positions[pickedVertex];
                _dragPlaneNormal = -rayDir;
                _dragPlaneDistance = glm::dot(hitPoint, _dragPlaneNormal);
                _femSystem.setKinematicVertex(_activeVertex, _femSystem.Positions[_activeVertex]);
            }
        }

        if (_isDraggingVertex && leftHeld) {
            float const denom = glm::dot(rayDir, _dragPlaneNormal);
            if (glm::abs(denom) > 1e-6f) {
                float const t = (_dragPlaneDistance - glm::dot(rayOrigin, _dragPlaneNormal)) / denom;
                glm::vec3 const targetPosition = rayOrigin + t * rayDir - _dragOffset;
                _femSystem.setKinematicVertex(_activeVertex, targetPosition);
            }
        }
    }
}