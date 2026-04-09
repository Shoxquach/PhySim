#include "Labs/1-RigidBody/CaseSingleRigid.h"
#include "Labs/Common/ImGuiHelper.h"

namespace VCX::Labs::RigidBody {

    CaseSingleRigid::CaseSingleRigid():
        _program(
            Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/flat.vert"),
                                        Engine::GL::SharedShader("assets/shaders/flat.frag") })),
        _boxItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Triangles),
        _lineItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines) {
        //     3-----2
        //    /|    /|
        //   0 --- 1 |
        //   | 7 - | 6
        //   |/    |/
        //   4 --- 5
        const std::vector<std::uint32_t> line_index = { 0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6, 6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7 }; // line index
        _lineItem.UpdateElementBuffer(line_index);

        // const std::vector<std::uint32_t> tri_index = { 0, 1, 2, 0, 2, 3, 1, 4, 0, 1, 4, 5, 1, 6, 5, 1, 2, 6, 2, 3, 7, 2, 6, 7, 0, 3, 7, 0, 4, 7, 4, 5, 6, 4, 6, 7 };
        const std::vector<std::uint32_t> tri_index = { 0, 1, 2, 0, 2, 3, 1, 0, 4, 1, 4, 5, 1, 5, 6, 1, 6, 2, 2, 7, 3, 2, 6, 7, 0, 3, 7, 0, 7, 4, 4, 6, 5, 4, 7, 6 };
        _boxItem.UpdateElementBuffer(tri_index);
        _cameraManager.AutoRotate = false;
        _cameraManager.EnableDamping = false;
        _cameraManager.EnablePan = false;
        _cameraManager.Save(_camera);
        _rigid.getBoxInertiaTensor(_dim);
    }

    void CaseSingleRigid::OnSetupPropsUI() {
        ImGui::Button("Reset Simulation", ImVec2(250, 0));
        if (ImGui::IsItemClicked()) {
            _rigid.x = glm::vec3(0.f);
            _rigid.v = glm::vec3(0.f);
            _rigid.L = glm::vec3(0.f);
            _rigid.q = glm::quat();
            _center = glm::vec3(0.f);
        }
        ImGui::Button("Reset Camera", ImVec2(250, 0));
        if (ImGui::IsItemClicked()) {
            _cameraManager.Reset(_camera);
        }
        ImGui::Checkbox("Pause", &_pause);
        ImGui::SliderFloat("Damping", &_rigid.Damping, 0.999f, 1.f, "%.4f");
        ImGui::Text("Position %.2f %.2f %.2f", _rigid.x.x, _rigid.x.y, _rigid.x.z);
        ImGui::Text("Velocity %.2f %.2f %.2f", _rigid.v.x, _rigid.v.y, _rigid.v.z);
        ImGui::Text("Orientation %.2f %.2f %.2f %.2f", _rigid.q.x, _rigid.q.y, _rigid.q.z, _rigid.q.w);
        ImGui::Spacing();
    }

    Common::CaseRenderResult CaseSingleRigid::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        if (ImGui::IsKeyDown(ImGuiKey_Space)) {
            _pause = !_pause;
        }
        // apply mouse control first
        OnProcessMouseControl(_cameraManager.getMouseMove());

        // rendering
        _frame.Resize(desiredSize);
        if (!_pause) {
            glm::vec3 force(0.f), torque(0.f);
            if (ImGui::IsKeyDown(ImGuiKey_LeftArrow)) {
                force += glm::vec3(-10.f, 0.f, 0.f);
                // printf("Pushing left\n");
            }
            if (ImGui::IsKeyDown(ImGuiKey_RightArrow)) {
                force += glm::vec3(10.f, 0.f, 0.f);
                // printf("Pushing right\n");
            }
            if (ImGui::IsKeyDown(ImGuiKey_UpArrow)) {
                force += glm::vec3(0.f, 10.f, 0.f);
                // printf("Pushing up\n");
            }
            if (ImGui::IsKeyDown(ImGuiKey_DownArrow)) {
                force += glm::vec3(0.f, -10.f, 0.f);
                // printf("Pushing down\n");
            }
            if (ImGui::IsKeyDown(ImGuiKey_Comma)) {
                force += glm::vec3(0.f, 0.f, 10.f);
                // printf("Pushing forward\n");
            }
            if (ImGui::IsKeyDown(ImGuiKey_Period)) {
                force += glm::vec3(0.f, 0.f, -10.f);
                // printf("Pushing backward\n");
            }
            if (ImGui::IsKeyDown(ImGuiKey_Q)) {
                torque += glm::vec3(0.f, 0.f, 10.f);
                // printf("Torque clockwise\n");
            }
            if (ImGui::IsKeyDown(ImGuiKey_E)) {
                torque += glm::vec3(0.f, 0.f, -10.f);
                // printf("Torque counterclockwise\n");
            }
            if (ImGui::IsKeyDown(ImGuiKey_S)) {
                torque += glm::vec3(10.f, 0.f, 0.f);
                // printf("Torque pitch up\n");
            }
            if (ImGui::IsKeyDown(ImGuiKey_W)) {
                torque += glm::vec3(-10.f, 0.f, 0.f);
                // printf("Torque pitch down\n");
            }
            if (ImGui::IsKeyDown(ImGuiKey_D)) {
                torque += glm::vec3(0.f, 10.f, 0.f);
                // printf("Torque yaw left\n");
            }
            if (ImGui::IsKeyDown(ImGuiKey_A)) {
                torque += glm::vec3(0.f, -10.f, 0.f);
                // printf("Torque yaw right\n");
            }
            _rigid.SimulateTimestep(0.001f, force, torque);
        }

        _cameraManager.Update(_camera);
        _program.GetUniforms().SetByName("u_Projection", _camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
        _program.GetUniforms().SetByName("u_View", _camera.GetViewMatrix());

        gl_using(_frame);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_LINE_SMOOTH);
        glLineWidth(.5f);

        std::vector<glm::vec3> VertsPosition;
        glm::vec3 sgn[8] = {
            {-1, 1, 1},
            {1, 1, 1},
            {1, 1, -1},
            {-1, 1, -1},
            {-1, -1, 1},
            {1, -1, 1},
            {1, -1, -1},
            {-1, -1, -1}
        };
        for (int i = 0; i < 8; i ++) {
            glm::vec3 localPos = _rigid.q * (sgn[i] * _dim / 2.f);
            VertsPosition.push_back(_center + localPos + _rigid.x);
        }

        auto span_bytes = Engine::make_span_bytes<glm::vec3>(VertsPosition);

        _program.GetUniforms().SetByName("u_Color", _boxColor);
        _boxItem.UpdateVertexBuffer("position", span_bytes);
        _boxItem.Draw({ _program.Use() });

        _program.GetUniforms().SetByName("u_Color", glm::vec3(1.f, 1.f, 1.f));
        _lineItem.UpdateVertexBuffer("position", span_bytes);
        _lineItem.Draw({ _program.Use() });

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

    void CaseSingleRigid::OnProcessInput(ImVec2 const & pos) {
        // printf("Mouse Pos: %f, %f\n", pos.x, pos.y);
        _cameraManager.ProcessInput(_camera, pos);
    }

    void CaseSingleRigid::OnProcessMouseControl(glm::vec3 mouseDelta) {
        // printf("Mouse Delta: %f, %f. %f\n", mouseDelta.x, mouseDelta.y, mouseDelta.z);
        // float movingScale = 0.1f;
        // _center += mouseDelta * movingScale;
    }
} // namespace VCX::Labs::RigidBody