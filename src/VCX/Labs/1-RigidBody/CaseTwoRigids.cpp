#include "Labs/1-RigidBody/CaseTwoRigids.h"
#include "Labs/Common/ImGuiHelper.h"

namespace VCX::Labs::RigidBody {
    static constexpr auto c_Cases = std::array<char const *, 3> {
        "Case 1",
        "Case 2",
        "Case 3"
    };

    void CaseTwoRigids::reset_simulation() {
        switch (_case) {
            case 0 :
                _rigid1 = Rigid(_rigid1.m, glm::vec3(-5.f, 0.f, 0.f), glm::vec3(7.f, 0.f, 0.f), glm::quat(glm::vec3(0.f, 0.f, glm::radians(35.f))), glm::vec3(0.f));
                _rigid2 = Rigid(_rigid2.m, glm::vec3(5.f, 0.f, 0.f), glm::vec3(-7.f, 0.f, 0.f), glm::quat(glm::vec3(0.f, glm::radians(-25.f), glm::radians(-10.f))), glm::vec3(0.f));
                break;
            case 1 :
                _rigid1 = Rigid(_rigid1.m, glm::vec3(-5.f, 0.f, 0.f), glm::vec3(7.f, 0.f, 0.f), glm::quat(), glm::vec3(0.f, 0.f, 0.f));
                _rigid2 = Rigid(_rigid2.m, glm::vec3(5.f, 0.f, 0.f), glm::vec3(-7.f, 0.f, 0.f), glm::quat(glm::vec3(glm::radians(45.f), 0.f, glm::radians(30.f))), glm::vec3(0.f, 0.f, 0.f));
                break;
            case 2 :
                _rigid1 = Rigid(_rigid1.m, glm::vec3(-5.f, 0.f, 0.f), glm::vec3(7.f, 0.f, 0.f), glm::quat(), glm::vec3(0.f));
                _rigid2 = Rigid(_rigid2.m, glm::vec3(5.f, 0.f, 0.f), glm::vec3(-7.f, 0.f, 0.f), glm::quat(), glm::vec3(0.f));
                break;
        }
    }

    CaseTwoRigids::CaseTwoRigids():
        _program(
            Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/flat.vert"),
                                        Engine::GL::SharedShader("assets/shaders/flat.frag") })),
        _boxItem1(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Triangles),
        _lineItem1(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines),
        _boxItem2(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Triangles),
        _lineItem2(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines) {
        //     3-----2
        //    /|    /|
        //   0 --- 1 |
        //   | 7 - | 6
        //   |/    |/
        //   4 --- 5
        const std::vector<std::uint32_t> line_index = { 0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6, 6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7 }; // line index
        _lineItem1.UpdateElementBuffer(line_index);
        _lineItem2.UpdateElementBuffer(line_index);

        // const std::vector<std::uint32_t> tri_index = { 0, 1, 2, 0, 2, 3, 1, 4, 0, 1, 4, 5, 1, 6, 5, 1, 2, 6, 2, 3, 7, 2, 6, 7, 0, 3, 7, 0, 4, 7, 4, 5, 6, 4, 6, 7 };
        const std::vector<std::uint32_t> tri_index = { 0, 1, 2, 0, 2, 3, 1, 0, 4, 1, 4, 5, 1, 5, 6, 1, 6, 2, 2, 7, 3, 2, 6, 7, 0, 3, 7, 0, 7, 4, 4, 6, 5, 4, 7, 6 };
        _boxItem1.UpdateElementBuffer(tri_index);
        _boxItem2.UpdateElementBuffer(tri_index);
        _cameraManager.AutoRotate = false;
        _cameraManager.EnableDamping = false;
        _cameraManager.EnablePan = false;
        _cameraManager.Save(_camera);
        _rigid1.m = 1.f;
        _rigid2.m = 1.f;
        _rigid1.getBoxInertiaTensor(_dim1);
        _rigid2.getBoxInertiaTensor(_dim2);
        reset_simulation();
    }

    void CaseTwoRigids::OnSetupPropsUI() {
        ImGui::Combo("Case", &_case, c_Cases.data(), c_Cases.size());
        ImGui::Button("Reset Simulation", ImVec2(250, 0));
        if (ImGui::IsItemClicked()) {
            reset_simulation();
            _center = glm::vec3(0.f);
        }
        ImGui::Button("Reset Camera", ImVec2(250, 0));
        if (ImGui::IsItemClicked()) {
            _cameraManager.Reset(_camera);
        }
        ImGui::Checkbox("Pause", &_pause);
        ImGui::SliderFloat("Restitution", &_restitution, 0.f, 1.f);
        ImGui::SliderFloat("Mass 1", &_rigid1.m, 0.1f, 10.f);
        ImGui::SliderFloat("Mass 2", &_rigid2.m, 0.1f, 10.f);
        ImGui::RadioButton("Control Rigid 1", &_controlling, 1);
        ImGui::RadioButton("Control Rigid 2", &_controlling, 2);
        ImGui::RadioButton("Control Both", &_controlling, 3);
        ImGui::Text("Position 1 %.2f %.2f %.2f", _rigid1.x.x, _rigid1.x.y, _rigid1.x.z);
        ImGui::Text("Velocity 1 %.2f %.2f %.2f", _rigid1.v.x, _rigid1.v.y, _rigid1.v.z);
        ImGui::Text("Position 2 %.2f %.2f %.2f", _rigid2.x.x, _rigid2.x.y, _rigid2.x.z);
        ImGui::Text("Velocity 2 %.2f %.2f %.2f", _rigid2.v.x, _rigid2.v.y, _rigid2.v.z);
        ImGui::Spacing();
    }

    Common::CaseRenderResult CaseTwoRigids::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        if (ImGui::IsKeyDown(ImGuiKey_Space)) {
            _pause = !_pause;
        }
        // apply mouse control first
        OnProcessMouseControl(_cameraManager.getMouseMove());
        _rigid2.Damping = _rigid1.Damping;

        // rendering
        _frame.Resize(desiredSize);
        _rigid1.getBoxInertiaTensor(_dim1);
        _rigid2.getBoxInertiaTensor(_dim2);
        if (!_pause) {
            collide_test(_rigid1, _dim1, _rigid2, _dim2, _restitution);
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
            if (_controlling == 1) {
                _rigid1.SimulateTimestep(0.001f, force, torque);
                _rigid2.SimulateTimestep(0.001f, glm::vec3(0.f), glm::vec3(0.f));
            } else if (_controlling == 2) {
                _rigid1.SimulateTimestep(0.001f, glm::vec3(0.f), glm::vec3(0.f));
                _rigid2.SimulateTimestep(0.001f, force, torque);
            } else if (_controlling == 3) {
                _rigid1.SimulateTimestep(0.001f, force, torque);
                _rigid2.SimulateTimestep(0.001f, -force, -torque);
            }
        }

        _cameraManager.Update(_camera);
        _program.GetUniforms().SetByName("u_Projection", _camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
        _program.GetUniforms().SetByName("u_View", _camera.GetViewMatrix());

        gl_using(_frame);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_LINE_SMOOTH);
        glLineWidth(.5f);

        std::vector<glm::vec3> VertsPosition1, VertsPosition2;
        glm::vec3 sgn[8] = {
            {-1, 1, 1},
            { 1, 1, 1},
            { 1, 1, -1},
            {-1, 1, -1},
            {-1, -1, 1},
            { 1, -1, 1},
            { 1, -1, -1},
            {-1, -1, -1}
        };
        for (int i = 0; i < 8; i ++) {
            glm::vec3 localPos1 = _rigid1.q * (sgn[i] * _dim1 / 2.f);
            VertsPosition1.push_back(_center + localPos1 + _rigid1.x);
            glm::vec3 localPos2 = _rigid2.q * (sgn[i] * _dim2 / 2.f);
            VertsPosition2.push_back(_center + localPos2 + _rigid2.x);
        }

        auto span_bytes1 = Engine::make_span_bytes<glm::vec3>(VertsPosition1);

        _program.GetUniforms().SetByName("u_Color", _boxColor1);
        _boxItem1.UpdateVertexBuffer("position", span_bytes1);
        _boxItem1.Draw({ _program.Use() });
        
        _program.GetUniforms().SetByName("u_Color", glm::vec3(1.f, 1.f, 1.f));
        _lineItem1.UpdateVertexBuffer("position", span_bytes1);
        _lineItem1.Draw({ _program.Use() });

        auto span_bytes2 = Engine::make_span_bytes<glm::vec3>(VertsPosition2);

        _program.GetUniforms().SetByName("u_Color", _boxColor2);
        _boxItem2.UpdateVertexBuffer("position", span_bytes2);
        _boxItem2.Draw({ _program.Use() });

        _program.GetUniforms().SetByName("u_Color", glm::vec3(1.f, 1.f, 1.f));
        _lineItem2.UpdateVertexBuffer("position", span_bytes2);
        _lineItem2.Draw({ _program.Use() });

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

    void CaseTwoRigids::OnProcessInput(ImVec2 const & pos) {
        // printf("Mouse Pos: %f, %f\n", pos.x, pos.y);
        _cameraManager.ProcessInput(_camera, pos);
    }

    void CaseTwoRigids::OnProcessMouseControl(glm::vec3 mouseDelta) {
        // printf("Mouse Delta: %f, %f. %f\n", mouseDelta.x, mouseDelta.y, mouseDelta.z);
        // float movingScale = 0.1f;
        // _center += mouseDelta * movingScale;
    }
} // namespace VCX::Labs::RigidBody