#include "Labs/1-RigidBody/CaseConstraintDynamics.h"
#include "Labs/Common/ImGuiHelper.h"
#include <random>

namespace VCX::Labs::RigidBody {
    void CaseConstraintDynamics::reset_simulation() {
        std::uniform_real_distribution<float> dist(-1.f, 1.f);
        std::mt19937 rng(std::random_device{}());
        for (int i = 0; i < _numRigids; i++) {
            _rigids[i] = Rigid(1.f, glm::vec3(dist(rng) * _random_offset, 0.f + i * 2.f, dist(rng) * _random_offset), glm::vec3(0.f), glm::quat(), glm::vec3(0.f));
            _rigids[i].getBoxInertiaTensor(_dims[i]);
        }
        _center = glm::vec3(0.f);
    }

    CaseConstraintDynamics::CaseConstraintDynamics():
        _program(
            Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/flat.vert"),
                                        Engine::GL::SharedShader("assets/shaders/flat.frag") })),
            _boxItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Triangles),
            _lineItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines),
            _groundboxItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Triangles),
            _groundlineItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines),
            _selectedboxItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Triangles),
            _selectedlineItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines) {
        //     3-----2
        //    /|    /|
        //   0 --- 1 |
        //   | 7 - | 6
        //   |/    |/
        //   4 --- 5
        std::vector<std::uint32_t> line_index = { 0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6, 6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7 };
        std::vector<std::uint32_t> tri_index = { 0, 1, 2, 0, 2, 3, 1, 0, 4, 1, 4, 5, 1, 5, 6, 1, 6, 2, 2, 7, 3, 2, 6, 7, 0, 3, 7, 0, 7, 4, 4, 6, 5, 4, 7, 6 };
        std::vector<std::uint32_t> selected_line_index = { 0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6, 6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7 };
        std::vector<std::uint32_t> selected_tri_index = { 0, 1, 2, 0, 2, 3, 1, 0, 4, 1, 4, 5, 1, 5, 6, 1, 6, 2, 2, 7, 3, 2, 6, 7, 0, 3, 7, 0, 7, 4, 4, 6, 5, 4, 7, 6 };
        _selectedboxItem.UpdateElementBuffer(selected_tri_index);
        _selectedlineItem.UpdateElementBuffer(selected_line_index);
        std::uniform_real_distribution<float> dist(-1.f, 1.f);
        std::mt19937 rng(std::random_device{}());
        for (int i = 0; i < _numRigids; i++) {
            if (i > 0 && i < _numRigids - 1) {
                for (int j = 0; j < 24; j ++) {
                    line_index.push_back(line_index[j] + 8 * i);
                }
                for (int j = 0; j < 36; j ++) {
                    tri_index.push_back(tri_index[j] + 8 * i);
                }
            }
            _dims.push_back(glm::vec3(1.f, 0.5f, 1.5f));
            _rigids.push_back(Rigid(1.f, glm::vec3(dist(rng) * _random_offset, 0.f + i * 2.f, dist(rng) * _random_offset), glm::vec3(0.f), glm::quat(), glm::vec3(0.f)));
            _rigids[i].getBoxInertiaTensor(_dims[i]);
        }
        std::vector<std::uint32_t> ground_line_index = { 0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6, 6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7, 8, 9, 9, 10, 10, 11, 11, 8 };
        std::vector<std::uint32_t> ground_tri_index = { 0, 1, 2, 0, 2, 3, 1, 0, 4, 1, 4, 5, 1, 5, 6, 1, 6, 2, 2, 7, 3, 2, 6, 7, 0, 3, 7, 0, 7, 4 };
        _ground = Rigid(0.f, glm::vec3(0.f, _groundY, 0.f), glm::vec3(0.f), glm::quat(), glm::vec3(0.f));
        _walls[0] = Rigid(0.f, glm::vec3(0.f, _groundY + _wall_height / 2.f, -_ground_size), glm::vec3(0.f), glm::quat(), glm::vec3(0.f));
        _walls[1] = Rigid(0.f, glm::vec3(0.f, _groundY + _wall_height / 2.f, _ground_size), glm::vec3(0.f), glm::quat(), glm::vec3(0.f));
        _walls[2] = Rigid(0.f, glm::vec3(-_ground_size, _groundY + _wall_height / 2.f, 0.f), glm::vec3(0.f), glm::quat(), glm::vec3(0.f));
        _walls[3] = Rigid(0.f, glm::vec3(_ground_size, _groundY + _wall_height / 2.f, 0.f), glm::vec3(0.f), glm::quat(), glm::vec3(0.f));

        _boxItem.UpdateElementBuffer(tri_index);
        _lineItem.UpdateElementBuffer(line_index);
        _groundboxItem.UpdateElementBuffer(ground_tri_index);
        _groundlineItem.UpdateElementBuffer(ground_line_index);
        _cameraManager.AutoRotate = false;
        _cameraManager.EnableDamping = false;
        _cameraManager.EnablePan = false;
        _cameraManager.Save(_camera);
    }

    void CaseConstraintDynamics::OnSetupPropsUI() {
        ImGui::Button("Reset Simulation", ImVec2(250, 0));
        if (ImGui::IsItemClicked()) {
            reset_simulation();
        }
        ImGui::Button("Reset Camera", ImVec2(250, 0));
        if (ImGui::IsItemClicked()) {
            _cameraManager.Reset(_camera);
        }
        ImGui::Checkbox("Pause", &_pause);
        ImGui::SliderFloat("Timestep", &_timestep, 0.001f, 0.01f, "%.6f");
        ImGui::SliderFloat("Damping", &_damping, 0.98f, 1.f, "%.4f");
        ImGui::SliderFloat("Mu", &_mu, 0.f, 1.f);
        ImGui::SliderFloat("Restitution", &_restitution, 0.f, 1.f);
        ImGui::SliderFloat("Random Offset", &_random_offset, 0.f, 1.f);
        ImGui::SliderFloat("Correction Factor", &_correction_factor, 0.f, 1.f);
        for (int i = 0; i < _numRigids; i ++) {
            ImGui::Text("P%d %.2f %.2f %.2f V%d %.2f %.2f %.2f",
                i + 1, _rigids[i].x.x, _rigids[i].x.y, _rigids[i].x.z, i + 1, _rigids[i].v.x, _rigids[i].v.y, _rigids[i].v.z);
        }
        ImGui::Spacing();
    }

    Common::CaseRenderResult CaseConstraintDynamics::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        if (ImGui::IsKeyDown(ImGuiKey_Space)) {
            _pause = !_pause;
        }
        if (ImGui::IsKeyReleased(ImGuiKey_GraveAccent)) {
            _selected = (_selected + 1) % _numRigids;
        }
        // apply mouse control first
        OnProcessMouseControl(_cameraManager.getMouseMove());
        for (int i = 0; i < _numRigids; i++) {
            _rigids[i].Damping = _damping;
        }

        // rendering
        _frame.Resize(desiredSize);
        if (!_pause) {
            glm::vec3 force(0.f), torque(0.f);
            if (ImGui::IsKeyDown(ImGuiKey_LeftArrow)) {
                force += glm::vec3(-1.f, 0.f, 0.f);
                // printf("Pushing left\n");
            }
            if (ImGui::IsKeyDown(ImGuiKey_RightArrow)) {
                force += glm::vec3(1.f, 0.f, 0.f);
                // printf("Pushing right\n");
            }
            if (ImGui::IsKeyDown(ImGuiKey_UpArrow)) {
                force += glm::vec3(0.f, 1.f, 0.f) - _gravity;
                // printf("Pushing up\n");
            }
            if (ImGui::IsKeyDown(ImGuiKey_DownArrow)) {
                force += glm::vec3(0.f, -1.f, 0.f);
                // printf("Pushing down\n");
            }
            if (ImGui::IsKeyDown(ImGuiKey_Comma)) {
                force += glm::vec3(0.f, 0.f, 1.f);
                // printf("Pushing forward\n");
            }
            if (ImGui::IsKeyDown(ImGuiKey_Period)) {
                force += glm::vec3(0.f, 0.f, -1.f);
                // printf("Pushing backward\n");
            }
            if (ImGui::IsKeyDown(ImGuiKey_Q)) {
                torque += glm::vec3(0.f, 0.f, 1.f);
                // printf("Torque clockwise\n");
            }
            if (ImGui::IsKeyDown(ImGuiKey_E)) {
                torque += glm::vec3(0.f, 0.f, -1.f);
                // printf("Torque counterclockwise\n");
            }
            if (ImGui::IsKeyDown(ImGuiKey_S)) {
                torque += glm::vec3(1.f, 0.f, 0.f);
                // printf("Torque pitch up\n");
            }
            if (ImGui::IsKeyDown(ImGuiKey_W)) {
                torque += glm::vec3(-1.f, 0.f, 0.f);
                // printf("Torque pitch down\n");
            }
            if (ImGui::IsKeyDown(ImGuiKey_D)) {
                torque += glm::vec3(0.f, 1.f, 0.f);
                // printf("Torque yaw left\n");
            }
            if (ImGui::IsKeyDown(ImGuiKey_A)) {
                torque += glm::vec3(0.f, -1.f, 0.f);
                // printf("Torque yaw right\n");
            }
            std::vector<Rigid*> rigid_list;
            std::vector<glm::vec3> dims_list, force_list;
            for (int i = 0; i < _numRigids; i++) {
                rigid_list.push_back(&_rigids[i]);
                dims_list.push_back(_dims[i]);
                if (i == _selected) {
                    force_list.push_back(force + _gravity);
                    force_list.push_back(torque);
                } else {
                    force_list.push_back(_gravity);
                    force_list.push_back(glm::vec3(0.f));
                }
            }
            rigid_list.push_back(&_ground);
            dims_list.push_back(glm::vec3(_ground_size * 2.f, 0.f, _ground_size * 2.f));
            rigid_list.push_back(&_walls[0]);
            dims_list.push_back(glm::vec3(_ground_size * 2.f, _wall_height, 0.f));
            rigid_list.push_back(&_walls[1]);
            dims_list.push_back(glm::vec3(_ground_size * 2.f, _wall_height, 0.f));
            rigid_list.push_back(&_walls[2]);
            dims_list.push_back(glm::vec3(0.f, _wall_height, _ground_size * 2.f));
            rigid_list.push_back(&_walls[3]);
            dims_list.push_back(glm::vec3(0.f, _wall_height, _ground_size * 2.f));
            simulation_constraint(rigid_list, dims_list, force_list, _timestep, _restitution, _correction_factor, _mu, _damping);
        }

        _cameraManager.Update(_camera);
        _program.GetUniforms().SetByName("u_Projection", _camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
        _program.GetUniforms().SetByName("u_View", _camera.GetViewMatrix());

        gl_using(_frame);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_LINE_SMOOTH);
        glLineWidth(.5f);

        const glm::vec3 sgn[8] = {
            {-1, 1, 1},
            {1, 1, 1},
            {1, 1, -1},
            {-1, 1, -1},
            {-1, -1, 1},
            {1, -1, 1},
            {1, -1, -1},
            {-1, -1, -1}
        };
        std::vector<glm::vec3> VertsPosition;
        for (int i = 0; i < _numRigids; i ++) {
            if (i != _selected) {
                for (int j = 0; j < 8; j ++) {
                    glm::vec3 localPos1 = _rigids[i].q * (sgn[j] * _dims[i] / 2.f);
                    VertsPosition.push_back(_center + localPos1 + _rigids[i].x);
                }
            }
        }
        auto span_bytes = Engine::make_span_bytes<glm::vec3>(VertsPosition);
        _program.GetUniforms().SetByName("u_Color", _boxColor);
        _boxItem.UpdateVertexBuffer("position", span_bytes);
        _boxItem.Draw({ _program.Use() });
        _program.GetUniforms().SetByName("u_Color", glm::vec3(1.f, 1.f, 1.f));
        _lineItem.UpdateVertexBuffer("position", span_bytes);
        _lineItem.Draw({ _program.Use() });

        VertsPosition.clear();
        for (int j = 0; j < 8; j ++) {
            glm::vec3 localPos1 = _rigids[_selected].q * (sgn[j] * _dims[_selected] / 2.f);
            VertsPosition.push_back(_center + localPos1 + _rigids[_selected].x);
        }
        auto selected_span_bytes = Engine::make_span_bytes<glm::vec3>(VertsPosition);
        _program.GetUniforms().SetByName("u_Color", _selectedColor);
        _selectedboxItem.UpdateVertexBuffer("position", selected_span_bytes);
        _selectedboxItem.Draw({ _program.Use() });
        _program.GetUniforms().SetByName("u_Color", glm::vec3(1.f, 1.f, 1.f));
        _selectedlineItem.UpdateVertexBuffer("position", selected_span_bytes);
        _selectedlineItem.Draw({ _program.Use() });


        VertsPosition.clear();
        VertsPosition.push_back(_center + glm::vec3(-_ground_size, _groundY, -_ground_size));
        VertsPosition.push_back(_center + glm::vec3(_ground_size, _groundY, -_ground_size));
        VertsPosition.push_back(_center + glm::vec3(_ground_size, _groundY, _ground_size));
        VertsPosition.push_back(_center + glm::vec3(-_ground_size, _groundY, _ground_size));
        VertsPosition.push_back(_center + glm::vec3(-_ground_size, _groundY + _wall_height, -_ground_size));
        VertsPosition.push_back(_center + glm::vec3(_ground_size, _groundY + _wall_height, -_ground_size));
        VertsPosition.push_back(_center + glm::vec3(_ground_size, _groundY + _wall_height, _ground_size));
        VertsPosition.push_back(_center + glm::vec3(-_ground_size, _groundY + _wall_height, _ground_size));
        VertsPosition.push_back(_center + glm::vec3(-_ground_size + _offset, _groundY, -_ground_size + _offset));
        VertsPosition.push_back(_center + glm::vec3(_ground_size - _offset, _groundY, -_ground_size + _offset));
        VertsPosition.push_back(_center + glm::vec3(_ground_size - _offset, _groundY, _ground_size - _offset));
        VertsPosition.push_back(_center + glm::vec3(-_ground_size + _offset, _groundY, _ground_size - _offset));
        auto span_bytes_ground = Engine::make_span_bytes<glm::vec3>(VertsPosition);
        _program.GetUniforms().SetByName("u_Color", _groundColor);
        _groundboxItem.UpdateVertexBuffer("position", span_bytes_ground);
        _groundboxItem.Draw({ _program.Use() });
        _program.GetUniforms().SetByName("u_Color", glm::vec3(1.f, 1.f, 1.f));
        _groundlineItem.UpdateVertexBuffer("position", span_bytes_ground);
        _groundlineItem.Draw({ _program.Use() });

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

    void CaseConstraintDynamics::OnProcessInput(ImVec2 const & pos) {
        // printf("Mouse Pos: %f, %f\n", pos.x, pos.y);
        _cameraManager.ProcessInput(_camera, pos);
    }

    void CaseConstraintDynamics::OnProcessMouseControl(glm::vec3 mouseDelta) {
        // printf("Mouse Delta: %f, %f. %f\n", mouseDelta.x, mouseDelta.y, mouseDelta.z);
        float movingScale = 0.1f;
        _center += mouseDelta * movingScale;
    }
} // namespace VCX::Labs::RigidBody