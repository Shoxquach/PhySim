#include "Labs/3-FEM/FEMSystem2d.h"

namespace VCX::Labs::FEM {

    void FEMSystem2D::reset(const std::vector<int> & tri_index, const std::vector<glm::vec3> & tri_vertex_positions) {
        n_Vertices = tri_vertex_positions.size();
        n_Triangles = tri_index.size() / 3;
        Positions = tri_vertex_positions;
        RestPositions = tri_vertex_positions;
        Velocities.assign(n_Vertices, glm::vec3(0.f));
        Masses.assign(n_Vertices, 0.f);
        Indices = tri_index;
        float density = .5f;
        E.clear();
        inv_E.clear();
        Area.clear();
        for (int i = 0; i < n_Triangles; i ++) {
            int i0 = Indices[i * 3];
            int i1 = Indices[i * 3 + 1];
            int i2 = Indices[i * 3 + 2];
            glm::vec3 R1 = Positions[i1] - Positions[i0];
            glm::vec3 R2 = Positions[i2] - Positions[i0];
            glm::vec3 u = glm::normalize(R1);
            glm::vec3 normal = glm::normalize(glm::cross(R1, R2));
            glm::vec3 v = glm::normalize(glm::cross(normal, u));
            glm::vec2 U1(glm::length(R1), 0.f);
            glm::vec2 U2(glm::dot(R2, u), glm::dot(R2, v));
            E.push_back(glm::mat2{U1, U2});
            inv_E.push_back(glm::inverse(E[i]));
            Area.push_back(glm::abs(glm::determinant(E[i])) / 2.f);
            Masses[i0] += Area[i] * density / 3.f;
            Masses[i1] += Area[i] * density / 3.f;
            Masses[i2] += Area[i] * density / 3.f;
        }
    }

    void FEMSystem2D::applyFixedTopConstraints() {
        if (FixedTopLeft >= 0 && FixedTopLeft < n_Vertices) {
            Positions[FixedTopLeft] = RestPositions[FixedTopLeft];
            Velocities[FixedTopLeft] = glm::vec3(0.f);
        }
        if (FixedTopRight >= 0 && FixedTopRight < n_Vertices) {
            Positions[FixedTopRight] = RestPositions[FixedTopRight];
            Velocities[FixedTopRight] = glm::vec3(0.f);
        }
    }

    void FEMSystem2D::resetRectangularCloth(float width, float height, int segmentsX, int segmentsY) {        
        std::vector<glm::vec3> vertices;
        vertices.clear();
        float dx = width / segmentsX;
        float dy = height / segmentsY;
        
        for (int row = 0; row <= segmentsY; row++) {
            for (int col = 0; col <= segmentsX; col++) {
                float x = col * dx;
                float y = 0.f;
                float z = row * dy;
                vertices.push_back(glm::vec3(x, y, z));
            }
        }
        
        int topRow = segmentsY;
        FixedTopLeft = topRow * (segmentsX + 1);
        FixedTopRight = topRow * (segmentsX + 1) + segmentsX;
        
        std::vector<int> indices;
        indices.clear();
        for (int row = 0; row < segmentsY; row++) {
            for (int col = 0; col < segmentsX; col++) {
                int i0 = row * (segmentsX + 1) + col;
                int i1 = row * (segmentsX + 1) + col + 1;
                int i2 = (row + 1) * (segmentsX + 1) + col;
                int i3 = (row + 1) * (segmentsX + 1) + col + 1;
                
                indices.push_back(i0);
                indices.push_back(i1);
                indices.push_back(i2);
                
                indices.push_back(i1);
                indices.push_back(i3);
                indices.push_back(i2);
            }
        }
        
        reset(indices, vertices);
        
        applyFixedTopConstraints();
    }

    void FEMSystem2D::reset() {
        Positions = RestPositions;
        Velocities.assign(n_Vertices, glm::vec3(0.f));
        applyFixedTopConstraints();
    }

    void FEMSystem2D::simulateTimestep(float dt, glm::vec3 f_ext) {
        forces.assign(n_Vertices, glm::vec3(0.f));
        for (int i = 0; i < n_Triangles; i++) {
            int i0 = Indices[i * 3];
            int i1 = Indices[i * 3 + 1];
            int i2 = Indices[i * 3 + 2];
            glm::mat2x3 Ds{
                Positions[i1] - Positions[i0],
                Positions[i2] - Positions[i0]
            };
            glm::mat2x3 F = Ds * inv_E[i];
            glm::mat2 G = (glm::transpose(F) * F - glm::mat2(1.f)) / 2.f;
            glm::mat2 S = 2 * mu * G + lambda * (G[0][0] + G[1][1]) * glm::mat2(1.f);
            glm::mat2x3 P = F * S;
            glm::mat2x3 f = -Area[i] * P * glm::transpose(inv_E[i]);
            forces[i0] -= f[0] + f[1];
            forces[i1] += f[0];
            forces[i2] += f[1];
        }
        integrateTimestep(dt, f_ext);
    }

    void FEMSystem2D::integrateTimestep(float dt, glm::vec3 f_ext) {
        applyFixedTopConstraints();
        
        if (KinematicVertex >= 0 && KinematicVertex < n_Vertices) {
            Positions[KinematicVertex] = KinematicPosition;
            Velocities[KinematicVertex] = glm::vec3(0.f);
        }

        for (int i = 0; i < n_Vertices; i++) {
            if (i == KinematicVertex || i == FixedTopLeft || i == FixedTopRight) {
                continue;
            }
            forces[i] += f_ext * Masses[i];
            Velocities[i] += forces[i] * dt / Masses[i];
            Positions[i] += Velocities[i] * dt;
            Velocities[i] *= damping;
            // if (Positions[i].y < -5.f) {
            //     Positions[i].y = -5.f;
            //     Velocities[i].y *= -0.5f;
            // }
        }
    }
}