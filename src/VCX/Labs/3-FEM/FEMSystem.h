#pragma once

#include <vector>
#include <glm/glm.hpp>
#include <Eigen/Dense>

namespace VCX::Labs::FEM {
    enum Method { StVK, Neo_Hookean, Corotated };

    struct FEMSystem {
        int n_Vertices, n_Tets;
        std::vector<glm::vec3> RestPositions;
        std::vector<glm::vec3> Positions;
        std::vector<int> Indices;
        std::vector<float> Masses;
        std::vector<glm::vec3> Velocities;
        std::vector<glm::mat3> E, inv_E;
        std::vector<float> Volumes;
        int KinematicVertex = -1;
        glm::vec3 KinematicPosition { 0.f };
        float mu = 8000.f, lambda = 5000.f;
        float damping = 0.999f;
        std::vector<glm::vec3> forces;
        
        // Reset the the whole object.
        void reset(const std::vector<int> & tet_index, const std::vector<glm::vec3> & tet_vertex_positions, float drop_height = 0.f);
        
        // Only reset the positions to the original state.
        void reset(float drop_height = 0.f);

        void setKinematicVertex(int vertex, glm::vec3 position) {
            if (vertex < 0 || vertex >= n_Vertices) {
                KinematicVertex = -1;
                return;
            }
            if (position.y < -5.f) position.y = -5.f;
            KinematicVertex = vertex;
            KinematicPosition = position;
            Positions[vertex] = position;
            Velocities[vertex] = glm::vec3(0.f);
        }

        void clearKinematicVertex() {
            KinematicVertex = -1;
        }

        FEMSystem() = default;

        std::vector<glm::mat3> F_p;
        std::vector<glm::mat3> F_prev;
        float eta = 0.5f, sigma_max = 1.2f, sigma_min = 0.8f;

        glm::mat3 computeStress(glm::mat3 F, Method method);
        void simulateTimestep(float dt, Method method, bool usePlastic, glm::vec3 f_ext = glm::vec3(0.f, -9.8f, 0.f));
        void integrateTimestep(float dt, glm::vec3 f_ext);
    };
}