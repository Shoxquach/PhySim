#pragma once

#include <vector>
#include <glm/glm.hpp>
#include <Eigen/Dense>

namespace VCX::Labs::FEM {
    struct FEMSystem2D {
        int n_Vertices, n_Triangles;
        std::vector<glm::vec3> RestPositions;
        std::vector<glm::vec3> Positions;
        std::vector<float> Masses;
        std::vector<glm::vec3> Velocities;
        std::vector<glm::vec3> forces;
        std::vector<int> Indices;
        std::vector<glm::mat2> E, inv_E;
        std::vector<float> Area;
        int KinematicVertex = -1;
        glm::vec3 KinematicPosition { 0.f };
        float mu, lambda;
        float damping = 0.999f;

        int FixedTopLeft = -1;
        int FixedTopRight = -1;
        
        // Reset the the whole object with given mesh.
        void reset(const std::vector<int> & tri_index, const std::vector<glm::vec3> & tri_vertex_positions);

        // Initialize a rectangular cloth hanging vertically.
        // width: cloth width in X direction
        // height: cloth height in Y direction (positive = up)
        // segmentsX: number of horizontal subdivisions
        // segmentsY: number of vertical subdivisions
        // Automatically fixes the top-left and top-right corners.
        void resetRectangularCloth(float width, float height, int segmentsX, int segmentsY);

        FEMSystem2D(int resolution) {
            resetRectangularCloth(10.f, 10.f, resolution, resolution);
        }
        
        // Only reset the positions to the original state.
        // Re-applies fixed constraints for rectangular cloth.
        void reset();

        void setKinematicVertex(int vertex, glm::vec3 const & position) {
            if (vertex < 0 || vertex >= n_Vertices) {
                KinematicVertex = -1;
                return;
            }
            KinematicVertex = vertex;
            KinematicPosition = position;
            Positions[vertex] = position;
            Velocities[vertex] = glm::vec3(0.f);
        }

        void clearKinematicVertex() {
            KinematicVertex = -1;
        }
        void simulateTimestep(float dt, glm::vec3 f_ext = glm::vec3(0.f, -9.8f, 0.f));
        void integrateTimestep(float dt, glm::vec3 f_ext);

        void applyFixedTopConstraints();
    };
}