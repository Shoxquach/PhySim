#pragma once

#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <glm/glm.hpp>
#include <iostream>
#include <utility>
#include <vector>
#include <random>

namespace VCX::Labs::Fluid {
    const int SOLID_CELL = 0, FLUID_CELL = 1, EMPTY_CELL = 2;
    struct Simulator {
        std::vector<glm::vec3> m_particlePos; // Particle m_particlePos
        std::vector<glm::vec3> m_particleVel; // Particle Velocity
        std::vector<glm::vec3> m_particleColor;

        float m_fRatio = .97f;
        int   m_iCellX;
        int   m_iCellY;
        int   m_iCellZ;
        float m_h;
        float m_fInvSpacing;
        int   m_iNumCells;

        int   m_iNumSpheres;
        float m_particleRadius;

        int resolution;

        std::vector<glm::vec3> m_vel;
        std::vector<glm::vec3> m_pre_vel;
        std::vector<glm::vec3> m_near_num;

        std::vector<int> m_hashtable;
        std::vector<int> m_hashtableindex;

        std::vector<float> m_p;               // Pressure array
        std::vector<float> m_s;               // 0.0 for solid cells, 1.0 for fluid cells, used to update m_type
        std::vector<int>   m_type;            // Flags array (const int EMPTY_CELL = 0; const int FLUID_CELL = 1; const int SOLID_CELL = 2;)
                                              // m_type = SOLID_CELL if m_s == 0.0;
                                              // m_type = FLUID_CELL if has particle and m_s == 1;
                                              // m_type = EMPTY_CELL if has No particle and m_s == 1;
        std::vector<float> m_particleDensity; // Particle Density per cell, saved in the grid cell
        float              m_particleRestDensity = .5f;

        glm::vec3 gravity { 0, -9.81f, 0 };
        const float boundary_size = .5f;

        glm::vec3 sphere_center { 0, .5f, 0 };
        glm::vec3 sphere_velocity { 0, 0, 0 };
        float sphere_radius = .2f;
        bool isSphereHeld = false;  // True when user is dragging the sphere
        float sphere_damping = 0.98f;  // Velocity damping when not held (1.0 = no damping, 0.0 = full stop)

        void integrateParticles(float timeStep);
        void pushParticlesApart(int numIters);
        void handleParticleCollisions(glm::vec3 obstaclePos, float obstacleRadius, glm::vec3 obstacleVel);
        void updateParticleDensity();

        void transferVelocities(bool toGrid, float flipRatio);
        void solveIncompressibility(int numIters, float dt, float overRelaxation, bool compensateDrift);
        void solveIncompressibility_CG(int numIters, float dt, bool compensateDrift);
        void updateParticleColors(int type);
        
        // Color gradient helpers
        glm::vec3   getGradientColor(float t, 
            const glm::vec3& c1, const glm::vec3& c2, 
            const glm::vec3& c3, const glm::vec3& c4, const glm::vec3& c5);
        glm::vec3   getGradientColor(float t, 
            const glm::vec3& c1, const glm::vec3& c2, const glm::vec3& c3);
        glm::vec3   estimateVorticity(int particleIdx);
        
        inline bool isValidVelocity(int i, int j, int k, int dir);
        inline int  index2GridOffset(glm::ivec3 index);
        inline int  index2GridOffset(int x, int y, int z);

        void SimulateTimestep(float const dt, int type, bool useCGSolver) {
            int   numSubSteps       = 1;
            int   numParticleIters  = 5;
            int   numPressureIters  = 30;
            bool  separateParticles = true;
            float overRelaxation    = 1.5f;
            bool  compensateDrift   = true;

            float     flipRatio = m_fRatio;

            float sdt = dt / numSubSteps;

            for (int step = 0; step < numSubSteps; step++) {
                integrateParticles(sdt);
                handleParticleCollisions(sphere_center, sphere_radius, sphere_velocity);
                if (separateParticles)
                    pushParticlesApart(numParticleIters);
                handleParticleCollisions(sphere_center, sphere_radius, sphere_velocity);
                transferVelocities(true, flipRatio);
                updateParticleDensity();
                if (useCGSolver) {
                    solveIncompressibility_CG(numPressureIters, sdt, compensateDrift);
                } else {
                    solveIncompressibility(numPressureIters, sdt, overRelaxation, compensateDrift);
                }
                transferVelocities(false, flipRatio);
            }
            updateParticleColors(type);
        }

        /*
         * Initialization: 
         * The tank is a cube with side length 2 * boundary_size, centered at the origin.
         * res determines the number of cells in each dimension.
         * The cell is (res + 2) ^ 3, and the boundary cell is solid.
         * Each cell has one particle in the center originally.
         */
        void setupScene(int res) {
            resolution = res;
            // update object member attributes
            m_iNumSpheres    = res * (res / 2) * (res / 2);
            m_iCellX         = res + 2;
            m_iCellY         = res + 2;
            m_iCellZ         = res + 2;
            m_iNumCells      = m_iCellX * m_iCellY * m_iCellZ;
            m_h              = 2 * boundary_size / float(res); // the size of each cell
            m_fInvSpacing    = 1 / m_h; // the inverse of the size of each cell
            m_particleRadius = 0.5 * m_h; // the radius of each particle

            // update particle array
            m_particlePos.clear();
            m_particlePos.resize(m_iNumSpheres, glm::vec3(0.0f));
            m_particleVel.clear();
            m_particleVel.resize(m_iNumSpheres, glm::vec3(0.0f));
            m_particleColor.clear();
            m_particleColor.resize(m_iNumSpheres, glm::vec3(1.0f));
            m_hashtable.clear();
            m_hashtable.resize(m_iNumSpheres, 0);
            m_hashtableindex.clear();
            m_hashtableindex.resize(m_iNumCells + 1, 0);

            // update grid array
            m_vel.clear();
            m_vel.resize(m_iNumCells, glm::vec3(0.0f));
            m_pre_vel.clear();
            m_pre_vel.resize(m_iNumCells, glm::vec3(0.0f));
            m_near_num.clear();
            m_near_num.resize(m_iNumCells, glm::vec3(0.0f));

            m_p.clear();
            m_p.resize(m_iNumCells, 0.0);
            m_s.clear();
            m_s.resize(m_iNumCells, 0.0);
            m_type.clear();
            m_type.resize(m_iNumCells, 0);
            m_particleDensity.clear();
            m_particleDensity.resize(m_iNumCells, 0.0f);

            // the rest density can be assigned after scene initialization

            // create particles
            int p = 0;
            std::mt19937 rng(std::random_device{}());
            std::uniform_real_distribution<float> offset_distribution(-0.5f, 0.5f);
            for (int i = 0; i < res; i++) {
                for (int j = 0; j < res / 2; j++) {
                    for (int k = 0; k < res / 2; k++) {
                        glm::vec3 random_offset(
                            offset_distribution(rng),
                            offset_distribution(rng),
                            offset_distribution(rng)
                        );
                        m_particlePos[p ++] = (glm::vec3(i, j, k) + random_offset + 0.5f) * m_h - boundary_size;
                    }
                }
            }
            // setup grid cells for tank

            for (int i = 0; i < m_iCellX; i++) {
                for (int j = 0; j < m_iCellY; j++) {
                    for (int k = 0; k < m_iCellZ; k++) {
                        float s = 1.0; // fluid
                        if (i == 0 || i == m_iCellX - 1 || j == 0 || j == m_iCellY - 1 || k == 0 || k == m_iCellZ - 1)
                            s = 0.0f; // solid
                        m_s[index2GridOffset(i, j, k)] = s;
                    }
                }
            }
        }
    };
} // namespace VCX::Labs::Fluid