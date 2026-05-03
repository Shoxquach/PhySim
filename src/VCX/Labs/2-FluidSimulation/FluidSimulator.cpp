#include "FluidSimulator.h"

namespace VCX::Labs::Fluid {
    void Simulator::integrateParticles(float timeStep) {
        for (int i = 0; i < m_iNumSpheres; i++) {
            m_particleVel[i] += gravity * timeStep;
            m_particlePos[i] += m_particleVel[i] * timeStep;
        }
    }
    void Simulator::pushParticlesApart(int numIters) {
        while (numIters--) {
            for (int i = 0; i < m_iNumCells; i ++) {
                m_hashtableindex[i] = 0;
            }
            for (int i = 0; i < m_iNumSpheres; i ++) {
                glm::vec3 vox_coord = (m_particlePos[i] + boundary_size) * m_fInvSpacing;
                vox_coord = glm::clamp(vox_coord, glm::vec3(.0f), glm::vec3(resolution - 1.1f)) + 1.f;
                glm::ivec3 vox_id = glm::floor(vox_coord);
                m_hashtableindex[index2GridOffset(vox_id)] ++;
            }
            for (int i = 0; i < m_iNumCells; i ++) {
                m_hashtableindex[i + 1] += m_hashtableindex[i];
            }
            m_hashtableindex[m_iNumCells] = m_iNumSpheres;
            for (int i = 0; i < m_iNumSpheres; i ++) {
                glm::vec3 vox_coord = (m_particlePos[i] + boundary_size) * m_fInvSpacing;
                vox_coord = glm::clamp(vox_coord, glm::vec3(.0f), glm::vec3(resolution - 1.1f)) + 1.f;  
                glm::ivec3 vox_id = glm::floor(vox_coord);
                m_hashtable[-- m_hashtableindex[index2GridOffset(vox_id)]] = i;
            }
            for (int i = 0; i < m_iNumSpheres; i++) {
                glm::vec3 vox_coord = (m_particlePos[i] + boundary_size) * m_fInvSpacing;
                vox_coord = glm::clamp(vox_coord, glm::vec3(.0f), glm::vec3(resolution - 1.1f)) + 1.f;
                glm::ivec3 vox_id = glm::floor(vox_coord);
                for (int dx = -1; dx <= 1; dx ++) {
                    for (int dy = -1; dy <= 1; dy ++) {
                        for (int dz = -1; dz <= 1; dz ++) {
                            glm::ivec3 nid = vox_id + glm::ivec3(dx, dy, dz);
                            if (nid.x < 0 || nid.x >= m_iCellX || nid.y < 0 || nid.y >= m_iCellY || nid.z < 0 || nid.z >= m_iCellZ) {
                                continue;
                            }
                            int index_offset = index2GridOffset(nid);
                            for (int k = m_hashtableindex[index_offset]; k < m_hashtableindex[index_offset + 1]; k ++) {
                                int j = m_hashtable[k];
                                if (i == j) continue;
                                float d = glm::length(m_particlePos[i] - m_particlePos[j]);
                                if (d < m_particleRadius * 2.f && d > 1e-3f) {
                                    glm::vec3 r = m_particlePos[i] - m_particlePos[j];
                                    glm::vec3 s = (m_particleRadius * 2.f - d) * r / d;
                                    m_particlePos[i] += s / 2.f;
                                    m_particlePos[j] -= s / 2.f;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    void Simulator::handleParticleCollisions(glm::vec3 obstaclePos, float obstacleRadius, glm::vec3 obstacleVel) {
        for (int i = 0; i < m_iNumSpheres; i++) {
            for (int j = 0; j < 3; j ++) {
                if (m_particlePos[i][j] < -boundary_size) {
                    m_particlePos[i][j] = -boundary_size;
                    m_particleVel[i][j] = glm::max(m_particleVel[i][j], 0.f);
                } else if (m_particlePos[i][j] > boundary_size) {
                    m_particlePos[i][j] = boundary_size;
                    m_particleVel[i][j] = glm::min(m_particleVel[i][j], 0.f);
                }
            }
            glm::vec3 r = m_particlePos[i] - obstaclePos;
            float d = glm::length(r);
            if (d < obstacleRadius && d > 1e-3f) {
                glm::vec3 v_rel = m_particleVel[i] - obstacleVel;
                glm::vec3 normal = r / d;
                float v_n = glm::dot(v_rel, normal);
                if (v_n < 0.f) {
                    m_particleVel[i] = obstacleVel;
                }
                m_particlePos[i] = obstaclePos + normal * obstacleRadius;
            }
        }
    }
    void Simulator::updateParticleDensity() {
        for (int i = 0; i < m_iNumCells; i++) {
            m_particleDensity[i] = 0.f;
        }
        for (int i = 0; i < m_iNumSpheres; i++) {
            glm::vec3 vox_coord = (m_particlePos[i] + boundary_size) * m_fInvSpacing;
            vox_coord = glm::clamp(vox_coord, glm::vec3(.0f), glm::vec3(resolution - 1.1f)) + 1.f;
            glm::vec3 vox_id = glm::floor(vox_coord);
            glm::vec3 rd = vox_coord - vox_id;
            for (int j = 0; j < 8; j ++) {
                int idx = vox_id[0], idy = vox_id[1], idz = vox_id[2];
                float weight = 1.f;
                if (j & 1) idx ++, weight *= rd[0];
                else weight *= 1 - rd[0];
                if (j & 2) idy ++, weight *= rd[1];
                else weight *= 1 - rd[1];
                if (j & 4) idz ++, weight *= rd[2];
                else weight *= 1 - rd[2];
                if (idx < 0 || idx >= m_iCellX || idy < 0 || idy >= m_iCellY || idz < 0 || idz >= m_iCellZ) {
                    continue;
                }
                if (m_type[index2GridOffset(idx, idy, idz)] == SOLID_CELL) {
                    continue;
                }
                m_particleDensity[index2GridOffset(idx, idy, idz)] += weight;
            }
        }
    }

    void Simulator::transferVelocities(bool toGrid, float flipRatio) {
        if (toGrid) {
            for (int i = 1; i < m_iCellX - 1; i++) {
                for (int j = 1; j < m_iCellY - 1; j++) {
                    for (int k = 1; k < m_iCellZ - 1; k++) {
                        int offset = index2GridOffset(i, j, k);
                        glm::vec3 center = (glm::vec3(i, j, k) - 1.f) * m_h - boundary_size;
                        if (glm::distance(center, sphere_center) < sphere_radius) {
                            m_s[offset] = 0.f; // Mark as solid cell to prevent fluid from entering sphere
                        } else {
                            m_s[offset] = 1.f; // Mark as fluid cell
                        }
                    }
                }
             }
            for (int i = 0; i < m_iNumCells; i++) {
                m_vel[i] = glm::vec3(0.f);
                m_near_num[i] = glm::vec3(0.f);
                if (m_s[i] < 0.5f) {
                    m_type[i] = SOLID_CELL;
                } else {
                    m_type[i] = EMPTY_CELL;
                }
            }

            for (int i = 0; i < m_iNumSpheres; i++) {
                for (int j = 0; j < 3; j++) {
                    glm::vec3 adjust_pos = m_particlePos[i];
                    adjust_pos -= m_h / 2.f;
                    adjust_pos[j] += m_h / 2.f;

                    glm::vec3 vox_coord = (adjust_pos + boundary_size) * m_fInvSpacing;
                    vox_coord = glm::clamp(vox_coord, glm::vec3(.0f), glm::vec3(resolution - 1.1f)) + 1.f;
                    
                    glm::ivec3 vox_id = glm::floor(vox_coord);
                    glm::vec3 rd = vox_coord - glm::vec3(vox_id);

                    for (int k = 0; k < 8; k++) {
                        int idx = vox_id.x + ((k & 1) ? 1 : 0);
                        int idy = vox_id.y + ((k & 2) ? 1 : 0);
                        int idz = vox_id.z + ((k & 4) ? 1 : 0);

                        float weight = 1.0f;
                        weight *= (k & 1) ? rd.x : (1.0f - rd.x);
                        weight *= (k & 2) ? rd.y : (1.0f - rd.y);
                        weight *= (k & 4) ? rd.z : (1.0f - rd.z);

                        int offset = index2GridOffset(idx, idy, idz);
                        m_vel[offset][j] += weight * m_particleVel[i][j];
                        m_near_num[offset][j] += weight;

                        if (idx < 0 || idx >= m_iCellX || idy < 0 || idy >= m_iCellY || idz < 0 || idz >= m_iCellZ) {
                            continue;
                        }
                        if (m_type[offset] == EMPTY_CELL && weight > 1e-4f) {
                            m_type[offset] = FLUID_CELL;
                        }
                    }
                }
            }
            for (int i = 0; i < m_iNumCells; i++) {
                for (int j = 0; j < 3; j++) {
                    if (m_near_num[i][j] > 0.f) {
                        m_vel[i][j] /= m_near_num[i][j];
                    }
                }
            }
        } else {
            for (int i = 0; i < m_iNumSpheres; i++) {
                glm::vec3 PIC_vel(0.f);
                glm::vec3 FLIP_correction(0.f);
                float total_weight = 0.f;

                for (int j = 0; j < 3; j++) {
                    glm::vec3 adjust_pos = m_particlePos[i] - m_h / 2.f;
                    adjust_pos[j] += m_h / 2.f;

                    glm::vec3 vox_coord = (adjust_pos + boundary_size) * m_fInvSpacing;
                    vox_coord = glm::clamp(vox_coord, glm::vec3(.0f), glm::vec3(resolution - 1.1f)) + 1.f;
                    
                    glm::ivec3 vox_id = glm::floor(vox_coord);
                    glm::vec3 rd = vox_coord - glm::vec3(vox_id);

                    for (int k = 0; k < 8; k++) {
                        int idx = vox_id.x + ((k & 1) ? 1 : 0);
                        int idy = vox_id.y + ((k & 2) ? 1 : 0);
                        int idz = vox_id.z + ((k & 4) ? 1 : 0);

                        float weight = 1.0f;
                        weight *= (k & 1) ? rd.x : (1.0f - rd.x);
                        weight *= (k & 2) ? rd.y : (1.0f - rd.y);
                        weight *= (k & 4) ? rd.z : (1.0f - rd.z);

                        if (idx < 0 || idx >= m_iCellX || idy < 0 || idy >= m_iCellY || idz < 0 || idz >= m_iCellZ) {
                            continue;
                        }
                        int offset = index2GridOffset(idx, idy, idz);
                        PIC_vel[j] += weight * m_vel[offset][j];
                        FLIP_correction[j] += weight * (m_vel[offset][j] - m_pre_vel[offset][j]);
                        total_weight += weight;
                    }
                }
                if (total_weight > 1e-4f) {
                    PIC_vel /= total_weight;
                    FLIP_correction /= total_weight;
                } else {
                    PIC_vel = m_particleVel[i];
                    FLIP_correction = glm::vec3(0.f);
                }
                glm::vec3 FLIP_vel = m_particleVel[i] + FLIP_correction;
                m_particleVel[i] = PIC_vel * (1.0f - flipRatio) + FLIP_vel * flipRatio;
            }
        }
    }
    void Simulator::solveIncompressibility(int numIters, float dt, float overRelaxation, bool compensateDrift) {
        m_pre_vel = m_vel;

        const int Nx = m_iCellY * m_iCellZ;
        const int Ny = m_iCellZ;
        const int Nz = 1;

        for (int i = 0; i < m_iNumCells; i++) {
            m_p[i] = 0.f;
        }

        for (int iter = 0; iter < numIters; iter++) {
            for (int i = 1; i < m_iCellX - 1; i++) {
                for (int j = 1; j < m_iCellY - 1; j++) {
                    for (int k = 1; k < m_iCellZ - 1; k++) {
                        int center = index2GridOffset(i, j, k);

                        if (m_type[center] != FLUID_CELL) continue;

                        float sL = m_s[center - Nx];
                        float sR = m_s[center + Nx];
                        float sB = m_s[center - Ny];
                        float sT = m_s[center + Ny];
                        float sF = m_s[center - Nz];
                        float sN = m_s[center + Nz];
                        
                        float sSum = sL + sR + sB + sT + sF + sN;
                        if (sSum < 0.5f) continue;

                        float div = m_vel[center + Nx][0] - m_vel[center][0] +
                                    m_vel[center + Ny][1] - m_vel[center][1] +
                                    m_vel[center + Nz][2] - m_vel[center][2];

                        if (compensateDrift && m_particleRestDensity > 0.0f) {
                            float k_stiff = .3f;
                            float compression = m_particleDensity[center] - m_particleRestDensity;
                            if (compression > 0.0f) {
                                div -= k_stiff * compression;
                            }
                        }

                        float p = -div / sSum;
                        p *= overRelaxation;

                        m_vel[center].x      -= sL * p;
                        m_vel[center + Nx].x += sR * p;
                        m_vel[center].y      -= sB * p;
                        m_vel[center + Ny].y += sT * p;
                        m_vel[center].z      -= sF * p;
                        m_vel[center + Nz].z += sN * p;
                        
                        m_p[center] += p * (m_h / dt);
                    }
                }
            }
        }
    }

    void Simulator::solveIncompressibility_CG(int numIters, float dt, bool compensateDrift) {
        m_pre_vel = m_vel;

        std::vector<int> gridToMat(m_iNumCells, -1);
        std::vector<int> matToGrid;

        for (int i = 0; i < m_iNumCells; ++i) {
            if (m_type[i] == FLUID_CELL) {
                gridToMat[i] = matToGrid.size();
                matToGrid.push_back(i);
            }
        }

        const int n = matToGrid.size();
        if (n == 0) return;

        Eigen::VectorXf b(n);
        Eigen::SparseMatrix<float> A(n, n);
        std::vector<Eigen::Triplet<float>> triplets;

        const int Nx = m_iCellY * m_iCellZ;
        const int Ny = m_iCellZ;
        const int Nz = 1;

        // Only iterate over interior cells (excluding boundary)
        for (int i = 1; i < m_iCellX - 1; i++) {
            for (int j = 1; j < m_iCellY - 1; j++) {
                for (int k = 1; k < m_iCellZ - 1; k++) {
                    int center = index2GridOffset(i, j, k);
                    int r = gridToMat[center];
                    if (m_type[center] != FLUID_CELL) {
                        continue;
                    }
                    float sL = m_s[center - Nx];
                    float sR = m_s[center + Nx];
                    float sB = m_s[center - Ny];
                    float sT = m_s[center + Ny];
                    float sF = m_s[center - Nz];
                    float sN = m_s[center + Nz];

                    float sSum = sL + sR + sB + sT + sF + sN;
                    if (sSum < 0.5f) {
                        continue;
                    }
                    float div = m_vel[center + Nx].x - m_vel[center].x +
                                m_vel[center + Ny].y - m_vel[center].y +
                                m_vel[center + Nz].z - m_vel[center].z;
                    if (compensateDrift && m_particleRestDensity > 0.0f) {
                        float k_stiff = 0.3f;
                        float compression = m_particleDensity[center] - m_particleRestDensity;
                        if (compression > 0.0f) {
                            div -= k_stiff * compression;
                        }
                    }
                    b(r) = -div;
                    triplets.push_back(Eigen::Triplet<float>(r, r, sSum));
                    if (m_type[center - Nx] == FLUID_CELL) {
                        triplets.push_back(Eigen::Triplet<float>(r, gridToMat[center - Nx], -sL));
                    }
                    if (m_type[center + Nx] == FLUID_CELL) {
                        triplets.push_back(Eigen::Triplet<float>(r, gridToMat[center + Nx], -sR));
                    }
                    if (m_type[center - Ny] == FLUID_CELL) {
                        triplets.push_back(Eigen::Triplet<float>(r, gridToMat[center - Ny], -sB));
                    }
                    if (m_type[center + Ny] == FLUID_CELL) {
                        triplets.push_back(Eigen::Triplet<float>(r, gridToMat[center + Ny], -sT));
                    }
                    if (m_type[center - Nz] == FLUID_CELL) {
                        triplets.push_back(Eigen::Triplet<float>(r, gridToMat[center - Nz], -sF));
                    }
                    if (m_type[center + Nz] == FLUID_CELL) {
                        triplets.push_back(Eigen::Triplet<float>(r, gridToMat[center + Nz], -sN));
                    }
                }
            }
        }
        A.setFromTriplets(triplets.begin(), triplets.end());

        Eigen::ConjugateGradient<Eigen::SparseMatrix<float>, Eigen::Lower|Eigen::Upper> solver;
        solver.setMaxIterations(numIters);
        solver.setTolerance(1e-5);
        solver.compute(A);
        Eigen::VectorXf p = solver.solve(b);

        // Apply pressure gradient to fluid cells only
        // Use the same logic as Gauss-Seidel: only apply to faces adjacent to fluid
        for (int r = 0; r < n; ++r) {
            int center = matToGrid[r];
            float press = p(r);

            m_p[center] = press * (m_h / dt);

            // Only apply pressure gradient on faces that are not adjacent to solid boundary
            // sX > 0.5f ensures the neighbor is fluid (not solid)
            float sL = m_s[center - Nx];
            float sR = m_s[center + Nx];
            float sB = m_s[center - Ny];
            float sT = m_s[center + Ny];
            float sF = m_s[center - Nz];
            float sN = m_s[center + Nz];

            if (sL > 0.5f) m_vel[center].x -= press;
            if (sR > 0.5f) m_vel[center + Nx].x += press;
            if (sB > 0.5f) m_vel[center].y -= press;
            if (sT > 0.5f) m_vel[center + Ny].y += press;
            if (sF > 0.5f) m_vel[center].z -= press;
            if (sN > 0.5f) m_vel[center + Nz].z += press;
        }

        // Enforce solid boundary conditions: zero velocity on solid cells
        for (int i = 0; i < m_iCellX; i++) {
            for (int j = 0; j < m_iCellY; j++) {
                for (int k = 0; k < m_iCellZ; k++) {
                    int center = index2GridOffset(i, j, k);
                    if (m_s[center] < 0.5f) {
                        // Solid cell - zero out all velocities stored at this cell
                        m_vel[center] = glm::vec3(0.0f);
                    }
                }
            }
        }
    }

    void Simulator::updateParticleColors(int type) {
        const glm::vec3 blue(0.1f, 0.3f, 0.8f);
        const glm::vec3 cyan(0.2f, 0.7f, 0.9f);
        const glm::vec3 green(0.2f, 0.8f, 0.3f);
        const glm::vec3 yellow(0.9f, 0.9f, 0.2f);
        const glm::vec3 red(0.9f, 0.2f, 0.2f);

        for (int i = 0; i < m_iNumSpheres; i++) {
            float t = 0.0f;

            glm::vec3 vox_coord = (m_particlePos[i] + boundary_size) * m_fInvSpacing;
            vox_coord = glm::clamp(vox_coord, glm::vec3(.0f), glm::vec3(resolution - 1.1f)) + 1.f;
            glm::ivec3 vox_id = glm::floor(vox_coord);
            int cellIdx = index2GridOffset(vox_id);

            switch (type) {
                case 0: {
                    float speed = glm::length(m_particleVel[i]);
                    t = glm::smoothstep(0.f, 3.f, speed);
                    m_particleColor[i] = getGradientColor(t, blue, cyan, green, yellow, red);
                    break;
                }
                case 1: {
                    float density = m_particleDensity[cellIdx];
                    float densityRatio = density / m_particleRestDensity;
                    t = glm::smoothstep(0.f, 2.f, densityRatio);
                    m_particleColor[i] = getGradientColor(t, blue, cyan, green, yellow, red); 
                    break;
                }
                case 2: {
                    float pressure = m_p[cellIdx];
                    t = glm::smoothstep(0.f, 10.f, pressure);
                    
                    m_particleColor[i] = getGradientColor(t, blue, cyan, green, yellow, red);
                    break;
                }
                case 3: {
                    m_particleColor[i] = m_particlePos[i] + boundary_size;
                    break;
                }
                case 4: {
                    glm::vec3 dir = glm::normalize(m_particleVel[i]);
                    m_particleColor[i] = glm::abs(dir);
                    break;
                }
            }
        }
    }
    
    // Multi-stop gradient interpolation
    glm::vec3 Simulator::getGradientColor(float t, 
        const glm::vec3& c1, const glm::vec3& c2, 
        const glm::vec3& c3, const glm::vec3& c4, const glm::vec3& c5) {
        t = glm::clamp(t, 0.0f, 1.0f);
        if (t < 0.25f) {
            return glm::mix(c1, c2, t * 4.0f);
        } else if (t < 0.5f) {
            return glm::mix(c2, c3, (t - 0.25f) * 4.0f);
        } else if (t < 0.75f) {
            return glm::mix(c3, c4, (t - 0.5f) * 4.0f);
        } else {
            return glm::mix(c4, c5, (t - 0.75f) * 4.0f);
        }
    }
    
    // Simplified 3-stop gradient
    glm::vec3 Simulator::getGradientColor(float t, 
        const glm::vec3& c1, const glm::vec3& c2, const glm::vec3& c3) {
        t = glm::clamp(t, 0.0f, 1.0f);
        if (t < 0.5f) {
            return glm::mix(c1, c2, t * 2.0f);
        } else {
            return glm::mix(c2, c3, (t - 0.5f) * 2.0f);
        }
    }
    
    inline bool Simulator::isValidVelocity(int i, int j, int k, int dir) {

    }
    inline int Simulator::index2GridOffset(glm::ivec3 index) {
        return index.x * m_iCellY * m_iCellZ + index.y * m_iCellZ + index.z;
    }
    inline int Simulator::index2GridOffset(int x, int y, int z) {
        return x * m_iCellY * m_iCellZ + y * m_iCellZ + z;
    }

}