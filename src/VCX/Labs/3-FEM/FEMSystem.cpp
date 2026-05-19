#include "Labs/3-FEM/FEMSystem.h"

namespace VCX::Labs::FEM {
    void SVD(const glm::mat3& F, glm::mat3& U, glm::vec3& sigma, glm::mat3& V) {
        Eigen::Map<const Eigen::Matrix3f> eigenF(&F[0][0]);

        Eigen::JacobiSVD<Eigen::Matrix3f> svd(eigenF, Eigen::ComputeFullU | Eigen::ComputeFullV);

        Eigen::Matrix3f eigenU = svd.matrixU();
        Eigen::Vector3f eigenSigma = svd.singularValues();
        Eigen::Matrix3f eigenV = svd.matrixV();

        if (eigenU.determinant() < 0.f) {
            eigenU.col(2) *= -1.f;
            eigenSigma(2) *= -1.f;
        }
        if (eigenV.determinant() < 0.f) {
            eigenV.col(2) *= -1.f;
            eigenSigma(2) *= -1.f;
        }

        for (int i = 0; i < 3; i ++) {
            for (int j = 0; j < 3; j ++) {
                U[i][j] = eigenU(j, i);
                V[i][j] = eigenV(j, i);
            }
            sigma[i] = eigenSigma(i);
        }
    }

    void extractRotation(glm::mat3 F, glm::mat3& R, glm::mat3& S) {
        glm::mat3 U, V;
        glm::vec3 sigma;
        SVD(F, U, sigma, V);
        R = U * glm::transpose(V);
        glm::mat3 Sigma(0.f);
        Sigma[0][0] = sigma[0]; Sigma[1][1] = sigma[1]; Sigma[2][2] = sigma[2];
        S = V * Sigma * glm::transpose(V);
    }

    void FEMSystem::reset(const std::vector<int> & tet_index, const std::vector<glm::vec3> & tet_vertex_positions, float drop_height) {
        RestPositions = tet_vertex_positions;
        Indices = tet_index;
        Positions = RestPositions;
        n_Vertices = Positions.size();
        for (int i = 0; i < n_Vertices; i ++) {
            Positions[i].y += drop_height;
        }
        n_Tets = Indices.size() >> 2;
        Masses = std::vector<float>(n_Vertices, 0.f);
        Velocities = std::vector<glm::vec3>(n_Vertices, glm::vec3(0.f));
        KinematicVertex = -1;
        KinematicPosition = glm::vec3(0.f);
        E.clear();
        inv_E.clear();
        Volumes.clear();
        F_p.assign(n_Tets, glm::mat3(1.f));
        F_prev.assign(n_Tets, glm::mat3(1.f));
        float density = 1.f;
        for (int i = 0; i < n_Tets; i ++) {
            E.push_back(glm::mat3{
                RestPositions[Indices[i << 2 | 1]] - RestPositions[Indices[i << 2]],
                RestPositions[Indices[i << 2 | 2]] - RestPositions[Indices[i << 2]],
                RestPositions[Indices[i << 2 | 3]] - RestPositions[Indices[i << 2]]
            });
            inv_E.push_back(glm::inverse(E[i]));
            Volumes.push_back(glm::abs(glm::determinant(E[i])) / 6.f);
            for (int j = 0; j < 4; j ++) {
                Masses[Indices[i << 2 | j]] += Volumes[i] / 4.f * density;
            }
        }
        forces.resize(n_Vertices);
    }

    void FEMSystem::reset(float drop_height) {
        for (int i = 0; i < n_Vertices; i ++) {
            Positions[i] = RestPositions[i] + glm::vec3(0.f, drop_height, 0.f);
            Velocities[i] = glm::vec3(0.f);
        }
        F_p.assign(n_Tets, glm::mat3(1.f));
        F_prev.assign(n_Tets, glm::mat3(1.f));
    }

    glm::mat3 FEMSystem::computeStress(glm::mat3 F, Method method) {
        if (method == StVK) {
            glm::mat3 G = (glm::transpose(F) * F - glm::mat3(1.f)) / 2.f;
            glm::mat3 S = 2 * mu * G + lambda * (G[0][0] + G[1][1] + G[2][2]) * glm::mat3(1.f);
            return F * S;
        } else if (method == Neo_Hookean) {
            float J = glm::determinant(F);
            glm::mat3 inv_F_T = glm::transpose(glm::inverse(F));
            return mu * (F - inv_F_T) + lambda * glm::log(J) * inv_F_T;
        } else if (method == Corotated) {
            glm::mat3 R, S;
            extractRotation(F, R, S);
            glm::mat3 E = S - glm::mat3(1.f);
            return R * (2 * mu * E + lambda * (E[0][0] + E[1][1] + E[2][2]) * glm::mat3(1.f));
        }
    }

    void FEMSystem::simulateTimestep(float dt, Method method, bool usePlastic, glm::vec3 f_ext) {
        for (int i = 0; i < n_Vertices; i ++) {
            forces[i] = glm::vec3(0.f);
        }
        for (int i = 0; i < n_Tets; i ++) {
            glm::mat3 F = glm::mat3{
                Positions[Indices[i << 2 | 1]] - Positions[Indices[i << 2]],
                Positions[Indices[i << 2 | 2]] - Positions[Indices[i << 2]],
                Positions[Indices[i << 2 | 3]] - Positions[Indices[i << 2]]
            } * inv_E[i];
            glm::mat3 P;
            if (!usePlastic) {
                P = computeStress(F, method);
                F_prev[i] = glm::mat3(1.f);
                F_p[i] = glm::mat3(1.f);
            } else {
                glm::mat3 F_e_trial = F * glm::inverse(F_p[i]);
                glm::mat3 U, V;
                glm::vec3 sigma;
                SVD(F_e_trial, U, sigma, V);
                
                glm::vec3 sigma_new = glm::clamp(sigma, sigma_min, sigma_max);
                // glm::vec3 sigma_new = glm::min(sigma, glm::vec3(sigma_max, sigma_max, sigma_max));
                
                glm::mat3 Sigma_new(0.f);
                Sigma_new[0][0] = sigma_new.x;
                Sigma_new[1][1] = sigma_new.y;
                Sigma_new[2][2] = sigma_new.z;
                glm::mat3 F_e = U * Sigma_new * glm::transpose(V);
                F_p[i] = glm::inverse(F_e) * F;
                
                glm::mat3 dF = (F - F_prev[i]) / dt;
                glm::mat3 P_viscous = eta * dF;
                F_prev[i] = F;
                
                glm::mat3 P_e = computeStress(F_e, method);
                P = P_e * glm::transpose(glm::inverse(F_p[i])) + P_viscous;
            }
            glm::mat3 f = -Volumes[i] * P * glm::transpose(inv_E[i]);
            forces[Indices[i << 2]] -= f[0] + f[1] + f[2];
            forces[Indices[i << 2 | 1]] += f[0];
            forces[Indices[i << 2 | 2]] += f[1];
            forces[Indices[i << 2 | 3]] += f[2];
        }
        integrateTimestep(dt, f_ext);
    }

    void FEMSystem::integrateTimestep(float dt, glm::vec3 f_ext) {
        if (KinematicVertex >= 0 && KinematicVertex < n_Vertices) {
            Positions[KinematicVertex] = KinematicPosition;
            Velocities[KinematicVertex] = glm::vec3(0.f);
        }
        for (int i = 0; i < n_Vertices; i ++) {
            if (i == KinematicVertex) {
                Positions[i] = KinematicPosition;
                Velocities[i] = glm::vec3(0.f);
                continue;
            }
            forces[i] += f_ext * Masses[i];
            Velocities[i] += forces[i] * dt / Masses[i];
            Positions[i] += Velocities[i] * dt;
            Velocities[i] *= damping;

            if (Positions[i].y < -5.f) {
                Positions[i].y = -5.f;
                if (Velocities[i].y < 0) {
                    Velocities[i].y *= -.7f;
                    Velocities[i].x *= .8f;
                    Velocities[i].z *= .8f;
                }
            }
        }
    }
}