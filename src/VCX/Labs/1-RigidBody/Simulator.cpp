#include "Labs/1-RigidBody/Simulator.h"

namespace VCX::Labs::RigidBody {
    void Rigid::SimulateTimestep(float const dt, glm::vec3 const & force, glm::vec3 const & torque) {
        x += v * dt;
        v += force * inv_m * dt;
        
        L += torque * dt;
        glm::mat3 R = RotationMatrix();
        glm::mat3 inv_I_world = R * inv_I * glm::transpose(R);
        glm::vec3 w = inv_I_world * L;
        q += 0.5f * glm::quat(0.f, w) * q * dt;
        q = glm::normalize(q);

        v *= Damping;
        L *= Damping;
    }

    /*
        * Impulse-based collision response.
     */
    fcl::CollisionResultf collide_detect(Rigid & rigid1, glm::vec3 dim1, Rigid & rigid2, glm::vec3 dim2) {
        auto boxA = std::make_shared<fcl::Boxf>(dim1.x, dim1.y, dim1.z);
        auto boxB = std::make_shared<fcl::Boxf>(dim2.x, dim2.y, dim2.z);

        fcl::Transform3f tfA = fcl::Transform3f::Identity();
        tfA.translation() = Eigen::Vector3f(rigid1.x.x, rigid1.x.y, rigid1.x.z);
        tfA.linear() = Eigen::Quaternionf(rigid1.q.w, rigid1.q.x, rigid1.q.y, rigid1.q.z).toRotationMatrix();

        fcl::Transform3f tfB = fcl::Transform3f::Identity();
        tfB.translation() = Eigen::Vector3f(rigid2.x.x, rigid2.x.y, rigid2.x.z);
        tfB.linear() = Eigen::Quaternionf(rigid2.q.w, rigid2.q.x, rigid2.q.y, rigid2.q.z).toRotationMatrix();

        fcl::CollisionObjectf objA(boxA, tfA);
        fcl::CollisionObjectf objB(boxB, tfB);

        fcl::CollisionRequestf request;
        request.enable_contact = true;
        request.num_max_contacts = 8;
        fcl::CollisionResultf result;

        fcl::collide(&objA, &objB, request, result);

        return result;
    }
    
    const float eps = 1e-6f, slop = 0.f;
    void collide_test(Rigid & rigid1, glm::vec3 dim1, Rigid & rigid2, glm::vec3 dim2, float restitution, float correction_factor, float mu) {
        auto result = collide_detect(rigid1, dim1, rigid2, dim2);

        if (result.isCollision()) {
            glm::vec3 normal(0.f);
            glm::vec3 pos(0.f);
            float penetration_depth = 0.f;
            // printf("Collision detected!(%zu)\n", result.numContacts());
            for (int i = 0; i < result.numContacts(); i++) {
                auto contact = result.getContact(i);
                // printf("Contact %d: Normal: (%f, %f, %f), Position: (%f, %f, %f), Penetration Depth: %f\n",
                //     i,
                //     contact.normal[0], contact.normal[1], contact.normal[2],
                //     contact.pos[0], contact.pos[1], contact.pos[2],
                //     contact.penetration_depth);
                normal += glm::vec3(contact.normal[0], contact.normal[1], contact.normal[2]);
                pos += glm::vec3(contact.pos[0], contact.pos[1], contact.pos[2]);
                penetration_depth += contact.penetration_depth;
            }
            normal = glm::normalize(normal);
            pos /= result.numContacts();
            penetration_depth /= result.numContacts();
            
            glm::mat3 R1 = rigid1.RotationMatrix();
            glm::mat3 inv_I1_world = R1 * rigid1.inv_I * glm::transpose(R1);
            glm::mat3 R2 = rigid2.RotationMatrix();
            glm::mat3 inv_I2_world = R2 * rigid2.inv_I * glm::transpose(R2);
            glm::vec3 w1 = inv_I1_world * rigid1.L;
            glm::vec3 w2 = inv_I2_world * rigid2.L;
            glm::vec3 v1 = rigid1.v + glm::cross(w1, pos - rigid1.x);
            glm::vec3 v2 = rigid2.v + glm::cross(w2, pos - rigid2.x);
            glm::vec3 v_rel = v2 - v1;
            if (glm::dot(v_rel, normal) > 0.f) {
                return;
            }
            glm::vec3 v_n = glm::dot(v_rel, normal) * normal;
            glm::vec3 v_t = v_rel - v_n;


            glm::vec3 v_n_new = -restitution * v_n;
            float alpha = 0.f;
            if (mu * (1.f + restitution) * glm::length(v_n) < glm::length(v_t)) {
                alpha = std::max(1.f - mu * (1.f + restitution) * glm::length(v_n) / glm::length(v_t), 0.f);
            }
            glm::vec3 v_t_new = alpha * v_t;

            // float v_threshold = 0.1f;
            // if (glm::length(v_rel) < v_threshold) {
            //     v_n_new = glm::vec3(0.0f);
            //     v_t_new = glm::vec3(0.0f);
            // }

            glm::vec3 v_rel_new = v_n_new + v_t_new;

            glm::vec3 r1 = pos - rigid1.x;
            glm::vec3 r2 = pos - rigid2.x;
            glm::mat3 r1_cross{
                {0.f, -r1.z, r1.y},
                {r1.z, 0.f, -r1.x},
                {-r1.y, r1.x, 0.f}
            }, r2_cross{
                {0.f, -r2.z, r2.y},
                {r2.z, 0.f, -r2.x},
                {-r2.y, r2.x, 0.f}
            };
            glm::mat3 K1 = glm::mat3(rigid1.inv_m) - r1_cross * inv_I1_world * r1_cross;
            glm::mat3 K2 = glm::mat3(rigid2.inv_m) - r2_cross * inv_I2_world * r2_cross;
            glm::mat3 K = K1 + K2 + glm::mat3(eps);
            glm::vec3 impulse = glm::inverse(K) * (v_rel_new - v_rel);

            rigid1.v -= impulse * rigid1.inv_m;
            rigid1.L -= glm::cross(r1, impulse);
            rigid2.v += impulse * rigid2.inv_m;
            rigid2.L += glm::cross(r2, impulse);

            if (penetration_depth > slop) {
                glm::vec3 correction = ((penetration_depth - slop) / (rigid1.inv_m + rigid2.inv_m)) * normal * correction_factor;
                rigid1.x -= correction * rigid1.inv_m;
                rigid2.x += correction * rigid2.inv_m;
            }
        }
    }

    void collide_multiple(std::vector<Rigid*> const & rigids, std::vector<glm::vec3> const & dims,
        float restitution, float correction_factor, float mu, bool & pause) {
        int n = rigids.size();
        int m = n;
        while (rigids[m - 1]->IsStatic && m > 0) {
            m --;
        }
        struct ContactInfo {
            int idx1, idx2;
            glm::vec3 normal;
            glm::vec3 pos;
            float penetration_depth;
        };
        std::vector<ContactInfo> contact_infos;
        std::vector<std::vector<int>> contact_indices(m);
        for (int i = 0; i < m; i ++) {
            for (int j = i + 1; j < n; j ++) {
                auto result = collide_detect(*rigids[i], dims[i], *rigids[j], dims[j]);
                if (result.isCollision()) {
                    // for (int k = 0; k < result.numContacts(); k++) {
                    //     auto contact = result.getContact(k);
                    //     glm::vec3 normal(contact.normal[0], contact.normal[1], contact.normal[2]);
                    //     glm::vec3 pos(contact.pos[0], contact.pos[1], contact.pos[2]);
                    //     float penetration_depth = contact.penetration_depth;
                    //     ContactInfo contact_info(i, j, normal, pos, penetration_depth);
                    //     contact_infos.push_back(contact_info);
                    //     contact_indices[i].push_back(contact_infos.size() - 1);
                    //     if (j < m) {
                    //         contact_indices[j].push_back(contact_infos.size() - 1);
                    //     }
                    // }
                    glm::vec3 normal(0.f);
                    glm::vec3 pos(0.f);
                    float penetration_depth = 0.f;
                    for (int k = 0; k < result.numContacts(); k++) {
                        auto contact = result.getContact(k);
                        normal += glm::vec3(contact.normal[0], contact.normal[1], contact.normal[2]);
                        pos += glm::vec3(contact.pos[0], contact.pos[1], contact.pos[2]);
                        penetration_depth += contact.penetration_depth;
                    }
                    normal = glm::normalize(normal);
                    pos /= result.numContacts();
                    penetration_depth /= result.numContacts();
                    ContactInfo contact_info{i, j, normal, pos, penetration_depth};
                    contact_infos.push_back(contact_info);
                    contact_indices[i].push_back(contact_infos.size() - 1);
                    if (j < m) {
                        contact_indices[j].push_back(contact_infos.size() - 1);
                    }
                }
            }
        }
        if (contact_infos.empty()) {
            return;
        }
        int contact_num = contact_infos.size();
        Eigen::MatrixXf A = Eigen::MatrixXf::Zero(contact_num * 3, contact_num * 3);
        Eigen::VectorXf b = Eigen::VectorXf::Zero(contact_num * 3);
        for (int i = 0; i < contact_num; i ++) {
            auto & c = contact_infos[i];
            int idx1 = c.idx1, idx2 = c.idx2;
            glm::vec3 r1 = c.pos - rigids[idx1]->x;
            glm::vec3 r2 = c.pos - rigids[idx2]->x;
            glm::mat3 R1 = rigids[idx1]->RotationMatrix();
            glm::vec3 w1 = R1 * (rigids[idx1]->inv_I * glm::transpose(R1) * rigids[idx1]->L);
            glm::mat3 R2 = rigids[idx2]->RotationMatrix();
            glm::vec3 w2 = R2 * (rigids[idx2]->inv_I * glm::transpose(R2) * rigids[idx2]->L);
            glm::vec3 v1 = rigids[idx1]->v + glm::cross(w1, r1);
            glm::vec3 v2 = rigids[idx2]->v + glm::cross(w2, r2);
            glm::vec3 v_rel = v2 - v1;
            glm::vec3 v_n = glm::dot(v_rel, c.normal) * c.normal;
            glm::vec3 v_t = v_rel - v_n;
            glm::vec3 v_n_new = -restitution * v_n;
            float alpha = 0.f;
            if (mu * (1.f + restitution) * glm::length(v_n) < glm::length(v_t)) {
                alpha = std::max(1.f - mu * (1.f + restitution) * glm::length(v_n) / glm::length(v_t), 0.f);
            }
            glm::vec3 v_t_new = alpha * v_t;
            glm::vec3 v_rel_new = v_n_new + v_t_new;
            glm::vec3 delta_v = v_rel_new - v_rel;
            for (int j = 0; j < 3; j ++) {
                b(i * 3 + j) = delta_v[j];
            }
            for (auto j : contact_indices[idx1]) {
                int sgn = idx1 == contact_infos[j].idx1 ? 1 : -1;
                glm::mat3 r1_cross{
                    {0.f, -r1.z, r1.y},
                    {r1.z, 0.f, -r1.x},
                    {-r1.y, r1.x, 0.f}
                };
                glm::mat3 K = glm::mat3(rigids[idx1]->inv_m) - r1_cross * rigids[idx1]->inv_I * r1_cross;
                for (int k = 0; k < 3; k ++) {
                    for (int l = 0; l < 3; l ++) {
                        A(i * 3 + k, j * 3 + l) += sgn * K[k][l];
                    }
                }
            }
            if (idx2 < m) {
                for (auto j : contact_indices[idx2]) {
                    int sgn = idx2 == contact_infos[j].idx1 ? -1 : 1;
                    glm::mat3 r2_cross{
                        {0.f, -r2.z, r2.y},
                        {r2.z, 0.f, -r2.x},
                        {-r2.y, r2.x, 0.f}
                    };
                    glm::mat3 K = glm::mat3(rigids[idx2]->inv_m) - r2_cross * rigids[idx2]->inv_I * r2_cross;
                    for (int k = 0; k < 3; k ++) {
                        for (int l = 0; l < 3; l ++) {
                            A(i * 3 + k, j * 3 + l) += sgn * K[k][l];
                        }
                    }
                }
            }
        }
        Eigen::VectorXf x = A.fullPivHouseholderQr().solve(b);
        if (x.norm() > 100.f) {
            for (int i = 0; i < contact_num * 3; i ++) {
                for (int j = 0; j < contact_num * 3; j ++) {
                    printf("%03.3f ", A(i, j));
                }
                printf("| %03.3f | %03.3f\n", x(i), b(i));
            }
            for (auto c : contact_infos) {
                int idx1 = c.idx1, idx2 = c.idx2;
                glm::vec3 r1 = c.pos - rigids[idx1]->x;
                glm::vec3 r2 = c.pos - rigids[idx2]->x;
                glm::mat3 R1 = rigids[idx1]->RotationMatrix();
                glm::vec3 w1 = R1 * (rigids[idx1]->inv_I * glm::transpose(R1) * rigids[idx1]->L);
                glm::mat3 R2 = rigids[idx2]->RotationMatrix();
                glm::vec3 w2 = R2 * (rigids[idx2]->inv_I * glm::transpose(R2) * rigids[idx2]->L);
                glm::vec3 v1 = rigids[idx1]->v + glm::cross(w1, r1);
                glm::vec3 v2 = rigids[idx2]->v + glm::cross(w2, r2);
                glm::vec3 v_rel = v2 - v1;
                glm::vec3 v_n = glm::dot(v_rel, c.normal) * c.normal;
                glm::vec3 v_t = v_rel - v_n;
                glm::vec3 v_n_new = -restitution * v_n;
                float alpha = 0.f;
                if (mu * (1.f + restitution) * glm::length(v_n) < glm::length(v_t)) {
                    alpha = std::max(1.f - mu * (1.f + restitution) * glm::length(v_n) / glm::length(v_t), 0.f);
                }
                glm::vec3 v_t_new = alpha * v_t;
                glm::vec3 v_rel_new = v_n_new + v_t_new;
                glm::vec3 delta_v = v_rel_new - v_rel;
                printf("Contact: (%d, %d), Normal: (%f, %f, %f), Position: (%f, %f, %f), Penetration Depth: %f\n",
                    c.idx1, c.idx2,
                    c.normal.x, c.normal.y, c.normal.z,
                    c.pos.x, c.pos.y, c.pos.z,
                    c.penetration_depth);
                printf("%.2f %.2f %.2f -> %.2f %.2f %.2f\n", v_rel.x, v_rel.y, v_rel.z, v_rel_new.x, v_rel_new.y, v_rel_new.z);
            }
            printf("=========================\n");
            pause = true;
        }
        for (int i = 0; i < contact_num; i ++) {
            glm::vec3 impulse(x(i * 3), x(i * 3 + 1), x(i * 3 + 2));
            auto & c = contact_infos[i];
            int idx1 = c.idx1, idx2 = c.idx2;
            glm::vec3 r1 = c.pos - rigids[idx1]->x;
            glm::vec3 r2 = c.pos - rigids[idx2]->x;
            rigids[idx1]->v -= impulse * rigids[idx1]->inv_m;
            rigids[idx1]->L -= glm::cross(r1, impulse);
            rigids[idx2]->v += impulse * rigids[idx2]->inv_m;
            rigids[idx2]->L += glm::cross(r2, impulse);
            float penetration_depth = c.penetration_depth;
            if (penetration_depth > slop) {
                glm::vec3 correction = ((penetration_depth - slop) / (rigids[idx1]->inv_m + rigids[idx2]->inv_m)) * c.normal * correction_factor;
                rigids[idx1]->x -= correction * rigids[idx1]->inv_m;
                rigids[idx2]->x += correction * rigids[idx2]->inv_m;
            }
        }
    }

    /*
        * Constraint-based collision response.
    */
    void simulation_constraint(std::vector<Rigid*> const & rigids, std::vector<glm::vec3> const & dims, std::vector<glm::vec3> const & force,
        float const dt, float const restitution, float const correction_factor, float const mu, float const damping) {
        int n = rigids.size();
        int m = n;
        while (rigids[m - 1]->IsStatic && m > 0) {
            m --;
        }
        struct Constraint {
            int idx1, idx2;
            glm::vec3 normal;
            glm::vec3 pos;
            float penetration_depth;
        };
        std::vector<Constraint> constraints;
        for (int i = 0; i < m; i ++) {
            for (int j = i + 1; j < n; j ++) {
                auto result = collide_detect(*rigids[i], dims[i], *rigids[j], dims[j]);
                if (result.isCollision()) {
                    glm::vec3 normal(0.f);
                    glm::vec3 pos(0.f);
                    float penetration_depth = 0.f;
                    for (int i = 0; i < result.numContacts(); i++) {
                        auto contact = result.getContact(i);
                        normal += glm::vec3(contact.normal[0], contact.normal[1], contact.normal[2]);
                        pos += glm::vec3(contact.pos[0], contact.pos[1], contact.pos[2]);
                        penetration_depth += contact.penetration_depth;
                    }
                    normal = glm::normalize(normal);
                    pos /= result.numContacts();
                    penetration_depth /= result.numContacts();
                    constraints.push_back({i, j, normal, pos, penetration_depth});
                }
            }
        }
        int constraint_num = constraints.size();
        if (constraint_num == 0) {
            for (int i = 0; i < m; i ++) {
                rigids[i]->v = (rigids[i]->v + force[i * 2] * rigids[i]->inv_m * dt) * damping;
                glm::mat3 R = rigids[i]->RotationMatrix();
                glm::mat3 inv_I_world = R * rigids[i]->inv_I * glm::transpose(R);
                rigids[i]->L = (rigids[i]->L + glm::cross(inv_I_world * rigids[i]->L, force[i * 2 + 1]) * dt) * damping;
                rigids[i]->x += rigids[i]->v * dt;
                rigids[i]->q += 0.5f * glm::quat(0.f, inv_I_world * rigids[i]->L) * rigids[i]->q * dt;
                // rigids[i]->q += 0.5f * glm::quat(0.f, rigids[i]->w) * rigids[i]->q * dt;
                rigids[i]->q = glm::normalize(rigids[i]->q);
            }
        } else {
            Eigen::MatrixXf J = Eigen::MatrixXf::Zero(constraint_num * 3, 6 * m);
            Eigen::MatrixXf M_inv = Eigen::MatrixXf::Zero(6 * m, 6 * m);
            Eigen::VectorXf d = Eigen::VectorXf::Zero(constraint_num * 3);
            Eigen::VectorXf v_old = Eigen::VectorXf::Zero(6 * m);
            Eigen::VectorXf f_ext = Eigen::VectorXf::Zero(6 * m);
            for (int i = 0; i < m; i ++) {
                glm::mat3 R = rigids[i]->RotationMatrix();
                glm::mat3 inv_I_world = R * rigids[i]->inv_I * glm::transpose(R);
                glm::vec w = inv_I_world * rigids[i]->L;
                for (int j = 0; j < 3; j ++) {
                    M_inv(i * 6 + j, i * 6 + j) = rigids[i]->inv_m;
                    v_old(i * 6 + j) = rigids[i]->v[j];
                    v_old(i * 6 + 3 + j) = w[j];
                    // v_old(i * 6 + 3 + j) = rigids[i]->w[j];
                }
                for (int j = 0; j < 3; j ++) {
                    for (int k = 0; k < 3; k ++) {
                        M_inv(i * 6 + 3 + j, i * 6 + 3 + k) = inv_I_world[j][k];
                    }
                }
                for (int j = 0; j < 3; j ++) {
                    f_ext(i * 6 + j) = force[i * 2][j];
                    f_ext(i * 6 + 3 + j) = force[i * 2 + 1][j];
                }
            }
            for (int i = 0; i < constraint_num; i++) {
                auto & c = constraints[i];
                auto & r1 = rigids[c.idx1];
                auto & r2 = rigids[c.idx2];
                glm::vec3 r1c = c.pos - r1->x;
                glm::vec3 r2c = c.pos - r2->x;
                glm::vec3 v1 = r1->v + glm::cross(r1->inv_I * r1->L, r1c);
                glm::vec3 v2 = r2->v + glm::cross(r2->inv_I * r2->L, r2c);
                glm::vec3 v_rel = v2 - v1;
                glm::vec3 t1 = (std::abs(c.normal.x) > 0.9f) ? glm::vec3(0, 1, 0) : glm::vec3(1, 0, 0);
                t1 = glm::normalize(glm::cross(t1, c.normal));
                glm::vec3 t2 = glm::cross(c.normal, t1);
                for (int j = 0; j < 3; j ++) {
                    J(i * 3, c.idx1 * 6 + j) = -c.normal[j];
                    J(i * 3, c.idx1 * 6 + 3 + j) = -glm::cross(r1c, c.normal)[j];
                    J(i * 3 + 1, c.idx1 * 6 + j) = -t1[j];
                    J(i * 3 + 1, c.idx1 * 6 + 3 + j) = -glm::cross(r1c, t1)[j];
                    J(i * 3 + 2, c.idx1 * 6 + j) = -t2[j];
                    J(i * 3 + 2, c.idx1 * 6 + 3 + j) = -glm::cross(r1c, t2)[j];
                }
                if (c.idx2 < m) {
                    for (int j = 0; j < 3; j ++) {
                        J(i * 3, c.idx2 * 6 + j) = c.normal[j];
                        J(i * 3, c.idx2 * 6 + 3 + j) = glm::cross(r2c, c.normal)[j];
                        J(i * 3 + 1, c.idx2 * 6 + j) = t1[j];
                        J(i * 3 + 1, c.idx2 * 6 + 3 + j) = glm::cross(r2c, t1)[j];
                        J(i * 3 + 2, c.idx2 * 6 + j) = t2[j];
                        J(i * 3 + 2, c.idx2 * 6 + 3 + j) = glm::cross(r2c, t2)[j];
                    }
                }
                d(i * 3) = correction_factor / dt * std::min(c.penetration_depth, 0.01f) - restitution * glm::dot(v_rel, c.normal);
            }
            Eigen::MatrixXf A = J * M_inv * J.transpose();
            const float eps = 1e-6f;
            for (int i = 0; i < constraint_num * 3; i ++) {
                A(i, i) += eps;
            }
            Eigen::VectorXf b = d - J * (M_inv * f_ext * dt + v_old);
            Eigen::VectorXf lambda = A.fullPivLu().solve(b);
            for (int i = 0; i < constraint_num; i ++) {
                if (lambda(i * 3) < 0.f) {
                    lambda(i * 3) = 0.f;
                }
                lambda(i * 3 + 1) = glm::clamp(lambda(i * 3 + 1), -mu * lambda(i * 3), mu * lambda(i * 3));
                lambda(i * 3 + 2) = glm::clamp(lambda(i * 3 + 2), -mu * lambda(i * 3), mu * lambda(i * 3));
            }
            Eigen::VectorXf v_new = v_old + M_inv * (f_ext * dt + J.transpose() * lambda);
            for (int i = 0; i < m; i ++) {
                rigids[i]->v = glm::vec3{
                    v_new(i * 6),
                    v_new(i * 6 + 1),
                    v_new(i * 6 + 2)
                } * damping;
                // rigids[i]->w = glm::vec3{
                //     v_new(i * 6 + 3),
                //     v_new(i * 6 + 4),
                //     v_new(i * 6 + 5)
                // } * damping;
                glm::vec3 w_new = glm::vec3{
                    v_new(i * 6 + 3),
                    v_new(i * 6 + 4),
                    v_new(i * 6 + 5)
                } * damping;
                glm::mat3 R = rigids[i]->RotationMatrix();
                glm::mat3 I_world = R * rigids[i]->I * glm::transpose(R);
                glm::mat3 inv_I_world = R * rigids[i]->inv_I * glm::transpose(R);
                rigids[i]->L = I_world * w_new;
                rigids[i]->x += rigids[i]->v * dt;
                rigids[i]->q += 0.5f * glm::quat(0.f, inv_I_world * rigids[i]->L) * rigids[i]->q * dt;
                // rigids[i]->q += 0.5f * glm::quat(0.f, rigids[i]->w) * rigids[i]->q * dt;
                rigids[i]->q = glm::normalize(rigids[i]->q);
            }
        }
    }

}