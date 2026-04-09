#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <Eigen/Dense>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/contact.h>
#include <fcl/narrowphase/distance.h>

namespace VCX::Labs::RigidBody {
    struct Rigid {
        bool IsStatic;
        float m, inv_m;
        glm::mat3 I, inv_I;

        glm::vec3 x;
        glm::vec3 v;
        glm::quat q;
        glm::vec3 L;
        glm::vec3 w;

        Rigid() = default;

        Rigid(float _m, glm::vec3 _x, glm::vec3 _v, glm::quat _q, glm::vec3 _L) :
            m(_m), x(_x), v(_v), q(_q), L(_L), IsStatic(_m == 0.f), inv_m(_m != 0.f ? 1.f / _m : 0.f) {}

        glm::mat3 RotationMatrix() const {
            return glm::mat3_cast(q);
        }

        void getBoxInertiaTensor(glm::vec3 dim) {
            I = glm::mat3(
                m / 12.f * (dim.y * dim.y + dim.z * dim.z), 0.f, 0.f,
                0.f, m / 12.f * (dim.x * dim.x + dim.z * dim.z), 0.f,
                0.f, 0.f, m / 12.f * (dim.x * dim.x + dim.y * dim.y)
            );
            if (IsStatic) {
                inv_I = glm::mat3(0.f);
            } else {
                inv_I = glm::inverse(I);
            }
            inv_m = m != 0.f ? 1.f / m : 0.f;
        }
        
        float Damping = 1.f;
        void SimulateTimestep(float const dt, glm::vec3 const & force, glm::vec3 const & torque);
    };

    void collide_test(Rigid & rigid1, glm::vec3 dim1, Rigid & rigid2, glm::vec3 dim2, float restitution = 1.f, float correction_factor = 0.f, float mu = 0.f);

    void collide_multiple(std::vector<Rigid*> const & rigids, std::vector<glm::vec3> const & dims,
        float restitution, float correction_factor, float mu, bool & pause);

    void simulation_constraint(std::vector<Rigid*> const & rigids, std::vector<glm::vec3> const & dims, std::vector<glm::vec3> const & force,
        float const dt, float const restitution, float const correction_factor, float const mu, float const damping);
}