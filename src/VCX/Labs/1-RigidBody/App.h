#pragma once

#include <vector>

#include "Engine/app.h"
#include "Labs/1-RigidBody/CaseSingleRigid.h"
#include "Labs/1-RigidBody/CaseTwoRigids.h"
#include "Labs/1-RigidBody/CaseFourRigids.h"
#include "Labs/1-RigidBody/CaseConstraintDynamics.h"
#include "Labs/Common/UI.h"

namespace VCX::Labs::RigidBody {
    class App : public Engine::IApp {
    private:
        Common::UI _ui;

        CaseSingleRigid _caseSingleRigid;
        CaseTwoRigids _caseTwoRigids;
        CaseFourRigids _caseFourRigids;
        CaseConstraintDynamics _caseConstraintDynamics;

        std::size_t _caseId = 0;

        std::vector<std::reference_wrapper<Common::ICase>> _cases = { 
            _caseSingleRigid,
            _caseTwoRigids,
            _caseFourRigids,
            _caseConstraintDynamics,
        };

    public:
        App();

        void OnFrame() override;
    };
}