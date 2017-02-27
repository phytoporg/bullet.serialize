#pragma once

#include <cinder/cinderglm.h>
#include <vector>

// Forward declarations
class btDiscreteDynamicsWorld;

// The app stores the state of every rigidbody at the end of
// a simulation; this object ensures that the state is identical
// each and every time.
class DeterminismValidator
{
public:
    // If we have no state, always returns true. Otherwise,
    // compares incoming state to saved state. Values much match
    // exactly, or CheckWorld() returns false.
    bool CheckWorld(btDiscreteDynamicsWorld const* pWorld);

private:
    std::vector<glm::vec3> m_origins;
    std::vector<glm::quat> m_rotations;
};
