#include "DeterminismValidator.h"
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>

namespace
{
    void PopulateState(
        btDiscreteDynamicsWorld const* pWorld,
        std::vector<glm::vec3>* pOrigins,
        std::vector<glm::quat>* pRotations
        )
    {
        pOrigins->clear();
        pRotations->clear();
        auto& collisionObjects = pWorld->getCollisionObjectArray();
        for (int i = 0; i < collisionObjects.size(); ++i)
        {
            btTransform transform = collisionObjects[i]->getWorldTransform();
            const btVector3& Origin = transform.getOrigin();
            const btQuaternion& Rotation = transform.getRotation();
            pOrigins->push_back(
                glm::vec3(Origin.x(), Origin.y(), Origin.z())
                );
            pRotations->push_back(
                glm::quat(Rotation.w(), Rotation.x(), Rotation.y(), Rotation.z())
                );
        }
    }

    bool CheckState(
        btDiscreteDynamicsWorld const* pWorld,
        std::vector<glm::vec3>* pOrigins,
        std::vector<glm::quat>* pRotations
        )
    {
        const auto NumCollisionObjects = static_cast<size_t>(pWorld->getNumCollisionObjects());
        if (NumCollisionObjects != pOrigins->size() || NumCollisionObjects != pRotations->size())
        {
            return false;
        }

        auto& collisionObjects = pWorld->getCollisionObjectArray();
        for (size_t i = 0; i < NumCollisionObjects; i++)
        {
            btTransform transform = collisionObjects[i]->getWorldTransform();
            const btVector3& Origin = transform.getOrigin();
            const btQuaternion& Rotation = transform.getRotation();

            if (Origin.x() != (*pOrigins)[i].x ||
                Origin.y() != (*pOrigins)[i].y ||
                Origin.z() != (*pOrigins)[i].z)
            {
                return false;
            }

            if (Rotation.x() != (*pRotations)[i].x ||
                Rotation.y() != (*pRotations)[i].y ||
                Rotation.z() != (*pRotations)[i].z ||
                Rotation.w() != (*pRotations)[i].w)
            {
                return false;
            }
        }

        return true;
    }
}

bool DeterminismValidator::CheckWorld(btDiscreteDynamicsWorld const* pWorld)
{
    if (m_origins.empty() && m_rotations.empty())
    {
        PopulateState(pWorld, &m_origins, &m_rotations);
        return true;
    }
    else
    {
        if (CheckState(pWorld, &m_origins, &m_rotations))
        {
            PopulateState(pWorld, &m_origins, &m_rotations);
            return true;
        }

        return false;
    }
}
