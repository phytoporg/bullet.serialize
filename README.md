This is a quick project intended to play with determinism in BulletPhysics world serialization.

The project is in a state where this goal is met for the simulation in question. However, the requisite modifications to bullet won't be part of the project as I am a stupid what cloned bullet as a submodule!

A quick note on that change:

in btWorldImporter.cpp at line 2051, add:
        body->setLinearVelocity(
            btVector3(
                colObjData->m_linearVelocity.m_floats[0],
                colObjData->m_linearVelocity.m_floats[1],
                colObjData->m_linearVelocity.m_floats[2]
                )
            );
        body->setAngularVelocity(
            btVector3(
                colObjData->m_angularVelocity.m_floats[0],
                colObjData->m_angularVelocity.m_floats[1],
                colObjData->m_angularVelocity.m_floats[2]
                )
            );

...and that's it!
