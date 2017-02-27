#pragma once

#include <cinder/app/app.h>
#include <cinder/camera.h>
#include <cinder/gl/gl.h>

#include "DeterminismValidator.h"

#include <memory>
#include <vector>

//
// Forward declarations
//
class btDiscreteDynamicsWorld;
class btRigidBody;
class btSerializer;
class btCollisionConfiguration;
class btCollisionDispatcher;
class btBroadphaseInterface;
class btConstraintSolver;

class CinderApp : public ci::app::App
{
public:
    CinderApp();
	virtual ~CinderApp() = default;

    virtual void setup() override;
    virtual void update() override;
	virtual void draw() override;

public:
    void SerializeState();
    void LoadState();

    btDiscreteDynamicsWorld* CreateDynamicsWorld();

    ci::CameraPersp  m_perspectiveCam;

    // Indices correspond
    std::vector<ci::gl::BatchRef>   m_balls;
    std::vector<btRigidBody*>       m_pRigidBodies;

    std::unique_ptr<btSerializer>   m_spSerializer;

    std::unique_ptr<btCollisionConfiguration> m_spCollisionConfig;
    std::unique_ptr<btCollisionDispatcher>    m_spDispatcher;
    std::unique_ptr<btBroadphaseInterface>    m_spBroadphase;
    std::unique_ptr<btConstraintSolver>       m_spSolver;

    std::unique_ptr<btDiscreteDynamicsWorld>  m_spDynamicsWorld;

    DeterminismValidator                      m_validator;

    double m_lastFrameTime;
    unsigned int m_frameIndex;
};
