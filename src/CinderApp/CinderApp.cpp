#include "CinderApp.h"

#include <cinder/app/renderergl.h>
#include <cinder/app/app.h>

#include <btBulletDynamicsCommon.h>
#include <BulletWorldImporter/btBulletWorldImporter.h>

#include <cassert>
#include <limits>

#define _USE_MATH_DEFINES
#include <cmath>

using namespace ci;

namespace
{ 
    const float BallSpeed = 10.0f;
    const float BallRadius = 0.2f;
    const size_t NumberOfBalls = 10;
}

CinderApp::CinderApp()
    : m_lastFrameTime(0.0),
      m_frameIndex(0)
{}

void CinderApp::setup()
{
    gl::enableVerticalSync();

    m_perspectiveCam.lookAt(ci::vec3(0, 0, -30.0f), ci::vec3(0));

    // Set up shaders, program
    auto lambert = gl::ShaderDef().lambert().color();
    gl::GlslProgRef program = gl::getStockShader(lambert);

    m_spDynamicsWorld.reset(CreateDynamicsWorld());

    // Set the balls up in a circle of radius 10
    const float AngleStep = (2 * M_PI) / NumberOfBalls;
    for (float a = 0; a < 2 * M_PI; a += AngleStep)
    {
        // rotate in the x-y plane
        const float X = 10 * cos(a);
        const float Y = 10 * sin(a);

        btTransform originalTransform(btQuaternion(0, 0, 0, 1), btVector3(X, Y, 0));
        auto ballRef = gl::Batch::create(
                ci::geom::Sphere().radius(BallRadius),
                program
            );
        m_balls.push_back(ballRef);
        auto pRigidBody = new btRigidBody(
                2.0, 
                nullptr, 
                new btSphereShape(BallRadius)
                );
        pRigidBody->setWorldTransform(originalTransform);

        // Should be drifting towards the center
        vec3 d(-X, -Y, 0);
        vec3 n = normalize(d) * BallSpeed;
        pRigidBody->setLinearVelocity(btVector3(n.x, n.y, n.z));

        m_pRigidBodies.emplace_back(pRigidBody);
        m_spDynamicsWorld->addRigidBody(pRigidBody);
    }

    // No gravity for now
    m_spDynamicsWorld->setGravity(btVector3(0.0, 0.0, 0.0));
}

void CinderApp::update()
{
    if (m_frameIndex == 0)
    {
        // Some piece of the initial state is never serialized, so
        // take the initial state and "clean" it up artificially to 
        // ensure that all following iterations are deterministic.
        SerializeState();
        LoadState();
    }
    else if (m_frameIndex == 1)
    {
        SerializeState();
    }
    m_frameIndex++;

    if (m_lastFrameTime > std::numeric_limits<float>::epsilon())
    {
        static const float InvFrameRate = 1.0f / getFrameRate();
        m_spDynamicsWorld->stepSimulation(InvFrameRate);
    }

    assert(m_spDynamicsWorld->getNumCollisionObjects() == m_pRigidBodies.size());
    assert(m_spDynamicsWorld->getNumCollisionObjects() == m_balls.size());

    m_lastFrameTime = getElapsedSeconds();

    if (m_frameIndex == 300)
    {
        LoadState();
        if (!m_validator.CheckWorld(m_spDynamicsWorld.get()))
        {
            throw std::exception("Validation FAILED: Nondeterminism detected in simulation.");
        }

        m_frameIndex = 1;
    }
}

void CinderApp::draw()
{
    gl::clear();
    gl::enableDepthRead();
    gl::enableDepthWrite();

    gl::setMatrices(m_perspectiveCam);

    for (size_t i = 0; i < m_spDynamicsWorld->getNumCollisionObjects(); ++i)
    {
        gl::pushModelMatrix();
        gl::color(Color(CM_RGB, 1.0, 0.0, 0.0));

        auto& pRigidBody = m_pRigidBodies[i];

        btTransform transform = pRigidBody->getWorldTransform();
        const auto& Origin = transform.getOrigin();
        const auto& Rotation = transform.getRotation();

        gl::translate(Origin.x(), Origin.y(), Origin.z());
        gl::rotate(quat(Rotation.getX(), Rotation.getY(), Rotation.getZ(), Rotation.getW()));

        auto& ballRef = m_balls[i];
        ballRef->draw();
        gl::popModelMatrix();
    }
}

btDiscreteDynamicsWorld* CinderApp::CreateDynamicsWorld()
{
    m_spCollisionConfig.reset(new btDefaultCollisionConfiguration());
    m_spDispatcher.reset(new btCollisionDispatcher(m_spCollisionConfig.get()));
    m_spBroadphase.reset(new btDbvtBroadphase());
    m_spSolver.reset(new btSequentialImpulseConstraintSolver());

    return 
        new btDiscreteDynamicsWorld(
            m_spDispatcher.get(),
            m_spBroadphase.get(),
            m_spSolver.get(),
            m_spCollisionConfig.get()
            );
}

void CinderApp::SerializeState()
{
    assert(!m_spSerializer);
    m_spSerializer.reset(new btDefaultSerializer());
    m_spDynamicsWorld->serialize(m_spSerializer.get());
}

void CinderApp::LoadState()
{
    assert(m_spSerializer);
    m_spDynamicsWorld.reset();
    m_spDynamicsWorld.reset(CreateDynamicsWorld());
    std::unique_ptr<btBulletWorldImporter> spWorldLoader(
        new btBulletWorldImporter(m_spDynamicsWorld.get())
        );

    unsigned char* pBuffer = const_cast<unsigned char*>(m_spSerializer->getBufferPointer());
    spWorldLoader->loadFileFromMemory(
        reinterpret_cast<char*>(pBuffer),
        m_spSerializer->getCurrentBufferSize()
        );
    m_spSerializer.reset();

    auto& collisionObjects = m_spDynamicsWorld->getCollisionObjectArray();
    assert(collisionObjects.size() == m_pRigidBodies.size());
    for (int i = 0; i < m_spDynamicsWorld->getNumCollisionObjects(); i++)
    {
        m_pRigidBodies[i] = static_cast<btRigidBody*>(collisionObjects[i]);
    }
}

CINDER_APP(CinderApp, app::RendererGl, [](cinder::app::AppBase::Settings* pSettings)
{
    pSettings->setWindowSize(640, 480);
    pSettings->setFrameRate(60.0f);
});
