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
    : m_lastFrameTime(0.0)
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
                new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0))), 
                new btSphereShape(BallRadius)
                );
        pRigidBody->setWorldTransform(originalTransform);
        pRigidBody->getMotionState()->setWorldTransform(originalTransform);

        // Should be drifting towards the center
        vec3 d(-X, -Y, 0);
        vec3 n = normalize(d) * BallSpeed;
        pRigidBody->setLinearVelocity(btVector3(n.x, n.y, n.z));

        m_spRigidBodies.emplace_back(pRigidBody);
        m_spDynamicsWorld->addRigidBody(pRigidBody);
    }

    // No gravity for now
    m_spDynamicsWorld->setGravity(btVector3(0.0, 0.0, 0.0));
}

void CinderApp::update()
{
    static float dtBuffer[100] = {};
    static int i = 0;
    if (m_lastFrameTime > std::numeric_limits<float>::epsilon())
    {
        float dt = getElapsedSeconds() - m_lastFrameTime;
        dtBuffer[i++ % 100] = dt;
        m_spDynamicsWorld->stepSimulation(1.0 / 60.0);
    }

    assert(m_spDynamicsWorld->getNumCollisionObjects() == m_spRigidBodies.size());
    assert(m_spDynamicsWorld->getNumCollisionObjects() == m_balls.size());

    m_lastFrameTime = getElapsedSeconds();
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

        auto& spRigidBody = m_spRigidBodies[i];

        btTransform transform;
        spRigidBody->getMotionState()->getWorldTransform(transform);
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
}

CINDER_APP(CinderApp, app::RendererGl, [](cinder::app::AppBase::Settings* pSettings)
{
    pSettings->setWindowSize(640, 480);
});