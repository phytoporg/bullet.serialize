#include "CinderApp.h"

#include <cinder/app/renderergl.h>
#include <cinder/app/app.h>

#include <btBulletDynamicsCommon.h>

#include <cassert>

using namespace ci;

namespace
{
    btDiscreteDynamicsWorld* CreateDynamicsWorld()
    {
        auto pCollisionConfig = new btDefaultCollisionConfiguration();
        auto pDispatcher = new btCollisionDispatcher(pCollisionConfig);
        auto pBroadphase = new btDbvtBroadphase();
        auto pSolver = new btSequentialImpulseConstraintSolver();

        return new btDiscreteDynamicsWorld(
            pDispatcher, pBroadphase, pSolver, pCollisionConfig
            );
    }

    const float BallRadius = 0.2f;
}

CinderApp::CinderApp()
    : m_lastFrameTime(0.0)
{}

void CinderApp::setup()
{
    auto lambert = gl::ShaderDef().lambert().color();
    gl::GlslProgRef program = gl::getStockShader(lambert);

    m_ball = gl::Batch::create(ci::geom::Sphere().center(vec3(0.0f)).radius(BallRadius), program);

    m_perspectiveCam.lookAt(ci::vec3(0, 0, -10.0f), ci::vec3(0));

    m_spDynamicsWorld.reset(CreateDynamicsWorld());

    // No gravity for now-- let's see how collisions work
    m_spDynamicsWorld->setGravity(btVector3(0.0, 0.0, 0.0));

    m_spRigidBody.reset(
        new btRigidBody(
            2.0, 
            new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0))), 
            new btSphereShape(BallRadius)
            ));
    m_spDynamicsWorld->addRigidBody(m_spRigidBody.get());
}

void CinderApp::update()
{
    float dt = getElapsedSeconds() - m_lastFrameTime;
    m_spDynamicsWorld->stepSimulation(dt);

    // For now
    assert(m_spDynamicsWorld->getNumCollisionObjects() == 1);

    m_lastFrameTime = getElapsedSeconds();
}

void CinderApp::draw()
{
    gl::clear();
    gl::enableDepthRead();
    gl::enableDepthWrite();

    gl::setMatrices(m_perspectiveCam);

    gl::pushModelMatrix();
    gl::color(Color(CM_RGB, 1.0, 0.0, 0.0));

    btTransform transform;
    m_spRigidBody->getMotionState()->getWorldTransform(transform);
    const auto& Origin = transform.getOrigin();
    const auto& Rotation = transform.getRotation();

    gl::translate(Origin.x(), Origin.y(), Origin.z());
    gl::rotate(quat(Rotation.getX(), Rotation.getY(), Rotation.getZ(), Rotation.getW()));

    m_ball->draw();
    gl::popModelMatrix();
}

CINDER_APP(CinderApp, app::RendererGl, [](cinder::app::AppBase::Settings* pSettings)
{
    pSettings->setWindowSize(640, 480);
});