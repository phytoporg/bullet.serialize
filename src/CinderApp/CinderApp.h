#pragma once

#include <cinder/app/app.h>
#include <cinder/camera.h>
#include <cinder/gl/gl.h>

#include <memory>

//
// Forward devlarations
//
class btDiscreteDynamicsWorld;
class btRigidBody;

class CinderApp : public ci::app::App
{
public:
    CinderApp();
	virtual ~CinderApp() = default;

    virtual void setup() override;
    virtual void update() override;
	virtual void draw() override;

public:
    ci::CameraPersp  m_perspectiveCam;
    ci::gl::BatchRef m_ball;

    std::unique_ptr<btRigidBody> m_spRigidBody;
    std::unique_ptr<btDiscreteDynamicsWorld> m_spDynamicsWorld;
    double m_lastFrameTime;
};
