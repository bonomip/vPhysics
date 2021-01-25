/*
VERLET PHYSISC

author: Paolo Bonomi

Real-Time Graphics Programming's Project - 2019/2020
*/

#pragma once

using namespace std;

//Physics class
#include <physics/verlet/verlet_rb_v1.h>
#include <physics/collision_solver_v1.h>

//for_each loop
#include<algorithm>

//OpenGL extension
#include <glad/glad.h>

class vPhysics
{

typedef glm::vec3 vec3;
float m_worldSize;
vector<vRigidBody*> m_rBodies;

static const int COLLISION_SOLVER = 1;
CollisionSolver * m_colSolv;

public:
    vPhysics(){}

    void setWorld(const float &worldSize) 
    {
        m_worldSize = worldSize; //world center is implicit at 0 0 0

        if(COLLISION_SOLVER) m_colSolv = new CollisionSolver(worldSize);
    }

    void addBox(vec3 pos, GLfloat* color, vec3 rot, vec3 scale, float mass, float drag, bool useGravity, bool isKinematic)
    {
        m_rBodies.push_back(new Box(m_rBodies.size(), pos, color, rot, scale, mass, drag, useGravity, isKinematic, m_worldSize));
    }

    void addSphere(vec3 pos, GLfloat* color, vec3 rot, vec3 scale, float mass, float drag, bool useGravity, bool isKinematic)
    {
       //todo
    }

    void cleanWorld()
    {
        for(int i = 0; m_rBodies.size(); i++)
            delete m_rBodies.at(i);

        vector<vRigidBody*>().swap(m_rBodies);
        if(COLLISION_SOLVER) m_colSolv->clean();
    }

    void step(float dt)
    {      
        for_each(m_rBodies.begin(), m_rBodies.end(),
            [&](vRigidBody *body)
            {
                body->update(dt);
            });

        if(COLLISION_SOLVER)
        {
            m_colSolv->setBodies(&m_rBodies);
            m_colSolv->update();
        }

        for_each(m_rBodies.begin(), m_rBodies.end(),
            [&](vRigidBody *body)
            {
                body->updateConstraint();
            });
    }

    vector<vRigidBody*>* getRigidBodies()
    {
        return &m_rBodies;
    }
};