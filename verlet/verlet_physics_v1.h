/*
VERLET PHYSISC

author: Paolo Bonomi

Real-Time Graphics Programming's Project - 2019/2020
*/

#pragma once

using namespace std;

//Physics class
#include <physics/verlet/verlet_rb_v1.h>
#include <physics/collision_detection_v1.h>

//vector classw
#include <vector>

//for_each loop
#include<algorithm>

//OpenGL extension
#include <glad/glad.h>

class vPhysics
{

typedef glm::vec3 vec3;
float m_worldSize;
vector<vRigidBody*> m_rBodies;
CollisionDetection<vRigidBody> * m_collisionDetection;

public:
    vPhysics()
    {
    }

    void setWorld(const float &worldSize) 
    {
        m_worldSize = worldSize; //world center is implicit at 0 0 0
        m_collisionDetection = new CollisionDetection<vRigidBody>(worldSize);
    }

    /*virtual ~vPhysics()
    {
        vector<vRigidBody>().swap(m_rBodies);
    }*/

    void addRigidBody(int id, int kind, vec3 pos, GLfloat* color, vec3 rot, vec3 scale, float mass, float drag, bool useGravity, bool isKinematic)
    {
        vRigidBody *ptr = new vRigidBody(id, kind, pos, color, rot, scale, mass, drag, useGravity, isKinematic, m_worldSize); 
        m_rBodies.push_back(ptr);
        m_collisionDetection->addRigidBody(ptr);
    }

    void cleanWorld()
    {
        vector<vRigidBody*>().swap(m_rBodies);
    }

    void step(float dt)
    {

        m_collisionDetection->update(); 
        
        for_each(m_rBodies.begin(), m_rBodies.end(),
            [&](vRigidBody *body)
            {
                body->update(dt);
            });
    }

    vector<vRigidBody*>* getRigidBodies()
    {
        return &m_rBodies;
    }

    void debugOctree(vector<Octree<vRigidBody>::OctreeNode*> *nodes)
    {
        m_collisionDetection->debugOctree(nodes);
    }
};