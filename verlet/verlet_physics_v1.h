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

#include <physics/struct_v1.h>

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
        m_collisionDetection->clean();
        vector<vRigidBody*>().swap(m_rBodies);
    }

    void step(float dt)
    {      
        for_each(m_rBodies.begin(), m_rBodies.end(),
            [&](vRigidBody *body)
            {
                body->update(dt);
            });

        m_collisionDetection->update();

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

    vector<triangle> debugTri()
    {

        vector<triangle> res; 
        for_each(m_rBodies.begin(), m_rBodies.end(),
            [&](vRigidBody *body)
            {
                vector<triangle> t;
                box<vRigidBody>().getTrianglesfromBox(t, box<vRigidBody>().create(body));
                for_each(t.begin(), t.end(),
                [&](triangle tri){
                    res.push_back(tri);
                });
            });
        return res;
       /* vector<vec3> res; //SHOW TRINAGLE VERTICES ///method returns vector<vec3>
        for_each(m_rBodies.begin(), m_rBodies.end(),
            [&](vRigidBody *body)
            {
                vector<triangle> t;
                box<vRigidBody>().getTrianglesfromBox(t, box<vRigidBody>().create(body));
                for_each(t.begin(), t.end(),
                [&](triangle tri){
                    res.push_back(tri.v0);
                    res.push_back(tri.v1);
                    res.push_back(tri.v2);
                });
            });
        return res; /*

        /*vector<vec3> res; //SHOW VERTICES ///method returns vector<vec3>
        vector<vec3> tmp;

        int bodyn = 1;
        for_each(m_rBodies.begin(), m_rBodies.end(),
            [&](vRigidBody *body)
            {
                int point = 0;
                vector<triangle> t;
                box<vRigidBody>().debug(t, tmp, box<vRigidBody>().create(body));
                for_each(tmp.begin(), tmp.end(),
                [&](vec3 v){
                    int asd = bodyn*point;
                    std::cout << "point n =\t" << asd << std::endl;
                    point++;
                    res.push_back(v);
                });
                body++;
            });
        return res; */
    }

    int debugCollision(vector<vec3>* t, vector<vec3>* i, vector<vec3>* p, vector<vec3>* l)
    {
        return m_collisionDetection->debugCollision(t, i, p, l);
    }

    int debugCollision2(CollisionResponse<RigidBody::Response r)
    {
        return m_collisionDetection->debugCollision2(r);
    }
    void debugOctree(vector<Octree<vRigidBody>::OctreeNode*> *nodes)
    {
        m_collisionDetection->debugOctree(nodes);
    }
};