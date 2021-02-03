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

class vPhysics
{

typedef glm::vec3 vec3;
float m_worldSize;
vector<vRigidBody*> m_rBodies;
int m_countRb;

static const int COLLISION_SOLVER = 1;
CollisionSolver * m_colSolv;

public:
    vPhysics(){}

    struct spherePrefab
    {
        vec3 pos = vec3(.0f,.0f,.0f);
        vec3 rot = vec3(.0f,.0f,.0f); 
        GLfloat* color;
        float radius = .5f; 
        float mass = .5f;
        float drag = .5f;
        float bounciness = .5f;
        bool gravity = true;
        bool kinematic = false;
    };

    struct boxPrefab
    {
        vec3 pos = vec3(.0f,.0f,.0f);
        GLfloat* color;
        vec3 rot = vec3(.0f,.0f,.0f); 
        vec3 scale = vec3(.2f, .4f, .2f); 
        float mass = .5f;
        float drag = .5f;
        bool gravity = true;
        bool kinematic = false;
    };

    void setWorld(const float &worldSize) 
    {
        m_worldSize = worldSize; //world center is implicit at 0 0 0
        this->m_countRb = 0;
        if(COLLISION_SOLVER) m_colSolv = new CollisionSolver(worldSize);
    }

    vRigidBody* addBox(vec3 pos, GLfloat* color, vec3 rot, vec3 scale, float mass, float drag, bool useGravity, bool isKinematic)
    {
        m_rBodies.push_back(new Box(this->m_countRb++, pos, color, rot, scale, mass, drag, useGravity, isKinematic, this->m_worldSize));
        return m_rBodies.back();
    }

    vRigidBody* addBox(boxPrefab b)
    {
        return this->addBox(b.pos, b.color, b.rot, b.scale, b.mass, b.drag, b.gravity, b.kinematic);
    }

    vRigidBody* addSphere(vec3 pos, GLfloat* color, vec3 rot, const float &radius, float mass, float drag, float bounciness, bool useGravity, bool isKinematic)
    {
       m_rBodies.push_back(new Sphere(this->m_countRb++, pos, color, rot, radius, mass, drag, bounciness, useGravity, isKinematic, this->m_worldSize));
       return m_rBodies.back();
    }

    vRigidBody* addSphere(spherePrefab s)
    {
        return this->addSphere(s.pos, s.color, s.rot, s.radius, s.mass, s.drag, s.bounciness, s.gravity, s.kinematic);
    }

    void cleanWorld()
    {
        //call the decostructor of each obj
        this->m_rBodies.clear();

        this->m_countRb = 0;

        //make sure mem clear
        vector<vRigidBody*>().swap(this->m_rBodies);

        if(COLLISION_SOLVER) this->m_colSolv->clean();
    }

    void step(float dt)
    {      
        for(int i = 0; i < this->m_rBodies.size(); i ++)
        {
            this->m_rBodies.at(i)->update(dt);

            if( this->m_rBodies.at(i)->getParticles()->at(0).stop )
            {   //only for sphere to prevent bugs
                this->m_rBodies.at(i)->getParticles()->at(0).reset( this->m_rBodies.at(i)->getStartPos());
                this->m_rBodies.at(i)->update(dt); //if its the first frame it will catch up
            }

            this->m_rBodies.at(i)->updateConstraint();
        }

        if(COLLISION_SOLVER)
        {
            m_colSolv->setBodies(&m_rBodies);
            m_colSolv->update();

        }
    }

    vector<vRigidBody*>* getRigidBodies()
    {
        return &m_rBodies;
    }

    //debug
    void getOctreeNodes(vector<std::pair<vec3, vec3>> &result)
    {
        if(!COLLISION_SOLVER) return;

        for_each(this->m_colSolv->octreeLeafs.begin(), this->m_colSolv->octreeLeafs.end(),
        [&](Octree<vRigidBody>::OctreeNode *n)
        {
            result.push_back(std::make_pair(n->m_pos, vec3(n->m_side_size/2.0f, n->m_side_size/2.0f, n->m_side_size/2.0f)));
        });
    }

};