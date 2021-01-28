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
int m_countRb;

static const int COLLISION_SOLVER = 1;
CollisionSolver * m_colSolv;

public:
    vPhysics(){}

    void setWorld(const float &worldSize) 
    {
        m_worldSize = worldSize; //world center is implicit at 0 0 0
        this->m_countRb = 0;
        if(COLLISION_SOLVER) m_colSolv = new CollisionSolver(worldSize);
    }

    void addBox(vec3 pos, GLfloat* color, vec3 rot, vec3 scale, float mass, float drag, bool useGravity, bool isKinematic)
    {
        m_rBodies.push_back(new Box(this->m_countRb++, pos, color, rot, scale, mass, drag, useGravity, isKinematic, this->m_worldSize));
    }

    void addSphere(vec3 pos, GLfloat* color, vec3 rot, const float &radius, float mass, float drag, float bounciness, bool useGravity, bool isKinematic)
    {
       m_rBodies.push_back(new Sphere(this->m_countRb++, pos, color, rot, radius, mass, drag, bounciness, useGravity, isKinematic, this->m_worldSize));
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
            //only for collision debug
            if(this->m_rBodies.at(i)->getParticles()->at(0).stop2)
            {
                continue;
            }

            this->m_rBodies.at(i)->update(dt);

            if( this->m_rBodies.at(i)->getParticles()->at(0).stop )
            {   //only for sphere to prevent bugs
                this->m_rBodies.at(i)->getParticles()->at(0).reset( this->m_rBodies.at(i)->getStartPos());
                this->m_rBodies.at(i)->update(dt); //if its the first frame it will catch up
            }
        }

        if(COLLISION_SOLVER)
        {
            m_colSolv->setBodies(&m_rBodies);
            m_colSolv->update();
        }

        for(int i = 0; i < this->m_rBodies.size(); i ++)
        {
            this->m_rBodies.at(i)->updateConstraint();
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

    bool frameByFrame()
    {
        return this->m_colSolv->framebyframe;
    }

};