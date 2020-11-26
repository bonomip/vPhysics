/*
PHYSISC

author: Paolo Bonomi

Real-Time Graphics Programming's Project - 2019/2020
*/

#pragma once

#include <physics/verlet/verlet_physics_v1.h>
#include <physics/octree_v1.h>
#include <physics/collision_response_v1.h>
#include <physics/struct_v1.h>
#include <physics/collision_v1.h>

template <class RigidBody> class CollisionDetection
{
    public:
    CollisionResponse<RigidBody>* collisionResp;
    vector<RigidBody*> m_rBodies;
    vector<class Collision<RigidBody>*> colls;

    float octreeSize = 10.0f;
    glm::vec3 octreeCenter = glm::vec3(0.0f, 0.0f, 0.0f);
    int octreeDepth = 5;
    vector<class Octree<RigidBody>::OctreeNode*> octreeLeafs;
    Octree<RigidBody> * m_tree;

    CollisionDetection(const float &worldSize) //world center is implicit at 0 0 0
    {
        collisionResp = new CollisionResponse<RigidBody>();
        m_tree = new Octree<RigidBody>(octreeCenter, worldSize*2.0f, octreeDepth);
    }

    void addRigidBody(RigidBody * rb)
    {
        m_rBodies.push_back(rb);
    }

    void update() //called each physics step
    {
        //struct used for collision detection
        box<RigidBody> a;
        box<RigidBody> b;

        vector<class Collision<RigidBody>*>().swap(colls);

        //realease the memory
        vector<class Octree<RigidBody>::OctreeNode*>().swap(octreeLeafs);
        
        //update the tree
        m_tree->updateTree(m_rBodies);
        
        //we fetch the leafs of the tree (where rigidbodies are)
        //only leaf containing more than one rigidbody are returned 
        m_tree->getLeafsContainingMoreThanOneObject(&octreeLeafs);
        
        //for each leafs we check if there are collision between the rigidbodies inside it
        for(int i = 0; i < octreeLeafs.size(); i++)
        {
            RigidBody *pt_a;
            RigidBody *pt_b;
            
            //check collision between all the rb in the leaf
            for(int j = 0; j < octreeLeafs.at(i)->m_rBodies.size(); j++)
            {
                //rigidbody A
                pt_a = octreeLeafs.at(i)->m_rBodies.at(j);
                a = box<RigidBody>::create(pt_a);
                for(int k = j+1; k < octreeLeafs.at(i)->m_rBodies.size(); k++)
                {
                    //rigidbody B
                    pt_b = octreeLeafs.at(i)->m_rBodies.at(k);
                    b = box<RigidBody>::create(pt_b);
                    vec3 n;
                    
                    //if the two boxes collide
                    if(box<RigidBody>::collide(a, b, n))
                    {
                        
                        collisionResp->addCollision(new Collision<RigidBody>(pt_a, a, pt_b, b, n));
                    }
                }
            }
        }

        //than we resolve the collisions
        collisionResp->resolveCollisions();
    }

    void debugOctree(vector<class Octree<RigidBody>::OctreeNode*> *nodes)
    {   
        m_tree->getLeafs(nodes);
    }
};