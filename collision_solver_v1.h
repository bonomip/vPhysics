/*
PHYSISC

author: Paolo Bonomi

Real-Time Graphics Programming's Project - 2019/2020
*/

#pragma once

#include <physics/verlet/verlet_physics_v1.h>
#include <physics/octree_v1.h>
#include <physics/struct_v1.h>
#include <physics/collision_v1.h>

template <class RigidBody> class CollisionSolver
{
    public:
    vector<RigidBody*> m_rBodies;
    vector<Collision*> colls;
    vector<Response*> resp;
    vector<Response*> debug;
    vector<vector<int>> collisionId;

    float octreeSize = 10.0f;
    glm::vec3 octreeCenter = glm::vec3(0.0f, 0.0f, 0.0f);
    int octreeDepth = 5;
    vector<class Octree<RigidBody>::OctreeNode*> octreeLeafs;
    Octree<RigidBody> * m_tree;
    vector<vector<int>> colcheck;
    bool coltores = false;


    CollisionSolver(const float &worldSize) //world center is implicit at 0 0 0
    {
        m_tree = new Octree<RigidBody>(octreeCenter, worldSize*2.0f, octreeDepth);
    }

    void addRigidBody(RigidBody * rb)
    {
        m_rBodies.push_back(rb);
    }

    void clean()
    {
        vector<RigidBody*>().swap(m_rBodies);
    }

    void update() //called each physics step
    {
        clearResp();

        //struct used for collision detection
        box<RigidBody> a;
        box<RigidBody> b;

        vector<Collision*>().swap(colls);

        //realease the memory
        vector<class Octree<RigidBody>::OctreeNode*>().swap(octreeLeafs);
        
        //update the tree
        m_tree->updateTree(m_rBodies);
        
        //we fetch the leafs of the tree (where rigidbodies are)
        //only leaf containing more than one rigidbody are returned 
        m_tree->getLeafsWithObj(&octreeLeafs);
        
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
                    
                    if( canAddColl(Collision::genId(pt_a->getId(), pt_b->getId())) )
                        if(box<RigidBody>::collide(a, b, n))
                        {
                            coltores = true;
                            vector<int> df = Collision::genId(pt_a->getId(), pt_b->getId());
                            //std::cout << "COLLISION SOLVER - UPDATE -> collision id " << df.at(0) << "." << df.at(1);
                            //std::cout << " in leaf number " << i << std::endl;
                            addCollision(new Collision(pt_a, a, pt_b, b, n));
                        }
                }
            }
        }
        if(coltores)
        { //if there are collision to resolve
            //than we resolve the collisions
            resolveCollisions();
            coltores = false;
        } else 
        {
            clearDebug();
        }
        
    }

    bool canAddColl(vector<int> col_id)
    {
        for(int i = 0; i < collisionId.size(); i++)
            if(col_id.at(0) == collisionId.at(i).at(0) && col_id.at(1) == collisionId.at(i).at(1))
                return false;
        return true;
    }

    void addCollision(Collision * c)
    {   
        collisionId.push_back( Collision::genId( c->pt_a->getId(),c->pt_b->getId() ) );
        addResponses(*c);
    } 

    void addResponses(Collision c)
    { //TO OPTIMIZE (sort indexes? )
        for(int i = 0; i < c.getResponses().size(); i++)
        {
            for(int j = 0; j < resp.size(); j++)
                if(c.getResponses().at(i)->getId() == resp.at(j)->getId() )
                    resp.at(j)->update(c.getResponses().at(i));
            resp.push_back(c.getResponses().at(i));
        }
    }

    void resolveCollisions()
    {
        for_each(resp.begin(), resp.end(),
            [&](Response* r)
            {
                r->apply();
            });
        //clearDebug();    
        //updateDebug();
        clearResp();
        //std::cout << "\t\t\tCOLLISION SOLVER -> CLEAR DATA" << std::endl;
    }

    void clearResp()
    {
        vector<Response*>().swap(resp);
        vector<vector<int>>().swap(collisionId);
    }
    
    void clearDebug()
    {
        vector<Response*>().swap(debug);
    }

    void updateDebug()
    {
        resp.swap(debug);
    }

    void debugOctree(vector<class Octree<RigidBody>::OctreeNode*> *nodes)
    {   
        m_tree->getLeafs(nodes);
    }
};