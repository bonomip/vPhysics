/*
PHYSISC

author: Paolo Bonomi

Real-Time Graphics Programming's Project - 2019/2020
*/

#pragma once

#include <physics/verlet/verlet_physics_v1.h>
#include <physics/octree_v1.h>
#include <physics/collision_v1.h>

class CollisionSolver
{
    public:
    vector<vRigidBody*>* m_rBodies;
    vector<Collision*> colls;
    vector<Response*> resp;
    vector<vector<int>> collisionId;

    float octreeSize = 10.0f;
    glm::vec3 octreeCenter = glm::vec3(0.0f, 0.0f, 0.0f);
    int octreeDepth = 5;
    vector<class Octree<vRigidBody>::OctreeNode*> octreeLeafs;
    Octree<vRigidBody> * m_tree;
    vector<vector<int>> colcheck;
    bool coltores = false;


    CollisionSolver(const float &worldSize) //world center is implicit at 0 0 0
    {
        this->m_tree = new Octree<vRigidBody>(this->octreeCenter, worldSize*2.0f, this->octreeDepth);
    }

    void setBodies(vector<vRigidBody*>* v)
    {
        this->m_rBodies = v;
    }

    void clean()
    {

    }

    void freeMemory()
    {
        vector<Collision*>().swap(this->colls);
        vector<class Octree<vRigidBody>::OctreeNode*>().swap(this->octreeLeafs);     
    }

    void update() //called each physics step
    {
        clearResp();
        freeMemory();
        
        //update the tree
        m_tree->updateTree(*m_rBodies);
        
        //we fetch the leafs of the tree (where rigidbodies are)
        //only leaf containing more than one rigidbody are returned 
        m_tree->getLeafsWithObj(&octreeLeafs);
        
        vRigidBody *pt_a, *pt_b;

        //for each leafs we check if there are collision between the rigidbodies inside it
        for(int i = 0; i < octreeLeafs.size(); i++)
        {
            //check collision between all the rb in the leaf
            for(int j = 0; j < octreeLeafs.at(i)->m_items.size(); j++)
            {   //rigidbody A
                pt_a = dynamic_cast<vRigidBody*>(octreeLeafs.at(i)->m_items.at(j));

                for(int k = j+1; k < octreeLeafs.at(i)->m_items.size(); k++)
                {   //rigidbody B
                    pt_b = dynamic_cast<vRigidBody*>(octreeLeafs.at(i)->m_items.at(k));
                    vec3 intersection;
            
                    if( canAddColl(Collision::genId(pt_a->getId(), pt_b->getId())) )
                        if(vRigidBody::collide(pt_a, pt_b, intersection))
                        {
                            coltores = true;
                            vector<int> df = Collision::genId(pt_a->getId(), pt_b->getId());
                            //std::cout << "COLLISION SOLVER - UPDATE -> collision id " << df.at(0) << "." << df.at(1);
                            //std::cout << " in leaf number " << i << std::endl;
                            addCollision(new Collision(pt_a, pt_b, intersection));
                        }
                }
            }
        }
        resolveCollisions();
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
        delete c;
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
        if(!coltores) return;
        coltores = false;
        for_each(resp.begin(), resp.end(),
            [&](Response* r)
            {
                r->apply();
            });
        clearResp();
        //std::cout << "\t\t\tCOLLISION SOLVER -> CLEAR DATA" << std::endl;
    }

    void clearResp()
    {
        vector<Response*>().swap(resp);
        vector<vector<int>>().swap(collisionId);
    }
};