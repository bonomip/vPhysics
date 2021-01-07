/*
VERLET PHYSISC

author: Paolo Bonomi

Real-Time Graphics Programming's Project - 2019/2020
*/
#pragma once

#include <glm/glm.hpp>
#include <vector>
#include <physics/struct_v1.h>

using std::vector;
using std::abs;
using glm::dot;


template <class RigidBody> class Octree
{
public:    
    enum NodePosition
    {
        topRightBack,       //000   0
        topRightFront,      //001   1
        topLeftBack,        //010   2
        topLeftFront,       //011   3
        bottomRightBack,    //100   4
        bottomRightFront,   //101   5
        bottomLeftBack,     //110   6
        bottomLeftFront,    //111   7
    };
    typedef glm::vec3 vec3;

    class OctreeNode;
    OctreeNode * root;
    int m_depth;

public:
    Octree(vec3 &position, const float &size, int &depth)
    {
        root = new OctreeNode(position, size, NULL, -1);
        m_depth = depth;
    }

    void updateTree(vector<RigidBody*> bodies)
    {
        root->clear();
        root->update(bodies, m_depth);
    }

    void getLeafsWithObj(vector<OctreeNode*> *result)
    {
        if(root->m_isLeaf){
            if(root->m_rBodies.size() > 1)
                result->push_back(root);
            return;
        }
        root->getLeafsWithObj(result);
    }

    void getLeafs(vector<OctreeNode*> *nodes)
    {
        if(root->m_isLeaf){
            nodes->push_back(root);
            return;
        }
        root->getLeafs(nodes);
    }    

    class OctreeNode
    {

    public:
        vector<RigidBody*> m_rBodies;
        vector<OctreeNode*> m_subNodes;

        float m_size;
        vec3 m_pos;
        OctreeNode * m_parent;
        int m_id;

        bool m_isLeaf = true;
    
        OctreeNode(){}

        OctreeNode(const vec3 &position, const float &size, OctreeNode * parent, int id)
        {
            m_size = size;
            m_pos = position;
            m_parent = parent;
            m_id = id;
        }

        //we dont take in cosideration the case were the rigidbody are outside the root of the octree
        void update(vector<RigidBody*> bodies, const int &depth)
        {
            if(depth == 0)
            {
                this->m_rBodies = bodies; 
            } 
            else 
            {
                vector<RigidBody*> temp;
                vec3 newPos;
                for(int i = 0; i < 8; i++)
                { // for each subnode

                    //find the position of the new subnode
                    newPos = this->m_pos;
                    newPos.x = ((i & 2) == 2) ? newPos.x + this->m_size*0.25f : newPos.x - this->m_size*0.25f;
                    newPos.y = ((i & 4) == 4) ? newPos.y - this->m_size*0.25f : newPos.y + this->m_size*0.25f;
                    newPos.z = ((i & 1) == 1) ? newPos.z + this->m_size*0.25f : newPos.z - this->m_size*0.25f;
                    
                    //check thought all rigidbody in the scene if they collide with the node i
                    //if they collide we add the rigidbody reference to the temp list
                    for(int j = 0; j < bodies.size(); j++)
                    { // for each rigid body in the scene

                        RigidBody *pt = bodies.at(j);
                        box<RigidBody> a, b;

                        //sub node
                        a = box<RigidBody>::create(    newPos,
                                        vec3(1.0f, .0f, .0f),
                                        vec3(.0f, 1.0f, .0f),
                                        vec3(.0f, .0f, 1.0f),
                                        m_size*0.25f,
                                        m_size*0.25f,
                                        m_size*0.25f
                                    );
                        //rigidbody
                        b = box<RigidBody>::create(pt);
                        vec3 n;

                        //check if rigidbody collide with the subnode
                        if(box<RigidBody>::collide(a, b, n))
                        {   //if they collide, add the rigidbody to the temp list of this subnode
                            temp.push_back(pt);
                        }
                    }

                    //if only one rigidbodies collides with the subnode. We create it, 
                    //but no further recursion is nedeed
                    if(temp.size() == 1)
                    {
                        this->m_isLeaf = false;
                        this->m_subNodes.push_back(new OctreeNode(newPos, this->m_size*0.5f, this, i));
                        vector<RigidBody*>().swap(temp);
                        //and we pass to the next subnode
                        continue;
                    }
                    //if more then one rigidbody collides with the subnode, we create it
                    //and we recursively update the branch
                    if(temp.size() > 1)
                    {
                        this->m_isLeaf = false;
                        this->m_subNodes.push_back(new OctreeNode(newPos, this->m_size*0.5f, this, i));
                        this->m_subNodes.back()->update(temp, depth-1);
                        //we empty the temp vector
                        vector<RigidBody*>().swap(temp);
                        //and we pass to the next subdnode
                        continue;
                    }
                    vector<RigidBody*>().swap(temp);
                    //if we are here temp.size() is equal to 0, we pass to the next node.
                }
            }
        }

        bool isLeaf()
        {
            return m_isLeaf;
        }

        void getLeafsWithObj(vector<OctreeNode*> *result)
        {
            for(int i = 0; i < this->m_subNodes.size(); i++)
            {
                if(this->m_subNodes.at(i)->isLeaf())
                {
                    if(this->m_subNodes.at(i)->m_rBodies.size() > 1)
                        result->push_back(this->m_subNodes.at(i));
                }
                else
                    this->m_subNodes.at(i)->getLeafs(result);
            }
        }

        void getLeafs(vector<OctreeNode*> *nodes)
        {
            for(int i = 0; i < this->m_subNodes.size(); i++)
            {
                if(this->m_subNodes.at(i)->isLeaf())
                    nodes->push_back(this->m_subNodes.at(i));
                else
                    this->m_subNodes.at(i)->getLeafs(nodes);
            }
        }

        void clear()
        {
            for(int i = 0; i < m_subNodes.size(); i++)
            {
                m_subNodes.at(i)->clear();
            }
            vector<OctreeNode*>().swap(m_subNodes);
            vector<RigidBody*>().swap(m_rBodies);
        }    
    };
};

