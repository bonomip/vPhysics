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

    void getLeafsContainingMoreThanOneObject(vector<OctreeNode*> *result)
    {
        if(root->m_isLeaf){
            if(root->m_rBodies.size() > 1)
                result->push_back(root);
            return;
        }
        root->getLeafsContainingMoreThanOneObject(result);
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

       /* void update(vector<RigidBody> *bodies, const int &depth)
        {
            /*
                if(depth == 0)
                {

                }



            
            m_isLeaf = false;
            subNodes = new OctreeNode[8];

            for(int i = 0; i < 8; i++)
            { //for each potential subnodes...

                vector<RigidBody*> temp;
                vec3 newPos = m_pos;
                newPos.x = ((i & 2) == 2) ? newPos.x + m_size*0.25f : newPos.x - m_size*0.25f;
                newPos.y = ((i & 4) == 4) ? newPos.y - m_size*0.25f : newPos.y + m_size*0.25f;
                newPos.z = ((i & 1) == 1) ? newPos.z + m_size*0.25f : newPos.z - m_size*0.25f;

                for(int j = 0; j < m_bodies->size(); j++) //for each entry in the rigidbody list...
                {
                    box a, b;
                    a.position = newPos;
                    a.x_axis = vec3(1.0f, .0f, .0f);
                    a.y_axis = vec3(.0f, 1.0f, .0f);
                    a.z_axis = vec3(.0f, .0f, 1.0f);
                    a.h_width = m_size*0.25f;
                    a.h_height = m_size*0.25f;
                    a.h_depth = m_size*0.25f;

                    RigidBody rb = m_bodies->at(j);
                    vector<vec3> axis = rb.getXYZAxis();
                    b.position = rb.getPosition();
                    b.x_axis = axis.at(0);
                    b.y_axis = axis.at(0);
                    b.z_axis = axis.at(0);
                    b.h_width = rb.getSize().x;
                    b.h_height = rb.getSize().y;
                    b.h_depth = rb.getSize().z;

                    std::cout << "a pos = "<< glm::to_string(a.position) << " b pos = "<< glm::to_string(b.position) << std::endl;
                    vec3 d = a.position-b.position;
                    std::cout << "distance = " << glm::length(d) << std::endl;
                    if(collide(a,b)) //if the rigidbody collides with the sub nodes i hadd it to the temp list
                    {
                        std::cout << "rb id = " <<rb.getId() << std::endl;
                        std::cout << "node id = ";
                        for(int i = 0; i < this->getFullId().size(); i++)
                        {
                             std::cout << this->getFullId().at(i);
                        }

                        std::cout << "" << std::endl;
                        temp.push_back(&rb);
                        m_bodies->erase(m_bodies->begin()+j);
                    }
                }

                std::cout << "---" << std::endl;
                std::cout << temp.size() << std::endl;
                if(temp.size() == 1) //if only one rb is within the node
                {
                    subNodes[i] = OctreeNode(newPos, m_size * 0.5f, this, i);
                    subNodes[i].m_bodies->push_back(*temp.back());
                }
                if(temp.size() > 1) //if more than one rb is within the node
                {
                    subNodes[i] = OctreeNode(newPos, m_size * 0.5f, this, i);
                    for(int k = 0; k < temp.size(); k++)
                        subNodes[i].m_bodies->push_back(*temp.at(k));
                    if(depth>1)
                        subNodes[i].update(depth-1);
                }
            }
        } 
*/
       /* vector<int> getFullId()
        {
            vector<int> x;
            OctreeNode * parent;
            parent = m_parent;
            x.push_back(m_id);
            while(parent != NULL)
            {
                x.push_back(parent->m_id);
                parent = parent->m_parent;
            }

            return x;
        } */

        bool isLeaf()
        {
            return m_isLeaf;
        }

        void getLeafsContainingMoreThanOneObject(vector<OctreeNode*> *result)
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

