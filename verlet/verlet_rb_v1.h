/*
VERLET PHYSISC

author: Paolo Bonomi

Real-Time Graphics Programming's Project - 2020/2021
*/

#pragma once

#include <glm/glm.hpp>
#include "glm/ext.hpp"
#include "glm/ext.hpp"
#include "glm/gtx/euler_angles.hpp"
#include "glm/gtx/string_cast.hpp"

#include <physics/octree_v1.h>
#include <physics/verlet/verlet_particle_v1.h>
#include <physics/verlet/verlet_connection_v1.h>

#include <vector>
#include <stdlib.h>
#include <physics/struct_v1.h>

class Box;

class vRigidBody : public OItem
{
    protected:
    typedef glm::vec3 vec3;

    vec3 m_scale;
    int m_kind;
    GLfloat * m_diffuseColor;

    int m_id;
    bool m_useGravity;
    bool m_isKinematic;

    vector<vParticle> m_particles;
    vector<vConnection> m_connections;

public:
    vRigidBody(const int &id,const int &kind,const vec3 &pos, GLfloat* color, const vec3 &e_rot, const vec3 &scale,const float &mass,const float &drag,const bool &useGravity,const bool &isKinematic, const float worldSize)
    {
        m_scale = scale;
        m_useGravity = useGravity;
        m_isKinematic = isKinematic;
        m_kind = kind;
        m_id = id;
        m_diffuseColor = new GLfloat[3]{color[0], color[1], color[2]};
    }

    virtual ~vRigidBody() {}

    int getId()
    {
        return m_id;
    }

    void update(float dt)
    {
        for_each(m_particles.begin(), m_particles.end(),
            [&](vParticle &p)
            {
                p.update(dt);
            });
    }  

    void updateConstraint()
    {
        for(int i = 0; i < 2; i++) 
            for_each(m_connections.begin(), m_connections.end(),
                [&](vConnection &c)
                {
                    c.enforceConstraint();
                });
    }      

    int getKind()
    {
        return m_kind;
    }

    vec3 getSize()
    {
        return m_scale;
    }

    GLfloat* getColor()
    {
        return m_diffuseColor;
    }

    void setColor(GLfloat* color)
    {
        delete[] m_diffuseColor;
        m_diffuseColor = new GLfloat[3]{color[0], color[1], color[2]};
    }

    vector<vParticle>* getParticles()
    {
        return &m_particles;
    }
    
    vector<vConnection>* getConnections()
    {
        return &m_connections;
    }

    static bool collide(Box * a, Box * b)
    {
        std::cout << "cooooooolliiiidee" << std::endl;
        return false;
    }

    virtual vec3 getPosition() = 0;
    virtual vec3 getLastPosition() = 0;
    virtual vec3 getXAxis() = 0;
    virtual vec3 getYAxis() = 0;
    virtual vec3 getZAxis() = 0;
    virtual vector<vec3> getXYZAxis() = 0;
    virtual glm::mat4 getRotation() = 0; //return rotation from 0f 0f 0f to actual rotation
};

class Box : public vRigidBody
{
    public:
    Box(const int &id, const vec3 &pos, GLfloat* color, const vec3 &e_rot, const vec3 &scale,const float &mass,const float &drag,const bool &useGravity,const bool &isKinematic, const float worldSize)
    : vRigidBody(id, 0,pos, color, e_rot, scale, mass, drag, useGravity, isKinematic, worldSize)
    {
        vector<vec3> obj_pos;

            obj_pos.push_back(vec3( scale.x,    scale.y,    scale.z     ));
            obj_pos.push_back(vec3( -scale.x,   scale.y,    scale.z     ));
            obj_pos.push_back(vec3( -scale.x,   scale.y,    -scale.z    ));
            obj_pos.push_back(vec3( scale.x,    scale.y,    -scale.z    ));
            obj_pos.push_back(vec3( scale.x,    -scale.y,   scale.z     ));
            obj_pos.push_back(vec3( -scale.x,   -scale.y,   scale.z     ));
            obj_pos.push_back(vec3( -scale.x,   -scale.y,   -scale.z    ));
            obj_pos.push_back(vec3( scale.x,    -scale.y,   -scale.z    ));

            glm::mat4 rot = glm::eulerAngleYXZ(e_rot.y, e_rot.x, e_rot.z);
            for(int i = 0; i < 8; i++){
                glm::vec4 p = glm::vec4(obj_pos[i], 1) * rot;
                m_particles.push_back(vParticle(id, i, vec3(pos.x+p.x, pos.y+p.y, pos.z+p.z), mass/8, drag, worldSize));
            }

            //ORIZONTAL CONNECTION CLOCK WISE UP TO BOTTOM
            m_connections.push_back(vConnection(&m_particles.at( 0 ),&m_particles.at( 1 )));
            m_connections.push_back(vConnection(&m_particles.at( 1 ),&m_particles.at( 2 )));
            m_connections.push_back(vConnection(&m_particles.at( 2 ),&m_particles.at( 3 )));
            m_connections.push_back(vConnection(&m_particles.at( 3 ),&m_particles.at( 0 )));
            m_connections.push_back(vConnection(&m_particles.at( 4 ),&m_particles.at( 5 )));
            m_connections.push_back(vConnection(&m_particles.at( 5 ),&m_particles.at( 6 )));
            m_connections.push_back(vConnection(&m_particles.at( 6 ),&m_particles.at( 7 )));
            m_connections.push_back(vConnection(&m_particles.at( 7 ),&m_particles.at( 4 )));

            //VERTICAL CONNECTION CLOCK WISE UP TO BOTTOM
            m_connections.push_back(vConnection(&m_particles.at( 0 ),&m_particles.at( 4 )));
            m_connections.push_back(vConnection(&m_particles.at( 1 ),&m_particles.at( 5 )));
            m_connections.push_back(vConnection(&m_particles.at( 2 ),&m_particles.at( 6 )));
            m_connections.push_back(vConnection(&m_particles.at( 3 ),&m_particles.at( 7 )));

            //OBLIQUAL CONNECTION CLOCK WISE UP TO BOTTOM
            m_connections.push_back(vConnection(&m_particles.at( 0 ),&m_particles.at( 6 )));
            m_connections.push_back(vConnection(&m_particles.at( 1 ),&m_particles.at( 7 )));
            m_connections.push_back(vConnection(&m_particles.at( 2 ),&m_particles.at( 4 )));
            m_connections.push_back(vConnection(&m_particles.at( 3 ),&m_particles.at( 5 )));

            //BOTTOM AND TOP SQUARE 
            m_connections.push_back(vConnection(&m_particles.at( 0 ),&m_particles.at( 2 )));
            m_connections.push_back(vConnection(&m_particles.at( 2 ),&m_particles.at( 3 )));
            m_connections.push_back(vConnection(&m_particles.at( 4 ),&m_particles.at( 6 )));
            m_connections.push_back(vConnection(&m_particles.at( 5 ),&m_particles.at( 7 )));

            //LATERAL SQUARE
            m_connections.push_back(vConnection(&m_particles.at( 0 ),&m_particles.at( 7 )));
            m_connections.push_back(vConnection(&m_particles.at( 3 ),&m_particles.at( 4 )));
            m_connections.push_back(vConnection(&m_particles.at( 3 ),&m_particles.at( 6 )));
            m_connections.push_back(vConnection(&m_particles.at( 2 ),&m_particles.at( 7 )));
            m_connections.push_back(vConnection(&m_particles.at( 1 ),&m_particles.at( 6 )));
            m_connections.push_back(vConnection(&m_particles.at( 2 ),&m_particles.at( 5 )));
            m_connections.push_back(vConnection(&m_particles.at( 0 ),&m_particles.at( 5 )));
            m_connections.push_back(vConnection(&m_particles.at( 1 ),&m_particles.at( 4 )));
    }

    vec3 getPosition()
    {
        //The center of the box is calculated as the half distance between particles to the external counterparts

            return (m_particles.at(6).getPosition()+m_particles.at(0).getPosition())*0.5f;

    }

    vec3 getLastPosition()    
    {
        return m_particles.at(0).getLastPosition()+
                (m_particles.at(6).getLastPosition() - m_particles.at(0).getLastPosition())*.5f;

    }

    vec3 getXAxis(){
            vec3 v0 = m_particles.at(0).getPosition();
            vec3 v1 = m_particles.at(1).getPosition();
            return glm::normalize(v0 - v1);
    }

    vec3 getYAxis(){
            vec3 v0 = m_particles.at(0).getPosition();
            vec3 v2 = m_particles.at(2).getPosition();
            
            vec3 x = getXAxis();
            return glm::normalize(glm::cross(x, v2-v0));
    }

    vec3 getZAxis(){
            vec3 x = getXAxis();
            vec3 y = getYAxis();
            return glm::cross(x, y);
    }

    vector<vec3> getXYZAxis()
    {
            vec3 v0 = m_particles.at(0).getPosition();
            vec3 v1 = m_particles.at(1).getPosition();
            vec3 v2 = m_particles.at(2).getPosition();
            vec3 x = glm::normalize(v0 - v1);
            vec3 y = glm::normalize(glm::cross(x, v2-v0));
            vec3 z = glm::cross(x, y);

            vector<vec3> v;
            v.push_back(x);
            v.push_back(y);
            v.push_back(z);
            return v;
    }

    glm::mat4 getRotation() //return rotation from 0f 0f 0f to actual rotation
    {
        glm::mat4 result;

        vector<vec3> axis = getXYZAxis();
        
        result[0] = glm::vec4(axis.at(0), 0);
        result[1] = glm::vec4(axis.at(1), 0);
        result[2] = glm::vec4(axis.at(2), 0);
        result[3] = glm::vec4(0, 0, 0, 1);

        return result;
    }

    bool isMember(vec3 node_pos, float node_size)
    {
        box<vRigidBody> a, b;

        //sub node
        a = box<vRigidBody>::create(    node_pos,
                        vec3(1.0f, .0f, .0f),
                        vec3(.0f, 1.0f, .0f),
                        vec3(.0f, .0f, 1.0f),
                        node_size*0.25f,
                        node_size*0.25f,
                        node_size*0.25f
                    );
        //rigidbody
        b = box<vRigidBody>::create(this);
        vec3 n;

        //check if rigidbody collide with the subnode
        return box<vRigidBody>::collide(a, b, n);
    }
};