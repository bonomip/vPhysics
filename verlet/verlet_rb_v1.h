/*
VERLET PHYSISC

author: Paolo Bonomi

Real-Time Graphics Programming's Project - 2019/2020
*/

#pragma once

#include <glm/glm.hpp>
#include "glm/ext.hpp"
#include "glm/ext.hpp"
#include "glm/gtx/euler_angles.hpp"
#include "glm/gtx/string_cast.hpp"
#include <physics/verlet/verlet_particle_v1.h>
#include <physics/verlet/verlet_connection_v1.h>
#include <vector>


#include <stdlib.h>

class vRigidBody
{
    typedef glm::vec3 vec3;

    vec3 m_scale;
    int m_kind;
    GLfloat * m_diffuseColor;

    int BOX = 0;
    int SPHERE = 1;

    int m_id;
    bool m_useGravity;
    bool m_isKinematic;

    vector<vParticle> m_particles;
    vector<vConnection> m_connections;

public:
    vRigidBody(){}

    vRigidBody(const int &id,const int &kind,const vec3 &pos, GLfloat* color, const vec3 &e_rot, const vec3 &scale,const float &mass,const float &drag,const bool &useGravity,const bool &isKinematic, const float worldSize)
    {
        m_scale = scale;
        m_useGravity = useGravity;
        m_isKinematic = isKinematic;
        m_kind = kind;
        m_id = id;
        m_diffuseColor = new GLfloat[3]{color[0], color[1], color[2]};

        if(m_kind == BOX)
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
        if(m_kind == SPHERE)
        {

        }
    }

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

    vector<vParticle>* getParticles()
    {
        return &m_particles;
    }
    
    vector<vConnection>* getConnections()
    {
        return &m_connections;
    }

    vec3 getPosition()
    {
        //The center of the box is calculated as the half distance between particles to the external counterparts
        if(m_kind == BOX)
        {
            return (m_particles.at(6).getPosition()+m_particles.at(0).getPosition())*0.5f;
        }
        if(m_kind == SPHERE)
        {
            
        }
    }

    vec3 getLastPosition()
    {
        //The center of the box is calculated as the half distance between particles to the external counterparts
        if(m_kind == BOX)
        {
            return m_particles.at(0).getLastPosition()+
                            (m_particles.at(6).getLastPosition() - m_particles.at(0).getLastPosition())*.5f;
        }
        if(m_kind == SPHERE)
        {
            
        }
    }

    vec3 getXAxis(){
        if(m_kind == BOX){
            vec3 v0 = m_particles.at(0).getPosition();
            vec3 v1 = m_particles.at(1).getPosition();
            return glm::normalize(v1 - v0);
        }
    }

    vec3 getYAxis(){
        if(m_kind == BOX){
            vec3 v0 = m_particles.at(0).getPosition();
            vec3 v2 = m_particles.at(3).getPosition();
            
            vec3 x = getXAxis();
            return glm::normalize(glm::cross(v2-v0, x));
        }
    }

    vec3 getZAxis(){
        if(m_kind == BOX){
            vec3 x = getXAxis();
            vec3 y = getYAxis();
            return glm::cross(x, y);
        }
    }

    vector<vec3> getXYZAxis()
    {
        if(m_kind == BOX){
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

    glm::mat4 getRotation() //return rotation from 0f 0f 0f to actual rotation
    {
        glm::mat4 result;
        
        if(m_kind == BOX)
        {
        vector<vec3> axis = getXYZAxis();
        
        result[0] = glm::vec4(axis.at(0), 0);
        result[1] = glm::vec4(axis.at(1), 0);
        result[2] = glm::vec4(axis.at(2), 0);
        result[3] = glm::vec4(0, 0, 0, 1);

        return result;
        }
        if(m_kind == SPHERE)
        {

        }
    }
};