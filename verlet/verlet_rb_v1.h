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
    protected:
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

    virtual vec3 getPosition(){}

    virtual vec3 getLastPosition(){}

    virtual vec3 getXAxis(){}

    virtual vec3 getYAxis(){}

    virtual vec3 getZAxis(){}

    virtual vector<vec3> getXYZAxis(){}

    //return rotation from 0f 0f 0f to actual rotation
    virtual glm::mat4 getRotation(){}

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
};