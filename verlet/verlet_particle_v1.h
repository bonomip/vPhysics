/*
VERLET PHYSISC

author: Paolo Bonomi

Real-Time Graphics Programming's Project - 2019/2020
*/

#pragma once

//GLM classes used in the application
#include <glm/glm.hpp>

//Physics class
#include <physics/verlet/verlet_physics_v1.h>

class vParticle
{
    typedef glm::vec3 vec3;

    vec3 m_pNow;
    vec3 m_pOld;

    float m_dt;

    vec3 m_forces;

    float m_mass;
    float m_drag;

    float m_worldSize;

public:
    int m_rbid, m_id; //rigidbody id, particle id 

    vParticle(const int &rb_id,const int &id,const vec3 &pos,const float &mass,const float &drag, const float worldSize)
    {
        m_pNow = pos;
        m_pOld = pos;

        m_mass = mass;
        m_drag = drag;

        m_rbid = rb_id;
        m_id = id;

        m_worldSize = worldSize;
    }

    int getId()
    {
        return m_id;
    }

    void update(const float &dt)
    {
        vec3 new_acc = apply_gravity()+m_forces / m_mass;
        float dump = 1.0f-m_drag*dt;
        vec3 new_pos = (1.0f+dump)*m_pNow - (dump)*m_pOld + new_acc * dt*dt;

        //ENFORCE WORLD COSTRAIN
        if(new_pos.y > m_worldSize) new_pos.y = m_worldSize;
        if(new_pos.y < -m_worldSize) new_pos.y = -m_worldSize;

        if(new_pos.x > m_worldSize) new_pos.x = m_worldSize;
        if(new_pos.x < -m_worldSize) new_pos.x = -m_worldSize;

        if(new_pos.z > m_worldSize) new_pos.z = m_worldSize;
        if(new_pos.z < -m_worldSize) new_pos.z = -m_worldSize;

        m_pOld = m_pNow;
        m_pNow = new_pos;
        m_dt = dt;
        m_forces = vec3(0.0f,0.0f,0.0f);
    }

    vec3 getPosition()
    {
        return m_pNow;
    }

    vec3 getLastPosition()
    {
        return m_pOld;
    }

    vec3 getVelocity()
    {
        return (m_pNow-m_pOld)/m_dt;
    }

    void setPosition(const vec3 &pos)
    {
        m_pNow = pos;
    }

    void setPositionConservingMomentum(const vec3 &p0)
    {
        /*vec3 v1 = (p0-m_pNow);
        float post = abs(glm::length(v1));
        std::cout << "PARTICLE CHANGE POSITION" << std::endl;
        if(post > 0.0f){
        vec3 p1 = m_pNow;
        m_pOld = m_pNow;
        m_pNow = p1+(v1)*(glm::length(m_pNow-m_pOld)/glm::length(v1));
        return;
        }*/
        //m_pOld = m_pNow;
        m_pNow = p0;
    }

    float getDt()
    {
        return m_dt;
    }

    float getMass()
    {
        return m_mass;
    }
private:
    vec3 apply_gravity()
    {
        return vec3(.0f, -9.81f, .0f)*m_mass;
    } 

};