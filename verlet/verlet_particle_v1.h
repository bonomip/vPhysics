/*
VERLET PHYSISC

author: Paolo Bonomi

Real-Time Graphics Programming's Project - 2019/2020
*/

#pragma once

//GLM classes used in the application
#include <glm/glm.hpp>

typedef glm::vec3 vec3;

class vParticle
{
public:
protected:
    vec3 m_pNow;
    vec3 m_pOld;

    float m_dt;

    vec3 m_forces;

    float m_mass;
    float m_drag;

    float m_worldSize;

    //for sphere particle
    float m_radius;
    float m_bounciness;

    bool stop;

public:
    int m_rbid, m_id; //rigidbody id, particle id 

    vParticle(const int &rb_id,const int &id,const vec3 &pos,const float &mass,const float &drag, const float worldSize)
    {
        this->stop = false;

        m_pNow = pos;
        m_pOld = pos;

        m_mass = mass;
        m_drag = drag;

        m_rbid = rb_id;
        m_id = id;

        m_worldSize = worldSize;
    }

    bool getStop()
    {
        return this->stop;
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
protected:
    vec3 apply_gravity()
    {
        return vec3(.0f, -9.81f, .0f)*m_mass;
    } 

};

class SphereParticle : public vParticle 
{
    public:
    SphereParticle(const int &rb_id,const int &id,const vec3 &pos, const float &radius, const float &mass,const float &drag, const float &bounciness, const float &worldSize) :
    vParticle(rb_id, id, pos, mass, drag, worldSize)
    {
        this->m_radius = radius;
        this->m_bounciness = bounciness;
        //this->m_pOld = vec3(m_pOld.x+0.15f, m_pOld.y+0.15f, m_pOld.z);
    }

    void update(const float &dt)
    {
        vec3 new_acc = apply_gravity()+m_forces / m_mass;
        float dump = 1.0f-m_drag*dt;
        vec3 new_pos = (1.0f+dump)*m_pNow - (dump)*m_pOld + new_acc * dt*dt;

        enforceWorldConstraint(&new_pos);
        
        m_pOld = m_pNow;
        m_pNow = new_pos;

        m_dt = dt;
        m_forces = vec3(0.0f,0.0f,0.0f);
    }

    void enforceWorldConstraint(vec3 *new_pos)
    {
        enforcePositive(new_pos->x, this->m_pNow.x, this->m_radius, this->m_worldSize ); //positive x
        enforceNegative(new_pos->x, this->m_pNow.x, this->m_radius, this->m_worldSize ); //negative x
        enforcePositive(new_pos->y, this->m_pNow.y, this->m_radius, this->m_worldSize ); //positive x
        enforceNegative(new_pos->y, this->m_pNow.y, this->m_radius, this->m_worldSize ); //negative x
        enforcePositive(new_pos->z, this->m_pNow.z, this->m_radius, this->m_worldSize ); //positive x
        enforceNegative(new_pos->z, this->m_pNow.z, this->m_radius, this->m_worldSize ); //negative x
    }

    /*
        s: new position
        p: old position
        r: radius
        q: bound
    */

    void enforcePositive(float &s, float &p, float r, float q)
    {
        if(s + r > q )
        {
            if( s > q ) // first case -> point outside bound
            {
                std::cout << "verlet patricle enforce neg or pos -> not tested" << std::endl;
                s = s - 2.0f*(s-q); // reflect new position respect q
                p = p - 2.0f*(p-q); // reflect old position respect q
            } else //second case -> point inside bound
            {
                p = (q - r) + (s - p)*this->m_bounciness;
                s = q - r;
            }
        }
    }

    /*
        s: new position
        p: old position
        r: radius
        q: bound
    */
    void enforceNegative(float &s, float &p, float r, float q)
    {
        if(r - s > q )
        {
            if( -s > q ) // first case -> point outside bound
            {
                std::cout << "verlet patricle enforce neg or pos -> not tested" << std::endl;
                s = s + 2.0f*(s+q); // reflect new position respect q
                p = p + 2.0f*(p+q); // reflect old position respect q
            } else //second case -> point inside bound //COMMON ONE
            {
                p = (r - q) + (s - p)*this->m_bounciness;
                s = r - q;
            }
        }
    }
};