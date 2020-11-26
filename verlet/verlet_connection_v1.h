#pragma once

//Physics class
#include <physics/verlet/verlet_particle_v1.h>
#include <stdio.h>

class vConnection
{
    vParticle * m_pa, * m_pb;
    float m_length;

public:
    vConnection(vParticle* pa, vParticle* pb)
    {
        m_pb = pb;
        m_pa = pa;
        m_length = glm::distance(pa->getPosition(), pb->getPosition());
    }

    /*virtual ~vConnection()
    {
        //free(m_pa);
        //free(m_pb);
    }*/

    //this function returns the difference of original and actual distance between two particles.
    void enforceConstraint()
    {
        glm::vec3 d = m_pb->getPosition() - m_pa->getPosition();
        float delta =  glm::length(d) - m_length;
        glm::vec3 nd = glm::normalize(d);
        m_pa->setPosition(m_pa->getPosition() + (m_pb->getMass()/(m_pa->getMass()+m_pb->getMass())) * delta * nd);
        m_pb->setPosition(m_pb->getPosition() - (m_pa->getMass()/(m_pa->getMass()+m_pb->getMass())) * delta * nd);
    }
};