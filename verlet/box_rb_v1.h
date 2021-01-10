#pragma once

#include <physics/verlet/verlet_rb_v1.h>


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
};