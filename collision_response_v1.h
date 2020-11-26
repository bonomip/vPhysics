/*
PHYSISC

author: Paolo Bonomi

Real-Time Graphics Programming's Project - 2019/2020
*/

#pragma once

#include <physics/struct_v1.h>
#include <physics/verlet/verlet_particle_v1.h>
#include <physics/collision_detection_v1.h>
#include <physics/collision_v1.h>
#include <physics/verlet/verlet_rb_v1.h>

template <class RigidBody> class CollisionResponse
{
    class Response
    {
        vector<int> apId, bpId ; //particle's ID of A, B
        vector<vec3> apPos, bpPos; //new Positionof particles A, B
        vRigidBody *a, *b;
        Collision<RigidBody>* collision;

        public:

        Response(Collision<RigidBody> *col)
        {   
            this->collision = col;
            evaluate(col);
        }
        
        void apply()
        {
            for(int i = 0; i < apId.size(); i++)
            {
                collision->pt_a->getParticles()->at(i).setPosition(apPos.at(i));
            }

            for(int i = 0; i < bpId.size(); i++)
            {
                collision->pt_b->getParticles()->at(i).setPosition(bpPos.at(i));
            }
        }

        private:
        void evaluate(Collision<vRigidBody> *col)
        {
            halfEval(col->pt_a->getParticles(), col->b, apId, apPos );
            halfEval(col->pt_b->getParticles(), col->a, bpId, bpPos);
        }
     
        void halfEval(vector<vParticle> *particles, box<RigidBody> b, vector<int> &pId, vector<vec3> &pPos)
        {
            vec3 pp; //patricle position
            float px, py, pz;
            vector<triangle> tris;
            b.getTrianglesfromBox(tris, b);
            for(int i = 0; i < 8; i++)
            {   //for each particles of A
                
                pp = particles->at(i).getPosition();

                vec3 v = pp-b.position;

                px = std::abs(glm::dot(v, b.x));
                py = std::abs(glm::dot(v, b.y));
                pz = std::abs(glm::dot(v, b.z));
       
                if(px <= b.w && py <= b.h && pz <= b.d) //the i-th patricle of A is inside B
                {   
                    /*//local coords
                    vec4 lp = ir * glm::vec4((pp-t)/s, 1); // local position - inverse rotation
                    vec4 lw = ir * glm::vec4(-glm::normalize(a_patricles->at(i).getVelocity()), 0); //local direction vector
                    ///find collision using triangle struct 
                    //world coords
                    vec4 wp = (r * lp) + t*s
                    vec3 p = vec3(wp.x, wp.y, wp.z) */
                    for(int j = 0; j < 12; j++) //for each triangle
                    {
                        vec3 q;
                        vec3 plp = particles->at(i).getLastPosition();
                        vec3 pd = glm::normalize(pp-plp);
                        if(triangle::rayIntersect(
                                plp+(pd*-5.0f),
                                pd,
                                tris.at(j),
                                q
                            ))
                        {
                            pId.push_back(i);
                            pPos.push_back(q);
                            break;
                        } 
                    }
                }   
            }
        } 
    };

    vector<Response*> resp;
    public:
    CollisionResponse()
    {
    }

    void addCollision(Collision<RigidBody> *col)
    {   
        resp.push_back(new Response(col));
    } 
    
    void resolveCollisions()
    {
        for_each(resp.begin(), resp.end(),
            [&](Response* r)
            {
                r->apply();
            });
        vector<Response*>().swap(resp);
    }
};