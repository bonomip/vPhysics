/*
PHYSISC

author: Paolo Bonomi

Real-Time Graphics Programming's Project - 2019/2020
*/

#pragma once

#include <physics/struct_v1.h>
#include <physics/verlet/verlet_particle_v1.h>
#include <physics/collision_solver_v1.h>
#include <physics/collision_v1.h>
#include <physics/verlet/verlet_rb_v1.h>

class Response
    {
        vector<int> id;
        vParticle * partic;
        vec3 pos;

        public:
        Response(vector<int> response_id, vParticle * particle, vec3 new_position)
        {   
            partic = particle;
            id = response_id;
            pos = new_position;
        }

        static vector<int> genId(int rigidbody_id, int particle_id)
        {
            vector<int> r;
            r.push_back(rigidbody_id);
            r.push_back(particle_id);
            return r;
        }

        void apply()
        {
            this->partic->setPositionConservingMomentum(pos);
        }

        vector<int> getId()
        {
            return id;
        }

        void update(Response * r)
        {
            vec3 n = this->pos - this->partic->getPosition();
            vec3 v = r->pos - r->partic->getPosition();
            this->pos = this->partic->getPosition()+n+v;
        }
    };
