/*
PHYSISC

author: Paolo Bonomi

Real-Time Graphics Programming's Project - 2019/2020
*/

#pragma once

#include <physics/verlet/verlet_particle_v1.h>

class Response
    {
        //the id is composed by the id of the rb and the id of the patricle
        vector<int> id;
        Movable * mov;
        vec3 pos;
        vec3 old_pos;

        public:
        Response(vector<int> response_id, Movable * movable, vec3 new_position, vec3 old_position)
        {   
            this->mov = movable;
            this->id = response_id;
            this->pos = new_position;
            this->old_pos = old_position;
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
            this->mov->setPosition(this->pos, this->old_pos);
        }

        vector<int> getId()
        {
            return this->id;
        }

        //this need to be review
        void update(Response * r)
        {
            vec3 n = this->pos - this->mov->getPosition();
            vec3 v = r->pos - r->mov->getPosition();
            this->pos = this->mov->getPosition()+n+v;
        }
    };
