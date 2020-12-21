/*
PHYSISC

author: Paolo Bonomi

Real-Time Graphics Programming's Project - 2019/2020
*/

#pragma once

#include <physics/verlet/verlet_rb_v1.h>

template <class RigidBody> class Collision
{
        public:
        RigidBody *pt_a, *pt_b;
        box<RigidBody> a, b;
        vec3 normal;
        vector<int> id;

        Collision(RigidBody* pt_a, box<RigidBody> a, RigidBody* pt_b, box<RigidBody> b, vec3 n)
        {
            this->pt_a = pt_a;
            this->pt_b = pt_b;
            this->a = a;
            this->b = b; 
            this->normal = n;
            this->id = genId(pt_a->getId(), pt_b->getId());

            //find collision point
        }
        private:
        static vector<int> genId2(int a, int b)
        {
            vector<int> id; id.push_back(a); id.push_back(b);
            return id;
        }
        public:
        static vector<int> genId(int a, int b)
        {
            if(a > b)
                return genId2(a,b);
            else
                return genId2(b,a);
        }

        vector<int> getId()
        {
            return id;
        }
}; 