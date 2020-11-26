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

        Collision(RigidBody* pt_a, box<RigidBody> a, RigidBody* pt_b, box<RigidBody> b, vec3 n)
        {
            this->pt_a = pt_a;
            this->pt_b = pt_b;
            this->a = a;
            this->b = b; 
            this->normal = n;

            //find collision point
        }
}; 