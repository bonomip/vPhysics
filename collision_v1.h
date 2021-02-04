/*
PHYSISC

author: Paolo Bonomi

Real-Time Graphics Programming's Project - 2019/2020
*/

#pragma once

#include <physics/verlet/verlet_rb_v1.h>
#include <physics/response_v1.h>
#include <physics/tools_v1.h>

class Collision
{
    public:
    vRigidBody *pt_a, *pt_b;

    vec3 normal;
    vector<int> m_id;
    vector<int> apId, bpId ; //particle's ID of A, B
    vector<vec3> apPos, bpPos; //new Position of particles A, B
    vector<Response*> resp;


    Collision(vRigidBody* a, vRigidBody* b, vec3 n)
    {
        this->pt_a = a;
        this->pt_b = b;
        this->normal = n;
        this->m_id = genId(pt_a->getId(), pt_b->getId());

        //std::cout << "\t\tCOLLISION - EVALUATE" << std::endl;
        evaluate();
    }

    private:

    struct triangleResp
    {
        vRigidBody::triangle tri;
        vec3 direction;
        vec3 intersection;

        triangleResp static create(vRigidBody::triangle t, vec3 dir, vec3 intersect)
        {
            triangleResp tr;
            tr.tri = t;
            tr.direction = dir;
            tr.intersection = intersect;
            return tr;
        }

        triangleResp static cumulate(triangleResp t0, triangleResp t1)
        {
            triangleResp r;
            r.tri = t0.tri;
            r.intersection = middlePoint(t0.intersection, t1.intersection);
            r.direction = middleVector(t0.direction, t1.direction);
            return r;
        }

        void static baricentricDistribution(vec3 t0, vec3 t1, vec3 t2, vec3 intersection, float &u, float &v, float &w)
        { //homogeneous
            vec3 v0 = t1 - t0;
            vec3 v1 = t2 - t0;
            vec3 v2 = intersection - t0;
            float d00 = dot(v0, v0);
            float d01 = dot(v0, v1);
            float d11 = dot(v1, v1);
            float d20 = dot(v2, v0);
            float d21 = dot(v2, v1);
            float denom = d00 * d11 - d01 * d01;
            v = (d11 * d20 - d01 * d21) / denom;
            w = (d00 * d21 - d01 * d20) / denom;
            u = 1.0f - v - w;
        }
    };

    void evaluate(Box * rb_a, Box * rb_b)
        {
            vec3 p_pos; //patricle pre collision position
            vec3 po_pos; //patricle pre collision laste frame position
            
            float px, py, pz;
            vector<vRigidBody::triangle> tris;
            vRigidBody::box b = rb_b->getBox();
            vRigidBody::box::getTrianglesfromBox(tris, rb_b->getBox());
        
        for(int i = 0; i < rb_a->getParticles()->size(); i++)
        { //for each particles of A
            p_pos = rb_a->getParticles()->at(i).getPosition();
            po_pos = rb_a->getParticles()->at(i).getLastPosition();

            vec3 v = p_pos-b.position;

            px = std::abs(glm::dot(v, b.x));
            py = std::abs(glm::dot(v, b.y));
            pz = std::abs(glm::dot(v, b.z));
            
            if(px <= b.w && py <= b.h && pz <= b.d) //the i-th patricle of A is inside B
            {   
                vec3 a_out, ao_out;
                vec3 particle_vel = rb_a->getParticles()->at(i).getPosition() - rb_a->getParticles()->at(i).getLastPosition();

                for(int j = 0; j < tris.size(); j++) //for each triangle
                {
                    vec3 intersection;
                    vec3 rayDir = normalize(p_pos-po_pos); //patricle position - patricle last position
                    vec3 origin = po_pos+(rayDir*(-2.0f)); //the origin is put far away from the collision point

                    if(vRigidBody::triangle::rejectTri(rayDir, tris.at(j))) continue;

                    if(vRigidBody::triangle::rayIntersect(origin, rayDir, tris.at(j), intersection))
                    {
                        //this method dosen't fit
                        //it dosen't take into consideration angular velocity 

                        reflectpatricle( a_out, ao_out, p_pos, rb_a->getParticles()->at(i).getLastPosition(),
                                rb_a->getMass(), rb_b->getVelocity(), rb_b->getMass(),
                                intersection, vRigidBody::triangle::getNormal(tris.at(j))
                        );
                            
                        resp.push_back(new Response(
                            Response::genId(rb_a->getId(),  rb_a->getParticles()->at(i).getId()),
                            &rb_a->getParticles()->at(i),
                            a_out,
                            ao_out
                            )
                        );

                        vec3 p1_out, p2_out, p3_out;
                        vec3 po1_out, po2_out, po3_out;
                        
                        vec3 p1_pos = rb_b->getParticles()->at(tris.at(j).i0).getPosition();
                        vec3 p2_pos = rb_b->getParticles()->at(tris.at(j).i1).getPosition();
                        vec3 p3_pos = rb_b->getParticles()->at(tris.at(j).i2).getPosition();

                        float b1, b2, b3;

                        triangleResp::baricentricDistribution(p1_pos, p2_pos, p3_pos, intersection, b1, b2, b3);

                        break;
                    } 
                }
            }   
        }
    }


    //this method will calculate the new state of patricle A after hitting a rigidbody B
    // out_pos = A post collision position
    // out_last_pos = A post collision last frame position
    // pos = A pre collision position
    // last_pos = A pre collision last frame position
    // mass_a = mass of patricle A
    // vel_b = velocity of rigidbody B
    // mass_b = mass of rigidbody B
    // intersection = intersection point
    // normal = intersection plane normal pointing A
    // [optional] radius_a = radius of patricle A
    static void reflectpatricle(vec3 &out_pos, vec3 &out_last_pos, vec3 pos, vec3 last_pos, float mass_a, vec3 vel_b, float mass_b, vec3 intersection, vec3 normal, float radius_a = 0.0f)
    {
        //adjust patricle position respect to intersection point and its radius (if present)
        vec3 pos1 = intersection + glm::normalize(pos - intersection) * radius_a;
        //compute velocity pre collision
        vec3 vel_a = pos - last_pos;
        //compute the normal velocity using relative velocity (vel_a - vel_b)
        vec3 vel_norm = glm::dot( vel_a - vel_b, normal ) * normal;
        //compute velocity post collision respect to mass
        vec3 vel_a1 = vel_a - ( 2.0f * mass_b / ( mass_a + mass_b ) ) * vel_norm;
        //compute the adjust factor
        float s = glm::length(pos-pos1);

        //compute the new state of the patricle -> current position + old position
        out_pos = s == 0 ? pos1 : pos1 + glm::normalize(vel_a1) * s;
        out_last_pos = out_pos - vel_a1;
    }

    /*
    
    OLD EVALUATE METHOD A LITTE BIT MORE PRECISE DUE TO THE COLLISION NORMAL CALCULATED AFTER ADJUSTED THE CENTER OF THE PATRICLE IN A VALID POSITION
    BOTH METHOD ARE IMPRECISE THOUGH.

    THE BEST WAY TO DO THIS IS EXPLAINED ABOVE AT (1)

    // a = current sphere position, a1 = adjusted sphere position, vel = velocity
    // &out_a = calculated current sphere position
    // &out_o = calculated previous frame sphere position
    static void foo(vec3 a, vec3 a1, vec3 vel, vec3 &out_a, vec3& out_o)
    {
        //adjust factor
        float s = glm::length(a-a1);

        out_a = s == 0 ? a1 : a1 + glm::normalize(vel) * s;
        out_o = out_a - vel;
    }

    void evaluate(Sphere *rb_a, Sphere * rb_b)
    {
        //current sphere position
        vec3 a = rb_a->getPosition();
        vec3 b = rb_b->getPosition();
        //previous frame sphere position
        vec3 oa = rb_a->getLastPosition();
        vec3 ob = rb_b->getLastPosition();
        //radius
        float ra = rb_a->getRadius();
        float rb = rb_b->getRadius();

        //intersection plane normal pointing A
        vec3 na = glm::normalize( a - b );
        //compute intesection point
        vec3 q = b + na * rb;
    
        //adjust -- A B's center position respect to q
        // 1 ---------------------------------------------------- SOLVE THIS BUG !!!!!
        //the most correct way to do this is
        //solving this system:
        //  
        //  vb, va = velocity es: ( a.position() - a.lastPosition() )
        //  
        //  { || b' - a' || = a.radius + b.radius
        //  { b' = b.lastPosition() + ß(b.position() - b.lastPosition() )
        //  { a' = a.lastPosition() + ß(b.position() - b.lastPosition() )
        //
        vec3 a1 = q + glm::normalize( a - q ) * ra;
        vec3 b1 = q + glm::normalize( b - q ) * rb;

        //velocity pre collision (relative to dt)
        vec3 va = a - oa;
        vec3 vb = b - ob;
        //normal vector between two spheres
        vec3 n = glm::normalize(a1 - b1);
        //relative velocity -> va - vb
        //normal velocity
        vec3 vn = glm::dot(va - vb, n)*n;
        
        //velocity post collision
        float ma = rb_a->getMass();
        float mb = rb_b->getMass();

        vec3 va1 = va - ( 2.0f * mb / ( ma + mb ) ) * vn;
        vec3 vb1 = vb + ( 2.0f * ma / ( ma + mb ) ) * vn;
       
        vec3 a2, oa2; //sphere A new position (current and old) post collision
        vec3 b2, ob2; //sphere B new position (current and old) post collision

        foo(a, a1, va1, a2, oa2);
        foo(b, b1, vb1, b2, ob2);

        resp.push_back(new Response(
            Response::genId(rb_b->getId(), 0 ), //in sphere there is only one patricle, thus id is always 0
            &rb_b->getParticles()->at(0),
            b2,
            ob2
        ));

        resp.push_back(new Response(
            Response::genId(rb_a->getId(), 0 ), //in sphere there is only one patricle, thus id is always 0
            &rb_a->getParticles()->at(0),
            a2,
            oa2
        ));
    } */

    void evaluate(Sphere *rb_a, Sphere * rb_b)
    {
        vec3 b_pos, a_pos, b_last, a_last;
        float b_radius, a_radius, a_mass, b_mass;

        a_pos = rb_a->getPosition();
        a_last = rb_a->getLastPosition();
        a_radius = rb_a->getRadius();
        a_mass = rb_a->getMass();

        b_pos = rb_b->getPosition();
        b_last = rb_b->getLastPosition();
        b_radius = rb_b->getRadius();
        b_mass = rb_b->getMass();
        

        //compute normals & intesection point
        vec3 a_normal = glm::normalize( a_pos - b_pos );
        vec3 b_normal = glm::normalize( b_pos - a_pos );
        vec3 intersection = b_pos + a_normal * b_radius;


        //the following method will put the result in these variables
        vec3 a, oa, b, ob;

        reflectpatricle(a, 
                        oa,
                        a_pos,
                        a_last,
                        a_mass,
                        b_pos - b_last,
                        b_mass,
                        intersection,
                        a_normal,
                        a_radius
                        );

        reflectpatricle(b, 
                    ob,
                    b_pos,
                    b_last,
                    b_mass,
                    a_pos - a_last,
                    a_mass,
                    intersection,
                    b_normal,
                    b_radius
                    );

        resp.push_back(new Response(
            Response::genId(rb_b->getId(), 0 ), //in sphere there is only one patricle, thus id is always 0
            &rb_b->getParticles()->at(0),
            b,
            ob
        ));

        resp.push_back(new Response(
            Response::genId(rb_a->getId(), 0 ), //in sphere there is only one patricle, thus id is always 0
            &rb_a->getParticles()->at(0),
            a,
            oa
        ));
    }

    void evaluate()
    {
        if ( this->pt_a->isBox() && this->pt_b->isBox() )
        {
            evaluate(dynamic_cast<Box*>(this->pt_a), dynamic_cast<Box*>(this->pt_b));
            evaluate(dynamic_cast<Box*>(this->pt_b), dynamic_cast<Box*>(this->pt_a));
        }
        if ( this->pt_a->isSphere() && this->pt_b->isSphere() )
        {
            evaluate(dynamic_cast<Sphere*>(this->pt_a), dynamic_cast<Sphere*>(this->pt_b));
        }


    }

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

    vector<Response*> getResponses()
    {
        return resp;
    }

    vector<int> getId()
    {
            return m_id;
    }
}; 


