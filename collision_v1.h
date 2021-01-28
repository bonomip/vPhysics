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

        void static baricentricDistribution(triangleResp trsp, float &u, float &v, float &w)
        { //homogeneous
            vec3 v0 = trsp.tri.v1 - trsp.tri.v0;
            vec3 v1 = trsp.tri.v2 - trsp.tri.v0;
            vec3 v2 = trsp.intersection - trsp.tri.v0;
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

    // a = current sphere position, a1 = adjusted sphere position, o = previous frame sphere position, nvel = velocity normalized
    // &out_a = calculated current sphere position
    // &out_o = calculated previous frame sphere position
    static void foo(vec3 a, vec3 a1, vec3 o, vec3 nvel, vec3 &out_a, vec3& out_o)
    {
        //adjust factor
        float s = glm::length(a-a1);
        //velocity magnitude -> equal mass 
        float d = glm::length(o-a);

        //vec3 o1 = a + ( o - a ); usless due to the result

        out_a = a1 + nvel * s;
        out_o = out_a - nvel * d;
    }

    void evaluate(Box * rb_a, Box * rb_b)
        {
            vec3 pp; //patricle position
            float px, py, pz;
            vector<vRigidBody::triangle> tris;
            vRigidBody::box b = rb_b->getBox();
            vRigidBody::box::getTrianglesfromBox(tris, rb_b->getBox());
        
        for(int i = 0; i < rb_a->getParticles()->size(); i++)
        { //for each particles of A
            pp = rb_a->getParticles()->at(i).getPosition();

            vec3 v = pp-b.position;

            px = std::abs(glm::dot(v, b.x));
            py = std::abs(glm::dot(v, b.y));
            pz = std::abs(glm::dot(v, b.z));
            
            if(px <= b.w && py <= b.h && pz <= b.d) //the i-th patricle of A is inside B
            {   
                //std::cout << "\t\tCOLLISION - halfEval -> collision id " <<  m_id.at(0) << "." << m_id.at(1) << " patricle " << rb_a->getParticles()->at(i).m_id << " of rb_a " << rb_a->getParticles()->at(i).m_rbid;
                //std::cout << " is inside rb_a " << b_id << std::endl;
                for(int j = 0; j < tris.size(); j++) //for each triangle
                {
                    vec3 q;
                    vec3 rayDir = normalize(pp-rb_a->getParticles()->at(i).getLastPosition()); //patricle position - patricle last position
                    vec3 origin = rb_a->getParticles()->at(i).getLastPosition()+(rayDir*(-2.0f)); //the origin is put far away from the collision point

                    if(vRigidBody::triangle::rejectTri(rayDir, tris.at(j))){
                        //std::cout << "\tCOLLISION_RESPONSE - halfEval -> rejected triangle" << std::endl;
                        continue;
                    }

                    if(vRigidBody::triangle::rayIntersect(origin, rayDir, tris.at(j), q))
                    {
                        //std::cout << "\tCOLLISION - halfEval -> found collision points" << std::endl;
                
                        //responses created by the collision are put in vector "resp"
                        resp.push_back(new Response(
                            Response::genId(rb_a->getId(),  rb_a->getParticles()->at(i).getId()),
                            &rb_a->getParticles()->at(i),
                            q,
                            rb_a->getParticles()->at(i).getLastPosition()
                        ));

                        vec3 d = q - pp;
                        float k0, k1, k2;
                        triangleResp trsp = triangleResp::create(tris.at(j), d, q);
                        triangleResp::baricentricDistribution(trsp, k0, k1, k2);

                        resp.push_back(new Response(
                            Response::genId(rb_b->getId(), tris.at(j).i0),
                            &rb_b->getParticles()->at(tris.at(j).i0),
                            rb_b->getParticles()->at(tris.at(j).i0).getPosition()+(d*k0),
                            rb_b->getParticles()->at(tris.at(j).i0).getLastPosition()
                        ));
                        resp.push_back(new Response(
                            Response::genId(rb_b->getId(), tris.at(j).i1),
                            &rb_b->getParticles()->at(tris.at(j).i1),
                            rb_b->getParticles()->at(tris.at(j).i1).getPosition()+(d*k1),
                            rb_b->getParticles()->at(tris.at(j).i1).getLastPosition()
                        ));
                        resp.push_back(new Response(
                            Response::genId(rb_b->getId(), tris.at(j).i2),
                            &rb_b->getParticles()->at(tris.at(j).i2),
                            rb_b->getParticles()->at(tris.at(j).i2).getPosition()+(d*k2),
                            rb_b->getParticles()->at(tris.at(j).i2).getLastPosition()
                        ));
                        break;
                    } 
                }
            }   
        }
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
        vec3 a1 = q + glm::normalize( a - q ) * ra;
        vec3 b1 = q + glm::normalize( b - q ) * rb; 

        //velocity pre collision (relative to dt)
        vec3 va = a - oa;
        vec3 vb = b - ob;
        //normal vector between two spheres
        vec3 n = glm::normalize(a1 - b1);
        //relative velocity
        vec3 vr = va - vb;
        //normal velocity
        vec3 vn = glm::dot(vr, n)*n;
        //velocity post collision EQUAL MASS
        vec3 va1 = va - vn;
        vec3 vb1 = vb + vn;

        vec3 a2, oa2; //sphere A new position (current and old) post collision
        vec3 b2, ob2; //sphere B new position (current and old) post collision
        foo(a, a1, oa, glm::normalize(va1), a2, oa2);
        foo(b, b1, ob, glm::normalize(vb1), b2, ob2);

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

        




        /* /////// NEW ALGORITHM
        vec3 a1, b1;    // position post collision           //out
        vec3 oa, ob;    // last step position pre collision  //constant
        vec3 oa1, ob1;  //last step position post collision  //out
        vec3 va, vb;    // velocity of A B (relative to dt)
        vec3 va1, vb1;  // velocity of A B (relative to dt) post collision
        float ra, rb;   // radius of A and B                 //constant
        //calculate pre collision vel
        va = a - oa;
        vb = b - ob;

        //calculate post collision vel
        va1 = a + na;
        vb1 = b + nb;
        */
         
        /*/////// OLD PROCEDURE

        //intesection point
        vec3 q = b + na * rb;

        //adjust -- A B's center position respect to q
        a1 = q + glm::normalize( a - q ) * ra;
        b1 = q + glm::normalize( b - q ) * rb; 
        //adjust -- last step A B's center position respect to post collision A B's center position
        oa1 = oa + ( a1 - a );
        ob1 = ob + ( b1 - b ); 
    
        //adjust -- velocity pre collision (relative to dt)
        va = a1 - oa1;
        vb = b1 - ob1;
        //normal vector between two spheres
        vec3 n = glm::normalize(a1 - b1);
        //relative velocity
        vec3 vr = va - vb;
        //normal velocity
        vec3 vn = glm::dot(vr, n)*n;
        //velocity post collision EQUAL MASS
        va1 = va - vn;
        vb1 = vb + vn;

        resp.push_back(new Response(
            Response::genId(rb_b->getId(), 0 ), //in sphere there is only one patricle, thus id is always 0
            &rb_b->getParticles()->at(0),
            b1,
            ob1
        ));

        resp.push_back(new Response(
            Response::genId(rb_a->getId(), 0 ), //in sphere there is only one patricle, thus id is always 0
            &rb_a->getParticles()->at(0),
            a1,
            oa1
        ));

        */
        
        
        
        //only for debug -- freeze state post collision
        //rb_b->getParticles()->at(0).stop2 = true;
        //rb_a->getParticles()->at(0).stop2 = true;

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


