/*
PHYSISC

author: Paolo Bonomi

Real-Time Graphics Programming's Project - 2019/2020
*/

#pragma once

#include <physics/verlet/verlet_rb_v1.h>
#include <physics/response_v1.h>

class Collision
{
    public:
    vRigidBody *pt_a, *pt_b;
    box<vRigidBody> a, b;
    vec3 normal;
    vector<int> m_id;
    vector<int> apId, bpId ; //particle's ID of A, B
    vector<vec3> apPos, bpPos; //new Position of particles A, B
    //vector<triangleResp> aTresp, bTresp; // triangle to apply collision forces
    vector<Response*> resp;
    /*int n_col = 0;
    vector<triangle> tring; //debug;
    vector<vec3> inter; //debug;
    vector<vec3> partic; //debug;
    vector<vec3> particl;
    */

    Collision(vRigidBody* pt_a, box<vRigidBody> a, vRigidBody* pt_b, box<vRigidBody> b, vec3 n)
    {
        this->pt_a = pt_a;
        this->pt_b = pt_b;
        this->a = a;
        this->b = b; 
        this->normal = n;
        this->m_id = genId(pt_a->getId(), pt_b->getId());

        std::cout << "\t\tCOLLISION - EVALUATE - 1 halfeval" << std::endl;
        halfEval(pt_a, pt_b, b, pt_b->getId());
        std::cout << "\t\tCOLLISION - EVALUATE - 2 halfeval" << std::endl;
        halfEval(pt_b, pt_a, a, pt_a->getId());
    }

/*
    int debugCollision(vector<vec3>* t, vector<vec3>* i, vector<vec3>* p, vector<vec3>* l)
    {
        if(tring.size() == 0) return 0;

        std::cout << "COLLISION - debug collision" << std::endl; 
        std::cout << tring.size() << " " << inter.size() << " " << partic.size() << " " << particl.size() << std::endl;
        for(int i = 0 ; i < tring.size(); i++)
        {
            t->push_back(tring.at(i).v0);
            t->push_back(tring.at(i).v1);
            t->push_back(tring.at(i).v2);
        }
        for(int j = 0 ; j < inter.size(); j++)
        {
            i->push_back(inter.at(j));
            p->push_back(partic.at(j));
            l->push_back(particl.at(j));
        }
        return n_col;
    } */

    private:
    /* void addTresp(vector<triangleResp> &v, triangleResp trsp)
    {
        for(int i = 0; i < v.size(); i++)
            if( triangle::isEqual(v.at(i).tri, trsp.tri) )
            {
                v.at(i) = triangleResp::cumulate(v.at(i), trsp);
                return;
            }
        v.push_back(trsp);
    } */

    struct triangleResp
{
triangle tri;
vec3 direction;
vec3 intersection;

triangleResp static create(triangle t, vec3 dir, vec3 intersect)
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

    void halfEval(vRigidBody * rb_a, vRigidBody * rb_b, box<vRigidBody> b, int b_id)
    {
        vec3 pp; //patricle position
        float px, py, pz;
        vector<triangle> tris;
        b.getTrianglesfromBox(tris, b);
        for(int i = 0; i < rb_a->getParticles()->size(); i++)
        {   //for each particles of A
            pp = rb_a->getParticles()->at(i).getPosition();

            vec3 v = pp-b.position;

            px = std::abs(glm::dot(v, b.x));
            py = std::abs(glm::dot(v, b.y));
            pz = std::abs(glm::dot(v, b.z));
            if(px <= b.w && py <= b.h && pz <= b.d) //the i-th patricle of A is inside B
            {   
                std::cout << "\t\tCOLLISION - halfEval -> collision id " <<  m_id.at(0) << "." << m_id.at(1) << " patricle " << rb_a->getParticles()->at(i).m_id << " of rb_a " << rb_a->getParticles()->at(i).m_rbid;
                std::cout << " is inside rb_a " << b_id << std::endl;
                for(int j = 0; j < tris.size(); j++) //for each triangle
                {
                    vec3 q;
                    vec3 rayDir = normalize(pp-rb_a->getParticles()->at(i).getLastPosition()); //patricle position - patricle last position
                    vec3 origin = rb_a->getParticles()->at(i).getLastPosition()+(rayDir*(-2.0f)); //the origin is put far away from the collision point

                    /*tring.push_back(tris.at(j)); //DEBUG triangle normal
                    inter.push_back(triangle::getNormal(tris.at(j))+triangle::getCentroid(tris.at(j))); //DEBUG red
                    partic.push_back(triangle::getCentroid(tris.at(j))); //DEBUG blue*/

                    if(triangle::rejectTri(rayDir, tris.at(j))){
                        //std::cout << "\tCOLLISION_RESPONSE - halfEval -> rejected triangle" << std::endl;
                        continue;
                    }

                    if(triangle::rayIntersect(origin, rayDir, tris.at(j), q))
                    {
                        std::cout << "\tCOLLISION - halfEval -> found collision points" << std::endl;
                        /*n_col++;//debug
                        tring.push_back(tris.at(j)); //DEBUG
                        inter.push_back(q); //DEBUG
                        partic.push_back(pp); //DEBUG
                        particl.push_back(particles->at(i).getLastPosition()); //DEBUG
                        

                        addTresp(tresp, triangleResp::create(tris.at(j), q-pp, q));
                        pId.push_back(i);
                        pPos.push_back(q);*/

                        //responses created by the collision are put in vector "resp"
                        resp.push_back(new Response(
                            Response::genId(rb_a->getId(),  rb_a->getParticles()->at(i).getId()),
                            &rb_a->getParticles()->at(i),
                            q )
                        );

                        vec3 d = pp - q;
                        float k0, k1, k2;
                        triangleResp trsp = triangleResp::create(tris.at(j), d, q);
                        triangleResp::baricentricDistribution(trsp, k0, k1, k2);

                        resp.push_back(new Response(
                            Response::genId(rb_b->getId(), tris.at(j).i0),
                            &rb_b->getParticles()->at(tris.at(j).i0),
                            rb_b->getParticles()->at(tris.at(j).i0).getPosition()+(d*k0)
                        ));
                        resp.push_back(new Response(
                            Response::genId(rb_b->getId(), tris.at(j).i1),
                            &rb_b->getParticles()->at(tris.at(j).i1),
                            rb_b->getParticles()->at(tris.at(j).i1).getPosition()+(d*k1)
                        ));
                        resp.push_back(new Response(
                            Response::genId(rb_b->getId(), tris.at(j).i2),
                            &rb_b->getParticles()->at(tris.at(j).i2),
                            rb_b->getParticles()->at(tris.at(j).i2).getPosition()+(d*k2)
                        ));
                        break;
                    } 
                }
            }   
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