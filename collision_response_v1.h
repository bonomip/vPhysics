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
    public : class Response
    {
        vector<int> apId, bpId ; //particle's ID of A, B
        vector<vec3> apPos, bpPos; //new Position of particles A, B
        vRigidBody *a, *b;
        Collision<RigidBody>* collision;
        vector<triangleResp> aTresp, bTresp; // triangle to apply collision forces
        int n_col = 0;

        vector<triangle> tring; //debug;
        vector<vec3> inter; //debug;
        vector<vec3> partic; //debug;
        vector<vec3> particl;

        public:

        Response(Collision<RigidBody> *col)
        {   
            this->collision = col;
            evaluate(col);
        }

        void apply()
        {
            std::cout << "\t\tCOLLISION RESPONSE - APPLY - collision id " << collision->getId().at(0) << "." << collision->getId().at(1) << std::endl;
            solvePartCollision(collision->pt_a, apId, apPos);
            solvePartCollision(collision->pt_b, bpId, bpPos);
            solveTriCollision(collision->pt_b, aTresp);
            solveTriCollision(collision->pt_a, bTresp);
        }

        void solvePartCollision(RigidBody* pt, vector<int> idx, vector<vec3> pos)
        {
            for(int i = 0; i < idx.size(); i++)
            {
                pt->getParticles()->at(i).setPositionConservingMomentum(pos.at(i));
            }
        }

        void solveTriCollision(RigidBody* pt, vector<triangleResp> trsp)
        {
            for(int i = 0; i < trsp.size(); i++)
            {
                float u, v, w;
                triangleResp tr = trsp.at(i);
                triangleResp::baricentricDistribution(tr, u, v, w);
                pt->getParticles()->at(tr.tri.i0).setPositionConservingMomentum(tr.tri.v0+(tr.direction));
                pt->getParticles()->at(tr.tri.i1).setPositionConservingMomentum(tr.tri.v1+(tr.direction));
                pt->getParticles()->at(tr.tri.i2).setPositionConservingMomentum(tr.tri.v2+(tr.direction));
            }
        }      

        int debugCollision(vector<vec3>* t, vector<vec3>* i, vector<vec3>* p, vector<vec3>* l)
        {
            if(tring.size() == 0) return 0;

            std::cout << "COLLISION_RESPONSE - debug collision" << std::endl; 
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
        }

        private:
        void evaluate(Collision<vRigidBody> *col)
        {
            std::cout << "\t\tCOLLISION RESPONSE - EVALUATE - 1 halfeval" << std::endl;
            halfEval(col->pt_a->getParticles(), col->b, col->pt_b->getId(), apId, apPos, aTresp );
            std::cout << "\t\tCOLLISION RESPONSE - EVALUATE - 2 halfeval" << std::endl;
            halfEval(col->pt_b->getParticles(), col->a, col->pt_a->getId(), bpId, bpPos, bTresp );
        }

        void addTresp(vector<triangleResp> &v, triangleResp trsp)
        {
            for(int i = 0; i < v.size(); i++)
                if( triangle::isEqual(v.at(i).tri, trsp.tri) )
                {
                    v.at(i) = triangleResp::cumulate(v.at(i), trsp);
                    return;
                }
            v.push_back(trsp);
        }

        void halfEval(vector<vParticle> *particles, box<RigidBody> b, int b_id, vector<int> &pId, vector<vec3> &pPos, vector<triangleResp> &tresp)
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
                    std::cout << "\t\tCOLLISION_RESPONSE - halfEval -> collision id " <<  collision->getId().at(0) << "." << collision->getId().at(1) << " patricle " << particles->at(i).m_id << " of rb " << particles->at(i).m_rbid;
                    std::cout << " is inside rb " << b_id << std::endl;
                    for(int j = 0; j < tris.size(); j++) //for each triangle
                    {
                        vec3 q;
                        vec3 rayDir = normalize(pp-particles->at(i).getLastPosition()); //patricle position - patricle last position
                        vec3 origin = particles->at(i).getLastPosition()+(rayDir*(-2.0f)); //the origin is put far away from the collision point

                        /*tring.push_back(tris.at(j)); //DEBUG triangle normal
                        inter.push_back(triangle::getNormal(tris.at(j))+triangle::getCentroid(tris.at(j))); //DEBUG red
                        partic.push_back(triangle::getCentroid(tris.at(j))); //DEBUG blue*/

                        if(triangle::rejectTri(rayDir, tris.at(j))){
                            //std::cout << "\tCOLLISION_RESPONSE - halfEval -> rejected triangle" << std::endl;
                            continue;
                        }

                        if(triangle::rayIntersect(origin, rayDir, tris.at(j), q))
                        {
                            std::cout << "\tCOLLISION_RESPONSE - halfEval -> found collision points" << std::endl;
                            n_col++;//debug
                            tring.push_back(tris.at(j)); //DEBUG
                            inter.push_back(q); //DEBUG
                            partic.push_back(pp); //DEBUG
                            particl.push_back(particles->at(i).getLastPosition()); //DEBUG


                            addTresp(tresp, triangleResp::create(tris.at(j), q-pp, q));
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
    vector<Response*> debug;
    vector<vector<int>> collisionId;


    public:
    
    CollisionResponse()
    {
    }   

    bool canAddColl(vector<int> col_id)
    {
        for(int i = 0; i < collisionId.size(); i++)
            if(col_id.at(0) == collisionId.at(i).at(0) && col_id.at(1) == collisionId.at(i).at(1))
                return false;
        return true;
    }

    void addCollision(Collision<RigidBody> *c)
    {   
        collisionId.push_back( Collision<RigidBody>::genId( c->pt_a->getId(),c->pt_b->getId() ) );
        resp.push_back(new Response(c)); // __________to remove
        //addResponses(col);
    } 

    void addResponses(Collision<RigidBody> c)
    { //TO OPTIMIZE (sort indexes? )
        for(int i = 0; i < c.getResponses().size(); i++)
        {
            for(int j = 0; j < resp.size(); j++)
                if(c.getResponses().at(i).getId() == resp.at(j).getId )
                    resp.at(j).update(c.getResponses().at(i));
            resp.push_back(c.getResponses().at(i));
        }
    }



    void resolveCollisions()
    {
        for_each(resp.begin(), resp.end(),
            [&](Response* r)
            {
                r->apply();
            });
        clearDebug();    
        updateDebug();
        clear();
        std::cout << "\t\t\tCOLLISION RESPONSE -> CLEAR DATA" << std::endl;
    }

    void clear()
    {
        vector<Response*>().swap(resp);
        vector<vector<int>>().swap(collisionId);
    }
    
    int debugCollision(vector<vec3>* t, vector<vec3>* i, vector<vec3>* p, vector<vec3>* l)
    { 
        int n = 0;
        for_each(debug.begin(), debug.end(),
            [&](Response* r)
            {
                n +=r->debugCollision(t, i, p, l);
            });
        return n;
    }

    int debugCollision2(vector<CollisionResponse<vRigidBody>::Response> &r)
    {

    }
    
    void clearDebug()
    {
        vector<Response*>().swap(debug);
    }

    void updateDebug()
    {
        resp.swap(debug);
    }
};