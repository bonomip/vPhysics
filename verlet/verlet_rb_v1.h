/*
VERLET PHYSISC

author: Paolo Bonomi

Real-Time Graphics Programming's Project - 2020/2021
*/

#pragma once

#include <glm/glm.hpp>
#include "glm/ext.hpp"
#include "glm/ext.hpp"
#include "glm/gtx/euler_angles.hpp"
#include "glm/gtx/string_cast.hpp"

#include <physics/octree_v1.h>
#include <physics/verlet/verlet_particle_v1.h>
#include <physics/verlet/verlet_connection_v1.h>

#include <vector>
#include <stdlib.h>

class Box;

class vRigidBody : public OItem
{
    public:

    struct triangle
    {
        vec3 v0, v1, v2;
        int i0, i1, i2; //id of the patricle 

        static triangle create(vec3 a, vec3 b, vec3 c, int ida, int idb, int idc)
        {
            triangle t;
            t.v0 = a;
            t.v1 = b;
            t.v2 = c;
            t.i0 = ida;
            t.i1 = idb;
            t.i2 = idc;
            return t;
        }
        
        bool static isEqual(triangle t, triangle s)
        {
            return t.i0 == s.i0 && t.i1 == s.i1 && t.i2 == s.i2;
        }
        
        static vec3 getCentroid(triangle tri)
        {
            float x, y, z;
            x = (tri.v0.x+tri.v1.x+tri.v2.x)/3.0f;
            y = (tri.v0.y+tri.v1.y+tri.v2.y)/3.0f;
            z = (tri.v0.z+tri.v1.z+tri.v2.z)/3.0f;
            return vec3(x,y,z);
        }

        static vec3 getNormal(triangle tri)
        {
            return glm::normalize(glm::cross( tri.v1-tri.v0, tri.v2-tri.v0 ) );
        }

        static bool rejectTri(vec3 rayDir, triangle tri)
        {
            vec3 n = glm::normalize(glm::cross( tri.v1-tri.v0, tri.v2-tri.v0 ) );
            return glm::dot(n, rayDir) >= 0;
        }

        static bool rayIntersect(vec3 rayOrigin, vec3 rayDir, triangle tri, vec3 &intersection)
        {
            vec3 e1, e2, p, s, q;
            const float eps = 0.0001;
            float t, u, v, tmp;
            e1 = tri.v1 - tri.v0;
            e2 = tri.v2 - tri.v0;
            p = cross(rayDir, e2);
            tmp = dot(p, e1);

            if(tmp > -eps && tmp < eps)
            {
                return false; // This ray is parallel to this triangle.
            }

            tmp = 1.0 / tmp;
            s = rayOrigin - tri.v0;
            u = tmp * dot(s,p);
            if(u < 0.0 || u > 1.0)
            {
                return false;
            }

            q = cross(s, e1);
            v = tmp * dot(rayDir, q);

            if(v < 0.0 || u+v > 1.0)
            {
                return false;
            }

            t = tmp * dot(e2, q);

            if(t >= 0) // ray intersection
            {           
                intersection = rayOrigin + (t * rayDir);
                return true;
            } else // This means that there is a line intersection but not a ray intersection.
                return false; 
        }
    };

    struct box
    {
        vec3 position, x, y, z; //position + axis
        float w, h, d; //HALF SIZE
        int id;

        static void getTrianglesfromBox(vector<triangle>& result, box a)
        {
            vec3 x = a.x*a.w;
            vec3 y = a.y*a.h;
            vec3 z = a.z*a.d;

            //1 top face CW                                     //tri vertx -> partic id
            vec3 v1 = a.position+(-1.0f*x)+y+z;                 //1 -> 1
            vec3 v2 = a.position+x+y+z;                         //2 -> 0
            vec3 v3 = a.position+x+y+(-1.0f*z);                 //3 -> 3
            vec3 v4 = a.position+(-1.0f*x)+y+(-1.0f*z);         //4 -> 2
            //2 bottom face CW
            vec3 v5 = a.position+(-1.0f*x)+(-1.0f*y)+z;         //5 -> 5
            vec3 v6 = a.position+x+(-1.0f*y)+z;                 //6 -> 4
            vec3 v7 = a.position+x+(-1.0f*y)+(-1.0f*z);         //7 -> 7
            vec3 v8 = a.position+(x*-1.0f)+(-1.0f*y)+(-1.0f*z); //8 -> 6
            
            //left 
        result.push_back(triangle::create(v1, v4, v5, 1, 2, 5));
        result.push_back(triangle::create(v4, v8, v5, 2, 6, 5));


            //top
        result.push_back(triangle::create(v1, v2, v4, 1, 0, 2));
        result.push_back(triangle::create(v2, v3, v4, 0, 3, 2));

            //right
        result.push_back(triangle::create(v3, v2, v7, 3, 0, 7));
        result.push_back(triangle::create(v2, v6, v7, 0, 4, 7));

        //front
        result.push_back(triangle::create(v1, v6, v2, 1, 4, 0));
        result.push_back(triangle::create(v1, v5, v6, 1, 5, 4));

        //back
        result.push_back(triangle::create(v4, v3, v7, 2, 3, 7));
        result.push_back(triangle::create(v4, v7, v8, 2, 7, 6));

        //bottom
        result.push_back(triangle::create(v5, v7, v6, 5, 7, 4));
        result.push_back(triangle::create(v5, v8, v7, 5, 6, 7));
        }

        static bool collide(box a, box b, vec3 &intersection)
        {
            vec3 t = b.position-a.position;
            //std::cout << "a pos = " << glm::to_string(a.position) << " - b pos = " << glm::to_string(b.position) << std::endl;
            //case 1
            if( abs(dot(t, a.x) ) > 
                a.w+
                abs(b.w*dot(a.x,b.x))+
                abs(b.h*dot(a.x,b.y))+
                abs(b.d*dot(a.x,b.z)) )
                return false;
            int i = abs(dot(t, a.x));
            intersection = a.x;
            //case 2
            if( abs(dot(t, a.y)) >  
                a.h+
                abs(b.w*dot(a.y,b.x))+
                abs(b.h*dot(a.y,b.y))+
                abs(b.d*dot(a.y,b.z)) )
                return false;
            if(abs(dot(t, a.y)) < i)
            {
                i = abs(dot(t, a.y));
                intersection = a.y;
            }
            //case 3
            if( abs(dot(t,a.z)) >   
                a.d+
                abs(b.w*dot(a.z,b.x))+
                abs(b.h*dot(a.z,b.y))+
                abs(b.d*dot(a.z,b.z)) )
                return false;
            if(abs(dot(t, a.z)) < i)
            {
                i = abs(dot(t, a.z));
                intersection = a.z;
            }
            //case 4    
            if( abs(dot(t,b.x)) >   
                b.w+
                abs(a.w*dot(a.x,b.x))+
                abs(a.h*dot(a.y,b.x))+
                abs(a.d*dot(a.z,b.x)) )
                return false;
            if(abs(dot(t, b.x)) < i)
            {
                i = abs(dot(t, b.x));
                intersection = b.x;
            }
            //case 5
            if( abs(dot(t,b.y)) >   
                b.h+
                abs(a.w*dot(a.x,b.y))+
                abs(a.h*dot(a.y,b.y))+
                abs(a.d*dot(a.z,b.y)) )
                return false;
            if(abs(dot(t, b.y)) < i)
            {
                i = abs(dot(t, b.y));
                intersection = b.y;
            }
            //case 6
            if( abs(dot(t,b.z)) >
                b.d+
                abs(a.w*dot(a.x,b.z))+
                abs(a.h*dot(a.y,b.z))+
                abs(a.d*dot(a.z,b.z)) )
                return false;
            if(abs(dot(t, b.z)) < i)
            {
                i = abs(dot(t, b.z));
                intersection = b.z;
            }
            //case 7
            if( abs(dot(t,a.z)*dot(a.y,b.x)-dot(t,a.y)*dot(a.z,b.x)) > 
                abs(a.h*dot(a.z,b.x))+
                abs(a.d*dot(a.y,b.x))+
                abs(b.h*dot(a.x,b.z))+
                abs(b.d*dot(a.x,b.y)) )
                return false;
            
            if(abs(dot(t,a.z)*dot(a.y,b.x)-dot(t,a.y)*dot(a.z,b.x)) < i)
            {
                i = abs(dot(t,a.z)*dot(a.y,b.x)-dot(t,a.y)*dot(a.z,b.x));
                intersection = cross(a.x, b.x);
            }
            //case 8
            if( abs(dot(t,a.z)*dot(a.y,b.y)-dot(t,a.y)*dot(a.z,b.y)) > 
                abs(a.h*dot(a.z,b.y))+
                abs(a.d*dot(a.y,b.y))+
                abs(b.w*dot(a.x,b.z))+
                abs(b.d*dot(a.x,b.x)) )
                return false;
                
            if( abs(dot(t,a.z)*dot(a.y,b.y)-dot(t,a.y)*dot(a.z,b.y)) < i)
            {
                i = abs(dot(t,a.z)*dot(a.y,b.y)-dot(t,a.y)*dot(a.z,b.y));
                intersection = cross(a.x, b.y);
            }
            //case 9
            if( abs(dot(t,a.z)*dot(a.y,b.z)-dot(t,a.y)*dot(a.z,b.z)) > 
                abs(a.h*dot(a.z,b.z))+
                abs(a.d*dot(a.y,b.z))+
                abs(b.w*dot(a.x,b.y))+
                abs(b.h*dot(a.x,b.x)) )
                return false;
            if( abs(dot(t,a.z)*dot(a.y,b.z)-dot(t,a.y)*dot(a.z,b.z)) < i)
            {
                i = abs(dot(t,a.z)*dot(a.y,b.z)-dot(t,a.y)*dot(a.z,b.z));
                intersection = cross(a.x, b.z);
            }
            //case 10
            if( abs(dot(t,a.x)*dot(a.z,b.x)-dot(t,a.z)*dot(a.x,b.x)) > 
                abs(a.w*dot(a.z,b.x))+
                abs(a.d*dot(a.x,b.x))+
                abs(b.h*dot(a.y,b.z))+
                abs(b.d*dot(a.y,b.y)) )
                return false;
            if( abs(dot(t,a.x)*dot(a.z,b.x)-dot(t,a.z)*dot(a.x,b.x)) < i)
            {
                i = abs(dot(t,a.x)*dot(a.z,b.x)-dot(t,a.z)*dot(a.x,b.x));
                intersection = cross(a.y, b.x);
            }
            //case 11
            if( abs(dot(t,a.x)*dot(a.z,b.y)-dot(t,a.z)*dot(a.x,b.y)) > 
                abs(a.w*dot(a.z,b.y))+
                abs(a.d*dot(a.x,b.y))+
                abs(b.w*dot(a.y,b.z))+
                abs(b.d*dot(a.y,b.x)) )
                return false;  
            if( abs(dot(t,a.x)*dot(a.z,b.y)-dot(t,a.z)*dot(a.x,b.y)) < i)
            {
                i = abs(dot(t,a.x)*dot(a.z,b.y)-dot(t,a.z)*dot(a.x,b.y));
                intersection = cross(a.y, b.y);  
            }      
            //case 12
            if( abs(dot(t,a.x)*dot(a.z,b.z)-dot(t,a.z)*dot(a.x,b.z)) > 
                abs(a.w*dot(a.z,b.z))+
                abs(a.d*dot(a.x,b.z))+
                abs(b.w*dot(a.y,b.y))+
                abs(b.h*dot(a.y,b.x)) )
                return false;
            if( abs(dot(t,a.x)*dot(a.z,b.z)-dot(t,a.z)*dot(a.x,b.z)) < i)
            {
                i = abs(dot(t,a.x)*dot(a.z,b.z)-dot(t,a.z)*dot(a.x,b.z));
                intersection = cross(a.y, b.z);
            }
            //case 13
            if( abs(dot(t,a.y)*dot(a.x,b.x)-dot(t,a.x)*dot(a.y,b.x)) > 
                abs(a.w*dot(a.y,b.x))+
                abs(a.h*dot(a.x,b.x))+
                abs(b.h*dot(a.z,b.z))+
                abs(b.d*dot(a.z,b.y)) )
                return false;
            if( abs(dot(t,a.y)*dot(a.x,b.x)-dot(t,a.x)*dot(a.y,b.x)) < i)
            {
                i = abs(dot(t,a.y)*dot(a.x,b.x)-dot(t,a.x)*dot(a.y,b.x));
                intersection = cross(a.z, b.x);
            }
            //case 14
            if( abs(dot(t,a.y)*dot(a.x,b.y)-dot(t,a.x)*dot(a.y,b.y)) > 
                abs(a.w*dot(a.y,b.y))+
                abs(a.h*dot(a.x,b.y))+
                abs(b.w*dot(a.z,b.z))+
                abs(b.d*dot(a.z,b.x)) )
                return false;
            if( abs(dot(t,a.y)*dot(a.x,b.y)-dot(t,a.x)*dot(a.y,b.y)) < i)
            {
                i = abs(dot(t,a.y)*dot(a.x,b.y)-dot(t,a.x)*dot(a.y,b.y));
                intersection = cross(a.z, b.y);
            }
            //case 15
            if( abs(dot(t,a.y)*dot(a.x,b.z)-dot(t,a.x)*dot(a.y,b.z)) > 
                abs(a.w*dot(a.y,b.z))+
                abs(a.h*dot(a.x,b.z))+
                abs(b.w*dot(a.z,b.y))+
                abs(b.h*dot(a.z,b.x)) )
                return false;
            if( abs(dot(t,a.y)*dot(a.x,b.z)-dot(t,a.x)*dot(a.y,b.z)) < i)
            {
                i = abs(dot(t,a.y)*dot(a.x,b.z)-dot(t,a.x)*dot(a.y,b.z));
                intersection = cross(a.z, b.z);
            }
            return true;
        }
        
        static box create(Box* pt);

        static box create(vec3 pos, vec3 x_axis, vec3 y_axis, vec3 z_axis, float width, float height, float depth)
        {
            box a;
            a.position = pos;
            a.x = x_axis;
            a.y = y_axis;
            a.z = z_axis;
            a.w = width;
            a.h = height;
            a.d = depth;
            return a;
        }
    };

    protected:
    typedef glm::vec3 vec3;

    vec3 m_scale;
    GLfloat * m_diffuseColor;

    int m_id;
    int m_kind;
    bool m_useGravity;
    bool m_isKinematic;

    vector<vParticle> m_particles;
    vector<vConnection> m_connections;

public:

    vRigidBody(const vRigidBody& rb) = delete; //disallow copy
    
    vRigidBody& operator=(const vRigidBody& copy) = delete;

    vRigidBody(const int &id,const int &kind,const vec3 &pos, GLfloat* color, const vec3 &e_rot, const vec3 &scale,const float &mass,const float &drag,const bool &useGravity,const bool &isKinematic, const float worldSize)
    {
        m_scale = scale;
        m_useGravity = useGravity;
        m_isKinematic = isKinematic;
        m_kind = kind;
        m_id = id;
        m_diffuseColor = new GLfloat[3]{color[0], color[1], color[2]};
    }

    virtual ~vRigidBody() {}

    void update(float dt)
    {
        for_each(m_particles.begin(), m_particles.end(),
            [&](vParticle &p)
            {
                p.update(dt);
            });
    }  

    void updateConstraint()
    {
        for(int i = 0; i < 2; i++) 
            for_each(m_connections.begin(), m_connections.end(),
                [&](vConnection &c)
                {
                    c.enforceConstraint();
                });
    }      

    bool isBox(){ return m_kind == 0; }     // 0 for boxes

    bool isSphere(){ return m_kind == 1; }  // 1 for Spheres

    vector<vParticle>* getParticles() { return &m_particles; }
    
    vector<vConnection>* getConnections() { return &m_connections; }

    vec3 getSize() { return m_scale; }

    int getId() { return m_id; }

    GLfloat* getColor() { return m_diffuseColor; }

    virtual vec3 getPosition() = 0;
    
    virtual vec3 getLastPosition() = 0;
    
    virtual vec3 getXAxis() = 0;
    
    virtual vec3 getYAxis() = 0;
    
    virtual vec3 getZAxis() = 0;
    
    virtual vector<vec3> getXYZAxis() = 0;
    
    virtual glm::mat4 getRotation() = 0; //return rotation from 0f 0f 0f to actual rotation

    void setColor(GLfloat* color)
    {
        delete[] m_diffuseColor;
        m_diffuseColor = new GLfloat[3]{color[0], color[1], color[2]};
    }

    static bool collide(vRigidBody* a, vRigidBody* b, vec3 intersection);

    static bool collide(Box* a, Box* b, vec3 intersection);

};

class Box : public vRigidBody
{
    public:
    Box(const int &id, const vec3 &pos, GLfloat* color, const vec3 &e_rot, const vec3 &scale,const float &mass,const float &drag,const bool &useGravity,const bool &isKinematic, const float worldSize)
    : vRigidBody(id, 0,pos, color, e_rot, scale, mass, drag, useGravity, isKinematic, worldSize)
    {
        this->m_kind = 0;

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
                m_particles.push_back(vParticle(id, i, vec3(pos.x+p.x, pos.y+p.y, pos.z+p.z), mass, drag, worldSize));
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

    bool isMember(vec3 node_pos, float node_size)
    {
        box a, b;

        //sub node
        a = box::create(    node_pos,
                        vec3(1.0f, .0f, .0f),
                        vec3(.0f, 1.0f, .0f),
                        vec3(.0f, .0f, 1.0f),
                        node_size*0.25f,
                        node_size*0.25f,
                        node_size*0.25f
                    );
        //rigidbody
        b = box::create(this);
        vec3 n;

        //check if rigidbody collide with the subnode
        return box::collide(a, b, n);
    }

    box getBox()
    {
        return box::create(this);
    }
};

//COLLISION METHODS

inline bool vRigidBody::collide(vRigidBody* a, vRigidBody* b, vec3 intersection){
    if ( a->isBox() && b->isBox() ) //check between two boxes
        return Box::collide(dynamic_cast<Box*>(a), dynamic_cast<Box*>(b), intersection);
    return false;
};

inline bool vRigidBody::collide(Box* a, Box* b, vec3 intersection)
{
    return box::collide(a->getBox(), b->getBox(), intersection);
}

//FW DEC

inline vRigidBody::box vRigidBody::box::create(Box* pt)
{
    box a;
    vector<vec3> axis = pt->getXYZAxis();
    a.position = pt->getPosition();
    a.x = axis.at(0);
    a.y = axis.at(1);
    a.z = axis.at(2);
    a.w = pt->getSize().x;
    a.h = pt->getSize().y;
    a.d = pt->getSize().z;
    a.id = pt->getId();
    return a;
}