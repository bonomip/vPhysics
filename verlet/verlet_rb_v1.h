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
#include <cmath>

class Box;
class Sphere;

class vRigidBody : public OItem
{
    public:

    static float max(float a, float b)
    {
        return (a<b) ? b:a;
    }

    static float min(float a, float b)
    {
        return (a<b) ? a:b;
    }

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
        float w, h, d; //from center to side ( = half side)
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

        static box createFromAxisAligned(vec3 pos, float side_size)
        {
            return box::create( pos, 
                                vec3(1.0f, .0f, .0f),
                                vec3(.0f, 1.0f, .0f),
                                vec3(.0f, .0f, 1.0f),
                                side_size*0.5f,
                                side_size*0.5f,
                                side_size*0.5f 
                            );
        }
    };

    struct sphere
    {
        vec3 pos;
        float r;

        static sphere create(vec3 position, float radius)
        {
            sphere s;
            s.pos = position;
            s.r = radius;
            return s; 
        }

        //todo
        static bool collide(sphere s, box b, vec3 intersection)
        {
            return false;
        }

        static bool collideAxisAligned(sphere s, box b)
        {
            float x = max( b.position.x - b.w, min( s.pos.x, b.w  + b.position.x ) );
            float y = max( b.position.y - b.h, min( s.pos.y, b.h  + b.position.y ) );
            float z = max( b.position.z - b.d, min( s.pos.z, b.d  + b.position.z ) );

            //distanza^2
            float distance = (x - s.pos.x)*(x - s.pos.x) + (y - s.pos.y)*(y - s.pos.y) + (z - s.pos.z)*(z - s.pos.z); 

            return distance < s.r*s.r;
        }

        static bool collide(sphere s, sphere t, vec3 intersection)
        {
            //distance between two sphere's center not squared
            float d =   (s.pos.x - t.pos.x)*(s.pos.x - t.pos.x) +
                        (s.pos.y - t.pos.y)*(s.pos.y - t.pos.y) +
                        (s.pos.z - t.pos.z)*(s.pos.z - t.pos.z); 
            float r = s.r + t.r; //sum of the radius
            return d < r*r;
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

    vec3 m_start_pos;
    vec3 m_start_rot;

    vector<vParticle> m_particles;
    vector<vConnection> m_connections;

public:

    vRigidBody(const vRigidBody& rb) = delete; //disallow copy
    
    vRigidBody& operator=(const vRigidBody& copy) = delete;

    vRigidBody(const int id,const int kind, GLfloat* color, const vec3 scale, const bool useGravity,const bool isKinematic)
    {
        this->m_scale = scale;
        this->m_useGravity = useGravity;
        this->m_isKinematic = isKinematic;
        this->m_kind = kind;
        this->m_id = id;
        this->m_diffuseColor = new GLfloat[3]{color[0], color[1], color[2]};
    }

    virtual ~vRigidBody() {}

    void update(float dt)
    {
        if(this->m_kind == 0) for_each(this->m_particles.begin(), this->m_particles.end(), [&](vParticle &p) { p.update(dt); } );
        if(this->m_kind == 1) static_cast<SphereParticle&>(this->m_particles.at(0)).update(dt);
    }  

    void updateConstraint()
    {
        if(this->m_kind == 1) return;
        
        for(int i = 0; i < 2; i++) for_each(this->m_connections.begin(), this->m_connections.end(), [&](vConnection &c) { c.enforceConstraint(); } );
    }      

    bool isBox(){ return this->m_kind == 0; }     // 0 for boxes

    bool isSphere(){ return this->m_kind == 1; }  // 1 for Spheres

    vector<vParticle>* getParticles() { return &this->m_particles; }
    
    vector<vConnection>* getConnections() { return &this->m_connections; }

    vec3 getSize() { return this->m_scale; }

    int getId() { return this->m_id; }

    GLfloat* getColor() { return this->m_diffuseColor; }

    vec3 getStartPos() //only for sphere to prevent bugs
    {
        return this->m_start_pos;
    }

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
        this->m_diffuseColor = new GLfloat[3]{color[0], color[1], color[2]};
    }

    static bool collide(vRigidBody* a, vRigidBody* b, vec3 intersection);

    static bool collide(Box* a, Box* b, vec3 intersection);

    static bool collide(Sphere* a, Sphere* b, vec3 intersection);

};

class Box : public vRigidBody
{
    public:
    Box(int id, vec3 pos, GLfloat* color, vec3 e_rot, vec3 scale, float mass,float drag, bool useGravity,bool isKinematic, float worldSize)
    : vRigidBody(id, 0, color, scale, useGravity, isKinematic)
    {
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
            this->m_particles.push_back(vParticle(id, i, vec3(pos.x+p.x, pos.y+p.y, pos.z+p.z), mass, drag, worldSize));
        }

        for(int i = 0; i < this->m_particles.size()-1; i ++)
            for(int j = i+1; j < this->m_particles.size(); j++)
                this->m_connections.push_back(vConnection(&this->m_particles.at( i ),&this->m_particles.at( j )));
    }

    ~Box()
    {
        this->m_particles.clear();
        this->m_connections.clear();
        delete[] this->m_diffuseColor;
    }

    vec3 getPosition()
    {
        //The center of the box is calculated as the half distance between particles to the external counterparts
        return (this->m_particles.at(6).getPosition()+this->m_particles.at(0).getPosition())*0.5f;
    }

    vec3 getLastPosition()    
    {
        return this->m_particles.at(0).getLastPosition()+
                (this->m_particles.at(6).getLastPosition() - this->m_particles.at(0).getLastPosition())*.5f;
    }

    vec3 getXAxis(){
        vec3 v0 = this->m_particles.at(0).getPosition();
        vec3 v1 = this->m_particles.at(1).getPosition();
        return glm::normalize(v0 - v1);
    }

    vec3 getYAxis(){
        vec3 v0 = this->m_particles.at(0).getPosition();
        vec3 v2 = this->m_particles.at(2).getPosition();
        
        vec3 x = this->getXAxis();
        return glm::normalize(glm::cross(x, v2-v0));
    }

    vec3 getZAxis(){
        vec3 x = this->getXAxis();
        vec3 y = this->getYAxis();
        return glm::cross(x, y);
    }

    vector<vec3> getXYZAxis()
    {
        vec3 v0 = this->m_particles.at(0).getPosition();
        vec3 v1 = this->m_particles.at(1).getPosition();
        vec3 v2 = this->m_particles.at(2).getPosition();
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

        vector<vec3> axis = this->getXYZAxis();
        
        result[0] = glm::vec4(axis.at(0), 0);
        result[1] = glm::vec4(axis.at(1), 0);
        result[2] = glm::vec4(axis.at(2), 0);
        result[3] = glm::vec4(0, 0, 0, 1);

        return result;
    }

    bool isMember(vec3 node_pos, float node_side_size)
    {
        //sub node
        box a = box::createFromAxisAligned(node_pos, node_side_size);
        //rigidbody
        box b = box::create(this);

        vec3 n;

        //check if rigidbody collide with the subnode
        return box::collide(a, b, n);
    }

    box getBox()
    {
        return box::create(this);
    }
};

class Sphere : public vRigidBody
{
    public:
    Sphere(const int id, const vec3 pos, GLfloat* color, const vec3 e_rot, const float radius,const float mass,const float drag, const float bounciness, const bool useGravity,const bool isKinematic, const float &worldSize)
    : vRigidBody(id, 1, color, vec3(radius, radius, radius), useGravity, isKinematic)
    {
        this->m_start_pos = pos;
        this->m_start_rot = e_rot;

        glm::mat4 rot = glm::eulerAngleYXZ(e_rot.y, e_rot.x, e_rot.z);

        glm::vec4 p = glm::vec4(vec3(.0f,.0f,.0f), 1) * rot; //sphere is made up by 1 patricles in its center

        m_particles.push_back(SphereParticle(id, 0, vec3(pos.x+p.x, pos.y+p.y, pos.z+p.z), radius, mass, drag, bounciness, worldSize));
    }

    ~Sphere()
    {
        this->m_particles.clear();
        this->m_connections.clear();
        delete[] this->m_diffuseColor;
    }

    vec3 getPosition()
    {
        //The center of the box is calculated as the half distance between particles to the external counterparts
        return m_particles.at(0).getPosition();
    }

    vec3 getLastPosition()    
    {
        return m_particles.at(0).getLastPosition();
    }

    float getRadius()
    {
        return m_particles.at(0).getRadius();
    }

    sphere getSphere()
    {
        return sphere::create(this->getPosition(), this->getRadius());
    }

    void reset()
    {
        this->m_particles.at(0).reset(this->m_start_pos);
    }

    vec3 getXAxis(){
        return vec3(1.0f,.0f,.0f);
    }

    vec3 getYAxis(){
        return vec3(.0f,1.0f,.0f);
    }

    vec3 getZAxis(){
        return vec3(.0f,.0f,1.0f);
    }

    vector<vec3> getXYZAxis()
    {
        vector<vec3> v;
        v.push_back(vec3(1.0f,.0f,.0f));
        v.push_back(vec3(.0f,1.0f,.0f));
        v.push_back(vec3(.0f,.0f,1.0f));
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

    bool isMember(vec3 node_pos, float node_side_size)
    {
        box b = box::createFromAxisAligned(node_pos, node_side_size); 
        return sphere::collideAxisAligned(this->getSphere(), b);
    }
};


//COLLISION METHODS

inline bool vRigidBody::collide(vRigidBody* a, vRigidBody* b, vec3 intersection){ //method dispacher
    if (a->isBox() && b->isBox()) return vRigidBody::collide(dynamic_cast<Box*>(a), dynamic_cast<Box*>(b), intersection);
    if (a->isSphere() && b->isSphere()) return vRigidBody::collide(dynamic_cast<Sphere*>(a), dynamic_cast<Sphere*>(b), intersection);

    //if (a->isBox() && b->isSphere()) return vRigidBody::collide(dynamic_cast<Box*>(a), dynamic_cast<Sphere*>(b), intersection);
    //if (a->isSphere() && b->isBox()) return vRigidBody::collide(dynamic_cast<Box*>(b), dynamic_cast<Sphere*>(a), intersection);

    return false;
};

inline bool vRigidBody::collide(Box* a, Box* b, vec3 intersection)
{
    return box::collide(a->getBox(), b->getBox(), intersection);
}

inline bool vRigidBody::collide(Sphere* a, Sphere* b, vec3 intersection)
{
    return sphere::collide(a->getSphere(), b->getSphere(), intersection);
}

//-----------
inline vRigidBody::box vRigidBody::box::create(Box* pt)
{
    vector<vec3> axis = pt->getXYZAxis();
    return box::create(     pt->getPosition(),
                            axis.at(0),
                            axis.at(1),
                            axis.at(2),
                            pt->getSize().x,
                            pt->getSize().y,
                            pt->getSize().z
                    );
}