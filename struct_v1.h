#pragma once

#include <glm/glm.hpp>


typedef glm::vec3 vec3;
using std::abs;
using glm::dot;
using glm::cross;


struct triangle
{
    vec3 v0, v1, v2;

    static triangle create(vec3 a, vec3 b, vec3 c)
    {
        triangle t;
        t.v0 = a;
        t.v1 = b;
        t.v2 = c;
        return t;
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

        if(t > eps) // ray intersection
        {           
            intersection = rayOrigin + t * rayDir;
            return true;
        } else // This means that there is a line intersection but not a ray intersection.
            return false; 

    }
};

template <class RigidBody> struct box{

    static void getTrianglesfromBox(vector<triangle> &result, box<RigidBody> a)
    {
        vec3 x = a.x*a.w;
        vec3 y = a.y*a.h;
        vec3 z = a.z*a.d;

        //1 top face CW
        vec3 v1 = a.position+(-1.0f*x)+y+z; //1
        vec3 v2 = a.position+x+y+z; //2
        vec3 v3 = a.position+x+y+(-1.0f*z); //3
        vec3 v4 = a.position+(-1.0f*x)+y+(-1.0f*z); //4
        //2 bottom face CW
        vec3 v5 = a.position+(-1.0f*x)+(-1.0f*y)+z; //5
        vec3 v6 = a.position+x+(-1.0f*y)+z; //6
        vec3 v7 = a.position+x+(-1.0f*y)+(-1.0f*z); //7
        vec3 v8 = a.position+(x*-1.0f)+(-1.0f*y)+(-1.0f*z); //8
        
        //left 
       //145
       //458
       result.push_back(triangle::create(v1, v4, v5));
       result.push_back(triangle::create(v4, v5, v8));


        //top
       //124
       //243
       result.push_back(triangle::create(v1, v2, v4));
       result.push_back(triangle::create(v2, v4, v3));

        //right
       //327
       //376
       result.push_back(triangle::create(v3, v2, v7));
       result.push_back(triangle::create(v3, v7, v6));

       //back
       //126
       //165

       result.push_back(triangle::create(v1, v2, v6));
       result.push_back(triangle::create(v1, v6, v5));

       //front
       //437
       //478
       result.push_back(triangle::create(v4, v3, v7));
       result.push_back(triangle::create(v4, v7, v8));

       //bottom
       //567
       //578
       result.push_back(triangle::create(v5, v6, v7));
       result.push_back(triangle::create(v5, v7, v8));
    }

    static bool collide(box<RigidBody> a, box<RigidBody> b, vec3 &intersection)
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
    
    static box<RigidBody> create(RigidBody* pt)
    {
        box<RigidBody> a;
        vector<vec3> axis = pt->getXYZAxis();
        a.position = pt->getPosition();
        a.x = axis.at(0);
        a.y = axis.at(1);
        a.z = axis.at(2);
        a.w = pt->getSize().x;
        a.h = pt->getSize().y;
        a.d = pt->getSize().z;
        return a;
    }

    static box<RigidBody> create(vec3 pos, vec3 x_axis, vec3 y_axis, vec3 z_axis, float width, float height, float depth)
    {
        box<RigidBody> a;
        a.position = pos;
        a.x = x_axis;
        a.y = y_axis;
        a.z = z_axis;
        a.w = width;
        a.h = height;
        a.d = depth;
        return a;
    }

    vec3 position, x, y, z; //position + axis
    float w, h, d; //HALF SIZE
    int id;

};