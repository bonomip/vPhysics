#pragma once

#include <glm/glm.hpp>


typedef glm::vec3 vec3;
using std::abs;
using glm::dot;
using std::sqrtf;
using std::powf;
using glm::cross;


static float norma(vec3 v)
{
    float delta = powf(v.x, 2.0f)+powf(v.y, 2.0f)+powf(v.z, 2.0f);
    if(delta <= 0) return 0.0f;
    return std::sqrtf(delta);
}

static vec3 normalize(vec3 v)
{
    return v / norma(v);
}

static vec3 middlePoint(vec3 p0, vec3 p1)
{
    return vec3( (p0.x+p1.x)/2.0f,(p0.y+p1.y)/2.0f,(p0.z+p1.z)/2.0f );
}

static vec3 middleVector(vec3 v, vec3 w)
{
    float x = norma(v);
    float y = norma(w);
    return normalize( v + w ) * ( ( x + y ) / 2.0f );
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
    /*
    // Compute barycentric coordinates (u, v, w) for
// point p with respect to triangle (a, b, c)
void Barycentric(Point p, Point a, Point b, Point c, float &u, float &v, float &w)
{
    Vector v0 = b - a, v1 = c - a, v2 = p - a;
    float d00 = Dot(v0, v0);
    float d01 = Dot(v0, v1);
    float d11 = Dot(v1, v1);
    float d20 = Dot(v2, v0);
    float d21 = Dot(v2, v1);
    float denom = d00 * d11 - d01 * d01;
    v = (d11 * d20 - d01 * d21) / denom;
    w = (d00 * d21 - d01 * d20) / denom;
    u = 1.0f - v - w;
}
    */

template <class RigidBody> struct box{

    static void getTrianglesfromBox(vector<triangle> &result, box<RigidBody> a)
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