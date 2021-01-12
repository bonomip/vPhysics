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

