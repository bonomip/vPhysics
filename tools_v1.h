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



