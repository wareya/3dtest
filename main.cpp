/*
Copyright 2017 Alexander Nadeau <wareya@gmail.com>

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

bool debughard = false;

#include <GL/gl3w.h>
#include <GLFW/glfw3.h>

#define STB_IMAGE_IMPLEMENTATION
#include "include/stb_image_wrapper.h"

#include <stdio.h>
#include <math.h>
#ifndef M_PI
#define M_PI 3.1415926435
#endif

#ifndef INF
#define INF (1.0/0.0)
#endif

#include <signal.h>

#define deg2rad(X) ((X)/180.0f*(M_PI))
#define rad2deg(X) ((X)*180.0f/(M_PI))

#include <chrono>
#include <thread>
#include <vector>

struct vertex {
    float x, y, z, u, v, nx, ny, nz;
};
struct basicvertex {
    float x, y, z;
};
struct coord {
    double x, y, z;
    coord()
    {
        x = 0; y = 0; z = 0;
    }
    coord(double ax, double ay, double az)
    {
        x = ax; y = ay; z = az;
    }
    coord(basicvertex v)
    {
        x = v.x; y = v.y; z = v.z;
    }
    coord(vertex v)
    {
        x = v.x; y = v.y; z = v.z;
    }
    bool operator==(coord o)
    {
        return x == o.x && y == o.y && z == o.z;
    }
    bool operator!=(coord o)
    {
        return !(*this == o);
    }
    coord operator-()
    {
        return coord(-x, -y, -z);
    }
    bool operator>(coord o)
    {
        return (x > o.x and y > o.y and z > o.z);
    }
    bool operator<(coord o)
    {
        return (x < o.x and y < o.y and z < o.z);
    }
    coord operator+(coord b)
    {
        return coord(x+b.x, y+b.y, z+b.z);
    }
    coord operator-(coord b)
    {
        return coord(x-b.x, y-b.y, z-b.z);
    }
    coord operator*(double b)
    {
        return coord(x*b, y*b, z*b);
    }
    coord operator/(double b)
    {
        return coord(x/b, y/b, z/b);
    }
};

coord mult(coord a, double b)
{
    coord r;
    r.x = a.x*b;
    r.y = a.y*b;
    r.z = a.z*b;
    return r;
}

double dot(coord a, coord b)
{
    double r = 0;
    r += a.x*b.x;
    r += a.y*b.y;
    r += a.z*b.z;
    return r;
}

double magnitude(coord a)
{
    return sqrt(dot(a, a));
}

coord normalize(coord a)
{
    return a/magnitude(a);
}

double normalized_dot(coord a, coord b)
{
    a = normalize(a);
    b = normalize(b);
    return dot(a, b);
}

coord project(coord a, coord b)
{
    return b * (dot(a, b)/dot(b, b));
}

constexpr double fudge_radians = 0.00005;
coord reject(coord a, coord b)
{
    coord r = a - project(a, b);
    return r;
    //double dot_after = dot(r, b);
    //if(dot_after > fudge_radians)
    //double dot_before = dot(a, b);
    //double dot_doff
}

coord cross(coord a, coord b)
{
    coord r;
    r.x = a.y*b.z-a.z*b.y;
    r.y = a.z*b.x-a.x*b.z;
    r.z = a.x*b.y-a.y*b.x;
    return r;
}

coord make_minima(coord a, coord b)
{
    coord minima;
    
    if(a.x <= b.x) minima.x = a.x;
    else minima.x = b.x;
    
    if(a.y <= b.y) minima.y = a.y;
    else minima.y = b.y;
    
    if(a.z <= b.z) minima.z = a.z;
    else minima.z = b.z;
    
    return minima;
}

coord make_maxima(coord a, coord b)
{
    return -make_minima(-a, -b);
}

coord make_minima(std::vector<coord> coords)
{
    coord minima;
    
    minima = coords[0];
    
    for(auto & c : coords)
    {
        if(c.x < minima.x) minima.x = c.x;
        if(c.y < minima.y) minima.y = c.y;
        if(c.z < minima.z) minima.z = c.z;
    }
    
    return minima;
}

coord make_maxima(std::vector<coord> coords)
{
    coord maxima;
    
    maxima = coords[0];
    
    for(auto & c : coords)
    {
        if(c.x > maxima.x) maxima.x = c.x;
        if(c.y > maxima.y) maxima.y = c.y;
        if(c.z > maxima.z) maxima.z = c.z;
    }
    
    return maxima;
}

bool aabb_overlap(coord minima_a, coord maxima_a, coord minima_b, coord maxima_b)
{
    return minima_a.x <= maxima_b.x and maxima_a.x >= minima_b.x and
           minima_a.y <= maxima_b.y and maxima_a.y >= minima_b.y and
           minima_a.z <= maxima_b.z and maxima_a.z >= minima_b.z;
}

struct collision {
    // a flat triangle (with vertices in counter-clockwise direction when looking down at it) has a normal pointing up and acts like a floor
    std::vector<coord> points;
    coord normal;
    
    // bounding box
    coord minima;
    coord maxima;
    
    bool isstatic = true;
    collision()
    {
        points = {coord(), coord(), coord()};
        
        normal = coord(0,-1,0);
        
        minima = make_minima(points);
        maxima = make_maxima(points);
    }
    collision(coord i, coord j, coord k, bool invert = false, bool makestatic = true)
    {
        if(invert)
            points = {i, k, j};
        else
            points = {i, j, k};
        // This Is Right. cross((0,0,-1),(1,0,0)) (i.e. cross(backwards, rightwards)) should produce (0,-1,0) (i.e. upwards)
        normal = normalize(cross(points[1]-points[0], points[2]-points[0]));
        isstatic = makestatic;
        
        minima = make_minima(points);
        maxima = make_maxima(points);
    }
    collision(coord i, coord j, coord k, coord norm, coord minim, coord maxim, bool isstat)
    {
        points = {i, j, k};
        normal = norm;
        minima = minim;
        maxima = maxim;
        isstatic = isstat;
    }
    bool operator==(collision o)
    {
        return points[0] == o.points[0] && points[1] == o.points[1] && points[2] == o.points[2] && normal == o.normal;
    }
    bool operator!=(collision o)
    {
        return !(*this == o);
    }
    collision operator+(coord v)
    {
        return collision(points[0]+v, points[1]+v, points[2]+v, normal, minima, maxima, isstatic);
    }
};

coord center(collision c)
{
    return (c.points[0]+c.points[1]+c.points[2])/3.0;
}

struct plane {
    coord intersect;
    coord normal;
    bool isstatic;
    plane()
    {
        intersect = coord();
        normal = coord();
        isstatic = true;
    }
    bool operator==(plane o)
    {
        return /*intersect == o.intersect and*/ normal == o.normal && isstatic == o.isstatic;
    }
    bool operator!=(plane o)
    {
        return !(*this == o);
    }
};


coord make_minima(std::vector<collision> triangles)
{
    coord minima;
    
    minima = triangles[0].minima;
    
    for(auto & t : triangles)
        minima = make_minima(t.minima, minima);
    
    return minima;
}


coord make_maxima(std::vector<collision> triangles)
{
    coord maxima;
    
    maxima = triangles[0].maxima;
    
    for(auto & t : triangles)
        maxima = -make_minima(-t.maxima, -maxima);
    
    return maxima;
}

double ray_cast_triangle(coord o, coord m, collision c)
{
    // no collision from behind (motion in same direction as normal)
    
    double velocity = magnitude(m);
    if(velocity == 0) return 0;
    
    coord d = mult(m, 1/velocity);
    
    if(dot(d, c.normal) > 0)
        return INF;
    
    coord e1 = c.points[1]-c.points[0];
    coord e2 = c.points[2]-c.points[0];
    
    coord pvec = cross(d, e2);
    double det = dot(e1, pvec);
    
    if(det == 0)
        return INF;
    
    coord tvec = o-c.points[0];
    double u = dot(tvec, pvec) / det;
    if (u < 0 || u > 1)
        return INF;
    
    coord qvec = cross(tvec, e1);
    double v = dot(d, qvec) / det;
    if (v < 0 || u + v > 1)
        return INF;
    
    double ret = dot(e2, qvec) / det;
    
    //if(ret < 0)
    //    return INF;
    
    return ret;
}

double origin_distance_plane(collision c)
{
    coord o = coord(0,0,0);
    coord m = -c.normal;
    
    coord d = normalize(m);
    
    coord e1 = c.points[1]-c.points[0];
    coord e2 = c.points[2]-c.points[0];
    
    coord pvec = cross(d, e2);
    double det = dot(e1, pvec);
    
    if(det == INF)
        return INF;
    
    coord tvec = o-c.points[0];
    double u = dot(tvec, pvec) / det;
    //if (u < 0 || u > 1)
    //    return 0;
    
    coord qvec = cross(tvec, e1);
    double v = dot(d, qvec) / det;
    //if (v < 0 || u + v > 1)
    //    return 0;
    
    double ret = dot(e2, qvec) / det;
    
    return ret;
}

void triangle_castlines_triangle(collision r, coord d, collision c, double & d1, collision & contact_collision, bool positiveonly = true)
{
    d1 = INF;
    
    if(normalized_dot(d, c.normal) > 0 and normalized_dot(d, r.normal) < 0) return;
    
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            coord r1 = r.points[i];
            coord r2 = r.points[(i+1)%3];
            coord c1 = c.points[j];
            coord c2 = c.points[(j+1)%3];
            
            auto ro = r2-r1;
            
            auto tri1 = collision(c1, c2, c2+ro);
            auto tri2 = collision(c1, c1+ro, c2+ro);
            
            if(normalized_dot(d, tri1.normal) > 0)
                tri1 = collision(c1, c2+ro, c2);
            if(normalized_dot(d, tri2.normal) > 0)
                tri2 = collision(c1, c2+ro, c1+ro);
            
            double d2 = ray_cast_triangle(r2, d, tri1);
            double d3 = ray_cast_triangle(r2, d, tri2);
            
            if(d2 < d1 and d2 < d3 and (!positiveonly or d2 >= 0))
            {
                d1 = d2;
                contact_collision = tri1;
                contact_collision.isstatic = c.isstatic;
                //contact_plane.normal = tri1.normal;
                //contact_plane.isstatic = c.isstatic;
            }
            else if(d3 < d1 and d3 != INF and (!positiveonly or d3 >= 0))
            {
                d1 = d3;
                contact_collision = tri2;
                contact_collision.isstatic = c.isstatic;
                //contact_plane.normal = tri2.normal;
                //contact_plane.isstatic = c.isstatic;
            }
        }
    }
}

struct rigidbody {
    double x, y, z;
    double xspeed, yspeed, zspeed;
    coord minima; // AABB stuff
    coord maxima; // AABB stuff
    std::vector<collision> triangles; // relative to x, y, z
    std::vector<coord> points; // relative to x, y, z
};

collision zero_collision = collision();

void rigidbody_cast_triangle(rigidbody & body, coord position, coord d, collision c, double & d1, collision & contact_collision, bool positiveonly = true, bool verbose = false)
{
    auto unit = coord(1,1,1);
    auto minima = make_minima(body.minima+position-unit, body.minima+position+d-unit);
    auto maxima = make_maxima(body.maxima+position+unit, body.maxima+position+d+unit);
    
    d1 = INF;
    
    if(!aabb_overlap(minima, maxima, c.minima, c.maxima))
        return;
    
#if 0
    d1 = ray_cast_triangle(position, d, c);
    if(d1 != INF)
    {
        contact_collision = c;
        //contact_collision.intersect = center(c);
        //contact_collision.normal = c.normal;
        //contact_collision.isstatic = c.isstatic;
    }
    return;
#endif
    int type = 0;
    for(auto p : body.points)
    {
        double d2 = ray_cast_triangle(p+position, d, c);
        if(d2 < d1 and (!positiveonly or d2 >= 0))
        {
            type = 1;
            d1 = d2;
            contact_collision = c;
            //contact_collision.intersect = center(c);
            //contact_collision.normal = c.normal;
            //contact_collision.isstatic = c.isstatic;
        }
    }
    for(auto p : c.points)
    {
        for(auto tri : body.triangles)
        {
            double d2 = ray_cast_triangle(p, -d, tri+position);
            if(d2 < d1 and (!positiveonly or d2 >= 0))
            {
                type = 2;
                d1 = d2;
                contact_collision = tri;
                contact_collision.normal = -tri.normal;
                //contact_collision.intersect = center(c);
                //contact_collision.normal = -tri.normal;
                //contact_collision.isstatic = c.isstatic;
            }
        }
    }
    for(auto tri : body.triangles)
    {
        double d2;
        collision new_collision;
        triangle_castlines_triangle(tri+position, d, c, d2, new_collision, positiveonly);
        if(d2 < d1 and (!positiveonly or d2 >= 0))
        {
            type = 3;
            d1 = d2;
            contact_collision = new_collision;
        }
    }
    
    if(debughard and verbose)
    {
        if(type == 1) puts("TYPE A");
        if(type == 2) puts("TYPE B");
        if(type == 3) puts("TYPE C");
    }
}

void checkerr(int line)
{
    GLenum err;
    while((err = glGetError()) != GL_NO_ERROR)
    {
        printf("GL error %04X from line %d\n", err, line);
    }   
}

void error_callback(int error, const char* description)
{
    puts(description);
}

double x = 0;
double y = -256;
double z = 0;

double rotation_x = 0;
double rotation_y = 6;
double rotation_z = 0;

double units_per_meter = 64;

double gravity = 9.8*units_per_meter; // units per second per second
double friction = 10*units_per_meter; // units per second per second under normal gravity on a flat surface; when equal to gravity, 45 degree slopes are the breaking point of friction and gravity
//double shock = 5*units_per_meter; // units per second per impact


// multiplies a by b, storing the result in a
void m4mult(float * a, float * b)
{
    float output[16];
    for(int y = 0; y < 4; y++)
    {
        for(int x = 0; x < 4; x++)
        {
            int i = y*4+x;
            output[i] = 0;
            // a: follows y, iterates over x
            // b: follows x, iterates over y
            output[i] += a[y*4+0] * b[x+0];
            output[i] += a[y*4+1] * b[x+4];
            output[i] += a[y*4+2] * b[x+8];
            output[i] += a[y*4+3] * b[x+12];
        }
    }
    memcpy(a, output, sizeof(float)*16);
}

bool postprocessing = false;

// diagonal fov
double fov = 126.869898; // 90*atan(tan(45/180*pi)*2)/pi*4
//double fov = 110;

// fisheye projection post shader
bool polar = true;

int msaa = postprocessing?4:8;
double viewPortRes = postprocessing?3.0f:1.0f;

bool dosharpen = true;
double sharpenamount = 0.35;

// long term TODO: make the bloom blur buffers low res so that high blur radiuses are cheap instead of expensive
int bloomradius = 8;
int bloompasses = 0;

struct renderer {
    // TODO: FIXME: add a real reference counter
    struct texture {
        int w, h, n, boost;
        GLuint texid;
        texture(const unsigned char * data, int w, int h)
        {
            this->w = w;
            this->h = h;
            this->n = 4;
            
            checkerr(__LINE__);
            glActiveTexture(GL_TEXTURE0);
            
            checkerr(__LINE__);
            
            glGenTextures(1, &texid);
            glBindTexture(GL_TEXTURE_2D, texid);
            printf("Actual size: %dx%d\n", this->w, this->h);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, this->w, this->h, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
            
            checkerr(__LINE__);
            glGenerateMipmap(GL_TEXTURE_2D);
            checkerr(__LINE__);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
            checkerr(__LINE__);
            
            boost = 0;
        }
    };
    texture * load_texture(const char * filename)
    {
        auto start = glfwGetTime();
        puts("Starting load texture");
        fflush(stdout);
        int w, h, n;
        unsigned char * data = stbi_load(filename, &w, &h, &n, 4);
        puts("Done actual loading");
        if(!data) return puts("failed to open texture"), nullptr;
        else
        {
            printf("Building texture of size %dx%d\n", w, h);
            
            auto tex = new texture(data, w, h);
            
            puts("Built texture");
            stbi_image_free(data);
            auto end = glfwGetTime();
            printf("Time: %f\n", end-start);
            
            return tex;
        }
    }
    struct cubemap {
        int w, h, n;
        GLuint texid;
        cubemap(unsigned char ** data, int w, int h)
        {
            this->w = w;
            this->h = h;
            this->n = 4;
            
            checkerr(__LINE__);
            glActiveTexture(GL_TEXTURE0);
            
            glGenTextures(1, &texid);
            glBindTexture(GL_TEXTURE_CUBE_MAP, texid);
            
            glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_Z, 0, GL_RGBA8, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, data[0]);
            glTexImage2D(GL_TEXTURE_CUBE_MAP_NEGATIVE_Z, 0, GL_RGBA8, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, data[1]);
            glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X, 0, GL_RGBA8, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, data[2]);
            glTexImage2D(GL_TEXTURE_CUBE_MAP_NEGATIVE_X, 0, GL_RGBA8, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, data[3]);
            glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_Y, 0, GL_RGBA8, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, data[4]);
            glTexImage2D(GL_TEXTURE_CUBE_MAP_NEGATIVE_Y, 0, GL_RGBA8, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, data[5]);
            
            checkerr(__LINE__);
            glGenerateMipmap(GL_TEXTURE_CUBE_MAP);
            
            glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
            glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);  
            checkerr(__LINE__);
        }
    };
    cubemap * load_cubemap(const char * filename, const char * extension)
    {
        auto start = glfwGetTime();
        puts("Starting load texture");
        fflush(stdout);
        int w, h, n;
        unsigned char * data[6];
        data[0] = stbi_load((std::string(filename)+"Front" +extension).data(), &w, &h, &n, 4);
        data[1] = stbi_load((std::string(filename)+"Back"  +extension).data(), &w, &h, &n, 4);
        data[2] = stbi_load((std::string(filename)+"Right"  +extension).data(), &w, &h, &n, 4);
        data[3] = stbi_load((std::string(filename)+"Left" +extension).data(), &w, &h, &n, 4);
        data[4] = stbi_load((std::string(filename)+"Top"   +extension).data(), &w, &h, &n, 4);
        data[5] = stbi_load((std::string(filename)+"Bottom"+extension).data(), &w, &h, &n, 4);
        puts("Done actual loading");
        if(!data[0] or !data[1] or !data[2]) return puts("failed to open texture"), nullptr;
        else
        {
            printf("Building texture of size %dx%d\n", w, h);
            
            auto tex = new cubemap(data, w, h);
            
            puts("Built texture");
            stbi_image_free(data[0]);
            stbi_image_free(data[1]);
            stbi_image_free(data[2]);
            stbi_image_free(data[3]);
            stbi_image_free(data[4]);
            stbi_image_free(data[5]);
            
            auto end = glfwGetTime();
            printf("Time: %f\n", end-start);
            
            return tex;
        }
    }
    cubemap * load_cubemap_alt(const char * side, const char * top, const char * bottom)
    {
        auto start = glfwGetTime();
        puts("Starting load texture");
        fflush(stdout);
        int w, h, n;
        auto side_data = stbi_load(side, &w, &h, &n, 4);
        auto top_data = stbi_load(top, &w, &h, &n, 4);
        auto bottom_data = stbi_load(bottom, &w, &h, &n, 4);
        unsigned char * data[6];
        data[0] = side_data;
        data[1] = side_data;
        data[2] = side_data;
        data[3] = side_data;
        data[4] = top_data;
        data[5] = bottom_data;
        puts("Done actual loading");
        if(!data[0] or !data[1] or !data[2]) return puts("failed to open texture"), nullptr;
        else
        {
            printf("Building texture of size %dx%d\n", w, h);
            
            auto tex = new cubemap(data, w, h);
            
            puts("Built texture");
            stbi_image_free(side_data);
            stbi_image_free(top_data);
            stbi_image_free(bottom_data);
            
            auto end = glfwGetTime();
            printf("Time: %f\n", end-start);
            
            return tex;
        }
    }
    
    
    struct postprogram {
        unsigned int program;
        unsigned int fshader;
        unsigned int vshader;
        
        postprogram(const char * name, const char * fshadersource)
        {
            const char * vshadersource =
            "#version 330 core\n\
            layout (location = 0) in vec3 aPos;\n\
            layout (location = 1) in vec2 aTex;\n\
            layout (location = 2) in vec3 aNormal;\n\
            varying out vec2 myTexCoord;\n\
            varying out vec3 myVertNormal;\n\
            void main()\n\
            {\n\
                gl_Position = vec4(aPos.x, aPos.y, aPos.z, 1.0);\n\
                myTexCoord = aTex;\n\
            }\n"
            ;
            
            checkerr(__LINE__);
            vshader = glCreateShader(GL_VERTEX_SHADER);
            glShaderSource(vshader, 1, &vshadersource, NULL);
            glCompileShader(vshader);
            checkerr(__LINE__);
            
            fshader = glCreateShader(GL_FRAGMENT_SHADER);
            glShaderSource(fshader, 1, &fshadersource, NULL);
            glCompileShader(fshader);
            checkerr(__LINE__);
            
            program = glCreateProgram();
            glAttachShader(program, vshader);
            glAttachShader(program, fshader);
            glLinkProgram(program);
            checkerr(__LINE__);
            
            int v,f,p;
            glGetShaderiv(vshader, GL_COMPILE_STATUS, &v);
            glGetShaderiv(fshader, GL_COMPILE_STATUS, &f);
            glGetProgramiv(program, GL_LINK_STATUS, &p);
            checkerr(__LINE__);
            if(!v or !f or !p)
            {
                char info[512];
                puts("Failed to compile shader:");
                puts(name);
                if(!v)
                {
                    glGetShaderInfoLog(vshader, 512, NULL, info);
                    puts(info);
                }
                if(!f)
                {
                    glGetShaderInfoLog(fshader, 512, NULL, info);
                    puts(info);
                }
                if(!p)
                {
                    glGetProgramInfoLog(program, 512, NULL, info);
                    puts(info);
                }
                exit(0);
            }
            
            checkerr(__LINE__);
            
            glDeleteShader(vshader);
            glDeleteShader(fshader);
        }
    };
    
    // FBO 0: hi-res, 1: downsampled raw, 2/3: double wet buffers (e.g. multipass bloom)
    unsigned int MainVAO, VIO, VBO, FRBO, RBOC, RBOD, FBO, FBOtexture0, FBOtexture1, FBOtexture2, FBOtexture3, FBOtexture4, CubeVAO, CubeVBO, CubeVIO, jinctexid;
    int w, h;
    unsigned int vshader;
    unsigned int fshader;
    unsigned int program;
    
    unsigned int cubevshader;
    unsigned int cubefshader;
    unsigned int cubeprogram;
    
    GLFWwindow * win;
    postprogram * copy, * ssam, * distort, * meme, * sharpen, * bloom1, * bloom2, * bloom3;
    
    void checkcompile(int vshader, int fshader, int program)
    {
        int vsuccess, fsuccess, psuccess;
        glGetShaderiv(vshader, GL_COMPILE_STATUS, &vsuccess);
        glGetShaderiv(fshader, GL_COMPILE_STATUS, &fsuccess);
        glGetProgramiv(program, GL_LINK_STATUS, &psuccess);
        if(!vsuccess or !fsuccess or !psuccess)
        {
            char info[512];
            puts("Failed to compile shader");
            if(!vsuccess)
            {
                glGetShaderInfoLog(vshader, 512, NULL, info);
                puts(info);
            }
            if(!fsuccess)
            {
                glGetShaderInfoLog(fshader, 512, NULL, info);
                puts(info);
            }
            if(!psuccess)
            {
                glGetProgramInfoLog(program, 512, NULL, info);
                puts(info);
            }
            
            exit(0);
        }
    }
    void mainprogram(unsigned int & vshader, unsigned int & fshader, unsigned int & program, const char * vshadersource, const char * fshadersource)
    {
        vshader = glCreateShader(GL_VERTEX_SHADER);
        glShaderSource(vshader, 1, &vshadersource, NULL);
        glCompileShader(vshader);
        checkerr(__LINE__);
        
        fshader = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(fshader, 1, &fshadersource, NULL);
        glCompileShader(fshader);
        checkerr(__LINE__);
        
        program = glCreateProgram();
        glAttachShader(program, vshader);
        glAttachShader(program, fshader);
        glLinkProgram(program);
        checkerr(__LINE__);
        
        checkcompile(vshader, fshader, program);
        checkerr(__LINE__);
        
        glDeleteShader(fshader);
        glDeleteShader(vshader);
        checkerr(__LINE__);
    }
    
    double viewportscale;
    
    renderer()
    {
        if(postprocessing) viewportscale = viewPortRes;
        else viewportscale = 1.0f;
        glfwSwapInterval(0);
        
        if(!glfwInit()) puts("glfw failed to init"), exit(0);
        
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        
        win = glfwCreateWindow(1280, 720, "Hello, World!", NULL, NULL);
        
        if(!win) puts("glfw failed to init"), exit(0);
        glfwMakeContextCurrent(win);
        
        if(gl3wInit()) puts("gl3w failed to init"), exit(0);
        
        printf("OpenGL %s, GLSL %s\n", glGetString(GL_VERSION), glGetString(GL_SHADING_LANGUAGE_VERSION));
        
        //glfwSwapBuffers(win);
        glfwGetFramebufferSize(win, &w, &h);
        
        glViewport(0, 0, w, h);
        
        glEnable(GL_DEBUG_OUTPUT);
        glDebugMessageCallback([](GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar* message, const void* userParam)
        {
            puts(message);
        }, nullptr);
        
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LEQUAL);
        glEnable(GL_CULL_FACE);
        //glDisable(GL_CULL_FACE);
        glEnable(GL_PRIMITIVE_RESTART);
        glPrimitiveRestartIndex(65535);
        glFrontFace(GL_CCW);
        
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        
        
        glGenVertexArrays(1, &MainVAO);
        glGenBuffers(1, &VBO);
        glGenBuffers(1, &VIO);
        
        glBindVertexArray(MainVAO);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(vertex), (void*)0);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(vertex), (void*)offsetof(vertex, u));
        glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(vertex), (void*)offsetof(vertex, nx));
        glEnableVertexAttribArray(0);
        glEnableVertexAttribArray(1);
        glEnableVertexAttribArray(2);
        
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, VIO);
        
        
        glGenVertexArrays(1, &CubeVAO);
        glGenBuffers(1, &CubeVBO);
        glGenBuffers(1, &CubeVIO);
        
        glBindVertexArray(CubeVAO);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        
        glBindBuffer(GL_ARRAY_BUFFER, CubeVBO);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(basicvertex), (void*)0);
        glEnableVertexAttribArray(0);
        
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, CubeVIO);
        
        checkerr(__LINE__);
        
        glBindVertexArray(MainVAO);
        mainprogram(vshader, fshader, program, 
        "#version 330 core\n\
        uniform mat4 projection; // world and view transform\n\
        uniform mat4 translation; // object transform\n\
        layout (location = 0) in vec3 aPos;\n\
        layout (location = 1) in vec2 aTex;\n\
        layout (location = 2) in vec3 aNormal;\n\
        varying out vec2 myTexCoord;\n\
        varying out vec3 myNormal;\n\
        void main()\n\
        {\n\
            gl_Position = vec4(aPos.x, aPos.y, aPos.z, 1.0) * translation * projection;\n\
            myTexCoord = aTex;\n\
            myNormal = aNormal;\n\
        }\n",
        
        "#version 330 core\n\
        uniform sampler2D mytexture;\n\
        uniform int boost;\n\
        uniform float gamma;\n\
        varying vec2 myTexCoord;\n\
        varying vec3 myNormal;\n\
        \n\
        layout(location = 0) out vec4 fragColor;\n\
        void main()\n\
        {\n\
            vec4 color = texture2D(mytexture, myTexCoord, 0);\n\
            float dot = dot(normalize(myNormal), normalize(vec3(-1.0,-1.0,-1.0)));\n\
            dot = max(0, dot);\n\
            color.rgb = pow(color.rgb, vec3(gamma));\n\
            vec3 diffuse = color.rgb * dot;\n\
            vec3 ambient = color.rgb * 0.1;\n\
            fragColor = vec4(pow(diffuse+ambient, vec3(1/gamma)), 1);\n\
            if(boost == 1) fragColor.rgb *= 4;\n\
        }\n");
        
        glUseProgram(program);
        glUniform1i(glGetUniformLocation(program, "mytexture"), 0);
        glUniform1f(glGetUniformLocation(program, "gamma"), 2.2);
        
        glBindVertexArray(CubeVAO);
        mainprogram(cubevshader, cubefshader, cubeprogram, 
        "#version 330 core\n\
        uniform mat4 projection; // world and view transform\n\
        layout (location = 0) in vec3 aPos;\n\
        varying out vec3 myTexCoord;\n\
        void main()\n\
        {\n\
            gl_Position = (vec4(aPos.x, -aPos.y, aPos.z, 1.0) * projection).xyww;\n\
            myTexCoord = aPos;\n\
        }\n",
        
        "#version 330 core\n\
        uniform samplerCube skybox;\n\
        uniform int cylindrical;\n\
        varying vec3 myTexCoord;\n\
        \n\
        layout(location = 0) out vec4 fragColor;\n\
        void main()\n\
        {\n\
            if(true)//cylindrical != 1)\n\
                fragColor = texture(skybox, myTexCoord);\n\
            else\n\
            {\n\
                vec3 coord = myTexCoord;\n\
                vec2 pole = normalize(myTexCoord.xz);\n\
                float dist = sqrt(pole.x*pole.x + pole.y*pole.y);\n\
                float angle = atan(pole.y, pole.x);\n\
                coord.x = cos(angle)*dist;\n\
                coord.z = sin(angle)*dist;\n\
                //coord.y = sqrt(1 - coord.x*coord.x + coord.z*coord.z);\n\
                //coord = normalize(coord);\n\
                fragColor = texture(skybox, coord);\n\
            }\n\
        }\n");
        
        glUseProgram(cubeprogram);
        glUniform1i(glGetUniformLocation(cubeprogram, "skybox"), 0);
        glUniform1i(glGetUniformLocation(cubeprogram, "cylindrical"), 1);
        
        checkerr(__LINE__);
        
        checkerr(__LINE__);
        
        // FBO programs
        
        glGenFramebuffers(1, &FRBO); 
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, FRBO);
        
        glGenRenderbuffers(1, &RBOC); 
        glGenRenderbuffers(1, &RBOD); 
        glBindRenderbuffer(GL_RENDERBUFFER, RBOC);
        glRenderbufferStorageMultisample(GL_RENDERBUFFER, msaa, GL_RGBA16F, w*viewportscale, h*viewportscale);
        glBindRenderbuffer(GL_RENDERBUFFER, RBOD);
        glRenderbufferStorageMultisample(GL_RENDERBUFFER, msaa, GL_DEPTH_COMPONENT32F, w*viewportscale, h*viewportscale);
        
        // make framebuffer
        
        if(postprocessing)
        {
            glGenFramebuffers(1, &FBO); 
            glBindFramebuffer(GL_DRAW_FRAMEBUFFER, FBO);
            checkerr(__LINE__);
            
            static float jinctexture[512];
            
            for(int i = 0; i < 512; i++)
            {
                if(i == 0) jinctexture[i] = 1.0;
                else       jinctexture[i] = 2*std::cyl_bessel_j(1, float(i*M_PI)/8)/(float(i*M_PI)/8)*0.5+0.5;
            }
            
            ssam = new postprogram("ssam", 
            "#version 330 core\n\
            uniform sampler2D mytexture;\n\
            uniform sampler2D myJincLookup;\n\
            uniform float myScale;\n\
            varying vec2 myTexCoord;\n\
            #define M_PI 3.1415926435\n\
            vec2 coord;\n\
            ivec2 mySize;\n\
            float scale;\n\
            \n\
            layout(location = 0) out vec4 fragColor;\n\
            vec2 offsetRoundedCoord(int a, int b)\n\
            {\n\
                return vec2((floor(coord.x*(mySize.x)-0.5)+a+0.5)/(mySize.x),\n\
                            (floor(coord.y*(mySize.y)-0.5)+b+0.5)/(mySize.y));\n\
            }\n\
            vec4 offsetRoundedPixel(int a, int b)\n\
            {\n\
                return texture2D(mytexture, offsetRoundedCoord(a, b));\n\
            }\n\
            vec2 interpolationPhase()\n\
            {\n\
                return vec2(mod(coord.x*(mySize.x)+0.5, 1),\n\
                            mod(coord.y*(mySize.y)+0.5, 1));\n\
            }\n\
            vec2 lodRoundedCoord(int a, int b, vec2 size)\n\
            {\n\
                return vec2((floor(coord.x*(size.x)-0.5)+a+0.5)/(size.x),\n\
                            (floor(coord.y*(size.y)-0.5)+b+0.5)/(size.y));\n\
            }\n\
            vec4 lodRoundedPixel(int a, int b, vec2 size, int lod)\n\
            {\n\
                return textureLod(mytexture, lodRoundedCoord(a, b, size), lod);\n\
            }\n\
            vec2 downscalingPhase(vec2 size)\n\
            {\n\
                return vec2(mod(coord.x*(size.x)+0.5, 1),\n\
                            mod(coord.y*(size.y)+0.5, 1));\n\
            }\n\
            vec4 hermite(vec4 a, vec4 b, vec4 c, vec4 d, float i)\n\
            {\n\
                vec4  bw = (c-a)/2;\n\
                vec4  cw = (d-b)/2;\n\
                float h00 = i*i*i*2 - i*i*3 + 1;\n\
                float h10 = i*i*i - i*i*2 + i;\n\
                float h01 = -i*i*i*2 + i*i*3;\n\
                float h11 = i*i*i - i*i;\n\
                return b*h00 + bw*h10 + c*h01 + cw*h11;\n\
            }\n\
            vec4 hermiterow(int y, float i)\n\
            {\n\
                vec4 c1 = offsetRoundedPixel(-1,y);\n\
                vec4 c2 = offsetRoundedPixel(-0,y);\n\
                vec4 c3 = offsetRoundedPixel(+1,y);\n\
                vec4 c4 = offsetRoundedPixel(+2,y);\n\
                return hermite(c1, c2, c3, c4, i);\n\
            }\n\
            vec4 hermitegrid(float ix, float iy)\n\
            {\n\
                vec4 c1 = hermiterow(-1,ix);\n\
                vec4 c2 = hermiterow(-0,ix);\n\
                vec4 c3 = hermiterow(+1,ix);\n\
                vec4 c4 = hermiterow(+2,ix);\n\
                return hermite(c1, c2, c3, c4, iy);\n\
            }\n\
            float jinc(float x)\n\
            {\n\
                return texture2D(myJincLookup, vec2(x*8/512, 0)).r*2-1;\n\
            }\n\
            float jincwindow(float x, float radius)\n\
            {\n\
                if(x < -radius || x > radius) return 0;\n\
                return jinc(x) * cos(x*M_PI/2/radius);\n\
            }\n\
            bool supersamplemode;\n\
            vec4 supersamplegrid()\n\
            {\n\
                int lod = 0;\n\
                float radius = 1;\n\
                if(radius < 1) radius = 1;\n\
                vec2 phase = downscalingPhase(mySize);\n\
                float ix = phase.x;\n\
                float iy = phase.y;\n\
                int lowi  = int(floor(-radius/scale + iy));\n\
                int highi = int(ceil(radius/scale + iy));\n\
                int lowj  = int(floor(-radius/scale + ix));\n\
                int highj = int(ceil(radius/scale + ix));\n\
                vec4 c = vec4(0);\n\
                float sampleWeight = 0;\n\
                for(int i = lowi; i <= highi; i++)\n\
                {\n\
                    for(int j = lowj; j <= highj; j++)\n\
                    {\n\
                        float x = (i-ix)*scale;\n\
                        float y = (j-iy)*scale;\n\
                        if(sqrt(x*x+y*y) > radius) continue;\n\
                        float weight;\n\
                        weight = jincwindow(sqrt(x*x+y*y), radius);\n\
                        sampleWeight += weight;\n\
                        c += lodRoundedPixel(i, j, mySize, lod)*weight;\n\
                    }\n\
                }\n\
                c /= sampleWeight;\n\
                return c;\n\
            }\n\
            void main()\n\
            {\n\
                mySize = textureSize(mytexture, 0);\n\
                scale = myScale;\n\
                coord = myTexCoord;\n\
                if(scale < 1)\n\
                {\n\
                    fragColor = supersamplegrid();\n\
                }\n\
                else\n\
                {\n\
                    vec2 phase = interpolationPhase();\n\
                    vec4 c = hermitegrid(phase.x, phase.y);\n\
                    fragColor = c;\n\
                }\n\
            }\n");
            
            glUseProgram(ssam->program);
            checkerr(__LINE__);
            glUniform1i(glGetUniformLocation(ssam->program, "mytexture"), 0);
            glUniform1i(glGetUniformLocation(ssam->program, "myJincLookup"), 1);
            glUniform1f(glGetUniformLocation(ssam->program, "myScale"), 1/viewportscale);
            
            glActiveTexture(GL_TEXTURE1);
            glGenTextures(1, &jinctexid);
            glBindTexture(GL_TEXTURE_2D, jinctexid);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_R16F, 512, 1, 0, GL_RED, GL_FLOAT, jinctexture);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_MIRRORED_REPEAT);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_MIRRORED_REPEAT);

            checkerr(__LINE__);
            
            copy = new postprogram("copy", 
            "#version 330 core\n\
            uniform sampler2D mytexture;\n\
            varying vec2 myTexCoord;\n\
            \n\
            layout(location = 0) out vec4 fragColor;\n\
            void main()\n\
            {\n\
                fragColor = texture2D(mytexture, myTexCoord);\n\
            }\n");
            
            glUseProgram(copy->program);
            checkerr(__LINE__);
            glUniform1i(glGetUniformLocation(copy->program, "mytexture"), 0);
            checkerr(__LINE__);
            
            sharpen = new postprogram("sharpen", 
            "#version 330 core\n\
            uniform sampler2D mytexture;\n\
            varying vec2 myTexCoord;\n\
            uniform float amount;\n\
            \n\
            layout(location = 0) out vec4 fragColor;\n\
            vec2 offsetCoord(int x, int y, vec2 size)\n\
            {\n\
                return vec2((myTexCoord.x*size.x+x)/(size.x),\n\
                            (myTexCoord.y*size.y+y)/(size.y));\n\
            }\n\
            vec4 offsetPixel(int x, int y, vec2 size)\n\
            {\n\
                return texture(mytexture, offsetCoord(x, y, size));\n\
            }\n\
            void main()\n\
            {\n\
                vec2 mySize = textureSize(mytexture, 0);\n\
                vec4 blurred = vec4(0);\n\
                float weight = 0;\n\
                for(int y = -1; y <= 1; y++)\n\
                {\n\
                    for(int x = -1; x <= 1; x++)\n\
                    {\n\
                        float power = 1-y*y/2 * 1-x*x/2;\n\
                        weight += power;\n\
                        blurred += offsetPixel(x, y, mySize) * power;\n\
                    }\n\
                }\n\
                blurred /= weight;\n\
                fragColor = texture2D(mytexture, myTexCoord)*(1+amount) - blurred*amount;\n\
            }\n");
            
            glUseProgram(sharpen->program);
            checkerr(__LINE__);
            glUniform1i(glGetUniformLocation(sharpen->program, "mytexture"), 0);
            glUniform1f(glGetUniformLocation(sharpen->program, "amount"), sharpenamount);
            checkerr(__LINE__);
            
            distort = new postprogram("distort", 
            "#version 330 core\n\
            uniform sampler2D mytexture;\n\
            uniform float aspect; // aspect ratio w/h\n\
            uniform float fov; // half the diagonal fov in radians\n\
            varying vec2 myTexCoord;\n\
            #define M_PI 3.1415926435\n\
            \n\
            layout(location = 0) out vec4 fragColor;\n\
            void main()\n\
            {\n\
                vec2 aspect_v = normalize(vec2(aspect, 1));\n\
                // change coord range to -1 ~ +1 then apply aspect ratio within unit circle\n\
                // we want this coord's diagonals to have distance 1 from 0,0\n\
                vec2 badcoord = (myTexCoord*2-vec2(1, 1))*aspect_v;\n\
                float pole = sqrt(badcoord.x*badcoord.x + badcoord.y*badcoord.y);\n\
                // convert from polar angle to planar distance\n\
                float dist = tan(pole*fov)/tan(fov);\n\
                vec2 newcoord = badcoord/pole*dist/aspect_v/2 + vec2(0.5, 0.5);\n\
                fragColor = texture2D(mytexture, newcoord);\n\
            }\n");
            
            glUseProgram(distort->program);
            checkerr(__LINE__);
            glUniform1i(glGetUniformLocation(distort->program, "mytexture"), 0);
            glUniform1f(glGetUniformLocation(distort->program, "fov"), (fov/180.0*M_PI)/2.0);
            glUniform1f(glGetUniformLocation(distort->program, "aspect"), double(w)/double(h));
            checkerr(__LINE__);
            
            meme = new postprogram("meme", 
            "#version 330 core\n\
            uniform sampler2D mytexture;\n\
            varying vec2 myTexCoord;\n\
            \n\
            layout(location = 0) out vec4 fragColor;\n\
            void main()\n\
            {\n\
                vec4 c1 = texture2D(mytexture, myTexCoord);\n\
                vec4 c2 = texture2D(mytexture, myTexCoord*0.9+0.05);\n\
                fragColor = c1*0.8+c2*0.2;\n\
            }\n");
            
            glUseProgram(meme->program);
            checkerr(__LINE__);
            glUniform1i(glGetUniformLocation(meme->program, "mytexture"), 0);
            checkerr(__LINE__);
            
            bloom1 = new postprogram("bloom1", 
            "#version 330 core\n\
            uniform sampler2D mytexture;\n\
            uniform int radius;\n\
            uniform float gamma;\n\
            varying vec2 myTexCoord;\n\
            \n\
            layout(location = 0) out vec4 fragColor;\n\
            void main()\n\
            {\n\
                int diameter = radius*2+1;\n\
                ivec2 size = textureSize2D(mytexture, 0);\n\
                vec4 color = vec4(0,0,0,0);\n\
                float gather = 0;\n\
                for(int i = -radius; i <= radius; i++)\n\
                {\n\
                    float weight = cos(float(i)/(radius+1))*0.5+0.5;\n\
                    gather += weight;\n\
                    color += pow(texture2D(mytexture, myTexCoord+vec2(float(i)/size.x, 0)), vec4(gamma))*weight;\n\
                }\n\
                color /= gather;\n\
                fragColor = pow(color, vec4(1/gamma));\n\
            }\n");
            
            glUseProgram(bloom1->program);
            checkerr(__LINE__);
            glUniform1i(glGetUniformLocation(bloom1->program, "mytexture"), 0);
            glUniform1i(glGetUniformLocation(bloom1->program, "radius"), bloomradius); // FIXME don't base on raw pixel amount
            glUniform1f(glGetUniformLocation(bloom1->program, "gamma"), 2.2);
            checkerr(__LINE__);
            
            bloom2 = new postprogram("bloom2", 
            "#version 330 core\n\
            uniform sampler2D mytexture;\n\
            uniform int radius;\n\
            uniform float gamma;\n\
            varying vec2 myTexCoord;\n\
            \n\
            layout(location = 0) out vec4 fragColor;\n\
            void main()\n\
            {\n\
                int diameter = radius*2+1;\n\
                ivec2 size = textureSize2D(mytexture, 0);\n\
                vec4 color = vec4(0,0,0,0);\n\
                float gather = 0;\n\
                for(int i = -radius; i <= radius; i++)\n\
                {\n\
                    float weight = cos(float(i)/(radius+1))*0.5+0.5;\n\
                    gather += weight;\n\
                    color += pow(texture2D(mytexture, myTexCoord+vec2(0, float(i)/size.y)), vec4(gamma))*weight;\n\
                }\n\
                color /= gather;\n\
                fragColor = pow(color, vec4(1/gamma));\n\
            }\n");
            
            glUseProgram(bloom2->program);
            checkerr(__LINE__);
            glUniform1i(glGetUniformLocation(bloom2->program, "mytexture"), 0);
            glUniform1i(glGetUniformLocation(bloom2->program, "radius"), bloomradius); // FIXME don't base on raw pixel amount
            glUniform1f(glGetUniformLocation(bloom2->program, "gamma"), 2.2);
            checkerr(__LINE__);
            
            bloom3 = new postprogram("bloom3", 
            "#version 330 core\n\
            uniform sampler2D drybuffer;\n\
            uniform sampler2D wetbuffer;\n\
            uniform float ratio;\n\
            uniform float gamma;\n\
            varying vec2 myTexCoord;\n\
            \n\
            layout(location = 0) out vec4 fragColor;\n\
            void main()\n\
            {\n\
                vec4 color1 = pow(texture2D(drybuffer, myTexCoord), vec4(gamma));\n\
                vec4 color2 = pow(texture2D(wetbuffer, myTexCoord), vec4(gamma));\n\
                vec4 color = pow(color1*ratio + color2*(1-ratio), vec4(1/gamma));\n\
                fragColor = color;\n\
            }\n");
            
            glUseProgram(bloom3->program);
            checkerr(__LINE__);
            glUniform1i(glGetUniformLocation(bloom3->program, "wetbuffer"), 0);
            glUniform1i(glGetUniformLocation(bloom3->program, "drybuffer"), 1);
            glUniform1f(glGetUniformLocation(bloom3->program, "ratio"), 0.9);
            glUniform1f(glGetUniformLocation(bloom3->program, "gamma"), 2.2);
            checkerr(__LINE__);
            
            glActiveTexture(GL_TEXTURE0);
            
            glGenTextures(1, &FBOtexture0);
            glGenTextures(1, &FBOtexture1);
            glGenTextures(1, &FBOtexture2);
            glGenTextures(1, &FBOtexture3);
            glGenTextures(1, &FBOtexture4);
            
            glBindTexture(GL_TEXTURE_2D, FBOtexture0);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, w*viewportscale, h*viewportscale, 0, GL_RGB, GL_FLOAT, NULL);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, FBOtexture0, 0);
            
            glBindTexture(GL_TEXTURE_2D, FBOtexture1);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, w*viewportscale, h*viewportscale, 0, GL_RGB, GL_FLOAT, NULL);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, FBOtexture1, 0);
            checkerr(__LINE__);
            
            glBindTexture(GL_TEXTURE_2D, FBOtexture2);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, w, h, 0, GL_RGB, GL_FLOAT, NULL);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT2,  GL_TEXTURE_2D, FBOtexture2, 0);
            checkerr(__LINE__);
            
            glBindTexture(GL_TEXTURE_2D, FBOtexture3);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, w, h, 0, GL_RGB, GL_FLOAT, NULL);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT3, GL_TEXTURE_2D, FBOtexture3, 0);
            checkerr(__LINE__);
            
            glBindTexture(GL_TEXTURE_2D, FBOtexture4);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, w, h, 0, GL_RGB, GL_FLOAT, NULL);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT4, GL_TEXTURE_2D, FBOtexture4, 0);
            checkerr(__LINE__);
            
            checkerr(__LINE__);
        }
    }
    
    void cycle_start()
    {
        checkerr(__LINE__);
        int w2, h2;
        glfwGetFramebufferSize(win, &w2, &h2);
        
        checkerr(__LINE__);
        if(w2 != w or h2 != h)
        {
            w = w2;
            h = h2;
            checkerr(__LINE__);
            
            glBindFramebuffer(GL_DRAW_FRAMEBUFFER, FRBO);
            glBindRenderbuffer(GL_RENDERBUFFER, RBOC);
            glRenderbufferStorageMultisample(GL_RENDERBUFFER, msaa, GL_RGBA16F, w*viewportscale, h*viewportscale);
            glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, RBOC);
            
            glBindRenderbuffer(GL_RENDERBUFFER, RBOD);
            glRenderbufferStorageMultisample(GL_RENDERBUFFER, msaa, GL_DEPTH_COMPONENT32F, w*viewportscale, h*viewportscale);
            glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, RBOD);
            
            if(postprocessing)
            {
                glActiveTexture(GL_TEXTURE0);
                glBindFramebuffer(GL_DRAW_FRAMEBUFFER, FBO);
                
                glBindTexture(GL_TEXTURE_2D, FBOtexture0);
                checkerr(__LINE__);
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, w*viewportscale, h*viewportscale, 0, GL_RGB, GL_FLOAT, NULL);
                checkerr(__LINE__);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                checkerr(__LINE__);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
                checkerr(__LINE__);
                glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, FBOtexture0, 0);
                checkerr(__LINE__);
                
                glBindTexture(GL_TEXTURE_2D, FBOtexture1);
                checkerr(__LINE__);
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, w*viewportscale, h*viewportscale, 0, GL_RGB, GL_FLOAT, NULL);
                checkerr(__LINE__);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                checkerr(__LINE__);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
                checkerr(__LINE__);
                glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, FBOtexture1, 0);
                checkerr(__LINE__);
                
                glBindTexture(GL_TEXTURE_2D, FBOtexture2);
                checkerr(__LINE__);
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, w, h, 0, GL_RGB, GL_FLOAT, NULL);
                checkerr(__LINE__);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                checkerr(__LINE__);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
                checkerr(__LINE__);
                glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT2, GL_TEXTURE_2D, FBOtexture2, 0);
                checkerr(__LINE__);
                
                glBindTexture(GL_TEXTURE_2D, FBOtexture3);
                checkerr(__LINE__);
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, w, h, 0, GL_RGB, GL_FLOAT, NULL);
                checkerr(__LINE__);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                checkerr(__LINE__);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
                checkerr(__LINE__);
                glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT3, GL_TEXTURE_2D, FBOtexture3, 0);
                
                glBindTexture(GL_TEXTURE_2D, FBOtexture4);
                checkerr(__LINE__);
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, w, h, 0, GL_RGB, GL_FLOAT, NULL);
                checkerr(__LINE__);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                checkerr(__LINE__);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
                checkerr(__LINE__);
                glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT4, GL_TEXTURE_2D, FBOtexture4, 0);
                checkerr(__LINE__);
                
                glUseProgram(distort->program);
                checkerr(__LINE__);
                glUniform1f(glGetUniformLocation(distort->program, "aspect"), double(w)/double(h));
            }
        }
        
        glViewport(0, 0, w*viewportscale, h*viewportscale);
        
        glUseProgram(program);
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, FRBO);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, RBOC);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, RBOD);
        
        glDrawBuffer(GL_COLOR_ATTACHMENT0);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        
        glClearColor(0,0,0,1);
        glDepthMask(true);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
        double u_aspect = double(w)/double(h);
        double a_aspect = atan(1/u_aspect);
        double a_fovd = deg2rad(fov); // diagonal
        
        double u_d = tan(a_fovd/2); // distance to camera is our unit of distance, i.e. 1
        double u_x = u_d*cos(a_aspect);
        double u_y = u_d*sin(a_aspect);
        double a_fovx = atan(u_x)*2;
        double a_fovy = atan(u_y)*2;
        double fx = 1/tan((a_fovx/2));
        double fy = 1/tan((a_fovy/2));
        double nearf = 1;
        double farf = 10000;
        float projection[16] = {
              fx, 0.0f, 0.0f, 0.0f,
            0.0f,   fy, 0.0f, 0.0f,
            0.0f, 0.0f, (nearf+farf)/(nearf-farf), 2*nearf*farf/(nearf-farf),
            0.0f, 0.0f,-1.0f, 0.0f
        };
        
        float coordinates[16] = {
            1.0f, 0.0f, 0.0f, 0.0f,
            0.0f,-1.0f, 0.0f, 0.0f,
            0.0f, 0.0f,-1.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f
        };
        
        float translation[16] = {
            1.0f, 0.0f, 0.0f,   -x,
            0.0f, 1.0f, 0.0f,   -y,
            0.0f, 0.0f, 1.0f,   -z,
            0.0f, 0.0f, 0.0f, 1.0f
        };
        
        double r_x =  rotation_x/180.0*M_PI;
        double r_y =  rotation_y/180.0*M_PI;
        double r_z = -rotation_z/180.0*M_PI;
        float rotation_x[16] = {
                1.0f,     0.0f,     0.0f, 0.0f,
                0.0f, cos(r_x),-sin(r_x), 0.0f,
                0.0f, sin(r_x), cos(r_x), 0.0f,
                0.0f,     0.0f,     0.0f, 1.0f
        };
        float rotation_y[16] = {
            cos(r_y),     0.0f,-sin(r_y), 0.0f,
                0.0f,     1.0f,     0.0f, 0.0f,
            sin(r_y),     0.0f, cos(r_y), 0.0f,
                0.0f,     0.0f,     0.0f, 1.0f
        };
        float rotation_z[16] = {
            cos(r_z),-sin(r_z),     0.0f, 0.0f,
            sin(r_z), cos(r_z),     0.0f, 0.0f,
                0.0f,     0.0f,     1.0f, 0.0f,
                0.0f,     0.0f,     0.0f, 1.0f
        };
        
        m4mult(projection, coordinates);
        m4mult(rotation_x, rotation_y);
        m4mult(rotation_z, rotation_x);
        m4mult(projection, rotation_z);
        
        glUseProgram(cubeprogram);
        glUniformMatrix4fv(glGetUniformLocation(cubeprogram, "projection"), 1, 0, projection);
        
        m4mult(projection, translation);
        
        glUseProgram(program);
        glUniformMatrix4fv(glGetUniformLocation(program, "projection"), 1, 0, projection);
        
        glEnable(GL_CULL_FACE);
        
        checkerr(__LINE__);
    }
    void cycle_end()
    {
        
        if(!postprocessing)
        {
            glBindFramebuffer(GL_READ_FRAMEBUFFER, FRBO);
            glReadBuffer(GL_COLOR_ATTACHMENT0);
            glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
            glDrawBuffer(GL_BACK);
            glBlitFramebuffer(0,0,w,h,0,0,w,h, GL_COLOR_BUFFER_BIT, GL_NEAREST);
        }
        if(postprocessing)
        {
            glDisable(GL_CULL_FACE);
            glViewport(0, 0, w*viewportscale, h*viewportscale);
            //glUseProgram(program);
            glBindVertexArray(MainVAO);
            glBindBuffer(GL_ARRAY_BUFFER, VBO);
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, VIO);
            
            glDisable(GL_DEPTH_TEST);
            glDisable(GL_BLEND);
            
            const vertex vertices[] = {
                {-1.f, -1.f, 0.5f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f},
                { 1.f, -1.f, 0.5f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f},
                {-1.f,  1.f, 0.5f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f},
                { 1.f,  1.f, 0.5f, 1.0f, 1.0f, 0.0f, 1.0f, 0.0f},
            };
            glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices,  GL_DYNAMIC_DRAW);
            checkerr(__LINE__);
            
            glBindFramebuffer(GL_READ_FRAMEBUFFER, FRBO);
            glFramebufferRenderbuffer(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, RBOC);
            glBindFramebuffer(GL_DRAW_FRAMEBUFFER, FBO);
            glDrawBuffer(GL_COLOR_ATTACHMENT0);
            glBlitFramebuffer(0,0,w*viewportscale,h*viewportscale,0,0,w*viewportscale,h*viewportscale, GL_COLOR_BUFFER_BIT, GL_NEAREST);
            checkerr(__LINE__);
        
            unsigned int last_draw_buffer = GL_COLOR_ATTACHMENT0;
            unsigned int last_draw_texture = FBOtexture0;
            
            auto BUFFER_A = [&]()
            {
                glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
                glBindFramebuffer(GL_DRAW_FRAMEBUFFER, FBO);
            };
            auto BUFFER_DONE = [&]()
            {
                glBindFramebuffer(GL_READ_FRAMEBUFFER, FBO);
                glReadBuffer(last_draw_buffer);
                glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
                glDrawBuffer(GL_BACK);
                last_draw_buffer = GL_BACK;
            };
            
            auto FLIP_SOURCE = [&]()
            {
                if(last_draw_buffer == GL_COLOR_ATTACHMENT0)
                {
                    BUFFER_A();
                    glDrawBuffer(GL_COLOR_ATTACHMENT1);
                    last_draw_texture = FBOtexture1;
                    last_draw_buffer = GL_COLOR_ATTACHMENT1;
                    glBindTexture(GL_TEXTURE_2D, FBOtexture0);
                }
                else if(last_draw_buffer == GL_COLOR_ATTACHMENT1)
                {
                    BUFFER_A();
                    glDrawBuffer(GL_COLOR_ATTACHMENT2);
                    last_draw_texture = FBOtexture2;
                    last_draw_buffer = GL_COLOR_ATTACHMENT2;
                    glBindTexture(GL_TEXTURE_2D, FBOtexture1);
                }
                else if(last_draw_buffer == GL_COLOR_ATTACHMENT2)
                {
                    BUFFER_A();
                    glDrawBuffer(GL_COLOR_ATTACHMENT3);
                    last_draw_texture = FBOtexture3;
                    last_draw_buffer = GL_COLOR_ATTACHMENT3;
                    glBindTexture(GL_TEXTURE_2D, FBOtexture2);
                }
                else if(last_draw_buffer == GL_COLOR_ATTACHMENT3)
                {
                    BUFFER_A();
                    glDrawBuffer(GL_COLOR_ATTACHMENT4);
                    last_draw_texture = FBOtexture4;
                    last_draw_buffer = GL_COLOR_ATTACHMENT4;
                    glBindTexture(GL_TEXTURE_2D, FBOtexture3);
                }
                else if(last_draw_buffer == GL_COLOR_ATTACHMENT4)
                {
                    BUFFER_A();
                    glDrawBuffer(GL_COLOR_ATTACHMENT3);
                    last_draw_texture = FBOtexture3;
                    last_draw_buffer = GL_COLOR_ATTACHMENT3;
                    glBindTexture(GL_TEXTURE_2D, FBOtexture4);
                }
            };
            checkerr(__LINE__);
            
            if(polar)
            {
                FLIP_SOURCE();
                glUseProgram(distort->program);
                glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
            }
            else
            {
                FLIP_SOURCE();
                glUseProgram(copy->program);
                glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
            }
            
            glViewport(0, 0, w, h);
            
            if(viewportscale != 1.0f)
            {
                FLIP_SOURCE();
                glActiveTexture(GL_TEXTURE1);
                glBindTexture(GL_TEXTURE_2D, jinctexid);
                glActiveTexture(GL_TEXTURE0);
                glUseProgram(ssam->program);
                glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
                checkerr(__LINE__);
            }
            else
            {
                FLIP_SOURCE();
                glUseProgram(copy->program);
                glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
            }
            
            if(bloompasses > 0)
            {
                unsigned int bloom_dry_texture = last_draw_texture;
                for(int i = 0; i < bloompasses; i++)
                {
                    FLIP_SOURCE();
                    glUseProgram(bloom1->program);
                    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
                    checkerr(__LINE__);
                    
                    FLIP_SOURCE();
                    glUseProgram(bloom2->program);
                    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
                    checkerr(__LINE__);
                }
                
                FLIP_SOURCE();
                checkerr(__LINE__);
                glActiveTexture(GL_TEXTURE1);
                checkerr(__LINE__);
                glBindTexture(GL_TEXTURE_2D, bloom_dry_texture);
                checkerr(__LINE__);
                glActiveTexture(GL_TEXTURE0);
                glUseProgram(bloom3->program);
                glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
                checkerr(__LINE__);
            }
            
            if(dosharpen)
            {
                FLIP_SOURCE();
                glUseProgram(sharpen->program);
                glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
            }
            
            BUFFER_DONE();
            glBlitFramebuffer(0,0,w,h,0,0,w,h, GL_COLOR_BUFFER_BIT, GL_NEAREST);
        }
        
        checkerr(__LINE__);
        
        checkerr(__LINE__);
        glfwSwapBuffers(win);
        checkerr(__LINE__);
        glFinish();
        checkerr(__LINE__);
    }
    void draw_box(texture * texture, double x, double y, double z, double scale)
    {
        glUseProgram(program);
        glBindVertexArray(MainVAO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, VIO);
        
        double s = 0.5;
        const vertex vertices[4*6] = {
            // top
            {-s,-s,-s, 0.0f, 0.0f, 0.0f,-1.0f, 0.0f},
            { s,-s,-s, 1.0f, 0.0f, 0.0f,-1.0f, 0.0f},
            {-s,-s, s, 0.0f, 1.0f, 0.0f,-1.0f, 0.0f},
            { s,-s, s, 1.0f, 1.0f, 0.0f,-1.0f, 0.0f},
            // bottom
            { s, s,-s, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f},
            {-s, s,-s, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f},
            { s, s, s, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f},
            {-s, s, s, 1.0f, 1.0f, 0.0f, 1.0f, 0.0f},
            // left
            {-s, s,-s, 0.0f, 1.0f,-1.0f, 0.0f, 0.0f},
            {-s,-s,-s, 0.0f, 0.0f,-1.0f, 0.0f, 0.0f},
            {-s, s, s, 1.0f, 1.0f,-1.0f, 0.0f, 0.0f},
            {-s,-s, s, 1.0f, 0.0f,-1.0f, 0.0f, 0.0f},
            // right
            { s,-s,-s, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f},
            { s, s,-s, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f},
            { s,-s, s, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f},
            { s, s, s, 1.0f, 1.0f, 1.0f, 0.0f, 0.0f},
            // front or back
            { s,-s,-s, 0.0f, 0.0f, 0.0f, 0.0f,-1.0f},
            {-s,-s,-s, 1.0f, 0.0f, 0.0f, 0.0f,-1.0f},
            { s, s,-s, 0.0f, 1.0f, 0.0f, 0.0f,-1.0f},
            {-s, s,-s, 1.0f, 1.0f, 0.0f, 0.0f,-1.0f},
            // opposite
            {-s,-s, s, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f},
            { s,-s, s, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f},
            {-s, s, s, 1.0f, 1.0f, 0.0f, 0.0f, 1.0f},
            { s, s, s, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f},
        };
        // 65535
        const unsigned short indexes[] = {
            0, 1, 2, 3, 65535,
            4, 5, 6, 7, 65535,
            8, 9, 10, 11, 65535,
            12, 13, 14, 15, 65535,
            16, 17, 18, 19, 65535,
            20, 21, 22, 23
        };
        
        float translation[16] = {
            scale,  0.0f, 0.0f,    x,
             0.0f, scale, 0.0f,    y,
             0.0f,  0.0f, scale,   z,
             0.0f,  0.0f, 0.0f, 1.0f
        };
        
        glUniformMatrix4fv(glGetUniformLocation(program, "translation"), 1, 0, translation);
        glUniform1i(glGetUniformLocation(program, "boost"), texture->boost);
        glBindTexture(GL_TEXTURE_2D, texture->texid);
        
        glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices,  GL_DYNAMIC_DRAW);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indexes), indexes, GL_DYNAMIC_DRAW);
        glDrawElements(GL_TRIANGLE_STRIP, sizeof(indexes)/sizeof(indexes[0]), GL_UNSIGNED_SHORT, 0);
        
        //glBufferData(GL_ARRAY_BUFFER, sizeof(vertices2), vertices2,  GL_DYNAMIC_DRAW);
        //glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indexes2), indexes2, GL_DYNAMIC_DRAW);
        //glDrawElements(GL_TRIANGLE_STRIP, sizeof(indexes2)/sizeof(indexes2[0]), GL_UNSIGNED_BYTE, 0);
        checkerr(__LINE__);
    }
    void draw_cubemap(cubemap * map)
    {
        glUseProgram(cubeprogram);
        glBindVertexArray(CubeVAO);
        glBindBuffer(GL_ARRAY_BUFFER, CubeVBO);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, CubeVIO);
        
        const basicvertex vertices[] = {
            { 1, 1, 1},
            {-1, 1, 1},
            { 1,-1, 1},
            {-1,-1, 1},
            { 1,-1,-1},
            {-1,-1,-1},
            { 1, 1,-1},
            {-1, 1,-1},
        };
        // 65535
        const unsigned short indexes1[] = { 0, 1, 2, 3, 4, 5, 6, 7, 65535, 2, 4, 0, 6, 1, 7, 3, 5 };
        
        glBindTexture(GL_TEXTURE_CUBE_MAP, map->texid);
        
        glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices,  GL_DYNAMIC_DRAW);
        
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indexes1), indexes1, GL_DYNAMIC_DRAW);
        glDrawElements(GL_TRIANGLE_STRIP, sizeof(indexes1)/sizeof(indexes1[0]), GL_UNSIGNED_SHORT, 0);
        checkerr(__LINE__);
    }
    void draw_terrain(texture * texture, vertex * terrain, int terrainsize, unsigned short * terrainindexes, int terrainindexessize, double x, double y, double z, double scale)
    {
        glUseProgram(program);
        glBindVertexArray(MainVAO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, VIO);
        
        float translation[16] = {
            scale,  0.0f, 0.0f,    x,
             0.0f, scale, 0.0f,    y,
             0.0f,  0.0f, scale,   z,
             0.0f,  0.0f, 0.0f, 1.0f
        };
        
        glUniformMatrix4fv(glGetUniformLocation(program, "translation"), 1, 0, translation);
        glUniform1i(glGetUniformLocation(program, "boost"), texture->boost);
        glBindTexture(GL_TEXTURE_2D, texture->texid);
        glBufferData(GL_ARRAY_BUFFER, terrainsize, terrain,  GL_DYNAMIC_DRAW);
        
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, terrainindexessize, terrainindexes, GL_DYNAMIC_DRAW);
        glDrawElements(GL_TRIANGLE_STRIP, terrainindexessize/sizeof(terrainindexes[0]), GL_UNSIGNED_SHORT, 0);
        
        checkerr(__LINE__);
    }
};
    
std::vector<collision> worldtriangles;

const static int terrainsize = 64; // dimensions of terrain
const static int terrainscale = 64; // scale of each quad in terrain
vertex terrain[terrainsize*terrainsize];
unsigned short terrainindexes[(terrainsize*2+1)*(terrainsize-1)];

void generate_terrain()
{
    int w, h, n;
    unsigned char * data = stbi_load("heightmap.png", &w, &h, &n, 1);
    for(int y = 0; y < terrainsize; y++)
    {
        for(int x = 0; x < terrainsize; x++)
        {
            int i = y*terrainsize + x;
            terrain[i].x =  (x-terrainsize/2+0.5)*terrainscale;
            terrain[i].z = -(y-terrainsize/2+0.5)*terrainscale; // z is forward when looking slightly down
            
            terrain[i].y = (0.5-data[i]/255.0f)*8;
            terrain[i].y *= terrainscale;
            
            terrain[i].u = y/4.0;
            terrain[i].v = x/4.0;
        }
    }
    for(int y = 0; y < terrainsize; y++)
    {
        for(int x = 0; x < terrainsize; x++)
        {
            int i  = (y-0)*terrainsize + x;
            double nx;
            double ny;
            if(y == 0)
            {
                int iu = (y-0)*terrainsize + x;
                int id = (y+1)*terrainsize + x;
                ny = terrain[iu].y - terrain[id].y;
            }
            else if(y > 0 and y < terrainsize-1)
            {
                int iu = (y-1)*terrainsize + x;
                int id = (y+1)*terrainsize + x;
                ny = (terrain[iu].y - terrain[id].y)/2;
            }
            else if(y == terrainsize-1)
            {
                int iu = (y-1)*terrainsize + x;
                int id = (y+0)*terrainsize + x;
                ny = terrain[iu].y - terrain[id].y;
            }
            if(x == 0)
            {
                int il = y*terrainsize + (x-0);
                int ir = y*terrainsize + (x+1);
                nx = terrain[ir].y - terrain[il].y;
            }
            else if(x > 0 and x < terrainsize-1)
            {
                int il = y*terrainsize + (x-1);
                int ir = y*terrainsize + (x+1);
                nx = (terrain[ir].y - terrain[il].y)/2;
            }
            else if(x == terrainsize-1)
            {
                int il = y*terrainsize + (x-1);
                int ir = y*terrainsize + (x+0);
                nx = terrain[ir].y - terrain[il].y;
            }
            double dx = sin(atan(nx/terrainscale));
            double dy = sin(atan(ny/terrainscale));
            double ts = dx*dx - dy*dy;
            double dz;
            if(ts < 1.0f)
                dz = sqrt(1.0f - ts); 
            else
                dz = 0;
            
            terrain[i].nx =  dx;
            terrain[i].ny = -dz; // negative y is up but positive image value is up
            terrain[i].nz =  dy;
            //printf("n: %f %f %f\n", dx, dz, dy);
        }
    }
    
    // 65535
    int i = 0;
    for(int row = 0; row < terrainsize-1; row++)
    {
        for(int x = 0; x < terrainsize; x++) for(int y = 0; y < 2; y++) terrainindexes[i++] = x+(y+row)*terrainsize;
        terrainindexes[i++] = 65535;
    }
    
    for(int y = 0; y < terrainsize-1; y++)
    {
        for(int x = 0; x < terrainsize-1; x++)
        {
            auto ra = terrain[x  +(y  )*terrainsize];
            auto rb = terrain[x  +(y+1)*terrainsize];
            auto rc = terrain[x+1+(y  )*terrainsize];
            auto rd = terrain[x+1+(y+1)*terrainsize];
            
            auto t1 = collision(ra, rb, rc);
            auto t2 = collision(rc, rb, rd);
            
            worldtriangles.push_back(t1);
            worldtriangles.push_back(t2);
        }
    }
}

struct collider {
    rigidbody body;
    collision contact_collision = zero_collision;
    std::vector<collision> touching;
    bool collided = false;
};

struct projectile {
    double life;
    collider c;
};

struct box {
    double x, y, z, size;
};

std::vector<projectile *> shots;
std::vector<box *> boxes;

void insert_box_collisions(double x, double y, double z, double s, std::vector<collision> & triangles, bool makestatic)
{
    // top
    triangles.push_back(collision(coord(x-s, y-s, z-s), coord(x+s, y-s, z-s), coord(x-s, y-s, z+s), false, makestatic));
    triangles.push_back(collision(coord(x+s, y-s, z-s), coord(x-s, y-s, z+s), coord(x+s, y-s, z+s), true,  makestatic));
    // bottom
    triangles.push_back(collision(coord(x+s, y+s, z-s), coord(x-s, y+s, z-s), coord(x+s, y+s, z+s), false, makestatic));
    triangles.push_back(collision(coord(x-s, y+s, z-s), coord(x+s, y+s, z+s), coord(x-s, y+s, z+s), true,  makestatic));
    // left
    triangles.push_back(collision(coord(x-s, y+s, z-s), coord(x-s, y-s, z-s), coord(x-s, y+s, z+s), false, makestatic));
    triangles.push_back(collision(coord(x-s, y-s, z-s), coord(x-s, y+s, z+s), coord(x-s, y-s, z+s), true,  makestatic));
    // right
    triangles.push_back(collision(coord(x+s, y-s, z-s), coord(x+s, y+s, z-s), coord(x+s, y-s, z+s), false, makestatic));
    triangles.push_back(collision(coord(x+s, y+s, z-s), coord(x+s, y-s, z+s), coord(x+s, y+s, z+s), true,  makestatic));
    // out-from-screen
    triangles.push_back(collision(coord(x+s, y-s, z-s), coord(x-s, y-s, z-s), coord(x+s, y+s, z-s), false, makestatic));
    triangles.push_back(collision(coord(x-s, y-s, z-s), coord(x+s, y+s, z-s), coord(x-s, y+s, z-s), true,  makestatic));
    // into-screen
    triangles.push_back(collision(coord(x-s, y-s, z+s), coord(x+s, y-s, z+s), coord(x-s, y+s, z+s), false, makestatic));
    triangles.push_back(collision(coord(x+s, y-s, z+s), coord(x-s, y+s, z+s), coord(x+s, y+s, z+s), true,  makestatic));
}

void insert_box_points(double x, double y, double z, double s, std::vector<coord> & points, bool makestatic)
{
    points.push_back(coord(x-s, y-s, z-s));
    points.push_back(coord(x+s, y-s, z-s));
    points.push_back(coord(x-s, y-s, z+s));
    points.push_back(coord(x+s, y-s, z+s));
    points.push_back(coord(x-s, y+s, z-s));
    points.push_back(coord(x+s, y+s, z-s));
    points.push_back(coord(x-s, y+s, z+s));
    points.push_back(coord(x+s, y+s, z+s));
}

void insert_prism_oct_body(double x, double y, double z, double radius, double top, double bottom, std::vector<collision> & collisions, std::vector<coord> & points, bool makestatic)
{
    coord mypoints[16];
    int n = 0;
    for(int i = 0; i < 8; i++)
    {
        double r = (i+0.5)*45/180*M_PI;
        double forwards = cos(r)*radius;
        double rightwards = sin(r)*radius;
        printf("%f %f\n", forwards, rightwards);
        mypoints[n++] = coord(rightwards+x, top+y, forwards+z);
        mypoints[n++] = coord(rightwards+x, bottom+y, forwards+z);
    }
    for(int i = 0; i < 16; i++)
        collisions.push_back(collision(mypoints[i], mypoints[(i+1)%16], mypoints[(i+2)%16], !(i&1), makestatic));
    coord start = mypoints[0];
    for(int i = 1; i < 8-1; i++)
        collisions.push_back(collision(start, mypoints[(i*2)%16], mypoints[((i+1)*2)%16], true, makestatic));
    start = mypoints[1];
    for(int i = 1; i < 8-1; i++)
        collisions.push_back(collision(start, mypoints[((i*2)+1)%16], mypoints[((i+1)*2+1)%16], false, makestatic));
    
    for(int i = 0; i < 16; i++)
        points.push_back(mypoints[i]);
        
}

void insert_box_body(double x, double y, double z, double diameter, double top, double bottom, std::vector<collision> & collisions, std::vector<coord> & points, bool makestatic)
{
    double radius = diameter*sqrt(2);
    coord mypoints[8];
    int n = 0;
    for(int i = 0; i < 4; i++)
    {
        double r = (i+0.5)*90/180*M_PI;
        double forwards = cos(r)*radius;
        double rightwards = sin(r)*radius;
        printf("%f %f\n", forwards, rightwards);
        mypoints[n++] = coord(rightwards+x, top+y, forwards+z);
        mypoints[n++] = coord(rightwards+x, bottom+y, forwards+z);
    }
    for(int i = 0; i < 8; i++)
        collisions.push_back(collision(mypoints[i], mypoints[(i+1)%8], mypoints[(i+2)%8], !(i&1), makestatic));
    coord start = mypoints[0];
    for(int i = 1; i < 4-1; i++)
        collisions.push_back(collision(start, mypoints[(i*2)%8], mypoints[((i+1)*2)%8], true, makestatic));
    start = mypoints[1];
    for(int i = 1; i < 4-1; i++)
        collisions.push_back(collision(start, mypoints[((i*2)+1)%8], mypoints[((i+1)*2+1)%8], false, makestatic));
    
    for(int i = 0; i < 8; i++)
        points.push_back(mypoints[i]);
        
}

void add_box(double x, double y, double z, double size)
{
    boxes.push_back(new box({x, y, z, size}));
    
    std::vector<coord> dump;
    insert_box_body(x, y, z, size/2, -size/2, size/2, worldtriangles, dump, true);
    //insert_box_collisions(x, y, z, size/2, worldtriangles, true);
}

double shotsize = 8;

constexpr double safety = 0.001;
//constexpr double safety_bounce = 0;
constexpr double friction_gap = 1;

void body_find_contact(rigidbody & b, std::vector<collision> & world, coord motion, double speed, collision & goodcollision, double & gooddistance)
{
    // select closest collidable triangle
    for(auto p : world)
    {
        //double pseudo_safety = 2*safety/abs(dot(direction, p.normal));
        //if(pseudo_safety == INF or pseudo_safety == -INF) pseudo_safety = ;
        
        collision contact_collision = zero_collision;
        double dist;
        
        rigidbody_cast_triangle(b, coord(b.x, b.y, b.z), motion, p, dist, contact_collision, true, true);
        if(contact_collision != zero_collision)
        {
            double dottie = normalized_dot(motion, contact_collision.normal);
            if(dottie > 0)
            {
                puts("fgklasdfgZDFGHKLADG_-==25-32-=2544");
                continue;
            }
            double pseudo_safety = -safety/dottie;
            //double pseudo_safety = safety*2;
            //if(dist >= 0 and dist-pseudo_safety < gooddistance and dist-pseudo_safety < speed)
            if(dist >= 0 and dist-pseudo_safety < gooddistance and dist-pseudo_safety < speed)
            {
                goodcollision = contact_collision;
                gooddistance = dist-pseudo_safety;
            }
        }
    }
}

void collider_throw(collider & c, std::vector<collision> & world, double delta)
{
            
    rigidbody & b = c.body;
    
    collision & last_collision = c.contact_collision;
    
    //puts("frame boundary ---------------");
    
    std::vector<collision> & touching = c.touching;
    
    //touching = {};
    
    double time = delta;
    
    bool hit_anything_at_all = false;
    int baditers = 0;
    int reallybaditers = 0;
    bool refusedtohit = false;
    int iters = 0;
    while(time > 0)
    {
        iters++;
        bool reached = false;
        
        auto motion = coord({b.xspeed, b.yspeed, b.zspeed});
        auto speed = magnitude(motion)*time;
        
        if(iters%10 == 0)
            printf("iters %d speed %f\n", iters, speed);
        
        if(motion == coord())
        {
            time = 0;
            continue;
        }
        
        collision goodcollision = zero_collision;
        double gooddistance = INF;
        
        body_find_contact(b, world, motion*time, speed, goodcollision, gooddistance);
        
        if(gooddistance != INF)
        {
            refusedtohit = false;
            if(gooddistance >= -safety and gooddistance <= safety) reallybaditers++;
            
            auto p = goodcollision;
            
            double airtime = gooddistance/speed*time;
            
            double newtime = time - abs(airtime);
                
            bool already_touching = false;
            for(unsigned int j = 0; j < touching.size(); j++)
            {
                if(touching[j] == p) touching.erase(touching.begin()+(j--));
                else
                {
                    collision contact_collision = zero_collision;
                    double dist;
                    rigidbody_cast_triangle(b, coord(b.x, b.y, b.z), -touching[j].normal, touching[j], dist, contact_collision, false);
                    //rigidbody_cast_triangle(b, , -touching[j].normal, touching[j], dist, contact_collision);
                    if(dist < -(safety*2) or dist > (safety*2))
                    {
                        if(debughard) puts("erasing inside");
                        if(debughard) printf("%f\n", dist);
                        touching.erase(touching.begin()+(j--));
                    }
                }
            }
            touching.push_back(p);
            
            // FIXME: handle seams
            if(last_collision != zero_collision)
            {
                auto contact_direction = last_collision.normal;
                
                //if(false)//rigidbody_cast_triangle(b, contact_direction, last_triangle) < friction_gap*friction_gap)
                {
                    double speed = magnitude(motion);
                    if(speed != 0)
                    {
                        double normalforce = -normalized_dot(contact_direction, coord(0, 1, 0));
                        if(normalforce < 0) normalforce = 0;
                        double half_newspeed = speed-friction*abs(airtime)*normalforce;
                        if(half_newspeed < 0) half_newspeed = 0;
                        
                        motion = mult(motion, half_newspeed/speed);
                        
                        b.xspeed = motion.x;
                        b.yspeed = motion.y;
                        b.zspeed = motion.z;
                    }
                }
            }
            
            b.x += airtime*b.xspeed;
            b.y += airtime*b.yspeed;
            b.z += airtime*b.zspeed;
            
            if(touching.size() > 3)
            {
                puts("aslgjaeruogjadfhlaetrhAERFGIKERGAERHGAEUIRTH===========");
                return;
            }
            
            if(touching.size() == 1)
            {
                if(debughard) puts("hit");
                if(debughard) printf("distance %0.8f\n", gooddistance);
                if(debughard) printf("early  (%0.8f, %0.8f, %0.8f)\n", motion.x, motion.y, motion.z);
                auto dot1 = dot(motion, p.normal);
                motion = reject(motion, p.normal);
                if(debughard) printf("normal (%0.8f, %0.8f, %0.8f)\n", p.normal.x, p.normal.y, p.normal.z);
                if(debughard) printf("late   (%0.8f, %0.8f, %0.8f)\n", motion.x, motion.y, motion.z);
                auto dot2 = dot(motion, p.normal);
                if(debughard) printf("dots %0.8f %0.8f\n", dot1, dot2);
            }
            else if(touching.size() == 2)
            {
                auto previous = touching[0];
                auto current = touching[1];
                auto mydot = normalized_dot(current.normal, previous.normal);
                if(mydot <= 0)
                {
                    if(debughard) puts("seam");
                    auto seam = cross(current.normal, previous.normal);
                    motion = project(motion, seam);
                }
                else
                {
                    if(debughard) puts("skip");
                    touching = {current};
                    motion = reject(motion, current.normal);
                }
            }
            else if(touching.size() == 3)
            {
                auto previous_a = touching[0];
                auto previous_b = touching[1];
                auto current = touching[2];
                
                auto dot_a = normalized_dot(current.normal, previous_a.normal);
                auto dot_b = normalized_dot(current.normal, previous_b.normal);
                
                // skip off both old surfaces
                if(dot_a > 0 and dot_b > 0)
                {
                    if(debughard) puts("A");
                    touching = {current};
                    motion = reject(motion, current.normal);
                }
                // skip into both old surfaces
                else if(dot_a <= 0 and dot_b <= 0)
                {
                    if(debughard) puts("B");
                    motion = coord();
                }
                else if(dot_a <= 0)
                {
                    if(debughard) puts("C");
                    touching = {previous_a, current};
                    auto seam = cross(current.normal, previous_a.normal);
                    motion = project(motion, seam);
                }
                else if(dot_b <= 0)
                {
                    if(debughard) puts("D");
                    touching = {previous_b, current};
                    auto seam = cross(current.normal, previous_b.normal);
                    motion = project(motion, seam);
                }
                else
                {
                    puts("-----------------------MAYDAY STATE");
                    exit(0);
                }
            }
            
            b.xspeed = motion.x;
            b.yspeed = motion.y;
            b.zspeed = motion.z;
            
            last_collision = p;
            reached = true;
            hit_anything_at_all = true;
            
            if(newtime >= time)
            {
                //puts("backtracking");
                baditers++;
                if(baditers > 3)
                {
                    puts("----breaking early");
                    time = 0;
                }
            }
            else
            {
                time = newtime;
                //printf("looping %f\n", time);
                //printf("speed %f %f %f\n", motion.x, motion.y, motion.z);
            }
        }
        if(!hit_anything_at_all)
        {
            //puts("!hit anything at all");
            last_collision = zero_collision;
        }
        if(!reached)
        {
            if(time > 0)
            {
                // FIXME: handle seams
                if(last_collision != zero_collision)
                {
                    auto contact_direction = normalize(last_collision.normal);
                    auto motion = coord({b.xspeed, b.yspeed, b.zspeed});
                    
                    //if(false)//rigidbody_cast_triangle(b, contact_direction, last_collision) < friction_gap*friction_gap)
                    {
                        double speed = magnitude(motion);
                        if(speed != 0)
                        {
                            double normalforce = -normalized_dot(contact_direction, coord(0, 1, 0));
                            if(normalforce < 0) normalforce = 0;
                            double newspeed = speed-friction*time*normalforce;
                            if(newspeed < 0) newspeed = 0;
                            
                            motion = mult(motion, newspeed/speed);
                            
                            b.xspeed = motion.x;
                            b.yspeed = motion.y;
                            b.zspeed = motion.z;
                        }
                    }
                    //else
                    //    last_collision = zero_collision;
                }
            }
            break; // no collisions this iteration
        }
    }
    
    if(time > 0 and !refusedtohit)
    {
        b.z += time*b.zspeed;
        b.x += time*b.xspeed;
        b.y += time*b.yspeed;
    }
}

double height = 1.75*units_per_meter;
double width = height/2;
double offset = height*5/90;

collider myself;

int main (int argc, char ** argv)
{
    renderer myrenderer;
    
    auto & win = myrenderer.win;
    
    auto wood = myrenderer.load_texture("texture.png");
    if(!wood) return 0;
    wood->boost = 1;
    auto dirt = myrenderer.load_texture("ground.png");
    if(!dirt) return 0;
    auto sky = myrenderer.load_cubemap_alt("sky.png", "skytop.png", "skybottom.png");
    if(!sky) return 0;
    auto junk = myrenderer.load_texture("junk.png");
    if(!junk) return 0;
    
    generate_terrain();
    
    add_box(128, -256-8, 96+128, units_per_meter);
    add_box(0, -128, 96, units_per_meter);
    add_box(0, -128+64, 96, units_per_meter);
    add_box(64, -128+64, 96, units_per_meter);
    add_box(32, -96-64-128, 256-32, 128);
    add_box(0, -96-64-128, 256-32+128, 128);
    add_box(64, -96, 256, 256);
    add_box(1040, -890, 0, 256);
    
    myself.body.x = x;
    myself.body.y = y;
    myself.body.z = z;
    
    //myself.body.points = {coord(0, height-offset, 0)};
    
    //myself.body.minima = coord(0, 0, 0);
    //myself.body.maxima = coord(0, height-offset, 0);
    
    //void insert_prism_oct_body(double x, double y, double z, double radius, double top, double bottom, std::vector<collision> & collisions, std::vector<coord> & points, bool makestatic)
    insert_prism_oct_body(0, 0, 0, 0.5*units_per_meter, 0, height, myself.body.triangles, myself.body.points, false);
    myself.body.minima = make_minima(myself.body.points);
    myself.body.maxima = make_maxima(myself.body.points);
    
    while(!glfwWindowShouldClose(win))
    {
        glfwPollEvents();
        
        static double starttime = glfwGetTime();
        static double oldtime = starttime;
        double delta;
        
        auto newtime = glfwGetTime();
        constexpr double throttle = 1.0/60;
        if((newtime-starttime) < throttle)
        {
            std::this_thread::sleep_for(std::chrono::duration<double>(throttle-(newtime-starttime)));
            delta = throttle;
            oldtime = starttime;
            starttime = glfwGetTime();
        }
        else
        {
            starttime = glfwGetTime();
            //delta = starttime-oldtime;
            delta = throttle;
            oldtime = starttime;
        }
        
        glfwPollEvents();
        
        static double sensitivity = 1/8.0f;
        static bool continuation = false;
        if(glfwGetMouseButton(win, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS)
        {
            double xpos, ypos;
            glfwGetCursorPos(win, &xpos, &ypos);
            static double last_xpos = xpos;
            static double last_ypos = ypos;
            
            if(continuation)
            {
                double xd = xpos-last_xpos;
                double yd = ypos-last_ypos;
                
                rotation_y += xd*sensitivity; // AROUND y, i.e. horizontal
                rotation_x += yd*sensitivity;
                if(rotation_x >  90) rotation_x =  90;
                if(rotation_x < -90) rotation_x = -90;
            }
            last_xpos = xpos;
            last_ypos = ypos;
            continuation = true;
        }
        else
            continuation = false;
        
        
        coord walking;
        
        if(glfwGetKey(win, GLFW_KEY_E))
        {
            walking.z += delta*cos(deg2rad(rotation_y));//*cos(deg2rad(rotation_x));
            walking.x += delta*sin(deg2rad(rotation_y));//*cos(deg2rad(rotation_x));
            walking.y += delta*sin(deg2rad(rotation_x));
        }
        if(glfwGetKey(win, GLFW_KEY_D))
        {
            walking.z -= delta*cos(deg2rad(rotation_y));//*cos(deg2rad(rotation_x));
            walking.x -= delta*sin(deg2rad(rotation_y));//*cos(deg2rad(rotation_x));
            walking.y -= delta*sin(deg2rad(rotation_x));
        }
        
        if(glfwGetKey(win, GLFW_KEY_W))
        {
            walking.z += delta*sin(deg2rad(rotation_y));
            walking.x -= delta*cos(deg2rad(rotation_y));
        }
        if(glfwGetKey(win, GLFW_KEY_F))
        {
            walking.z -= delta*sin(deg2rad(rotation_y));
            walking.x += delta*cos(deg2rad(rotation_y));
        }
        if(glfwGetKey(win, GLFW_KEY_A))
        {
            walking.z /= 2;
            walking.x /= 2;
        }
        
        
        collision floor = zero_collision;
        double distance = INF;
        body_find_contact(myself.body, worldtriangles, coord(0,1,0), 1, floor, distance);
        static collision lastfloor = floor;
        bool jumped = false;
        double jumpspeed = -5*units_per_meter;
        double walkspeed = 8*units_per_meter;//4*units_per_meter;
        
        if(glfwGetKey(win, GLFW_KEY_SPACE) and floor != zero_collision)
        {
            myself.body.yspeed = jumpspeed;
            jumped = true;
        }
        
        double decay = 0;//pow(0.01, delta);
        if(walking != coord())
        {
            walking = normalize(walking);
            
            walking = walking*-normalized_dot(coord(0,1,0), floor.normal); // traction
            
            
            myself.body.xspeed = myself.body.xspeed*decay + walkspeed*walking.x*(1-decay);
            //myself.body.yspeed = walkspeed*walking.y;
            myself.body.zspeed = myself.body.zspeed*decay + walkspeed*walking.z*(1-decay);
        }
        else if(floor != zero_collision)
        {
            //myself.body.xspeed = myself.body.xspeed*decay;
            //myself.body.yspeed = 0;
            //myself.body.zspeed = myself.body.zspeed*decay;
        }
        //printf("speed %f %f\n", myself.body.xspeed, myself.body.zspeed);
        
        myself.body.yspeed += gravity*delta/2;
        
        collider_throw(myself, worldtriangles, delta);
        
        collision newfloor = zero_collision;
        double newdistance = INF;
        body_find_contact(myself.body, worldtriangles, coord(0,16,0), 16, newfloor, newdistance);
        
        if(floor != zero_collision and (newdistance > 1 or newfloor == zero_collision) and !jumped)
        {
            //puts("azerty");
            if(newdistance < 16 and -normalized_dot(coord(0,1,0), floor.normal) > sqrt(0.5))
            {
                //puts("asdf");
                coord velocity = coord(myself.body.xspeed, myself.body.yspeed, myself.body.zspeed);
                velocity = reject(velocity, floor.normal);
                myself.body.xspeed = velocity.x;
                myself.body.yspeed = velocity.y;
                myself.body.zspeed = velocity.z;
                myself.body.y += newdistance;
            }
            else
            {
                //puts("wasdrghd");
                myself.body.yspeed = 0;
            }
        }
        
        myself.body.yspeed += gravity*delta/2;
        
        x = myself.body.x;
        y = myself.body.y;
        z = myself.body.z;
        
        static bool right_waspressed = false;
        if(glfwGetMouseButton(win, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS)
        {
            if(!right_waspressed)
            {
                right_waspressed = true;
                puts("making box");
                
                double scatter_angle = 7;
                int scatter_size = 0;
                for(int rx = -scatter_size; rx <= scatter_size; rx++)
                {
                    for(int ry = -scatter_size; ry <= scatter_size; ry++)
                    {
                        auto newshot = new projectile;
                        newshot->c.body.x = x;
                        newshot->c.body.y = y+8;
                        newshot->c.body.z = z;
                        double shotspeed = 8*units_per_meter;
                        newshot->c.body.zspeed = shotspeed*cos(deg2rad(rotation_y+ry*scatter_angle))*cos(deg2rad(rotation_x+rx*scatter_angle));
                        newshot->c.body.xspeed = shotspeed*sin(deg2rad(rotation_y+ry*scatter_angle))*cos(deg2rad(rotation_x+rx*scatter_angle));
                        newshot->c.body.yspeed = shotspeed*sin(deg2rad(rotation_x+rx*scatter_angle));
                        newshot->life = 8;
    
                        //insert_box_collisions(0, 0, 0, shotsize/2, newshot->c.body.triangles, false);
                        //insert_box_points(0, 0, 0, shotsize/2, newshot->c.body.points, false);
                        insert_box_body(0, 0, 0, shotsize/2, -shotsize/2, shotsize/2, newshot->c.body.triangles, newshot->c.body.points, false);
                        
                        newshot->c.body.minima = make_minima(newshot->c.body.points);
                        newshot->c.body.maxima = make_maxima(newshot->c.body.points);
                        
                        shots.push_back(newshot);
                    }
                }
            }
        }
        else
            right_waspressed = false;
        
        for(unsigned int i = 0; i < shots.size(); i++)
        {
            auto s = shots[i];
            s->life -= delta;
            if(s->life <= 0)
            {
                puts("natural death");
                delete s;
                shots.erase(shots.begin()+(i--));
                continue;
            }
            else
            {
                s->c.body.yspeed += gravity*delta/2;
                
                collider_throw(s->c, worldtriangles, delta);
                
                s->c.body.yspeed += gravity*delta/2;
            }
        }
        
        myrenderer.cycle_start();
        
        for(auto s : shots)
            myrenderer.draw_box(junk, s->c.body.x, s->c.body.y, s->c.body.z, shotsize);
        for(auto b : boxes)
            myrenderer.draw_box(wood, b->x, b->y, b->z, b->size);
        
        myrenderer.draw_terrain(dirt, terrain, sizeof(terrain), terrainindexes, sizeof(terrainindexes), 0, 0, 0, 1);
        
        myrenderer.draw_cubemap(sky);
        
        myrenderer.cycle_end();
    }
    glfwDestroyWindow(win);
    
    return 0;
}
