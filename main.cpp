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
//bool debughard = true;

double time_spent_triangle = 0;
double time_spent_line = 0;
double time_spent_broadphase = 0;
double time_spent_searching = 0;
double time_spent_throwing = 0;

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
#include <set>
#include <functional>
#include <algorithm>
#include <array>
#include <list>

#define min(X,Y) (((X)<(Y))?(X):(Y))
#define max(X,Y) (((X)>(Y))?(X):(Y))

struct vertex {
    float x, y, z, u, v, nx, ny, nz;
};
struct simplevertex {
    float x, y, z, u, v;
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
    coord(double ax, double ay, double az) : x(ax), y(ay), z(az)
    {
        
    }
    coord(const basicvertex & v) : x(v.x), y(v.y), z(v.z)
    {
        
    }
    coord(const vertex & v) : x(v.x), y(v.y), z(v.z)
    {
        
    }
    bool operator==(const coord & o) const
    {
        return x == o.x && y == o.y && z == o.z;
    }
    bool operator!=(const coord & o) const
    {
        return !(*this == o);
    }
    coord operator-() const
    {
        return coord(-x, -y, -z);
    }
    bool operator>(const coord & o) const
    {
        return (x > o.x and y > o.y and z > o.z);
    }
    bool operator<(const coord & o) const
    {
        return (x < o.x and y < o.y and z < o.z);
    }
    coord operator+(const coord & b) const
    {
        return coord(x+b.x, y+b.y, z+b.z);
    }
    coord operator-(const coord & b) const
    {
        return coord(x-b.x, y-b.y, z-b.z);
    }
    coord operator*(const double & b) const
    {
        return coord(x*b, y*b, z*b);
    }
    coord operator/(const double & b) const
    {
        return coord(x/b, y/b, z/b);
    }
};

double dot(const coord & a, const coord & b)
{
    double r = 0;
    r += a.x*b.x;
    r += a.y*b.y;
    r += a.z*b.z;
    return r;
}

double magnitude(const coord & a)
{
    return sqrt(dot(a, a));
}

coord normalize(const coord & a)
{
    return a/magnitude(a);
}

coord project(const coord & a, const coord & b)
{
    return b * (dot(a, b)/dot(b, b));
}

constexpr double fudge_radians = 0.00005;
coord reject(const coord & a, const coord & b)
{
    return a - project(a, b);
}

coord cross(const coord & a, const coord & b)
{
    return coord(a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x);
}

coord make_minima(const coord & a, const coord & b)
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

coord make_maxima(const coord & a, const coord & b)
{
    return -make_minima(-a, -b);
}

template <typename T>
coord make_minima(const T & coords)
{
    coord minima;
    
    minima = *coords[0];
    
    for(const auto & c : coords)
    {
        if(c->x < minima.x) minima.x = c->x;
        if(c->y < minima.y) minima.y = c->y;
        if(c->z < minima.z) minima.z = c->z;
    }
    
    return minima;
}

template <typename T>
coord make_maxima(const T & coords)
{
    coord maxima;
    
    maxima = *coords[0];
    
    for(const auto & c : coords)
    {
        if(c->x > maxima.x) maxima.x = c->x;
        if(c->y > maxima.y) maxima.y = c->y;
        if(c->z > maxima.z) maxima.z = c->z;
    }
    
    return maxima;
}

template <int N>
coord array_make_minima(const std::array<coord, N> & coords)
{
    coord minima;
    
    minima = coords[0];
    
    for(const auto & c : coords)
    {
        if(c.x < minima.x) minima.x = c.x;
        if(c.y < minima.y) minima.y = c.y;
        if(c.z < minima.z) minima.z = c.z;
    }
    
    return minima;
}

template <int N>
coord array_make_maxima(const std::array<coord, N> & coords)
{
    coord maxima;
    
    maxima = coords[0];
    
    for(const auto & c : coords)
    {
        if(c.x > maxima.x) maxima.x = c.x;
        if(c.y > maxima.y) maxima.y = c.y;
        if(c.z > maxima.z) maxima.z = c.z;
    }
    
    return maxima;
}

// whether a and b meet
bool aabb_overlap(const coord & minima_a, const coord & maxima_a, const coord & minima_b, const coord & maxima_b)
{
    return minima_a.x <= maxima_b.x and maxima_a.x >= minima_b.x and
           minima_a.y <= maxima_b.y and maxima_a.y >= minima_b.y and
           minima_a.z <= maxima_b.z and maxima_a.z >= minima_b.z;
}

// whether a is fully inside b
bool aabb_contained(const coord & minima_a, const coord & maxima_a, const coord & minima_b, const coord & maxima_b)
{
    return minima_a.x >= minima_b.x and maxima_a.x <= maxima_b.x and
           minima_a.y >= minima_b.y and maxima_a.y <= maxima_b.y and
           minima_a.z >= minima_b.z and maxima_a.z <= maxima_b.z;
}

struct triangle {
    // a flat triangle (with vertices in counter-clockwise direction when looking down at it) has a normal pointing up and acts like a floor
    std::array<coord, 3> points;
    coord normal;
    triangle()
    {
        points = {coord(), coord(), coord()};
        
        normal = coord(0,-1,0);
    }
    triangle(const coord & i, const coord & j, const coord & k)
    {
        points = {i, j, k};
        // This Is Right. cross((0,0,-1),(1,0,0)) (i.e. cross(backwards, rightwards)) should produce (0,-1,0) (i.e. upwards)
        normal = normalize(cross(points[1]-points[0], points[2]-points[0]));
    }
    triangle(const coord & i, const coord & j, const coord & k, const coord & norm)
    {
        points = {i, j, k};
        normal = norm;
    }
    bool operator==(const triangle & o) const
    {
        return points[0] == o.points[0] && points[1] == o.points[1] && points[2] == o.points[2] && normal == o.normal;
    }
    bool operator!=(const triangle & o) const
    {
        return !(*this == o);
    }
    triangle operator+(const coord & v) const
    {
        return triangle(points[0]+v, points[1]+v, points[2]+v, normal);
    }
    triangle operator-(const coord & v) const
    {
        return triangle(points[0]-v, points[1]-v, points[2]-v, normal);
    }
};

struct line {
    std::array<coord, 2> points;
    
    coord normal1;
    coord normal2;
    
    line()
    {
        points = {coord(), coord()};
        normal1 = coord();
        normal2 = coord();
    }
    line(const coord & i, const coord & j, const coord & n1, const coord & n2)
    {
        points = {i, j};
        normal1 = n1;
        normal2 = n2;
    }
    bool operator==(const line & o) const
    {
        return points[0] == o.points[0] && points[1] == o.points[1];
    }
    bool operator!=(const line & o) const
    {
        return !(*this == o);
    }
    line operator+(const coord & v) const
    {
        return line(points[0]+v, points[1]+v, normal1, normal2);
    }
    line operator-(const coord & v) const
    {
        return line(points[0]-v, points[1]-v, normal1, normal2);
    }
};

struct lineholder {
    line lin;
    
    coord minima;
    coord maxima;
    
    lineholder()
    {
        lin = line();
        
        minima = coord();
        maxima = coord();
    }
    lineholder(const coord & i, const coord & j, const coord & n1, const coord & n2)
    {
        lin = line(i, j, n1, n2);
        minima = array_make_minima<2>(lin.points);
        maxima = array_make_maxima<2>(lin.points);
    }
    lineholder(const line & lina)
    {
        lin = lina;
        minima = array_make_minima<2>(lin.points);
        maxima = array_make_maxima<2>(lin.points);
    }
    lineholder(const line & lina, const coord & minim, const coord & maxim)
    {
        lin = lina;
        minima = minim;
        maxima = maxim;
    }
    bool operator==(const lineholder & o) const
    {
        return lin == o.lin;
    }
    bool operator!=(const lineholder & o) const
    {
        return !(*this == o);
    }
    lineholder operator+(const coord & v) const
    {
        return lineholder(lin+v, minima+v, maxima+v);
    }
};

struct triholder {
    triangle tri;
    
    coord minima;
    coord maxima;
    
    triholder()
    {
        tri = triangle();
        
        minima = coord();
        maxima = coord();
    }
    triholder(const coord & i, const coord & j, const coord & k, bool invert = false)
    {
        if(invert)
            tri = triangle(i, k, j);
        else
            tri = triangle(i, j, k);
        
        minima = array_make_minima<3>(tri.points);
        maxima = array_make_maxima<3>(tri.points);
    }
    triholder(const triangle & tria)
    {
        tri = tria;
        
        minima = array_make_minima<3>(tri.points);
        maxima = array_make_maxima<3>(tri.points);
    }
    triholder(const triangle & tria, const coord & minim, const coord & maxim)
    {
        tri = tria;
        minima = minim;
        maxima = maxim;
    }
    bool operator==(const triholder & o) const
    {
        return tri == o.tri;
    }
    bool operator!=(const triholder & o) const
    {
        return !(*this == o);
    }
    triholder operator+(const coord & v) const
    {
        return triholder(tri+v, minima+v, maxima+v);
    }
};

inline double ray_cast_triangle(const coord & o, const coord & m, const triangle & c)
{
    // no collision from behind (motion in same direction as normal)
    
    if(m == coord()) return 0;
    
    double velocity = magnitude(m);
    
    coord d = m/velocity;
    
    if(dot(d, c.normal) >= 0)
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
    
    return dot(e2, qvec) / det;
}


struct collisionstew {
    static constexpr int dimensions = 1024*16;
    
    std::vector<triholder *> triangles; // relative to x, y, z
    std::vector<lineholder *> lines; // relative to x, y, z
    std::vector<coord *> points; // relative to x, y, z
    
    triholder * insert(const triholder & value)
    {
        triholder * heap = new triholder;
        *heap = value;
        triangles.push_back(heap);
        return heap;
    }
    lineholder * insert(const lineholder & value)
    {
        lineholder * heap = new lineholder;
        *heap = value;
        lines.push_back(heap);
        return heap;
    }
    coord * insert(const coord & value)
    {
        coord * heap = new coord;
        *heap = value;
        points.push_back(heap);
        return heap;
    }
};


struct rigidbody {
    double x, y, z;
    double xspeed, yspeed, zspeed;
    coord minima; // AABB stuff
    coord maxima; // AABB stuff
    
    collisionstew collision;
};

triangle zero_triangle = triangle();

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
double rotation_y = 0;
double rotation_z = 0;

double units_per_meter = 64;
double step_size = units_per_meter/4;
double construct_stair_rise = units_per_meter/5;
double construct_stair_run = units_per_meter/5*7/5;

double gravity = 9.8*units_per_meter; // units per second per second
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

int msaa = postprocessing?1:8;
double viewPortRes = postprocessing?3.0f:1.0f;

bool dosharpen = true;
double sharpenamount = 0.35;

// long term TODO: make the bloom blur buffers low res so that high blur radiuses are cheap instead of expensive
int bloomradius = 8;
int bloompasses = 2;

int shadow_resolutioni = 1024;

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
            glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
            glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
            glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_BORDER);  
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
            out vec2 myTexCoord;\n\
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
            //if(!v or !f or !p)
            {
                char info[512];
                if(!v or !f or !p)
                {
                    puts("Failed to compile shader:");
                    puts(name);
                }
                //if(!v)
                {
                    GLint len;
                    glGetShaderInfoLog(vshader, 512, &len, info);
                    if(len != 0) puts(info);
                }
                //if(!f)
                {
                    GLint len;
                    glGetShaderInfoLog(fshader, 512, &len, info);
                    if(len != 0) puts(info);
                }
                //if(!p)
                {
                    GLint len;
                    glGetProgramInfoLog(program, 512, &len, info);
                    if(len != 0) puts(info);
                }
                if(!v or !f or !p)
                    exit(0);
            }
            
            checkerr(__LINE__);
            
            glDeleteShader(vshader);
            glDeleteShader(fshader);
        }
    };
    
    unsigned int
    BoxVAO, BoxVIO, BoxVBO,
    TerrainVAO, TerrainVIO, TerrainVBO,
    ScreenVAO, ScreenVIO, ScreenVBO,
    CubeVAO, CubeVBO, CubeVIO,
    FRBO, RBOC, RBOD,
    FBO_hires, FBOtexture0, FBOtexture1,
    FBO, FBOtexture2, FBOtexture3, FBOtexture4,
    jinctexid;
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
        glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, 1); 
        
        win = glfwCreateWindow(1280, 720, "Hello, World!", NULL, NULL);
        
        if(!win) puts("glfw failed to init"), exit(0);
        glfwMakeContextCurrent(win);
        
        if(gl3wInit()) puts("gl3w failed to init"), exit(0);
        
        printf("OpenGL %s, GLSL %s\n", glGetString(GL_VERSION), glGetString(GL_SHADING_LANGUAGE_VERSION));
        
        checkerr(__LINE__);
        
        //void (*glDebugMessageCallback)(DEBUGPROC, void *);
        //glDebugMessageCallback = gl3wGetProcAddress("glDebugMessageCallback");
        
        //glfwSwapBuffers(win);
        glfwGetFramebufferSize(win, &w, &h);
        
        checkerr(__LINE__);
        
        glViewport(0, 0, w, h);
        
        checkerr(__LINE__);
        
        glEnable(GL_DEBUG_OUTPUT);
        glDebugMessageCallback([](GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar* message, const void* userParam)
        {
            if(severity != GL_DEBUG_SEVERITY_NOTIFICATION)
                puts(message);
        }, nullptr);
        
        checkerr(__LINE__);
        
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LEQUAL);
        glEnable(GL_CULL_FACE);
        //glDisable(GL_CULL_FACE);
        glEnable(GL_PRIMITIVE_RESTART);
        glPrimitiveRestartIndex(65535);
        glFrontFace(GL_CCW);
        
        checkerr(__LINE__);
        
        glEnable(GL_SCISSOR_TEST);
        glScissor(0, 0, w*viewportscale, h*viewportscale);
        glDisable(GL_STENCIL_TEST);
        
        checkerr(__LINE__);
        
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        
        checkerr(__LINE__);
        
        glGenVertexArrays(1, &BoxVAO);
        glGenBuffers(1, &BoxVBO);
        glGenBuffers(1, &BoxVIO);
        glBindVertexArray(BoxVAO);
        glBindBuffer(GL_ARRAY_BUFFER, BoxVBO);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(vertex), (void*)0);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(vertex), (void*)offsetof(vertex, u));
        glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(vertex), (void*)offsetof(vertex, nx));
        glEnableVertexAttribArray(0);
        glEnableVertexAttribArray(1);
        glEnableVertexAttribArray(2);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, BoxVIO);
        
        checkerr(__LINE__);
        
        glGenVertexArrays(1, &TerrainVAO);
        glGenBuffers(1, &TerrainVBO);
        glGenBuffers(1, &TerrainVIO);
        glBindVertexArray(TerrainVAO);
        glBindBuffer(GL_ARRAY_BUFFER, TerrainVBO);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(vertex), (void*)0);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(vertex), (void*)offsetof(vertex, u));
        glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(vertex), (void*)offsetof(vertex, nx));
        glEnableVertexAttribArray(0);
        glEnableVertexAttribArray(1);
        glEnableVertexAttribArray(2);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, TerrainVIO);
        
        checkerr(__LINE__);
        
        glGenVertexArrays(1, &ScreenVAO);
        glGenBuffers(1, &ScreenVBO);
        glGenBuffers(1, &ScreenVIO);
        glBindVertexArray(ScreenVAO);
        glBindBuffer(GL_ARRAY_BUFFER, ScreenVBO);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(simplevertex), (void*)0);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(simplevertex), (void*)offsetof(simplevertex, u));
        glEnableVertexAttribArray(0);
        glEnableVertexAttribArray(1);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ScreenVIO);
        checkerr(__LINE__);
        
        glGenVertexArrays(1, &CubeVAO);
        glGenBuffers(1, &CubeVBO);
        glGenBuffers(1, &CubeVIO);
        glBindVertexArray(CubeVAO);
        glBindBuffer(GL_ARRAY_BUFFER, CubeVBO);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(basicvertex), (void*)0);
        glEnableVertexAttribArray(0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, CubeVIO);
        
        checkerr(__LINE__);
        
        mainprogram(vshader, fshader, program, 
        "#version 330 core\n\
        uniform mat4 projection; // world and view transform\n\
        uniform mat4 translation; // object transform\n\
        layout (location = 0) in vec3 aPos;\n\
        layout (location = 1) in vec2 aTex;\n\
        layout (location = 2) in vec3 aNormal;\n\
        out vec2 myTexCoord;\n\
        out vec3 myNormal;\n\
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
        in vec2 myTexCoord;\n\
        in vec3 myNormal;\n\
        \n\
        layout(location = 0) out vec4 fragColor;\n\
        void main()\n\
        {\n\
            vec4 color = texture2D(mytexture, myTexCoord, 0);\n\
            float dot = -dot(normalize(myNormal), normalize(vec3(1.0,1.0,-1.0)));\n\
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
        out vec3 myTexCoord;\n\
        void main()\n\
        {\n\
            gl_Position = (vec4(aPos.x, -aPos.y, aPos.z, 1.0) * projection).xyww;\n\
            myTexCoord = vec3(aPos.x, aPos.y, -aPos.z);\n\
        }\n",
        
        "#version 330 core\n\
        #define M_PI 3.1415926435\n\
        uniform samplerCube skybox;\n\
        uniform int cylindrical;\n\
        in vec3 myTexCoord;\n\
        \n\
        layout(location = 0) out vec4 fragColor;\n\
        void main()\n\
        {\n\
            if(cylindrical != 1)\n\
                fragColor = texture(skybox, myTexCoord);\n\
            else\n\
            {\n\
                vec3 coord = myTexCoord;\n\
                float angle = atan(coord.x, coord.z)*4/M_PI; // in eighth-rotations \n\
                if(angle <= -3 || angle > 3)\n\
                {\n\
                    coord.x = -1;\n\
                    coord.z = angle+4;\n\
                    if(coord.z > 1) coord.z -= 8;\n\
                    coord.z = -coord.z;\n\
                }\n\
                else if(angle <= -1)\n\
                {\n\
                    coord.x = angle+2;\n\
                    coord.z = -1;\n\
                }\n\
                else if(angle <= 1)\n\
                {\n\
                    coord.x = 1;\n\
                    coord.z = angle;\n\
                }\n\
                else\n\
                {\n\
                    coord.x = angle-2;\n\
                    coord.z = 1;\n\
                    coord.x = -coord.x;\n\
                }\n\
                vec2 pole = myTexCoord.xz;\n\
                float dist = sqrt(pole.x*pole.x + pole.y*pole.y);\n\
                dist /= 4/M_PI;\n\
                coord.xz *= dist;\n\
                coord = normalize(coord);\n\
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
        if(msaa != 1)
            glRenderbufferStorageMultisample(GL_RENDERBUFFER, msaa, GL_RGBA16F, w*viewportscale, h*viewportscale);
        else
            glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA16F, w*viewportscale, h*viewportscale);
        glBindRenderbuffer(GL_RENDERBUFFER, RBOD);
        if(msaa != 1)
            glRenderbufferStorageMultisample(GL_RENDERBUFFER, msaa, GL_DEPTH_COMPONENT32F, w*viewportscale, h*viewportscale);
        else
            glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT32F, w*viewportscale, h*viewportscale);
        
        // make framebuffer
        
        if(postprocessing)
        {
            glGenFramebuffers(1, &FBO_hires);
            glBindFramebuffer(GL_DRAW_FRAMEBUFFER, FBO_hires);
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
            in vec2 myTexCoord;\n\
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
            in vec2 myTexCoord;\n\
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
            in vec2 myTexCoord;\n\
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
            in vec2 myTexCoord;\n\
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
                //vec2 newnewcoord = myTexCoord*2-newcoord;\n\
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
            in vec2 myTexCoord;\n\
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
            in vec2 myTexCoord;\n\
            \n\
            layout(location = 0) out vec4 fragColor;\n\
            void main()\n\
            {\n\
                int diameter = radius*2+1;\n\
                ivec2 size = textureSize(mytexture, 0);\n\
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
            in vec2 myTexCoord;\n\
            \n\
            layout(location = 0) out vec4 fragColor;\n\
            void main()\n\
            {\n\
                int diameter = radius*2+1;\n\
                ivec2 size = textureSize(mytexture, 0);\n\
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
            in vec2 myTexCoord;\n\
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
            
            glBindFramebuffer(GL_DRAW_FRAMEBUFFER, FBO_hires);
            
            glGenTextures(1, &FBOtexture0);
            glGenTextures(1, &FBOtexture1);
            
            glBindTexture(GL_TEXTURE_2D, FBOtexture0);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, w*viewportscale, h*viewportscale, 0, GL_RGB, GL_FLOAT, NULL);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, FBOtexture0, 0);
            
            glBindTexture(GL_TEXTURE_2D, FBOtexture1);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, w*viewportscale, h*viewportscale, 0, GL_RGB, GL_FLOAT, NULL);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, FBOtexture1, 0);
            checkerr(__LINE__);
            
            glBindFramebuffer(GL_DRAW_FRAMEBUFFER, FBO);
            
            glGenTextures(1, &FBOtexture2);
            glGenTextures(1, &FBOtexture3);
            glGenTextures(1, &FBOtexture4);
            
            glBindTexture(GL_TEXTURE_2D, FBOtexture2);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, w, h, 0, GL_RGB, GL_FLOAT, NULL);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT2,  GL_TEXTURE_2D, FBOtexture2, 0);
            checkerr(__LINE__);
            
            glBindTexture(GL_TEXTURE_2D, FBOtexture3);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, w, h, 0, GL_RGB, GL_FLOAT, NULL);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT3, GL_TEXTURE_2D, FBOtexture3, 0);
            checkerr(__LINE__);
            
            glBindTexture(GL_TEXTURE_2D, FBOtexture4);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, w, h, 0, GL_RGB, GL_FLOAT, NULL);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT4, GL_TEXTURE_2D, FBOtexture4, 0);
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
            
            if(msaa != 1)
                glRenderbufferStorageMultisample(GL_RENDERBUFFER, msaa, GL_RGBA16F, w*viewportscale, h*viewportscale);
            else
                glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA16F, w*viewportscale, h*viewportscale);
            glBindRenderbuffer(GL_RENDERBUFFER, RBOD);
            if(msaa != 1)
                glRenderbufferStorageMultisample(GL_RENDERBUFFER, msaa, GL_DEPTH_COMPONENT32F, w*viewportscale, h*viewportscale);
            else
                glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT32F, w*viewportscale, h*viewportscale);
            
            if(postprocessing)
            {
                glActiveTexture(GL_TEXTURE0);
                
                glBindFramebuffer(GL_DRAW_FRAMEBUFFER, FBO_hires);
                
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
                
                glBindFramebuffer(GL_DRAW_FRAMEBUFFER, FBO);
                
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
        
        glUseProgram(program);
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, FRBO);
        glFramebufferRenderbuffer(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, RBOC);
        glFramebufferRenderbuffer(GL_DRAW_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, RBOD);
        
        glViewport(0, 0, w*viewportscale, h*viewportscale);
        glScissor(0, 0, w*viewportscale, h*viewportscale);
        
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
        double farf = 100000;
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
        for(const auto & b : boxes)
            draw_box(b);
        for(const auto & c : cubemaps)
            draw_cubemap(c);
        for(const auto & t : terrains)
            draw_terrain(t);
        
        boxes = {};
        cubemaps = {};
        terrains = {};
        
        if(!postprocessing)
        {
            glBindFramebuffer(GL_READ_FRAMEBUFFER, FRBO);
            glFramebufferRenderbuffer(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, RBOC);
            glReadBuffer(GL_COLOR_ATTACHMENT0);
            glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
            glDrawBuffer(GL_BACK);
            glBlitFramebuffer(0,0,w,h,0,0,w,h, GL_COLOR_BUFFER_BIT, GL_NEAREST);
            glFinish();
        }
        if(postprocessing)
        {
            glDisable(GL_CULL_FACE);
            
            glBindVertexArray(ScreenVAO);
            glBindBuffer(GL_ARRAY_BUFFER, ScreenVBO);
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ScreenVIO);
            
            glDisable(GL_DEPTH_TEST);
            glDisable(GL_BLEND);
            
            static bool init = false;
            if(!init)
            {
                init = true;
                const simplevertex vertices[] = {
                    {-1.f, -1.f, 0.5f, 0.0f, 0.0f},
                    { 1.f, -1.f, 0.5f, 1.0f, 0.0f},
                    {-1.f,  1.f, 0.5f, 0.0f, 1.0f},
                    { 1.f,  1.f, 0.5f, 1.0f, 1.0f},
                };
                glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices,  GL_STATIC_DRAW);
            }
            checkerr(__LINE__);
            
            glViewport(0, 0, w*viewportscale, h*viewportscale);
            glScissor(0, 0, w*viewportscale, h*viewportscale);
            
            glBindFramebuffer(GL_READ_FRAMEBUFFER, FRBO);
            glFramebufferRenderbuffer(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, RBOC);
            glReadBuffer(GL_COLOR_ATTACHMENT0);
            glBindFramebuffer(GL_DRAW_FRAMEBUFFER, FBO_hires);
            glDrawBuffer(GL_COLOR_ATTACHMENT0);
            glBlitFramebuffer(0,0,w*viewportscale,h*viewportscale,0,0,w*viewportscale,h*viewportscale, GL_COLOR_BUFFER_BIT, GL_NEAREST);
            
            // for debugging what's present in the framebuffer
            #if 0
            glBindFramebuffer(GL_READ_FRAMEBUFFER, FBO_hires);
            checkerr(__LINE__);
            glReadBuffer(GL_COLOR_ATTACHMENT0);
            checkerr(__LINE__);
            glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
            checkerr(__LINE__);
            glDrawBuffer(GL_BACK);
            checkerr(__LINE__);
            glBlitFramebuffer(0,0,w*viewportscale,h*viewportscale, 0,0,w,h, GL_COLOR_BUFFER_BIT, GL_NEAREST);
            checkerr(__LINE__);
            glfwSwapBuffers(win);
            checkerr(__LINE__);
            glFinish();
            checkerr(__LINE__);
            return;
            #endif
            
            unsigned int last_draw_buffer = GL_COLOR_ATTACHMENT0;
            unsigned int last_draw_texture = FBOtexture0;
            
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
                    glBindFramebuffer(GL_READ_FRAMEBUFFER, FBO_hires);
                    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, FBO_hires);
                    
                    glDrawBuffer(GL_COLOR_ATTACHMENT1);
                    last_draw_texture = FBOtexture1;
                    last_draw_buffer = GL_COLOR_ATTACHMENT1;
                    glBindTexture(GL_TEXTURE_2D, FBOtexture0);
                }
                else if(last_draw_buffer == GL_COLOR_ATTACHMENT1)
                {
                    glBindFramebuffer(GL_READ_FRAMEBUFFER, FBO_hires);
                    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, FBO);
                    
                    glDrawBuffer(GL_COLOR_ATTACHMENT2);
                    last_draw_texture = FBOtexture2;
                    last_draw_buffer = GL_COLOR_ATTACHMENT2;
                    glBindTexture(GL_TEXTURE_2D, FBOtexture1);
                }
                else if(last_draw_buffer == GL_COLOR_ATTACHMENT2)
                {
                    glBindFramebuffer(GL_READ_FRAMEBUFFER, FBO);
                    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, FBO);
                    
                    glDrawBuffer(GL_COLOR_ATTACHMENT3);
                    last_draw_texture = FBOtexture3;
                    last_draw_buffer = GL_COLOR_ATTACHMENT3;
                    glBindTexture(GL_TEXTURE_2D, FBOtexture2);
                }
                else if(last_draw_buffer == GL_COLOR_ATTACHMENT3)
                {
                    glBindFramebuffer(GL_READ_FRAMEBUFFER, FBO);
                    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, FBO);
                    
                    glDrawBuffer(GL_COLOR_ATTACHMENT4);
                    last_draw_texture = FBOtexture4;
                    last_draw_buffer = GL_COLOR_ATTACHMENT4;
                    glBindTexture(GL_TEXTURE_2D, FBOtexture3);
                }
                else if(last_draw_buffer == GL_COLOR_ATTACHMENT4)
                {
                    glBindFramebuffer(GL_READ_FRAMEBUFFER, FBO);
                    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, FBO);
                    
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
    struct scene_box {
        texture * texture;
        double x, y, z, scale, yangle;
    };
    std::vector<scene_box> boxes;
    void display_box(texture * texture, double x, double y, double z, double scale, double yangle = 0)
    {
        boxes.push_back({texture, x, y, z, scale, yangle});
    }
    
    void draw_box(const scene_box & box)
    {
        texture * texture = box.texture;
        double x = box.x;
        double y = box.y;
        double z = box.z;
        double scale = box.scale;
        double yangle = box.yangle;
        
        yangle *= M_PI/180.0;
        
        glUseProgram(program);
        glBindVertexArray(BoxVAO);
        glBindBuffer(GL_ARRAY_BUFFER, BoxVBO);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, BoxVIO);
        
        static bool init = false;
        // 65535
        const unsigned short indexes[] = {
            0, 1, 2, 3, 65535,
            4, 5, 6, 7, 65535,
            8, 9, 10, 11, 65535,
            12, 13, 14, 15, 65535,
            16, 17, 18, 19, 65535,
            20, 21, 22, 23
        };
        if(!init)
        {
            init = true;
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
            glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices,  GL_STATIC_DRAW);
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indexes), indexes, GL_STATIC_DRAW);
        }
        
        float translation[16] = {
            scale,  0.0f, 0.0f,    x,
             0.0f, scale, 0.0f,    y,
             0.0f,  0.0f, scale,   z,
             0.0f,  0.0f, 0.0f, 1.0f
        };
        
        float rotation_y[16] = {
            cos(yangle),  0.0f, sin(yangle), 0.0f,
                0.0f,     1.0f,        0.0f, 0.0f,
           -sin(yangle),  0.0f, cos(yangle), 0.0f,
                0.0f,     0.0f,        0.0f, 1.0f
        };
        
        m4mult(translation, rotation_y);
        
        glUniformMatrix4fv(glGetUniformLocation(program, "translation"), 1, 0, translation);
        glUniform1i(glGetUniformLocation(program, "boost"), texture->boost);
        glBindTexture(GL_TEXTURE_2D, texture->texid);
        
        glDrawElements(GL_TRIANGLE_STRIP, sizeof(indexes)/sizeof(indexes[0]), GL_UNSIGNED_SHORT, 0);
        
        //glBufferData(GL_ARRAY_BUFFER, sizeof(vertices2), vertices2,  GL_DYNAMIC_DRAW);
        //glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indexes2), indexes2, GL_DYNAMIC_DRAW);
        //glDrawElements(GL_TRIANGLE_STRIP, sizeof(indexes2)/sizeof(indexes2[0]), GL_UNSIGNED_BYTE, 0);
        checkerr(__LINE__);
    }
    
    struct scene_cubemap {
        cubemap * map;
    };
    std::vector<scene_cubemap> cubemaps;
    void display_cubemap(cubemap * map)
    {
        cubemaps.push_back({map});
    }
    void draw_cubemap(const scene_cubemap & cube)
    {
        cubemap * map = cube.map;
        
        glUseProgram(cubeprogram);
        glBindVertexArray(CubeVAO);
        glBindBuffer(GL_ARRAY_BUFFER, CubeVBO);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, CubeVIO);
        
        // 65535
        const unsigned short indexes[] = { 0, 1, 2, 3, 4, 5, 6, 7, 65535, 2, 4, 0, 6, 1, 7, 3, 5 };
        static bool init = false;
        if(!init)
        {
            init = true;
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
            glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices,  GL_STATIC_DRAW);
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indexes), indexes, GL_STATIC_DRAW);
        }
        
        glBindTexture(GL_TEXTURE_CUBE_MAP, map->texid);
        
        glDrawElements(GL_TRIANGLE_STRIP, sizeof(indexes)/sizeof(indexes[0]), GL_UNSIGNED_SHORT, 0);
        checkerr(__LINE__);
    }
    
    struct scene_terrain {
        texture * texture;
        vertex * terrain;
        int terrainsize;
        unsigned short * terrainindexes;
        int terrainindexessize;
        double x, y, z, scale;
    };
    std::vector<scene_terrain> terrains;
    void display_terrain(texture * texture, vertex * terrain, int terrainsize, unsigned short * terrainindexes, int terrainindexessize, double x, double y, double z, double scale)
    {
        terrains.push_back({texture, terrain, terrainsize, terrainindexes, terrainindexessize, x, y, z, scale});
    }
    void draw_terrain(const scene_terrain & terr)
    {
        texture * texture = terr.texture;
        vertex * terrain = terr.terrain;
        int terrainsize = terr.terrainsize;
        unsigned short * terrainindexes = terr.terrainindexes;
        int terrainindexessize = terr.terrainindexessize;
        double x = terr.x;
        double y = terr.y;
        double z = terr.z;
        double scale = terr.scale;
        
        glUseProgram(program);
        glBindVertexArray(TerrainVAO);
        glBindBuffer(GL_ARRAY_BUFFER, TerrainVBO);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, TerrainVIO);
        
        static bool init = false;
        if(!init)
        {
            init = true;
            glBufferData(GL_ARRAY_BUFFER, terrainsize, terrain,  GL_STATIC_DRAW);
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, terrainindexessize, terrainindexes, GL_STATIC_DRAW);
        }
            
        float translation[16] = {
            scale,  0.0f, 0.0f,    x,
             0.0f, scale, 0.0f,    y,
             0.0f,  0.0f, scale,   z,
             0.0f,  0.0f, 0.0f, 1.0f
        };
        
        glUniformMatrix4fv(glGetUniformLocation(program, "translation"), 1, 0, translation);
        glUniform1i(glGetUniformLocation(program, "boost"), texture->boost);
        glBindTexture(GL_TEXTURE_2D, texture->texid);
        
        glDrawElements(GL_TRIANGLE_STRIP, terrainindexessize/sizeof(terrainindexes[0]), GL_UNSIGNED_SHORT, 0);
        
        checkerr(__LINE__);
    }
};

template <typename T>
struct octnode {
    std::array<octnode *, 8> * nodes;
    std::vector<T *> tags;
    std::vector<T *> overtags;
    std::vector<T *> outertags;
    uint64_t contents = 0;
    bool isroot = false;
    coord minima, maxima;
    // top level constructor
    octnode(const coord & minima, const coord & maxima);
    // contained node constructor
    octnode(unsigned level, octnode * parent, int index);
    void allocate_nodes(unsigned level);
    bool contact(const coord & minima, const coord & maxima) const;
    void add(T * a, const coord & minima, const coord & maxima);
    void potentials(const coord & minima, const coord & maxima, std::vector<T *> & ret) const;
};


constexpr int depth = 6;
// Public domain fast power over integers
int ipow(int base, int exp)
{
    int result = 1;
    while (exp)
    {
        if (exp & 1)
            result *= base;
        exp >>= 1;
        base *= base;
    }
    return result;
}
// top level constructor
template <typename T>
octnode<T>::octnode(const coord & minima, const coord & maxima)
{
    isroot = true;
    
    this->minima = minima;
    this->maxima = maxima;
    
    printf("Building octree of %d bottom-level nodes (%dx%dx%d)\n", (int)ipow(8, depth), (int)ipow(2, depth), (int)ipow(2, depth), (int)ipow(2, depth));
    int size = ((int)ipow(8, depth+1)-1)/7;
    printf("(%d total nodes)\n", size);
    printf("node size: %d bytes\n", sizeof(octnode<T>));
    printf("minimum memory usage size: %d bytes\n", sizeof(octnode<T>)*size );
    
    allocate_nodes(depth);
}
// contained node constructor
template <typename T>
octnode<T>::octnode(unsigned level, octnode * parent, int index)
{
    if(index & 1)
    { // right half of parent
        minima.x = (parent->minima.x+parent->maxima.x)/2;
        maxima.x = parent->maxima.x;
    }
    else
    { // left half of parent
        minima.x = parent->minima.x;
        maxima.x = (parent->minima.x+parent->maxima.x)/2;
    }
    if(index & 2)
    { // bottom half of parent
        minima.y = (parent->minima.y+parent->maxima.y)/2;
        maxima.y = parent->maxima.y;
    }
    else
    { // top half of parent
        minima.y = parent->minima.y;
        maxima.y = (parent->minima.y+parent->maxima.y)/2;
    }
    if(index & 4)
    { // forwards half of parent
        minima.z = (parent->minima.z+parent->maxima.z)/2;
        maxima.z = parent->maxima.z;
    }
    else
    { // backwards half of parent
        minima.z = parent->minima.z;
        maxima.z = (parent->minima.z+parent->maxima.z)/2;
    }
    allocate_nodes(level);
}
template <typename T>
void octnode<T>::allocate_nodes(unsigned level)
{
    if(level > 0)
    {
        nodes = new std::array<octnode *, 8>
        {{new octnode(level-1,this,0), new octnode(level-1,this,1),
          new octnode(level-1,this,2), new octnode(level-1,this,3),
          new octnode(level-1,this,4), new octnode(level-1,this,5),
          new octnode(level-1,this,6), new octnode(level-1,this,7)}};
    }
    else
        nodes = nullptr;
}
template <typename T>
bool octnode<T>::contact(const coord & minima, const coord & maxima) const
{
    return aabb_overlap(minima, maxima, this->minima, this->maxima);
}

template <typename T>
void sorted_insert(std::vector<T> & a, const T & b)
{
    //for(const auto & e : b) if(a.count(e) == 0) a.insert(e);
    //a.merge(b);
    //for(const auto & e : b) a.insert(e);
    if(a.size() == 0)
        a = {b};
    else
    {
        size_t low = 0;
        size_t high = a.size()-1;
        while(1)
        {
            if(a[low] == b) return;
            if(a[high] == b) return;
            size_t half = (high-low)/2 + low;
            if(half == low or half == high)
                a.insert(a.begin()+half, b);
            else if(a[half] < b)
                low = half;
            else if(a[half] > b)
                high = half;
            else if(a[half] == b)
                return;
        }
    }
}

template <typename T>
void octnode<T>::add(T * a, const coord & minima, const coord & maxima)
{
    if(aabb_contained(this->minima, this->maxima, minima, maxima))
    {
        sorted_insert(overtags, a);
        contents++;
    }
    else if(contact(minima, maxima))
    {
        sorted_insert(tags, a);
        contents++;
        if(nodes)
            for(auto & e : *nodes)
                e->add(a, minima, maxima);
    }
    else if(isroot)
    {
        sorted_insert(outertags, a);
        contents++;
    }
}

// returns a sorted list of potential overlaps
template <>
void octnode<coord>::potentials(const coord & minima, const coord & maxima, std::vector<coord *> & ret) const
{
    if(contents == 0) return;
    #if 0
    dummy_union(ret, outertags);
    dummy_union(ret, tags);
    return;
    #endif
    // include alien objects if we're the root node and the object looking for stuff is not entirely inside us
    if(isroot and !aabb_contained(minima, maxima, this->minima, this->maxima))
    {
        for(const auto & e : outertags)
        {
            if(aabb_overlap(*e, *e, minima, maxima))
                sorted_insert(ret, e);
        }
    }
    // check if we're overlapping, then ask our child nodes, or insert if we're the last child
    if(contact(minima, maxima))
    {
        for(const auto & e : overtags)
            sorted_insert(ret, e);
        // pretend to be a child node if the object completely encompasses us
        if(nodes == nullptr or aabb_contained(this->minima, this->maxima, minima, maxima))
        {
            for(const auto & e : tags)
            {
                if(aabb_overlap(*e, *e, minima, maxima))
                    sorted_insert(ret, e);
            }
        }
        else
            for(const auto & e : *nodes)
                e->potentials(minima, maxima, ret);
    }
}

// returns a sorted list of potential overlaps
template <typename T>
void octnode<T>::potentials(const coord & minima, const coord & maxima, std::vector<T *> & ret) const
{
    if(contents == 0) return;
    #if 0
    for(const auto & e : outertags)
        sorted_insert(ret, e);
    for(const auto & e : tags)
        sorted_insert(ret, e);
    return;
    #endif
    // include alien objects if we're the root node and the object looking for stuff is not entirely inside us
    if(isroot and !aabb_contained(minima, maxima, this->minima, this->maxima))
    {
        for(const auto & e : outertags)
        {
            if(aabb_overlap(e->minima, e->maxima, minima, maxima))
                sorted_insert(ret, e);
        }
    }
    // check if we're overlapping, then ask our child nodes, or insert if we're the last child
    if(contact(minima, maxima))
    {
        for(const auto & e : overtags)
            sorted_insert(ret, e);
        if(nodes == nullptr or aabb_contained(this->minima, this->maxima, minima, maxima))
        {
            for(const auto & e : tags)
            {
                if(aabb_overlap(e->minima, e->maxima, minima, maxima))
                    sorted_insert(ret, e);
            }
        }
        else
            for(const auto & e : *nodes)
                e->potentials(minima, maxima, ret);
    }
}


struct worldstew {
    static constexpr int dimensions = 1024*16;
    octnode<triholder> * tri_tree = new octnode<triholder>(coord(-dimensions, -dimensions, -dimensions), coord(dimensions, dimensions, dimensions));
    octnode<lineholder> * line_tree = new octnode<lineholder>(coord(-dimensions, -dimensions, -dimensions), coord(dimensions, dimensions, dimensions));
    octnode<coord> * point_tree = new octnode<coord>(coord(-dimensions, -dimensions, -dimensions), coord(dimensions, dimensions, dimensions));
    triholder * insert(const triholder & value)
    {
        triholder * heap = new triholder;
        *heap = value;
        tri_tree->add(heap, heap->minima, heap->maxima);
        return heap;
    }
    lineholder * insert(const lineholder & value)
    {
        lineholder * heap = new lineholder;
        *heap = value;
        line_tree->add(heap, heap->minima, heap->maxima);
        return heap;
    }
    coord * insert(const coord & value)
    {
        coord * heap = new coord;
        *heap = value;
        point_tree->add(heap, value, value);
        return heap;
    }
    std::vector<triholder *> broadphase_tri(const coord & minima, const coord & maxima) const
    {
        std::vector<triholder *> ret;
        tri_tree->potentials(minima, maxima, ret);
        return ret;
    }
    std::vector<lineholder *> broadphase_line(const coord & minima, const coord & maxima) const
    {
        std::vector<lineholder *> ret;
        line_tree->potentials(minima, maxima, ret);
        return ret;
    }
    std::vector<coord *> broadphase_point(const coord & minima, const coord & maxima) const
    {
        std::vector<coord *> ret;
        point_tree->potentials(minima, maxima, ret);
        return ret;
    }
};

worldstew world;

inline void lines_cast_lines(const std::vector<lineholder *> & holder, const coord & position, const coord & motion,
                             const std::vector<lineholder *> & holder2,
                             double & d1, triangle & contact_collision,
                             const double & minimum, const double & maximum)
{
    d1 = INF;
    
    for(const auto & c : holder2)
    {
        if(motion != coord() and dot(motion, c->lin.normal1) > 0 and dot(motion, c->lin.normal2) > 0) continue;
        
        for(const auto & r : holder)
        {
            // line only belongs to back faces
            if(dot(motion, r->lin.normal1) < 0 and dot(motion, r->lin.normal2) < 0) continue;

            const coord r2 = r->lin.points[1]+position;
            
            const coord ro = r->lin.points[1]-r->lin.points[0];
            
            // line only belongs to back faces
            
            const coord c1 = c->lin.points[0];
            const coord c2 = c->lin.points[1];
            const coord c3 = c2+ro;
            const coord c4 = c1+ro;
            
            const triangle tri1a = triangle(c1, c2, c3);
            const triangle tri2a = triangle(c1, c4, c3, tri1a.normal);
            
            const bool forwards = (dot(motion, tri1a.normal) <= 0);
            
            const triangle tri1 = forwards?tri1a:triangle(c1, c3, c2, -tri1a.normal);
            const triangle tri2 = forwards?tri2a:triangle(c1, c3, c4, -tri1a.normal);
            
            const double d2 = ray_cast_triangle(r2, motion, tri1);
            const double d3 = ray_cast_triangle(r2, motion, tri2);
            
            if(d2 < d1 and d2 >= minimum and d2 <= maximum)
            {
                d1 = d2;
                contact_collision = tri1;
            }
            if(d3 < d1 and d3 >= minimum and d3 <= maximum)
            {
                d1 = d3;
                contact_collision = tri2;
            }
        }
    }
}

void rigidbody_cast_world(const collisionstew & body, const coord & position, const coord & motion,
                          const std::vector<triholder *>&  tris, const std::vector<lineholder *> & lines, const std::vector<coord *> & points,
                          double & d1, triangle & contact_collision,
                          const double & minimum, const double & maximum, bool verbose = false)
{
    d1 = INF;
    
    #if 0
    for(const auto & c : tris)
    {
        double d2 = ray_cast_triangle(position, motion, c->tri);
        if(d2 < d1 and d2 >= minimum and d2 <= maximum)
        {
            d1 = d2;
            contact_collision = c->tri;
        }
    }
    return;
    #else
    int type = 0;
    
    double start, end;
    start = glfwGetTime();
    for(const auto & c : tris)
    {
        for(const auto & p : body.points)
        {
            coord point = *p+position;
            const double d2 = ray_cast_triangle(point, motion, c->tri);
            if(d2 < d1 and d2 >= minimum and d2 <= maximum)
            {
                type = 1;
                d1 = d2;
                contact_collision = c->tri;
            }
        }
    }
    
    for(const auto & p : points)
    {
        for(const auto & c : body.triangles)
        {
            triangle mytri = c->tri+position;
            const double d2 = ray_cast_triangle(*p, -motion, mytri);
            if(d2 < d1 and d2 >= minimum and d2 <= maximum)
            {
                type = 2;
                d1 = d2;
                contact_collision = mytri + (normalize(motion)*d2);
                contact_collision.points = {contact_collision.points[0], contact_collision.points[2], contact_collision.points[1]};
                contact_collision.normal = -mytri.normal;
            }
        }
    }
    end = glfwGetTime();
    time_spent_triangle += end-start;
    
    
    start = glfwGetTime();
    double d2;
    triangle new_collision;
    lines_cast_lines(body.lines, position, motion, lines, d2, new_collision, minimum, maximum);
    if(d2 < d1 and d2 >= minimum and d2 <= maximum)
    {
        type = 3;
        d1 = d2;
        contact_collision = new_collision;
    }
    end = glfwGetTime();
    time_spent_line += end-start;
    
    
    if(verbose and debughard)
    {
        if(type == 0) puts("TYPE Z");
        if(type == 1) puts("TYPE A");
        if(type == 2) puts("TYPE B");
        if(type == 3) puts("TYPE C");
    }
    #endif
}

// The triangle we're colliding with will either be a world triangle, an moved and inverted body triangle, or a triangle derived from two edges. Point collisions will work for all of them.
void rigidbody_cast_virtual_triangle(const collisionstew & body, const coord & position, const coord & motion,
                             const triangle tri,
                             double & d1, triangle & contact_collision,
                             const double & minimum, const double & maximum, bool verbose = false)
{
    d1 = INF;
    for(const auto & p : body.points)
    //const coord point = position;
    {
        coord point = *p+position;
        
        const double d2 = ray_cast_triangle(point, motion, tri);
        if(d2 < d1 and d2 >= minimum and d2 <= maximum)
        {
            d1 = d2;
            contact_collision = tri;
        }
    }
}

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
    
    // insert triangles and diagonal lines
    for(int y = 0; y < terrainsize-1; y++)
    {
        for(int x = 0; x < terrainsize-1; x++)
        {
            auto ra = terrain[x  +(y  )*terrainsize];
            auto rb = terrain[x  +(y+1)*terrainsize];
            auto rc = terrain[x+1+(y  )*terrainsize];
            auto rd = terrain[x+1+(y+1)*terrainsize];
            // a---c
            // |  /|
            // | / |
            // |/  |
            // b---d
            auto t1 = triholder(ra, rb, rc);
            auto t2 = triholder(rc, rb, rd);
            
            world.insert(t1);
            world.insert(t2);
            
            auto myline = lineholder(rb, rc, t1.tri.normal, t2.tri.normal);
            
            world.insert(myline);
            
        }
    }
    // insert vertical lines
    for(int y = 0; y < terrainsize-1; y++)
    {
        for(int x = 0; x < terrainsize; x++)
        {
            auto ra = terrain[x  +(y  )*terrainsize];
            auto rb = terrain[x  +(y+1)*terrainsize];
            
            std::vector<coord> normals;
            normals.reserve(2);
            
            //     a---c
            //    /|  /
            //   / | /
            //  /  |/
            // d---b
            if(x > 0)
            {
                auto rd = terrain[x-1+(y+1)*terrainsize];
                auto tri = triangle(ra, rd, rb); // correspond to c, b, d in the triangle loop
                normals.push_back(tri.normal);
            }
            if(x < terrainsize-1)
            {
                auto rc = terrain[x+1+(y  )*terrainsize];
                auto tri = triangle(ra, rb, rc);
                normals.push_back(tri.normal);
            }
            
            coord n1;
            coord n2;
            if(normals.size() == 1)
            {
                n1 = normals[0];
                n2 = normals[0];
            }
            else
            {
                n1 = normals[0];
                n2 = normals[1];
            }
            
            auto myline = lineholder(ra, rb, n1, n2);
            
            world.insert(myline);
        }
    }
    // insert horizontal lines
    for(int y = 0; y < terrainsize; y++)
    {
        for(int x = 0; x < terrainsize-1; x++)
        {
            auto ra = terrain[x  +(y  )*terrainsize];
            auto rb = terrain[x+1+(y  )*terrainsize];
            
            std::vector<coord> normals;
            normals.reserve(2);
            
            //     d
            //    /|
            //   / |
            //  /  |
            // a---b
            // |  /
            // | /
            // |/
            // c
            if(y > 0)
            {
                auto rc = terrain[x  +(y+1)*terrainsize];
                auto tri = triangle(ra, rc, rb);
                normals.push_back(tri.normal);
            }
            if(y < terrainsize-1)
            {
                auto rd = terrain[x+1+(y-1)*terrainsize];
                auto tri = triangle(rd, ra, rb);
                normals.push_back(tri.normal);
            }
            
            coord n1;
            coord n2;
            if(normals.size() == 1)
            {
                n1 = normals[0];
                n2 = normals[0];
            }
            else
            {
                n1 = normals[0];
                n2 = normals[1];
            }
            
            auto myline = lineholder(ra, rb, n1, n2);
            
            world.insert(myline);
        }
    }
    // insert points
    for(int y = 0; y < terrainsize; y++)
    {
        for(int x = 0; x < terrainsize; x++)
        {
            auto ra = terrain[x  +(y  )*terrainsize];
            
            world.insert(ra);
        }
    }
}

struct collider {
    rigidbody body;
    triangle contact_collision = triangle();
    std::vector<triangle> touching;
    bool collided = false;
};

struct projectile {
    double life;
    collider c;
};

struct box {
    double x, y, z, size, yangle;
};

std::vector<projectile *> shots;
std::vector<box *> boxes;

template <typename T>
void insert_prism_oct_body(double x, double y, double z, double radius, double top, double bottom, T & collisionstew)
{
    coord mypoints[16];
    int n = 0;
    for(int i = 0; i < 8; i++)
    {
        double r = (i+0.5)*45/180*M_PI;
        double forwards = cos(r)*radius;
        double rightwards = sin(r)*radius;
        //printf("%f %f\n", forwards, rightwards);
        mypoints[n++] = coord(rightwards+x, top+y, forwards+z);
        mypoints[n++] = coord(rightwards+x, bottom+y, forwards+z);
    }
    
    // insert triangles
    
    // sides strip
    for(int i = 0; i < 16; i++)
        collisionstew.insert(triholder(mypoints[i], mypoints[(i+1)%16], mypoints[(i+2)%16], !(i&1)));
    // top(?) fan
    coord start = mypoints[0];
    for(int i = 1; i < 8-1; i++)
        collisionstew.insert(triholder(start, mypoints[(i*2)%16], mypoints[((i+1)*2)%16], true));
    // bottom(?) fan
    start = mypoints[1];
    for(int i = 1; i < 8-1; i++)
        collisionstew.insert(triholder(start, mypoints[((i*2)+1)%16], mypoints[((i+1)*2+1)%16], false));
    
    // insert lines
    
    for(int i = 0; i < 16; i+=2)
    {
        // sides are flat so this doesn't change anything
        auto prenormal = triangle(mypoints[(i-1+16)%16], mypoints[i], mypoints[i+1]).normal;
        auto postnormal = triangle(mypoints[i+1], mypoints[i], mypoints[(i+2)%16]).normal; // inverted
        
        //vertical
        collisionstew.insert(lineholder(mypoints[i], mypoints[i+1], prenormal, postnormal));
        // horizontal top
        collisionstew.insert(lineholder(mypoints[i], mypoints[(i+2)%16], postnormal, coord(0,-1,0)));
        // horizontal bottom
        // not made from the same vertices but the triangle they would make has the same normal
        collisionstew.insert(lineholder(mypoints[(i+1)%16], mypoints[(i+3)%16], postnormal, coord(0,1,0)));
    }
    
    // insert points
    
    for(int i = 0; i < 16; i++)
        collisionstew.insert(mypoints[i]);
        
}

template <typename T>
void insert_box_body(double x, double y, double z, double diameter, double top, double bottom, T & collisionstew, double yangle = 0)
{
    double radius = diameter*sqrt(2);
    coord mypoints[8];
    int n = 0;
    for(int i = 0; i < 4; i++)
    {
        double r = ((i+0.5)*90+yangle)/180*M_PI;
        double forwards = cos(r)*radius;
        double rightwards = sin(r)*radius;
        //printf("%f %f\n", forwards, rightwards);
        mypoints[n++] = coord(rightwards+x, top+y, forwards+z);
        mypoints[n++] = coord(rightwards+x, bottom+y, forwards+z);
    }
    
    // insert triangles
    
    // sides strip
    for(int i = 0; i < 8; i++)
        collisionstew.insert(triholder(mypoints[i], mypoints[(i+1)%8], mypoints[(i+2)%8], !(i&1)));
    // top(?) fan
    coord start = mypoints[0];
    for(int i = 1; i < 4-1; i++)
        collisionstew.insert(triholder(start, mypoints[(i*2)%8], mypoints[((i+1)*2)%8], true));
    // bottom(?) fan
    start = mypoints[1];
    for(int i = 1; i < 4-1; i++)
        collisionstew.insert(triholder(start, mypoints[((i*2)+1)%8], mypoints[((i+1)*2+1)%8], false));
    
    // insert lines
    
    for(int i = 0; i < 8; i+=2)
    {
        // sides are flat so this doesn't change anything
        auto prenormal = triangle(mypoints[(i-1+8)%8], mypoints[i], mypoints[i+1]).normal;
        auto postnormal = triangle(mypoints[i+1], mypoints[i], mypoints[(i+2)%8]).normal; // inverted
        
        //vertical
        collisionstew.insert(lineholder(mypoints[i], mypoints[i+1], prenormal, postnormal));
        // horizontal top
        collisionstew.insert(lineholder(mypoints[i], mypoints[(i+2)%8], postnormal, coord(0,-1,0)));
        // horizontal bottom
        // not made from the same vertices but the triangle they would make has the same normal
        collisionstew.insert(lineholder(mypoints[(i+1)%8], mypoints[(i+3)%8], postnormal, coord(0,1,0)));
    }
    
    // insert points
    
    for(int i = 0; i < 8; i++)
        collisionstew.insert(mypoints[i]);
        
}

void add_box(double x, double y, double z, double size, double yangle = 0)
{
    boxes.push_back(new box({x, y, z, size, yangle}));
    
    insert_box_body(x, y, z, size/2, -size/2, size/2, world, yangle);
}

double shotsize = 8;

constexpr double safety = 0.01;
//constexpr double safety_bounce = 0;
constexpr double friction_gap = 1;

// checks whether the given body, if placed at the given position, would collide with the given worldstew
// true if in contact
bool body_check_world(const rigidbody & body, const coord & position, const worldstew & world)
{
    const coord unit = coord(1,1,1);
    auto minima = body.minima+position-unit;
    auto maxima = body.maxima+position+unit;
    
    double start, end;
    start = glfwGetTime();
    const auto tris = world.broadphase_tri(minima, maxima);
    end = glfwGetTime();
    time_spent_broadphase += end-start;
    
    const collisionstew & b = body.collision;
    
    for(const auto & lh : b.lines)
    {
        const line newline = lh->lin+position;
        double length = magnitude(newline.points[1]-newline.points[0]);
        for(const auto & th : tris)
        {
            const triangle & newtri = th->tri;
            double d2 = ray_cast_triangle(newline.points[0], newline.points[1]-newline.points[0], newtri);
            if(d2 > length) d2 = ray_cast_triangle(newline.points[1], newline.points[0]-newline.points[1], newtri);
            if(d2 <= length) return true;
        }
    }
    
    start = glfwGetTime();
    const auto lines = world.broadphase_line(minima, maxima);
    end = glfwGetTime();
    time_spent_broadphase += end-start;
    
    for(const auto & lh : lines)
    {
        const line & newline = lh->lin;
        double length = magnitude(newline.points[1]-newline.points[0]);
        for(const auto & th : b.triangles)
        {
            const triangle newtri = th->tri+position;
            double d2 = ray_cast_triangle(newline.points[0], newline.points[1]-newline.points[0], newtri);
            if(d2 > length) d2 = ray_cast_triangle(newline.points[1], newline.points[0]-newline.points[1], newtri);
            if(d2 <= length) return true;
        }
    }
    return false;
}

void body_find_contact(const rigidbody & b, const worldstew & world, const coord & position, const coord & motion, const double & speed, triangle & goodcollision, double & gooddistance)
{
    goodcollision = triangle();
    gooddistance = INF;
    
    const coord unit = coord(1,1,1);
    auto minima = make_minima(b.minima, b.minima+motion)+position-unit;
    auto maxima = make_maxima(b.maxima, b.maxima+motion)+position+unit;
    
    double start, end;
    
    start = glfwGetTime();
    const auto world_tris = world.broadphase_tri(minima, maxima);
    const auto world_lines = world.broadphase_line(minima, maxima);
    const auto world_points = world.broadphase_point(minima, maxima);
    end = glfwGetTime();
    time_spent_broadphase += end-start;
    
    // select closest collidable triangle
    start = glfwGetTime();
    
    rigidbody_cast_world(b.collision, position, motion, world_tris, world_lines, world_points, gooddistance, goodcollision, 0, speed, true);
    
    if(goodcollision != zero_triangle)
    {
        double dottie = -dot(normalize(motion), goodcollision.normal);
        if(dottie < -0)
        {
            puts("bad dot in finding contact");
            printf("%.80f\n", dottie);
            printf("%f %f %f\n", motion.x, motion.y, motion.z);
            printf("%f %f %f\n", goodcollision.normal.x, goodcollision.normal.y, goodcollision.normal.z);
            exit(0);
        }
        gooddistance -= safety;
    }
    else
        gooddistance = INF;
    
    end = glfwGetTime();
    time_spent_searching += end-start;
}
void body_find_contact(const rigidbody & b, const worldstew & world, const coord & motion, const double & speed, triangle & goodcollision, double & gooddistance)
{
    coord position = coord(b.x, b.y, b.z);
    body_find_contact(b, world, position, motion, speed, goodcollision, gooddistance);
}

void collider_throw(collider & c, const worldstew & world, const double & delta, const double & friction, bool dostairs = false)
{
            
    rigidbody & b = c.body;
    
    triangle & last_collision = c.contact_collision;
    
    //puts("frame boundary ---------------");
    
    //std::vector<triangle> & touching = c.touching;
    //touching = {};
    std::vector<triangle> touching;
    touching.reserve(3);
    
    double time = delta;
    
    bool hit_anything_at_all = false;
    int baditers = 0;
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
            if(false)if(debughard) puts("still, breaking");
            time = 0;
            continue;
        }
        
        triangle goodcollision = zero_triangle;
        double gooddistance = INF;
        
        body_find_contact(b, world, motion*time, speed, goodcollision, gooddistance);
        gooddistance = max(0, gooddistance); // prevents us from ejecting into other walls
        
        double throw_start = glfwGetTime();
        
        if(gooddistance != INF)
        {
            refusedtohit = false;
            
            // check for stairstepping
            bool didstairs = false;
            if(dostairs and -dot(coord(0,1,0),goodcollision.normal) <= 0.7 and speed > 0)
            {
                double didtime = time * gooddistance/speed;
                double steptime = time - didtime;
                
                // throw to point of contact
                auto testposition = coord(b.x, b.y, b.z);
                testposition = testposition + motion*didtime;
                
                // on ground
                //auto asdfreg = normalize(motion);
                //printf("%f %f %f\n", asdfreg.x, asdfreg.y, asdfreg.z);
                //puts("checking for ground");
                
                // check for ground
                triangle testcollision = zero_triangle;
                double testdistance = INF;
                body_find_contact(b, world, testposition, coord(0, step_size+safety, 0), step_size+safety, testcollision, testdistance);
                testdistance = max(0, testdistance);
                if(testdistance != INF)
                {
                    // map to ground
                    testposition = testposition + coord(0, testdistance, 0);
                    // FIXME: There is still a bug around here somewhere!
                    //coord testmotion = coord(motion.x, motion.y, motion.z);
                    coord testmotion = coord(motion.x, 0, motion.z);
                    //testmotion = reject(testmotion, testcollision.normal);
                    
                    double testmotiondistance = magnitude(testmotion)*steptime; // in units of motion per frame, "distance", not "speed"
                    if(testmotiondistance != 0) // skip if no motion
                    {
                        // raise ourselves by up to step_size
                        body_find_contact(b, world, testposition, coord(0, -(step_size+safety), 0), (step_size+safety), testcollision, testdistance);
                        testdistance = max(0, testdistance);
                        if(testdistance == INF) testdistance = (step_size+safety);
                        testposition = testposition - coord(0, testdistance, 0);
                        
                        if(false) printf("raised by %f out of %f\n", testdistance, (step_size+safety));
                        
                        // check if this made it so we won't hit anything anymore
                        double step_run = min(step_size/2, testmotiondistance);
                        body_find_contact(b, world, testposition, normalize(testmotion)*step_run, step_run, testcollision, testdistance);
                        testdistance = max(0, testdistance);
                        
                        auto asdge = normalize(testmotion)*step_run;
                        if(false) printf("test motion %f %f %f\n", asdge.x, asdge.y, asdge.z);
                        
                        //puts("checking stairs from on ground");
                        if(testdistance == INF)
                        {
                            if(false) puts("A");
                            testdistance = step_run; // wouldn't hit anything
                        }
                        else if(testdistance < safety and -dot(coord(0,1,0),testcollision.normal) <= 0.7)
                        {
                            if(false) puts("B");
                            testdistance = INF; // wouldn't improve our free motion
                        }
                        // otherwise would hit something but further away than without stepping
                        
                        if(testdistance != INF)
                        {
                            // change test position to point of contact (if there was one) for the new step position
                            testposition = testposition + normalize(testmotion)*testdistance;
                            // make sure there's ground under the new position
                            if(false) puts("testing for ground from stair step position");
                            
                            triangle testcollision2 = zero_triangle;
                            double testdistance2 = INF;
                            body_find_contact(b, world, testposition, coord(0, step_size+safety, 0), step_size+safety, testcollision2, testdistance2);
                            testdistance2 = max(0, testdistance2);
                            
                            if(testdistance2 != INF)
                            {
                                if(-dot(coord(0,1,0),testcollision2.normal) > 0.7)
                                {
                                    testposition = testposition + coord(0, testdistance2, 0);
                                    
                                    if(false) puts("did stairs");
                                    if(false) printf("%f\n", testdistance);
                                    if(false) printf("%f\n", testdistance2);
                                    
                                    b.x = testposition.x;
                                    b.y = testposition.y;
                                    b.z = testposition.z;
                                    
                                    //testmotion = reject(testmotion, testcollision2.normal);
                                    //b.xspeed = testmotion.x;
                                    //b.yspeed = testmotion.y;
                                    //b.zspeed = testmotion.z;
                                    
                                    //touching = {testcollision2};
                                    touching = {};
                                    
                                    double airtime = steptime * testdistance/testmotiondistance;
                                    double newtime = steptime - abs(airtime);
                                    time = newtime; // FIXME: setting this to 0 fixes the bug, find out why
                                    
                                    didstairs = true;
                                    reached = true;
                                }
                            }
                        }
                    }
                }
            }
            
            if(!didstairs)
            {
                auto p = goodcollision;
                
                double airtime = time * gooddistance/speed;
                double newtime = time - abs(airtime);
                
                for(unsigned int j = 0; j < touching.size(); j++)
                {
                    if(touching[j] == p)
                    {
                        if(debughard) puts("erasing self");
                        touching.erase(touching.begin()+(j--));
                    }
                    else
                    {
                        triangle contact_collision = zero_triangle;
                        double dist = INF;
                        if(debughard) puts("erasure test");
                        rigidbody_cast_virtual_triangle(b.collision, coord(b.x, b.y, b.z), -touching[j].normal, touching[j], dist, contact_collision, -safety*3, safety*3);
                        if(dist == INF)
                        {
                            if(debughard) puts("erasing inside");
                            if(debughard) printf("%f\n", dist);
                            if(debughard) printf("%f\n", airtime);
                            if(debughard) printf("%f\n", time);
                            touching.erase(touching.begin()+(j--));
                        }
                    }
                }
                
                if(touching.size() >= 3)
                {
                    puts("aslgjaeruogjadfhlaetrhAERFGIKERGAERHGAEUIRTH===========");
                    exit(0);
                }
                
                touching.push_back(p);
                
                // FIXME: handle seams
                if(last_collision != zero_triangle)
                {
                    auto contact_direction = last_collision.normal;
                    
                    {
                        double speed = magnitude(motion);
                        if(speed != 0)
                        {
                            double normalforce = -dot(contact_direction, coord(0, 1, 0));
                            if(normalforce < 0) normalforce = 0;
                            double half_newspeed = speed-friction*abs(airtime)*normalforce;
                            if(half_newspeed < 0) half_newspeed = 0;
                            
                            motion = motion*(half_newspeed/speed);
                            
                            b.xspeed = motion.x;
                            b.yspeed = motion.y;
                            b.zspeed = motion.z;
                        }
                    }
                }
                
                b.x += airtime*b.xspeed;
                b.y += airtime*b.yspeed;
                b.z += airtime*b.zspeed;
                
                double fudge_space = 0;//.02;
                
                if(touching.size() == 1)
                {
                    if(debughard) puts("hit");
                    //if(debughard) printf("distance %0.8f\n", gooddistance);
                    //if(debughard) printf("early  (%0.8f, %0.8f, %0.8f)\n", motion.x, motion.y, motion.z);
                    //auto dot1 = dot(motion, p.normal);
                    motion = reject(motion, p.normal);
                    //if(debughard) printf("normal (%0.8f, %0.8f, %0.8f)\n", p.normal.x, p.normal.y, p.normal.z);
                    //if(debughard) printf("late   (%0.8f, %0.8f, %0.8f)\n", motion.x, motion.y, motion.z);
                    //auto dot2 = dot(motion, p.normal);
                    //if(debughard) printf("dots %0.8f %0.8f\n", dot1, dot2);
                }
                else if(touching.size() == 2)
                {
                    auto previous = touching[0];
                    auto current = touching[1];
                    auto mydot = dot(current.normal, previous.normal);
                    if(mydot <= fudge_space)
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
                    
                    auto dot_a = dot(current.normal, previous_a.normal);
                    auto dot_b = dot(current.normal, previous_b.normal);
                    
                    // skip off both old surfaces
                    if(dot_a > fudge_space and dot_b > fudge_space)
                    {
                        if(debughard) puts("A");
                        touching = {current};
                        motion = reject(motion, current.normal);
                    }
                    // skip into both old surfaces
                    else if(dot_a <= fudge_space and dot_b <= fudge_space)
                    {
                        if(debughard) puts("B");
                        //touching = {previous_b, current};
                        motion = coord();
                        //break;
                    }
                    // skip into surface B
                    else if(dot_a > fudge_space)
                    {
                        if(debughard) puts("C");
                        if(debughard) printf("%f %f %f\n", previous_a.normal.x, previous_a.normal.y, previous_a.normal.z);
                        if(debughard) printf("%f %f %f\n", previous_b.normal.x, previous_b.normal.y, previous_b.normal.z);
                        if(debughard) printf("%f %f %f\n", current.normal.x, current.normal.y, current.normal.z);
                        if(debughard) printf("%f %f %f\n", motion.x, motion.y, motion.z);
                        touching = {previous_b, current};
                        auto seam = cross(current.normal, previous_b.normal);
                        motion = project(motion, seam);
                        if(debughard) printf("%f %f %f\n", motion.x, motion.y, motion.z);
                    }
                    // skip into surface A
                    else if(dot_b > fudge_space)
                    {
                        if(debughard) puts("D");
                        touching = {previous_a, current};
                        auto seam = cross(current.normal, previous_a.normal);
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
                    if(debughard) printf("continuing %f\n", time);
                    if(debughard) printf("speed %f %f %f\n", motion.x, motion.y, motion.z);
                }
            }
        }
        if(!hit_anything_at_all)
        {
            if(debughard) puts("!hit anything at all");
            last_collision = zero_triangle;
        }
        if(!reached)
        {
            if(time > 0)
            {
                // FIXME: handle seams
                if(last_collision != zero_triangle)
                {
                    auto contact_direction = normalize(last_collision.normal);
                    auto motion = coord({b.xspeed, b.yspeed, b.zspeed});
                    
                    {
                        double speed = magnitude(motion);
                        if(speed != 0)
                        {
                            double normalforce = -dot(contact_direction, coord(0, 1, 0));
                            if(normalforce < 0) normalforce = 0;
                            double newspeed = speed-friction*time*normalforce;
                            if(newspeed < 0) newspeed = 0;
                            
                            motion = motion*(newspeed/speed);
                            
                            b.xspeed = motion.x;
                            b.yspeed = motion.y;
                            b.zspeed = motion.z;
                        }
                    }
                    //else
                    //    last_collision = zero_triangle;
                }
            }
            break; // no collisions this iteration
        }
        
        double throw_end = glfwGetTime();
        
        time_spent_throwing += throw_end-throw_start;
    }
    
    if(time > 0 and !refusedtohit)
    {
        if(debughard) puts("running auto motion");
        b.z += time*b.zspeed;
        b.x += time*b.xspeed;
        b.y += time*b.yspeed;
    }
    if(debughard) puts("throw over");
}

double height = 1.75*units_per_meter;
double width = height/2;
double offset = round(height*5/90);

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
    
    // far away
    add_box(0, 1024+256-8, 2048, 2048);
    add_box(0, 1024+256-8-2048, 2048+2048, 2048);
    
    
    add_box(128, -256, 96+128, units_per_meter);
    
    add_box(0, -128, 96, units_per_meter);
    add_box(0, -128+64, 96, units_per_meter);
    add_box(64, -128+64, 96, units_per_meter);
    add_box(-16, -128+16, 96-32, units_per_meter);
    add_box(-32, -128+32, 96-64, units_per_meter);
    add_box(-48, -128+48, 96-96, units_per_meter);
    
    add_box(32, -96-64-128-64-32, 256-32, units_per_meter, 5);
    add_box(32, -96-64-128, 256-32, 128);
    add_box(0, -96-64-128, 256-32+128, 128);
    add_box(64, -96, 256, 256);
    
    add_box(-512, 64, 256, 128);
    for(int i = 1; i < 10; i++)
        add_box(-512-i, 64-construct_stair_rise*i, 256+construct_stair_run*i, 128);
    
    
    // second stack
    add_box(128+256, -96, 256, 256, 20);
    add_box(128+256, -96-256, 256, 256, 10);
    
    // on top of the second stack
    add_box(256+128, -256*2, 128, units_per_meter, 45);
    double throwaway_scale = 1.0;
    double throwaway_scale2 = 12;
    add_box(256+128+units_per_meter*1*throwaway_scale, -256*2-units_per_meter*1, 128, units_per_meter, 45+1*throwaway_scale2);
    add_box(256+128+units_per_meter*2*throwaway_scale, -256*2-units_per_meter*2, 128, units_per_meter, 45+2*throwaway_scale2);
    add_box(256+128+units_per_meter*3*throwaway_scale, -256*2-units_per_meter*3, 128, units_per_meter, 45+3*throwaway_scale2);
    add_box(256+128+units_per_meter*4*throwaway_scale, -256*2-units_per_meter*4, 128, units_per_meter, 45+4*throwaway_scale2);
    add_box(256+128+units_per_meter*5*throwaway_scale, -256*2-units_per_meter*5, 128, units_per_meter, 45+5*throwaway_scale2);
    add_box(256+128+units_per_meter*6*throwaway_scale, -256*2-units_per_meter*6, 128, units_per_meter, 45+6*throwaway_scale2);
    add_box(256+128+units_per_meter*7*throwaway_scale, -256*2-units_per_meter*7, 128, units_per_meter, 45+7*throwaway_scale2);
    
    // far right
    add_box(1040, -890, 0, 256);
    
    myself.body.x = x;
    myself.body.y = y;
    myself.body.z = z;
    
    //myself.body.points = {coord(0, height-offset, 0)};
    
    //myself.body.minima = coord(0, 0, 0);
    //myself.body.maxima = coord(0, height-offset, 0);
    
    //void insert_prism_oct_body(double x, double y, double z, double radius, double top, double bottom, std::vector<collision> & collisions, std::vector<coord> & points, bool makestatic)
    insert_prism_oct_body(0, -offset, 0, 0.5*units_per_meter, 0, height, myself.body.collision);
    //insert_prism_oct_body(0, 0, 0, 32, 0, 32, myself.body.triangles, myself.body.points, false);
    myself.body.minima = make_minima(myself.body.collision.points);
    myself.body.maxima = make_maxima(myself.body.collision.points);
    
    
    double time_spent_rendering = 0;
    while(!glfwWindowShouldClose(win))
    {
        glfwPollEvents();
        
        static double starttime = glfwGetTime();
        static double oldtime = starttime;
        double delta;
        
        auto newtime = glfwGetTime();
        auto frametime = (newtime-starttime);
        
        if(0)
        {
            printf(
            "Frametime: %.2fms\n"
            "Possible framerate: %.2f\n"
            "live projectinoes %d\n"
            "time spent rendering %.2fms\n"
            "time spent broadphase %.2fms\n"
            "time spent searching %.2fms\n"
            //"time spent point-triangle colliding %.2fms\n"
            //"time spent line colliding %.2fms\n"
            //"time spent throwing %.2fms\n"
            , frametime*1000
            , 1/frametime
            , shots.size()
            , time_spent_rendering*1000
            , time_spent_broadphase*1000
            , time_spent_searching*1000
            //, time_spent_triangle*1000
            //, time_spent_line*1000
            //, time_spent_throwing*1000
            );
            
            time_spent_broadphase = 0;
            time_spent_searching = 0;
            time_spent_throwing = 0;
            time_spent_triangle = 0;
            time_spent_line = 0;
            
            double testtime = glfwGetTime();
            printf("time spent printing time stuff %.2fms\n", (testtime-newtime)*1000);
        }
        
        newtime = glfwGetTime();
        frametime = (newtime-starttime);
        
        constexpr double throttle = 1.0/125;
        if(frametime < throttle)
        {
            std::this_thread::sleep_for(std::chrono::duration<double>(throttle-frametime));
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
        static bool focused = false;
        
        static bool holding_m1 = false;
        if(glfwGetMouseButton(win, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS)
        {
            if(!holding_m1)
            {
                glfwSetInputMode(win, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
                focused = true;
            }
            holding_m1 = true;
        }
        else
        {
            if(holding_m1)
            {
                glfwSetInputMode(win, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
                focused = false;
            }
            holding_m1 = false;
        }
        
        static bool holding_z = false;
        if(glfwGetKey(win, GLFW_KEY_Z))
        {
            if(!holding_z)
            {
                if(focused)
                {
                    glfwSetInputMode(win, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
                    focused = false;
                }
                else
                {
                    glfwSetInputMode(win, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
                    focused = true;
                }
            }
            holding_z = true;
        }
        else
            holding_z = false;
        
        static double sensitivity = 1/16.0f;
        static bool continuation = false;
        if(focused)
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
            //walking.y += delta*sin(deg2rad(rotation_x));
        }
        if(glfwGetKey(win, GLFW_KEY_D))
        {
            walking.z -= delta*cos(deg2rad(rotation_y));//*cos(deg2rad(rotation_x));
            walking.x -= delta*sin(deg2rad(rotation_y));//*cos(deg2rad(rotation_x));
            //walking.y -= delta*sin(deg2rad(rotation_x));
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
        
        bool slowwalk = glfwGetKey(win, GLFW_KEY_A);
        
        triangle floor = zero_triangle;
        double distance = INF;
        body_find_contact(myself.body, world, coord(0,1,0), 1, floor, distance);
        static triangle lastfloor = floor;
        bool jumped = false;
        double jumpspeed = -5*units_per_meter;
        double walkspeed = 6.5*units_per_meter;//4*units_per_meter;
        
        if(glfwGetKey(win, GLFW_KEY_SPACE) and floor != zero_triangle)
        {
            myself.body.yspeed = jumpspeed;
            jumped = true;
        }
        
        bool onfloor = false;
        
        if(floor != zero_triangle and !jumped)
        {
            auto contact = -dot(coord(0,1,0), floor.normal);
            
            if(contact > 0.7) // ~45.57 degrees not exactly 45
                onfloor = true;
        }
        if(walking == coord())
        {
            if(onfloor)
            {
                // doing this instead of friction reduces "iceskating" (turns being wider than they should be while holding forward)
                double drag = pow(0.01, delta);
                myself.body.xspeed *= drag;
                myself.body.zspeed *= drag;
                
                double friction = 20*units_per_meter;
                double speed = sqrt(myself.body.xspeed*myself.body.xspeed + myself.body.zspeed*myself.body.zspeed);
                if(speed > 0)
                {
                    //puts("frictioning");
                    double newspeed = speed - friction*delta;
                    if(newspeed < 0) newspeed = 0;
                    myself.body.xspeed *= newspeed/speed;
                    myself.body.zspeed *= newspeed/speed;
                    myself.body.yspeed *= newspeed/speed;
                }
            }
        }
        if(walking != coord())
        {
            walking = normalize(walking);
            if(onfloor)
            {
                double mywalkspeed = walkspeed;
                if(slowwalk)
                {
                    mywalkspeed = mywalkspeed*0.5;
                    walking = walking*0.5;
                }
                // doing this instead of friction reduces "iceskating" (turns being wider than they should be while holding forward)
                double drag = pow(0.01, delta);
                myself.body.xspeed *= drag;
                myself.body.zspeed *= drag;
                
                double accel = 40*units_per_meter;
                auto startvector = coord(myself.body.xspeed, 0, myself.body.zspeed);
                double startspeed = magnitude(startvector);
                double current_relative_speed;
                double dottie = dot(normalize(startvector), walking);
                if(startspeed)
                    current_relative_speed = startspeed*dottie;
                else
                    current_relative_speed = 0;
                if(current_relative_speed < mywalkspeed)
                {
                    double addition = accel*delta;
                    if(addition > mywalkspeed-current_relative_speed) addition = mywalkspeed-current_relative_speed;
                    myself.body.xspeed += walking.x*addition;
                    myself.body.zspeed += walking.z*addition;
                    
                    double endspeed = sqrt(myself.body.xspeed*myself.body.xspeed + myself.body.zspeed*myself.body.zspeed);
                    if(endspeed > mywalkspeed)
                    {
                        double speedlimit = max(startspeed, mywalkspeed);
                        double factor = speedlimit/endspeed;
                        
                        myself.body.xspeed *= factor;
                        myself.body.zspeed *= factor;
                        
                    }
                }
            }
            else
            {
                double accel = 40*units_per_meter;
                auto startvector = coord(myself.body.xspeed, 0, myself.body.zspeed);
                double current_relative_speed;
                if(startvector != coord())
                {
                    double dottie = dot(normalize(startvector), walking);
                    current_relative_speed = magnitude(startvector)*dottie;
                }
                else
                    current_relative_speed = 0;
                if(current_relative_speed < 30)
                {
                    double addition = accel*delta;
                    if(addition > 30-current_relative_speed) addition = 30-current_relative_speed;
                    myself.body.xspeed = myself.body.xspeed + walking.x*addition;
                    myself.body.zspeed = myself.body.zspeed + walking.z*addition;
                }
            }
        }
        
        if(!onfloor)
            myself.body.yspeed += gravity*delta/2;
        
        collider_throw(myself, world, delta, 0, true);
        
        double speed = magnitude(coord(myself.body.xspeed, 0, myself.body.zspeed))*delta;
        
        triangle newfloor = zero_triangle;
        double newdistance = INF;
        body_find_contact(myself.body, world, coord(0,speed+step_size,0), speed+step_size, newfloor, newdistance);
        
        if(floor != zero_triangle and (newdistance > 1 or newfloor == zero_triangle) and !jumped)
        {
            // stick to floor
            if(newdistance != INF and -dot(coord(0,1,0), floor.normal) > 0.7) // ~45.57 degrees not exactly 45
            {
                //puts("asdf");
                coord velocity = coord(myself.body.xspeed, myself.body.yspeed, myself.body.zspeed);
                velocity = reject(velocity, floor.normal);
                myself.body.xspeed = velocity.x;
                myself.body.yspeed = velocity.y;
                myself.body.zspeed = velocity.z;
                myself.body.y += newdistance;
            }
            // run off ledge
            else
            {
                //puts("wasdrghd");
                myself.body.yspeed = 0;
            }
        }
        
        if(!onfloor)
            myself.body.yspeed += gravity*delta/2;
        
        //printf("speed %f\n", sqrt(myself.body.xspeed*myself.body.xspeed + myself.body.zspeed*myself.body.zspeed)/units_per_meter);
        
        x = myself.body.x;
        y = myself.body.y;
        z = myself.body.z;
        
        //printf("frame end position %f %f %f\n", x, y, z);
        
        static bool right_waspressed = false;
        if(glfwGetMouseButton(win, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS)
        {
            if(!right_waspressed)
            {
                right_waspressed = true;
                puts("making box");
                
                double scatter_angle = 7;
                int scatter_size = 5;
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
    
                        insert_box_body(0, 0, 0, shotsize/2, -shotsize/2, shotsize/2, newshot->c.body.collision);
                        
                        newshot->c.body.minima = make_minima(newshot->c.body.collision.points);
                        newshot->c.body.maxima = make_maxima(newshot->c.body.collision.points);
                        
                        shots.push_back(newshot);
                    }
                }
                printf("live projectinoes %d\n", shots.size());
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
                
                collider_throw(s->c, world, delta, 10*units_per_meter);
                
                s->c.body.yspeed += gravity*delta/2;
            }
        }
        
        double render_start = glfwGetTime();
        
        myrenderer.cycle_start();
        
        for(const auto & s : shots)
            myrenderer.display_box(junk, s->c.body.x, s->c.body.y, s->c.body.z, shotsize);
        for(const auto & b : boxes)
            myrenderer.display_box(wood, b->x, b->y, b->z, b->size, b->yangle);
        
        myrenderer.display_terrain(dirt, terrain, sizeof(terrain), terrainindexes, sizeof(terrainindexes), 0, 0, 0, 1);
        
        myrenderer.display_cubemap(sky);
        
        myrenderer.cycle_end();
        
        double render_end = glfwGetTime();
        
        time_spent_rendering = (render_end-render_start);
        
    }
    glfwDestroyWindow(win);
    
    return 0;
}
