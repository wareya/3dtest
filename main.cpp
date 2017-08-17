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

#include <GL/gl3w.h>
#include <GLFW/glfw3.h>

#define STB_IMAGE_IMPLEMENTATION
#include "include/stb_image_wrapper.h"

#include <stdio.h>
#include <math.h>
#ifndef M_PI
#define M_PI 3.1415926435
#endif

#include <signal.h>

#define deg2rad(X) (X/180.0f*M_PI)
#define rad2deg(X) (X*180.0f/M_PI)

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
};

coord sub(coord a, coord b)
{
    coord r;
    r.x = a.x-b.x;
    r.y = a.y-b.y;
    r.z = a.z-b.z;
    return r;
}

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

coord normalize(coord a)
{
    double magnitude = sqrt(dot(a, a));
    return mult(a, 1/magnitude);
}

coord project(coord a, coord b)
{
    return mult(b, dot(a, b)/dot(b, b));
}

coord reject(coord a, coord b)
{
    if(dot(a, a) == 0) return coord();
    return sub(a, project(a, b));
}

coord cross(coord a, coord b)
{
    coord r;
    r.x = a.y*b.z-a.z*b.y;
    r.y = a.z*b.x-a.x*b.z;
    r.z = a.x*b.y-a.y*b.x;
    return r;
}

struct collision {
    coord a, b, c, normal;
    collision()
    {
        a = coord();
        b = coord();
        c = coord();
        normal = coord();
    }
    collision(coord i, coord j, coord k, bool invert = false)
    {
        if(!invert)
        {
            a = i;
            b = j;
            c = k;
        }
        else
        {
            c = i;
            b = j;
            a = k;
        }
        normal = normalize(cross(sub(b, a), sub(c, a)));
    }
    bool operator==(collision o)
    {
        return a == o.a && b == o.b && c == o.c && normal == o.normal;
    }
    bool operator!=(collision o)
    {
        return !(*this == o);
    }
};

collision zero_collision = collision();

double triangle_intersection(coord o, coord d, collision c)
{
    coord e1 = sub(c.b, c.a);
    coord e2 = sub(c.c, c.a);
    // Calculate planes normal vector
    coord pvec = cross(d, e2);
    double det = dot(e1, pvec);
    
    if(det == 0)
        return 0;

    coord tvec = sub(o, c.a);
    double u = dot(tvec, pvec) / det;
    if (u < 0 || u > 1)
        return 0;

    coord qvec = cross(tvec, e1);
    double v = dot(d, qvec) / det;
    if (v < 0 || u + v > 1)
        return 0;
    
    return dot(e2, qvec) / det;
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

double rotation_x = 0;
double rotation_y = 0;
double rotation_z = 0;

double units_per_meter = 64;

double x = 0;
double y = -256;
double z = 0;

double height = 1.75*units_per_meter;

double gravity = 9.8*units_per_meter; // units per second per second
double friction = 7*units_per_meter; // units per second per second
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

bool postprocessing = true;

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
        win = glfwCreateWindow(1280, 720, "Hello, World!", NULL, NULL);
        
        if(!win) puts("glfw failed to init"), exit(0);
        glfwMakeContextCurrent(win);
        
        if(gl3wInit()) puts("gl3w failed to init"), exit(0);
        
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
        void main()\n\
        {\n\
            vec4 color = texture2D(mytexture, myTexCoord, 0);\n\
            float dot = dot(normalize(myNormal), normalize(vec3(-1.0,-1.0,-1.0)));\n\
            dot = max(0, dot);\n\
            color.rgb = pow(color.rgb, vec3(gamma));\n\
            vec3 diffuse = color.rgb * dot;\n\
            vec3 ambient = color.rgb * 0.1;\n\
            gl_FragColor = vec4(pow(diffuse+ambient, vec3(1/gamma)), 1);\n\
            if(boost == 1) gl_FragColor.rgb *= 4;\n\
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
        void main()\n\
        {\n\
            if(true)//cylindrical != 1)\n\
                gl_FragColor = texture(skybox, myTexCoord);\n\
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
                gl_FragColor = texture(skybox, coord);\n\
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
                    gl_FragColor = supersamplegrid();\n\
                }\n\
                else\n\
                {\n\
                    vec2 phase = interpolationPhase();\n\
                    vec4 c = hermitegrid(phase.x, phase.y);\n\
                    gl_FragColor = c;\n\
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
            void main()\n\
            {\n\
                gl_FragColor = texture2D(mytexture, myTexCoord);\n\
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
                gl_FragColor = texture2D(mytexture, myTexCoord)*(1+amount) - blurred*amount;\n\
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
                gl_FragColor = texture2D(mytexture, newcoord);\n\
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
            void main()\n\
            {\n\
                vec4 c1 = texture2D(mytexture, myTexCoord);\n\
                vec4 c2 = texture2D(mytexture, myTexCoord*0.9+0.05);\n\
                gl_FragColor = c1*0.8+c2*0.2;\n\
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
                gl_FragColor = pow(color, vec4(1/gamma));\n\
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
                gl_FragColor = pow(color, vec4(1/gamma));\n\
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
            void main()\n\
            {\n\
                vec4 color1 = pow(texture2D(drybuffer, myTexCoord), vec4(gamma));\n\
                vec4 color2 = pow(texture2D(wetbuffer, myTexCoord), vec4(gamma));\n\
                vec4 color = pow(color1*ratio + color2*(1-ratio), vec4(1/gamma));\n\
                gl_FragColor = color;\n\
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
            terrain[i].z = -(y-terrainsize/2+0.5)*terrainscale;
            
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

struct projectile {
    double x, y, z, xspeed, yspeed, zspeed, life;
    collision contact_triangle = zero_collision;
    std::vector<collision> touching;
    bool collided = false;
};

struct box {
    double x, y, z, size;
};

std::vector<projectile *> shots;
std::vector<box *> boxes;

void add_box(double x, double y, double z, double size)
{
    boxes.push_back(new box({x, y, z, size}));
    
    double s = size/2;
    
    // top
    worldtriangles.push_back(collision(coord(x-s, y-s, z-s), coord(x+s, y-s, z-s), coord(x-s, y-s, z+s)));
    worldtriangles.push_back(collision(coord(x+s, y-s, z-s), coord(x-s, y-s, z+s), coord(x+s, y-s, z+s), true));
    // bottom
    worldtriangles.push_back(collision(coord(x+s, y+s, z-s), coord(x-s, y+s, z-s), coord(x+s, y+s, z+s)));
    worldtriangles.push_back(collision(coord(x-s, y+s, z-s), coord(x+s, y+s, z+s), coord(x-s, y+s, z+s), true));
    // left
    worldtriangles.push_back(collision(coord(x-s, y+s, z-s), coord(x-s, y-s, z-s), coord(x-s, y+s, z+s)));
    worldtriangles.push_back(collision(coord(x-s, y-s, z-s), coord(x-s, y+s, z+s), coord(x-s, y-s, z+s), true));
    // right
    worldtriangles.push_back(collision(coord(x+s, y-s, z-s), coord(x+s, y+s, z-s), coord(x+s, y-s, z+s)));
    worldtriangles.push_back(collision(coord(x+s, y+s, z-s), coord(x+s, y-s, z+s), coord(x+s, y+s, z+s), true));
    // out-from-screen
    worldtriangles.push_back(collision(coord(x+s, y-s, z-s), coord(x-s, y-s, z-s), coord(x+s, y+s, z-s)));
    worldtriangles.push_back(collision(coord(x-s, y-s, z-s), coord(x+s, y+s, z-s), coord(x-s, y+s, z-s), true));
    // into-screen
    worldtriangles.push_back(collision(coord(x-s, y-s, z+s), coord(x+s, y-s, z+s), coord(x-s, y+s, z+s)));
    worldtriangles.push_back(collision(coord(x+s, y-s, z+s), coord(x-s, y+s, z+s), coord(x+s, y+s, z+s), true));
    
    
//     // top
//     {-s,-s,-s, 0.0f, 0.0f, 0.0f,-1.0f, 0.0f},
//     { s,-s,-s, 1.0f, 0.0f, 0.0f,-1.0f, 0.0f},
//     {-s,-s, s, 0.0f, 1.0f, 0.0f,-1.0f, 0.0f},
//     { s,-s, s, 1.0f, 1.0f, 0.0f,-1.0f, 0.0f},
//     // bottom
//     { s, s,-s, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f},
//     {-s, s,-s, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f},
//     { s, s, s, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f},
//     {-s, s, s, 1.0f, 1.0f, 0.0f, 1.0f, 0.0f},
//     // left
//     {-s, s,-s, 0.0f, 1.0f,-1.0f, 0.0f, 0.0f},
//     {-s,-s,-s, 0.0f, 0.0f,-1.0f, 0.0f, 0.0f},
//     {-s, s, s, 1.0f, 1.0f,-1.0f, 0.0f, 0.0f},
//     {-s,-s, s, 1.0f, 0.0f,-1.0f, 0.0f, 0.0f},
//     // right
//     { s,-s,-s, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f},
//     { s, s,-s, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f},
//     { s,-s, s, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f},
//     { s, s, s, 1.0f, 1.0f, 1.0f, 0.0f, 0.0f},
//     // front or back
//     { s,-s,-s, 0.0f, 0.0f, 0.0f, 0.0f,-1.0f},
//     {-s,-s,-s, 1.0f, 0.0f, 0.0f, 0.0f,-1.0f},
//     { s, s,-s, 0.0f, 1.0f, 0.0f, 0.0f,-1.0f},
//     {-s, s,-s, 1.0f, 1.0f, 0.0f, 0.0f,-1.0f},
//     // opposite
//     {-s,-s, s, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f},
//     { s,-s, s, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f},
//     {-s, s, s, 1.0f, 1.0f, 0.0f, 0.0f, 1.0f},
//     { s, s, s, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f},
}

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
    
    add_box(0, 128, 0, units_per_meter);
    add_box(32, -96-64-128, 256-32, 128);
    add_box(0, -96-64-128, 256-32+128, 128);
    add_box(64, -96, 256, 256);
    add_box(1040, -890, 0, 256);
    
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
            delta = starttime-oldtime;
            //delta = throttle;
            oldtime = starttime;
        }
        
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
        
        
        double walkspeed = 8*units_per_meter;//4*units_per_meter;
        if(glfwGetKey(win, GLFW_KEY_E))
        {
            z += walkspeed*delta*cos(deg2rad(rotation_y))*cos(deg2rad(rotation_x));
            x += walkspeed*delta*sin(deg2rad(rotation_y))*cos(deg2rad(rotation_x));
            y += walkspeed*delta*sin(deg2rad(rotation_x));
        }
        if(glfwGetKey(win, GLFW_KEY_D))
        {
            z -= walkspeed*delta*cos(deg2rad(rotation_y))*cos(deg2rad(rotation_x));
            x -= walkspeed*delta*sin(deg2rad(rotation_y))*cos(deg2rad(rotation_x));
            y -= walkspeed*delta*sin(deg2rad(rotation_x));
        }
        
        if(glfwGetKey(win, GLFW_KEY_W))
        {
            z += walkspeed*delta*sin(deg2rad(rotation_y));
            x -= walkspeed*delta*cos(deg2rad(rotation_y));
        }
        if(glfwGetKey(win, GLFW_KEY_F))
        {
            z -= walkspeed*delta*sin(deg2rad(rotation_y));
            x += walkspeed*delta*cos(deg2rad(rotation_y));
        }
        
        static bool right_waspressed = false;
        if(glfwGetMouseButton(win, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS)
        {
            if(!right_waspressed)
            {
                right_waspressed = true;
                puts("making box");
                
                auto newshot = new projectile;
                newshot->x = x;
                newshot->y = y+8;
                newshot->z = z;
                double shotspeed = 8*units_per_meter;
                newshot->zspeed = shotspeed*cos(deg2rad(rotation_y))*cos(deg2rad(rotation_x));
                newshot->xspeed = shotspeed*sin(deg2rad(rotation_y))*cos(deg2rad(rotation_x));
                newshot->yspeed = shotspeed*sin(deg2rad(rotation_x));
                newshot->life = 3;
                
                shots.push_back(newshot);
            }
        }
        else
            right_waspressed = false;
        
        for(unsigned int i = 0; i < shots.size(); i++)
        {
            constexpr double safety = 0.001;
            //constexpr double safety_bounce = 0;
            constexpr double friction_gap = 1;
            auto s = shots[i];
            s->life -= delta;
            if(s->life <= 0)
            {
                delete s;
                shots.erase(shots.begin()+(i--));
            }
            else
            {
                s->yspeed += gravity*delta/2;
                
                collision & last_triangle = s->contact_triangle;
                
                std::vector<collision> & touching = s->touching;
                double time = delta;
                
                bool hit_anything_at_all = false;
                while(time > 0)
                {
                    bool reached = false;
                    
                    collision goodtriangle = zero_collision;
                    double gooddistance = 0;
                    
                    auto pos = coord({s->x, s->y, s->z});
                    auto motion = coord({s->xspeed, s->yspeed, s->zspeed});
                    auto speed = sqrt(dot(motion, motion))*time;
                    
                    std::vector<collision> ignore;
                    
                    // select closest collidable triangle
                    for(auto t : worldtriangles)
                    {
                        // skip if we already interacted with it this frame
                        bool docontinue = false;
                        for(auto other : ignore) if (t == other) docontinue = true;
                        if(docontinue) continue;
                    
                        // no collision from behind
                        if(-dot(normalize(motion), normalize(t.normal)) < 0) continue;
                        
                        auto d = triangle_intersection(pos, normalize(motion), t);
                        // collide with things even if they're slightly in front of us
                        if(d <= speed and d > 0 and speed != 0) // speed != 0 is actually implicit based on the first two
                        {
                            if(gooddistance == 0 || d < gooddistance)
                            {
                                goodtriangle = t;
                                gooddistance = d;
                            }
                        }
                    }
                    if(gooddistance > 0)
                    {
                        auto t = goodtriangle;
                        auto d = gooddistance;
                        
                        double airtime = d/speed*time;
                        
                        double newtime = time - airtime;
                        if(newtime >= time)
                        {
                            time = 0;
                            break;
                        }
                        time = newtime;
                        
                        bool already_touching = false;
                        for(unsigned int j = 0; j < touching.size(); j++)
                        {
                            if(touching[j] == t) already_touching = true;
                            // too similar to the one we're on now and not it, we don't care about it anymore
                            if(dot(touching[j].normal, t.normal) > 0 && t != touching[j])
                                touching.erase(touching.begin()+(j--));
                            // not headed in the same direction, don't care about it anymore
                            else if(-dot(normalize(motion), touching[j].normal) < 0)
                                touching.erase(touching.begin()+(j--));
                        }
                        if(!already_touching)
                        {
                            touching.push_back(t);
                        }
                        ignore.push_back(t);
                        
                        // FIXME: handle seams
                        if(last_triangle != zero_collision)
                        {
                            auto contact_direction = normalize(last_triangle.normal);
                            
                            if(triangle_intersection(coord({s->x, s->y, s->z}), contact_direction, last_triangle) < friction_gap*friction_gap)
                            {
                                double speed = sqrt(dot(motion, motion));
                                if(speed != 0)
                                {
                                    double normalforce = -dot(contact_direction, coord(0, 1, 0));
                                    if(normalforce < 0) normalforce = 0;
                                    double half_newspeed = speed-friction*airtime*normalforce;
                                    if(half_newspeed < 0) half_newspeed = 0;
                                    
                                    motion = mult(motion, half_newspeed/speed);
                                    
                                    s->xspeed = motion.x;
                                    s->yspeed = motion.y;
                                    s->zspeed = motion.z;
                                }
                            }
                        }
                        
                        s->x += airtime*s->xspeed;
                        s->y += airtime*s->yspeed;
                        s->z += airtime*s->zspeed;
                        
                        s->x += t.normal.x*safety;
                        s->y += t.normal.y*safety;
                        s->z += t.normal.z*safety;
                        
                        if(touching.size() == 1)
                        {
                            puts("simple rejection");
                            motion = reject(motion, t.normal);
                        }
                        else if(touching.size() == 2)
                        {
                            puts("seam");
                            auto seam = cross(touching[0].normal, touching[1].normal);
                            motion = project(motion, seam);
                        }
                        else if(touching.size() == 3)
                        {
                            puts("crevice");
                            motion = coord(); // zero-vector
                        }
                        
                        s->xspeed = motion.x;
                        s->yspeed = motion.y;
                        s->zspeed = motion.z;
                        
                        last_triangle = t;
                        reached = true;
                        hit_anything_at_all = true;
                    }
                    if(!hit_anything_at_all)
                    {
                        //puts("!hit anything at all");
                        last_triangle = zero_collision;
                    }
                    if(!reached)
                    {
                        if(time > 0)
                        {
                            // FIXME: handle seams
                            if(last_triangle != zero_collision)
                            {
                                auto contact_direction = normalize(last_triangle.normal);
                                auto motion = coord({s->xspeed, s->yspeed, s->zspeed});
                                
                                if(triangle_intersection(coord({s->x, s->y, s->z}), contact_direction, last_triangle) < friction_gap*friction_gap)
                                {
                                    double speed = sqrt(dot(motion, motion));
                                    if(speed != 0)
                                    {
                                        double normalforce = -dot(contact_direction, coord(0, 1, 0));
                                        if(normalforce < 0) normalforce = 0;
                                        double newspeed = speed-friction*time*normalforce;
                                        if(newspeed < 0) newspeed = 0;
                                        
                                        motion = mult(motion, newspeed/speed);
                                        
                                        s->xspeed = motion.x;
                                        s->yspeed = motion.y;
                                        s->zspeed = motion.z;
                                    }
                                }
                                else
                                    last_triangle = zero_collision;
                            }
                        }
                        break; // no collisions this iteration
                    }
                }
                
                if(time > 0)
                {
                    s->z += time*s->zspeed;
                    s->x += time*s->xspeed;
                    s->y += time*s->yspeed;
                }
                
                s->yspeed += gravity*delta/2;
                
                //printf("asdf %f\n", s->yspeed);
            }
        }
        
        myrenderer.cycle_start();
        
        double shotsize = 8;
        
        for(auto s : shots)
            myrenderer.draw_box(junk, s->x, s->y, s->z, shotsize);
        for(auto b : boxes)
            myrenderer.draw_box(wood, b->x, b->y, b->z, b->size);
        
        myrenderer.draw_terrain(dirt, terrain, sizeof(terrain), terrainindexes, sizeof(terrainindexes), 0, 0, 0, 1);
        
        myrenderer.draw_cubemap(sky);
        
        myrenderer.cycle_end();
    }
    glfwDestroyWindow(win);
    
    return 0;
}
