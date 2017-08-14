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

#define deg2rad(X) (X/180.0f*M_PI)
#define rad2deg(X) (X*180.0f/M_PI)

#include <chrono>
#include <thread>

struct vertex {
    float x, y, z, u, v, nx, ny, nz;
};
struct basicvertex {
    float x, y, z;
};

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

float rotation_x = 0;
float rotation_y = 0;
float rotation_z = 0;

float x = 0;
float y = -256;
float z = 256;

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


int msaa = 8;
float viewPortRes = 2.0;

struct renderer {
    // TODO: FIXME: add a real reference counter
    struct texture {
        int w, h, n;
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
    
    unsigned int MainVAO, VIO, VBO, FRBO, RBOC, RBOD, FBO, FBOtexture0, FBOtexture1, FBOtexture2, CubeVAO, CubeVBO, CubeVIO;
    int w, h;
    unsigned int vshader;
    unsigned int fshader;
    unsigned int program;
    
    unsigned int cubevshader;
    unsigned int cubefshader;
    unsigned int cubeprogram;
    
    GLFWwindow * win;
    postprogram * copy, * ssam, * meme;
    
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
    renderer()
    {
        glfwSwapInterval(0);
        
        if(!glfwInit()) puts("glfw failed to init"), exit(0);
        
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        win = glfwCreateWindow(800, 600, "Hello, World!", NULL, NULL);
        
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
        //glFrontFace(GL_CCW);
        
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
        varying vec2 myTexCoord;\n\
        varying vec3 myNormal;\n\
        void main()\n\
        {\n\
            vec4 color = texture2D(mytexture, myTexCoord, 0);\n\
            float dot = dot(normalize(myNormal), normalize(vec3(-1.0,-1.0,-1.0)));\n\
            dot = max(0, dot);\n\
            vec3 diffuse = color.rgb * dot;\n\
            vec3 ambient = color.rgb * 0.25;\n\
            gl_FragColor = vec4(diffuse+ambient, 1);\n\
        }\n");
        
        glUseProgram(program);
        glUniform1i(glGetUniformLocation(program, "mytexture"), 0);
        
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
        varying vec3 myTexCoord;\n\
        void main()\n\
        {\n\
            gl_FragColor = texture(skybox, myTexCoord);\n\
        }\n");
        
        glUseProgram(cubeprogram);
        glUniform1i(glGetUniformLocation(cubeprogram, "skybox"), 0);
        
        checkerr(__LINE__);
        
        checkerr(__LINE__);
        
        // FBO programs
        
        glGenFramebuffers(1, &FRBO); 
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, FRBO);
        
        glGenRenderbuffers(1, &RBOC); 
        glGenRenderbuffers(1, &RBOD); 
        glBindRenderbuffer(GL_RENDERBUFFER, RBOC);
        glRenderbufferStorageMultisample(GL_RENDERBUFFER, msaa, GL_RGBA16F, w*viewPortRes, h*viewPortRes);
        glBindRenderbuffer(GL_RENDERBUFFER, RBOD);
        glRenderbufferStorageMultisample(GL_RENDERBUFFER, msaa, GL_DEPTH_COMPONENT32F, w*viewPortRes, h*viewPortRes);
        
        // make framebuffer
        
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
        vec2 lodRoundedPixel(int a, int b, vec2 size)\n\
        {\n\
            return vec2((floor(coord.x*(size.x)-0.5)+a+0.5)/(size.x),\n\
                        (floor(coord.y*(size.y)-0.5)+b+0.5)/(size.y));\n\
        }\n\
        vec4 lodRoundedPixel(int a, int b, vec2 size, int lod)\n\
        {\n\
            return textureLod(mytexture, lodRoundedPixel(a, b, size), lod);\n\
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
        glUniform1f(glGetUniformLocation(ssam->program, "myScale"), 1/viewPortRes);
        
        unsigned int jinctexid;
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
        
        glActiveTexture(GL_TEXTURE0);
        
        glGenTextures(1, &FBOtexture0);
        glGenTextures(1, &FBOtexture1);
        glGenTextures(1, &FBOtexture2);
        
        glBindTexture(GL_TEXTURE_2D, FBOtexture0);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, w*viewPortRes, h*viewPortRes, 0, GL_RGB, GL_FLOAT, NULL);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, FBOtexture0, 0);
        checkerr(__LINE__);
        
        glBindTexture(GL_TEXTURE_2D, FBOtexture1);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, w, h, 0, GL_RGB, GL_FLOAT, NULL);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1,  GL_TEXTURE_2D, FBOtexture1, 0);
        checkerr(__LINE__);
        
        glBindTexture(GL_TEXTURE_2D, FBOtexture2);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, w, h, 0, GL_RGB, GL_FLOAT, NULL);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT2, GL_TEXTURE_2D, FBOtexture2, 0);
        checkerr(__LINE__);
        
        checkerr(__LINE__);
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
            glRenderbufferStorageMultisample(GL_RENDERBUFFER, msaa, GL_RGBA8, w*viewPortRes, h*viewPortRes);
            glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, RBOC);
            
            glBindRenderbuffer(GL_RENDERBUFFER, RBOD);
            glRenderbufferStorageMultisample(GL_RENDERBUFFER, msaa, GL_DEPTH_COMPONENT32F, w*viewPortRes, h*viewPortRes);
            glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, RBOD);
            
            glActiveTexture(GL_TEXTURE0);
            glBindFramebuffer(GL_DRAW_FRAMEBUFFER, FBO);
            
            glBindTexture(GL_TEXTURE_2D, FBOtexture0);
            checkerr(__LINE__);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, w*viewPortRes, h*viewPortRes, 0, GL_RGB, GL_FLOAT, NULL);
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
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, w, h, 0, GL_RGB, GL_FLOAT, NULL);
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
        }
        
        glViewport(0, 0, w*viewPortRes, h*viewPortRes);
        
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
        
        float u_aspect = float(w)/float(h);
        float a_aspect = atan(1/u_aspect);
        float a_fovd = deg2rad(140.0f); // diagonal
        
        float u_d = tan(a_fovd/2); // distance to camera is our unit of distance, i.e. 1
        float u_x = u_d*cos(a_aspect);
        float u_y = u_d*sin(a_aspect);
        float a_fovx = atan(u_x)*2;
        float a_fovy = atan(u_y)*2;
        float fx = 1/tan((a_fovx/2));
        float fy = 1/tan((a_fovy/2));
        float nearf = 1;
        float farf = 10000;
        float projection[16] = {
              fx, 0.0f, 0.0f, 0.0f,
            0.0f,  -fy, 0.0f, 0.0f,
            0.0f, 0.0f, (nearf+farf)/(nearf-farf), 2*nearf*farf/(nearf-farf),
            0.0f, 0.0f,-1.0f, 0.0f
        };
        
        float translation[16] = {
            1.0f, 0.0f, 0.0f,   -x,
            0.0f, 1.0f, 0.0f,   -y,
            0.0f, 0.0f, 1.0f,   -z,
            0.0f, 0.0f, 0.0f, 1.0f
        };
        
        float r_x = rotation_x/180.0*M_PI;
        float r_y = rotation_y/180.0*M_PI;
        float r_z = rotation_z/180.0*M_PI;
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
                cos(r_z),-sin(r_z), 0.0f, 0.0f,
                sin(r_z), cos(r_z), 0.0f, 0.0f,
                0.0f,     0.0f,     1.0f, 0.0f,
                0.0f,     0.0f,     0.0f, 1.0f
        };
        
        m4mult(rotation_x, rotation_y);
        m4mult(rotation_z, rotation_x);
        m4mult(projection, rotation_z);
        
        glUseProgram(cubeprogram);
        glUniformMatrix4fv(glGetUniformLocation(cubeprogram, "projection"), 1, 0, projection);
        
        m4mult(projection, translation);
        
        glUseProgram(program);
        glUniformMatrix4fv(glGetUniformLocation(program, "projection"), 1, 0, projection);
        
        
        checkerr(__LINE__);
    }
    void cycle_end()
    {
        glViewport(0, 0, w*viewPortRes, h*viewPortRes);
        
        glUseProgram(program);
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
        glBlitFramebuffer(0,0,w*viewPortRes,h*viewPortRes,0,0,w*viewPortRes,h*viewPortRes, GL_COLOR_BUFFER_BIT, GL_NEAREST);
        checkerr(__LINE__);
        
        glViewport(0, 0, w, h);
        
        
        int currtex = 0;
        
        int last_draw_buffer = GL_COLOR_ATTACHMENT0;
        
        auto BUFFER_A = [&]()
        {
            glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
            glBindFramebuffer(GL_DRAW_FRAMEBUFFER, FBO);
            glDrawBuffer(GL_COLOR_ATTACHMENT1);
            last_draw_buffer = GL_COLOR_ATTACHMENT1;
        };
        auto BUFFER_B = [&]()
        {
            glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
            glBindFramebuffer(GL_DRAW_FRAMEBUFFER, FBO);
            glDrawBuffer(GL_COLOR_ATTACHMENT2);
            last_draw_buffer = GL_COLOR_ATTACHMENT2;
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
            if(currtex == 0)
            {
                BUFFER_A();
                currtex = 2;
                glBindTexture(GL_TEXTURE_2D, FBOtexture0);
            }
            else if(currtex == 2)
            {
                BUFFER_B();
                currtex = 1;
                glBindTexture(GL_TEXTURE_2D, FBOtexture1);
            }
            if(currtex == 1)
            {
                BUFFER_A();
                currtex = 2;
                glBindTexture(GL_TEXTURE_2D, FBOtexture2);
            }
        };
        checkerr(__LINE__);
        
        FLIP_SOURCE();
        glUseProgram(ssam->program);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
        checkerr(__LINE__);
        
        BUFFER_DONE();
        glBlitFramebuffer(0,0,w,h,0,0,w,h, GL_COLOR_BUFFER_BIT, GL_NEAREST);
        
        checkerr(__LINE__);
        
        checkerr(__LINE__);
        glfwSwapBuffers(win);
        checkerr(__LINE__);
        glFinish();
        checkerr(__LINE__);
    }
    void draw_box(texture * texture, float x, float y, float z, float scale)
    {
        glUseProgram(program);
        glBindVertexArray(MainVAO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, VIO);
        
        float s = 128;
        const vertex vertices[4*6] = {
            // top
            { s,-s,-s, 0.0f, 0.0f, 0.0f,-1.0f, 0.0f},
            {-s,-s,-s, 1.0f, 0.0f, 0.0f,-1.0f, 0.0f},
            { s,-s, s, 0.0f, 1.0f, 0.0f,-1.0f, 0.0f},
            {-s,-s, s, 1.0f, 1.0f, 0.0f,-1.0f, 0.0f},
            // bottom
            {-s, s,-s, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f},
            { s, s,-s, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f},
            {-s, s, s, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f},
            { s, s, s, 1.0f, 1.0f, 0.0f, 1.0f, 0.0f},
            // left
            {-s,-s,-s, 0.0f, 0.0f,-1.0f, 0.0f, 0.0f},
            {-s, s,-s, 1.0f, 0.0f,-1.0f, 0.0f, 0.0f},
            {-s,-s, s, 0.0f, 1.0f,-1.0f, 0.0f, 0.0f},
            {-s, s, s, 1.0f, 1.0f,-1.0f, 0.0f, 0.0f},
            // right
            { s, s,-s, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f},
            { s,-s,-s, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f},
            { s, s, s, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f},
            { s,-s, s, 1.0f, 1.0f, 1.0f, 0.0f, 0.0f},
            // front or back
            {-s,-s,-s, 0.0f, 0.0f, 0.0f, 0.0f,-1.0f},
            { s,-s,-s, 1.0f, 0.0f, 0.0f, 0.0f,-1.0f},
            {-s, s,-s, 0.0f, 1.0f, 0.0f, 0.0f,-1.0f},
            { s, s,-s, 1.0f, 1.0f, 0.0f, 0.0f,-1.0f},
            // opposite
            { s,-s, s, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f},
            {-s,-s, s, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f},
            { s, s, s, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f},
            {-s, s, s, 1.0f, 1.0f, 0.0f, 0.0f, 1.0f},
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
            {-1, 1, 1},
            { 1, 1, 1},
            {-1,-1, 1},
            { 1,-1, 1},
            {-1,-1,-1},
            { 1,-1,-1},
            {-1, 1,-1},
            { 1, 1,-1},
        };
        // 65535
        const unsigned short indexes1[] = { 0, 1, 2, 3, 4, 5, 6, 7, 65535, 2, 4, 0, 6, 1, 7, 3, 5 };
        
        glBindTexture(GL_TEXTURE_CUBE_MAP, map->texid);
        
        glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices,  GL_DYNAMIC_DRAW);
        
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indexes1), indexes1, GL_DYNAMIC_DRAW);
        glDrawElements(GL_TRIANGLE_STRIP, sizeof(indexes1)/sizeof(indexes1[0]), GL_UNSIGNED_SHORT, 0);
        checkerr(__LINE__);
    }

    const static int terrainsize = 64;
    const static int terrainscale = 64;
    vertex terrain[terrainsize*terrainsize];

    void generate_terrain()
    {
        int w, h, n;
        unsigned char * data = stbi_load("heightmap.png", &w, &h, &n, 1);
        for(int y = 0; y < terrainsize; y++)
        {
            for(int x = 0; x < terrainsize; x++)
            {
                int i = y*terrainsize + x;
                terrain[i].x = (x-terrainsize/2+0.5)*terrainscale;
                terrain[i].z = (y-terrainsize/2+0.5)*terrainscale;
                
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
                float nx;
                float ny;
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
                float dx = sin(atan(nx/terrainscale));
                float dy = sin(atan(ny/terrainscale));
                float ts = dx*dx - dy*dy;
                float dz;
                if(ts < 1.0f)
                    dz = sqrt(1.0f - ts); 
                else
                    dz = 0;
                
                terrain[i].nx = dx;
                terrain[i].ny = -dz;
                terrain[i].nz = -dy;
                //printf("n: %f %f %f\n", dx, dz, dy);
            }
        }
    }
    void draw_terrain(texture * texture, float x, float y, float z, float scale)
    {
        glUseProgram(program);
        glBindVertexArray(MainVAO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, VIO);
        
        static bool init = false;
        if(!init)
        {
            generate_terrain();
            init = true;
        }
        short indexes[terrainsize*2];
        
        float translation[16] = {
            scale,  0.0f, 0.0f,   -x,
             0.0f, scale, 0.0f,   -y,
             0.0f,  0.0f, scale,  -z,
             0.0f,  0.0f, 0.0f, 1.0f
        };
        
        glUniformMatrix4fv(glGetUniformLocation(program, "translation"), 1, 0, translation);
        glBindTexture(GL_TEXTURE_2D, texture->texid);
        glBufferData(GL_ARRAY_BUFFER, sizeof(terrain), terrain,  GL_DYNAMIC_DRAW);
        
        for(int row = 0; row < terrainsize-1; row++)
        {
            for(int x = 0; x < terrainsize; x++) for(int y = 0; y < 2; y++) indexes[y+x*2] = x+(y+row)*terrainsize;
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indexes), indexes, GL_DYNAMIC_DRAW);
            glDrawElements(GL_TRIANGLE_STRIP, sizeof(indexes)/sizeof(indexes[0]), GL_UNSIGNED_SHORT, 0);
        }
        
        checkerr(__LINE__);
    }
};



vertex boxes[8];

int main (int argc, char ** argv)
{
    renderer myrenderer;
    
    auto & win = myrenderer.win;
    
    auto wood = myrenderer.load_texture("texture.png");
    if(!wood) return 0;
    auto dirt = myrenderer.load_texture("ground.png");
    if(!dirt) return 0;
    auto sky = myrenderer.load_cubemap("Daylight Box_", ".bmp");
    if(!sky) return 0;
    
    float oldtime = glfwGetTime();
    while(!glfwWindowShouldClose(win))
    {
        float newtime;
        float delta;
        
        glfwPollEvents();
        newtime = glfwGetTime();
        delta = newtime-oldtime;
        oldtime = newtime;
        
        static float sensitivity = 1/8.0f;
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
                
                rotation_y -= xd*sensitivity; // AROUND y, i.e. horizontal
                rotation_x -= yd*sensitivity;
                if(rotation_x >  90) rotation_x =  90;
                if(rotation_x < -90) rotation_x = -90;
            }
            last_xpos = xpos;
            last_ypos = ypos;
            continuation = true;
        }
        else
            continuation = false;
        
        
        float walkspeed = 400;
        if(glfwGetKey(win, GLFW_KEY_E))
        {
            z -= walkspeed*delta*cos(deg2rad(rotation_y))*cos(deg2rad(rotation_x));
            x -= walkspeed*delta*sin(deg2rad(rotation_y))*cos(deg2rad(rotation_x));
            y -= walkspeed*delta*sin(deg2rad(rotation_x));
        }
        if(glfwGetKey(win, GLFW_KEY_D))
        {
            z += walkspeed*delta*cos(deg2rad(rotation_y))*cos(deg2rad(rotation_x));
            x += walkspeed*delta*sin(deg2rad(rotation_y))*cos(deg2rad(rotation_x));
            y += walkspeed*delta*sin(deg2rad(rotation_x));
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
        
        myrenderer.cycle_start();
        
        myrenderer.draw_box(wood, 0, -256-96, -256, 1);
        myrenderer.draw_box(wood, 64, -96, -256+32, 1);
        myrenderer.draw_terrain(dirt, 0, 0, 0, 1);
        myrenderer.draw_cubemap(sky);
        
        myrenderer.cycle_end();
        
        constexpr float throttle = 0.004;
        if(delta < throttle)
            std::this_thread::sleep_for(std::chrono::duration<float>(throttle-delta));
    }
    glfwDestroyWindow(win);
    
    return 0;
}
