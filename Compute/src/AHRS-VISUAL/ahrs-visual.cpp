/*
Copyright (c) 2012, Broadcom Europe Ltd
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include <unistd.h>

#include "bcm_host.h"

#include "GLES2/gl2.h"
#include "EGL/egl.h"
#include "EGL/eglext.h"

#include "revision.h"

// TODO: 
// 1) Use texturing on surfaces
// 2) use glMultMatrix instead of manually calculating mvp
// 3) Determine a good viewport and centering of cube in the viewport
// 4) Connect in bluetooth application 
// 5) 3-axis cube rotation and translation

#define VERTEX_DATA_ATTRIBUTE_SIZE 3
#define VIEWPORT_SIZE_DIVISOR 1
#define CUBE_NUM_FACES 6
#define SCREEN_SIZE_DIVISOR 2

typedef struct
{
   uint32_t screen_width;
   uint32_t screen_height;
   uint32_t viewport_dimension;
// OpenGL|ES objects
   EGLDisplay display;
   EGLSurface surface;
   EGLContext context;

   GLuint verbose;
   GLuint vshader;
   GLuint fshader;
   GLuint program;
   GLuint program2;
   GLuint tex_fb;
   GLuint tex;
   GLuint buf[2];
// attribs
   GLuint unif_mvp, attr_position, attr_color;
   // rotation about each axis -- Roll(z), Pitch(x), Yaw(y) measured in radians
   float rotation[3];
} GRAPHICS_STATE_T;

static GRAPHICS_STATE_T state;

static GLfloat mvp[4][4] = {
	{ 1.0, 0.0, 0.0, 0.0 },
	{ 0.0, 1.0, 0.0, 0.0 },
        { 0.0, 0.0, 1.0, 0.0 },
        { 0.0, 0.0, 0.0, 1.0 }   // x y z 1.0 (transpose)
};

static GLfloat xrot[4][4] = {
	{ 1.0, 0.0, 0.0, 0.0 },
	{ 0.0, 1.0, 0.0, 0.0 },
        { 0.0, 0.0, 1.0, 0.0 },
        { 0.0, 0.0, 0.0, 1.0 }   // x y z 1.0 (transpose)
};

static GLfloat yrot[4][4] = {
	{ 1.0, 0.0, 0.0, 0.0 },
	{ 0.0, 1.0, 0.0, 0.0 },
        { 0.0, 0.0, 1.0, 0.0 },
        { 0.0, 0.0, 0.0, 1.0 }   // x y z 1.0 (transpose)
};

static GLfloat zrot[4][4] = {
	{ 1.0, 0.0, 0.0, 0.0 },
	{ 0.0, 1.0, 0.0, 0.0 },
        { 0.0, 0.0, 1.0, 0.0 },
        { 0.0, 0.0, 0.0, 1.0 }   // x y z 1.0 (transpose)
};

static GLfloat tmp_mat[4][4] = {
	{ 0.0, 0.0, 0.0, 0.0 },
	{ 0.0, 0.0, 0.0, 0.0 },
        { 0.0, 0.0, 0.0, 0.0 },
        { 0.0, 0.0, 0.0, 0.0 } 
};


#define check() assert(glGetError() == 0)

static void showlog(GLint shader)
{
   // Prints the compile log for a shader
   char log[1024];
   glGetShaderInfoLog(shader,sizeof log,NULL,log);
   printf("%d:shader:\n%s\n", shader, log);
}

static void showprogramlog(GLint shader)
{
   // Prints the information log for a program object
   char log[1024];
   glGetProgramInfoLog(shader,sizeof log,NULL,log);
   printf("%d:program:\n%s\n", shader, log);
}
    
/***********************************************************
 * Name: init_ogl
 *
 * Arguments:
 *       GRAPHICS_STATE_T *state - holds OGLES model info
 *
 * Description: Sets the display, OpenGL|ES context and screen stuff
 *
 * Returns: void
 *
 ***********************************************************/
static void init_ogl(GRAPHICS_STATE_T *state)
{
   int32_t success = 0;
   EGLBoolean result;
   EGLint num_config;

   static EGL_DISPMANX_WINDOW_T nativewindow;

   DISPMANX_ELEMENT_HANDLE_T dispman_element;
   DISPMANX_DISPLAY_HANDLE_T dispman_display;
   DISPMANX_UPDATE_HANDLE_T dispman_update;
   VC_RECT_T dst_rect;
   VC_RECT_T src_rect;

   static const EGLint attribute_list[] =
   {
      EGL_RED_SIZE, 8,
      EGL_GREEN_SIZE, 8,
      EGL_BLUE_SIZE, 8,
      EGL_ALPHA_SIZE, 8,
      EGL_SURFACE_TYPE, EGL_WINDOW_BIT,
      EGL_NONE
   };
   
   static const EGLint context_attributes[] = 
   {
      EGL_CONTEXT_CLIENT_VERSION, 2,
      EGL_NONE
   };
   EGLConfig config;

   // get an EGL display connection
   state->display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
   assert(state->display!=EGL_NO_DISPLAY);
   check();

   // initialize the EGL display connection
   result = eglInitialize(state->display, NULL, NULL);
   assert(EGL_FALSE != result);
   check();

   // get an appropriate EGL frame buffer configuration
   result = eglChooseConfig(state->display, attribute_list, &config, 1, &num_config);
   assert(EGL_FALSE != result);
   check();

   // get an appropriate EGL frame buffer configuration
   result = eglBindAPI(EGL_OPENGL_ES_API);
   assert(EGL_FALSE != result);
   check();

   // create an EGL rendering context
   state->context = eglCreateContext(state->display, config, EGL_NO_CONTEXT, context_attributes);
   assert(state->context!=EGL_NO_CONTEXT);
   check();

   // create an EGL window surface
   success = graphics_get_display_size(0 /* LCD */, &state->screen_width, &state->screen_height);
   assert( success >= 0 );

   state->screen_width /= SCREEN_SIZE_DIVISOR;
   state->screen_height /= SCREEN_SIZE_DIVISOR;

   if (state->screen_width > state->screen_height)
   {
       state->viewport_dimension = state->screen_height;
   } else
   {
       state->viewport_dimension = state->screen_width;
   }

   if (SCREEN_SIZE_DIVISOR == 1)
   {
       dst_rect.x = 0;
       dst_rect.y = 0;
   } else {
       dst_rect.x = state->screen_width / SCREEN_SIZE_DIVISOR;
       dst_rect.y = state->screen_height /SCREEN_SIZE_DIVISOR;
   }
   dst_rect.width = state->screen_width;
   dst_rect.height = state->screen_height;
      
   src_rect.x = 0;
   src_rect.y = 0;
   src_rect.width = state->screen_width << 16;
   src_rect.height = state->screen_height << 16;        

   dispman_display = vc_dispmanx_display_open( 0 /* LCD */);
   dispman_update = vc_dispmanx_update_start( 0 );
         
   dispman_element = vc_dispmanx_element_add ( dispman_update, dispman_display,
      0/*layer*/, &dst_rect, 0/*src*/,
      &src_rect, DISPMANX_PROTECTION_NONE, 0 /*alpha*/, 0/*clamp*/, DISPMANX_NO_ROTATE/*transform*/);
      
   nativewindow.element = dispman_element;
   nativewindow.width = state->screen_width;
   nativewindow.height = state->screen_height;
   vc_dispmanx_update_submit_sync( dispman_update );
      
   check();

   state->surface = eglCreateWindowSurface( state->display, config, &nativewindow, NULL );
   assert(state->surface != EGL_NO_SURFACE);
   check();

   // connect the context to the surface
   result = eglMakeCurrent(state->display, state->surface, state->surface, state->context);
   assert(EGL_FALSE != result);
   check();

   // Set background color and clear buffers
   glClearColor ( 1.0, 1.0, 1.0, 1.0 );
   glClear( GL_COLOR_BUFFER_BIT );

   check();
}

static void init_shaders(GRAPHICS_STATE_T *state)
{

// CCW -- front facing
// CW -- rear facing
   static const GLfloat vertex_data[] = {
        -0.5, -0.5,  0.5,    // front side lower left
         0.5, -0.5,  0.5,    // front side lower right
         0.5,  0.5,  0.5,    // front side upper right
        -0.5,  0.5,  0.5,    // front side upper left

        -0.5, -0.5, -0.5,    // rear side 
        -0.5,  0.5, -0.5,    // rear side 
         0.5,  0.5, -0.5,    // rear side 
         0.5, -0.5, -0.5,    // rear side 

        -0.5, -0.5,  0.5,    // left side 
        -0.5,  0.5,  0.5,    // left side 
        -0.5,  0.5, -0.5,    // left side 
        -0.5, -0.5, -0.5,    // left side 

         0.5, -0.5,  0.5,    // right side
         0.5, -0.5, -0.5,    // right side
         0.5,  0.5, -0.5,    // right side
         0.5,  0.5,  0.5,    // right side

        -0.5,  0.5,  0.5,    // top side
         0.5,  0.5,  0.5,    // top side
         0.5,  0.5, -0.5,    // top side
        -0.5,  0.5, -0.5,    // top side

        -0.5, -0.5,  0.5,    // bottom side
        -0.5, -0.5, -0.5,    // bottom side
         0.5, -0.5, -0.5,    // bottom side
         0.5, -0.5,  0.5     // bottom side

   };

   static const GLfloat color_data[] = {
         0.5,  0.5,  0.1,
         0.5,  0.5,  0.1,
         0.5,  0.5,  0.1,
         0.5,  0.5,  0.1,

         0.5,  0.1,  0.8,
         0.5,  0.1,  0.8,
         0.5,  0.1,  0.8,
         0.5,  0.1,  0.8,

         0.1,  0.5,  0.8,
         0.1,  0.5,  0.8,
         0.1,  0.5,  0.8,
         0.1,  0.5,  0.8,

         0.1,  0.7,  0.8,
         0.1,  0.7,  0.8,
         0.1,  0.7,  0.8,
         0.1,  0.7,  0.8,

         0.7,  0.1,  0.8,
         0.7,  0.1,  0.8,
         0.7,  0.1,  0.8,
         0.7,  0.1,  0.8,

         0.5,  0.9,  0.1,
         0.5,  0.9,  0.1,
         0.5,  0.9,  0.1,
         0.5,  0.9,  0.1
   };

   const GLchar *vshader_source =
              "attribute vec4 a_position;"
              "attribute vec4 a_color;"
	      "uniform mat4 u_mvp_matrix;"
	      "varying vec4 g_color;"
              "void main(void) {"
              " gl_Position = u_mvp_matrix * a_position;"
	      " g_color = a_color;"
              "}";
     
   const GLchar *fshader_source =
           "varying vec4 g_color;"
           "void main(void) {"
           "  gl_FragColor = g_color;"
           "}";


        state->vshader = glCreateShader(GL_VERTEX_SHADER);
        glShaderSource(state->vshader, 1, &vshader_source, 0);
        glCompileShader(state->vshader);
        check();

	state->verbose=1;
        if (state->verbose)
            showlog(state->vshader);
            
        state->fshader = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(state->fshader, 1, &fshader_source, 0);
        glCompileShader(state->fshader);
        check();

        if (state->verbose)
            showlog(state->fshader);

        state->program = glCreateProgram();
        glAttachShader(state->program, state->vshader);
        glAttachShader(state->program, state->fshader);
        glLinkProgram(state->program);
        check();

        if (state->verbose)
            showprogramlog(state->program);
            
        state->attr_position = glGetAttribLocation(state->program, "a_position");
        state->attr_color  =   glGetAttribLocation(state->program, "a_color");
        state->unif_mvp    = glGetUniformLocation(state->program, "u_mvp_matrix");

        glClearColor ( 1.0, 1.0, 1.0, 1.0 );
        
        glGenBuffers(2, &state->buf[0]);
        check();

        // Prepare a framebuffer for rendering
        glGenFramebuffers(1,&state->tex_fb);
        check();
        glBindFramebuffer(GL_FRAMEBUFFER,state->tex_fb);
        check();
        glBindFramebuffer(GL_FRAMEBUFFER,0);
        check();
        // Prepare viewport
	printf("viewport dim %d screen w %d h %d\n", state->viewport_dimension, state->screen_width, state->screen_height);
        glViewport ( 0, 0, state->viewport_dimension, state->viewport_dimension );
        check();

        // Upload vertex data to a buffer
        glBindBuffer(GL_ARRAY_BUFFER, state->buf[0]);
        glBufferData(GL_ARRAY_BUFFER, sizeof(vertex_data), vertex_data, GL_STATIC_DRAW);
        glVertexAttribPointer(state->attr_position, VERTEX_DATA_ATTRIBUTE_SIZE, GL_FLOAT, 0, 0, 0);
        glEnableVertexAttribArray(state->attr_position);
        check();

	// Upoad color data to a buffer
        glBindBuffer(GL_ARRAY_BUFFER, state->buf[1]);
        glBufferData(GL_ARRAY_BUFFER, sizeof(color_data), color_data, GL_STATIC_DRAW);
        glVertexAttribPointer(state->attr_color, VERTEX_DATA_ATTRIBUTE_SIZE, GL_FLOAT, 0, 0, 0);
        glEnableVertexAttribArray(state->attr_color);
        check();

}

        
static void draw_cube(GRAPHICS_STATE_T *state)
{
        GLuint start_pos = 0;

        // Now render to the main frame buffer
        glBindFramebuffer(GL_FRAMEBUFFER,0);
        // Clear the background (not really necessary I suppose)
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
        check();
        
        glBindBuffer(GL_ARRAY_BUFFER, state->buf[0]);
        check();
        glUseProgram ( state->program );
        check();
        glBindTexture(GL_TEXTURE_2D,state->tex);
        check();

	// See: Computer Graphics Principles and Practice, Third Edition
	// John F. Hughes 2014
	// Section 11.2.1

	// rotation about Z axis (Roll)
	zrot[0][0] = zrot[1][1] = 1.0f; 
	zrot[0][1] = zrot[1][0] = 0.0f;
	zrot[0][0] = zrot[1][1] = (GLfloat)cos(state->rotation[0]);
	zrot[0][1] = (GLfloat)sin(state->rotation[0]);
	zrot[1][0] = -zrot[0][1];

	// rotation about X axis (Pitch)
	xrot[1][1] = xrot[2][2] = 1.0f; 
	xrot[1][2] = xrot[2][1] = 0.0f;
	xrot[1][1] = xrot[2][2] = (GLfloat)cos(state->rotation[1]);
	xrot[1][2] = (GLfloat)sin(state->rotation[1]);
	xrot[2][1] = -xrot[1][2];

	// rotation about Y axis (Yaw)
	yrot[0][0] = yrot[2][2] = 1.0f;
	yrot[2][0] = yrot[0][2] = 0.0f;
	yrot[0][0] = yrot[2][2] = (GLfloat)cos(state->rotation[2]);
	yrot[2][0] = (GLfloat)sin(state->rotation[2]);
	yrot[0][2] = -yrot[2][0];


	// mvp = X * Y * Z
	// tmp = X * Y
	// mvp = tmp * Z
	for (int row = 0 ; row < 4 ; row++)   // row
	{
	    for (int col = 0 ; col < 4 ; col++)   // col
	    {
                tmp_mat[row][col] = 0.0;
	        for (int k = 0 ; k < 4 ; k++)
		{
		    tmp_mat[row][col] += xrot[row][k] * yrot[k][col]; 
		}
            }
	}

	for (int row = 0 ; row < 4 ; row++)   // row
	{
	    for (int col = 0 ; col < 4 ; col++)   // col
	    {
                mvp[row][col] = 0.0;
            }
        }

	for (int row = 0 ; row < 4 ; row++)   // row
	{
	    for (int col = 0 ; col < 4 ; col++)   // col
	    {
	        for (int k = 0 ; k < 4 ; k++)
		{
		    mvp[row][col] += tmp_mat[row][k] * zrot[k][col]; 
		}
            }
	}


        glUniformMatrix4fv(state->unif_mvp, 16, GL_FALSE, &mvp[0][0]); 
        check();
	glFrontFace(GL_CCW);
	glCullFace(GL_BACK);
	glEnable(GL_CULL_FACE);
        check();

	for (unsigned int i = 0 ; i < CUBE_NUM_FACES ; i++)
	{
            glDrawArrays ( GL_TRIANGLE_FAN, start_pos, 4); 
	    start_pos += 4;
	}
        check();

        glFlush();
        glFinish();
        check();
        
        eglSwapBuffers(state->display, state->surface);
        check();
}

static int16_t swap_bytes(uint32_t data)
{
    uint32_t mask1 = 0x000000FF;
    uint32_t mask2 = 0x0000FF00;

    return ((data & mask1) << 8) | ((data & mask2) >> 8);
}

static int get_orientation(GRAPHICS_STATE_T *state)
{
    uint32_t xrot = 0, yrot = 0, zrot = 0;
    int rc = 0;
    static int pkt_num = 0;

    int num_fields = scanf("%x %x %x %d", &zrot, &xrot, &yrot, &pkt_num);
    if (num_fields == 4)
    {
        state->rotation[0] = swap_bytes(zrot) ; // Roll
        state->rotation[1] = swap_bytes(xrot) ; // Pitch
        state->rotation[2] = swap_bytes(yrot) ; // Yaw
	for (int i=0 ; i < 3 ; i++)
	{
            // Convert to Radians
	    state->rotation[i] = ( state->rotation[i] * M_PI ) / 180.0;
	}

#define PRINT_DEBUG_INTERVAL 0xF
	if (pkt_num & PRINT_DEBUG_INTERVAL)
	{
	    printf("Roll %.1f Pitch %.1f Yaw %.1f  Pkt %d\n", state->rotation[0] * (180.0/M_PI), state->rotation[1]* (180.0/M_PI), state->rotation[2]* (180.0/M_PI), pkt_num);
	}

    } else {
        rc = 1;
    }
    return rc;
}       
 
 
//==============================================================================

int main ()
{
   bcm_host_init();

   if (bcm_host_get_processor_id() == PROCESSOR_BCM2838)
   {
      puts("This demo application is not available on the Pi4\n\n");
      exit(0);
   }

   // Clear application state
   memset( &state, 0, sizeof( state ) );
      
   // Start OGLES
   init_ogl(&state);
   init_shaders(&state);

   while (1) 
   {
      if (get_orientation(&state)) break;
      draw_cube(&state);
   } 
   return 0;
}

