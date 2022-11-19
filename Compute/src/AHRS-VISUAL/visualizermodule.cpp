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

#define check() assert(glGetError() == 0)

#define PY_SSIZE_T_CLEAN
#include <Python.h>

/* 
 * 3-D gear wheels.  This program is in the public domain.
 *
 * Brian Paul
 *
 *
 * Modified to work under Togl as a widget for TK 1997
 *
 * Philip Quaife
 *
 */

//DSH4
#define USE_TOGL_STUBS
#define USE_TCL_STUBS
#define USE_TK_STUBS

#include "togl.h"
#include "toglpy.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

//DSH4
#define DEBUG_CODE



#undef TCL_STORAGE_CLASS
#define TCL_STORAGE_CLASS DLLEXPORT

#ifndef M_PI
#  define M_PI 3.14159265
#endif
#define FM_PI ((float) M_PI)

#ifdef _MSC_VER
__inline float
sinf(double a)
{
    return (float) sin(a);
}
__inline float
cosf(double a)
{
    return (float) cos(a);
}
__inline float
sqrtf(double a)
{
    return (float) sqrt(a);
}

#  define sin sinf
#  define cos cosf
#  define sqrt sqrtf
#endif

struct WHIRLYGIZMO
{
    int     Gear1, Gear2, Gear3;
    double  Rotx, Roty, Rotz;
    double  Angle;
    int     Height, Width;
};

typedef struct WHIRLYGIZMO WHIRLYGIZMO;

/* 
 * Draw a gear wheel.  You'll probably want to call this function when
 * building a display list since we do a lot of trig here.
 *
 * Input:  inner_radius - radius of hole at center
 *         outer_radius - radius at center of teeth
 *         width - width of gear
 *         teeth - number of teeth
 *         tooth_depth - depth of tooth
 */
static void
gear(GLfloat inner_radius, GLfloat outer_radius, GLfloat width,
        GLint teeth, GLfloat tooth_depth)
{
    GLint   i;
    GLfloat r0, r1, r2;
    GLfloat angle, da;
    GLfloat u, v, len;

#ifdef DEBUG_CODE
printf("GEAR start %s %d\n", __FILE__, __LINE__);
#endif

    r0 = inner_radius;
    r1 = outer_radius - tooth_depth / 2;
    r2 = outer_radius + tooth_depth / 2;

    da = 2 * FM_PI / teeth / 4;

    glShadeModel(GL_FLAT);

    glNormal3f(0, 0, 1);

    /* draw front face */
    glBegin(GL_QUAD_STRIP);
    for (i = 0; i <= teeth; i++) {
        angle = i * 2 * FM_PI / teeth;
        glVertex3f(r0 * cos(angle), r0 * sin(angle), width * 0.5f);
        glVertex3f(r1 * cos(angle), r1 * sin(angle), width * 0.5f);
        glVertex3f(r0 * cos(angle), r0 * sin(angle), width * 0.5f);
        glVertex3f(r1 * cos(angle + 3 * da), r1 * sin(angle + 3 * da),
                width * 0.5f);
    }
    glEnd();

    /* draw front sides of teeth */
    glBegin(GL_QUADS);
    da = 2 * FM_PI / teeth / 4;
    for (i = 0; i < teeth; i++) {
        angle = i * 2 * FM_PI / teeth;

        glVertex3f(r1 * cos(angle), r1 * sin(angle), width * 0.5f);
        glVertex3f(r2 * cos(angle + da), r2 * sin(angle + da), width * 0.5f);
        glVertex3f(r2 * cos(angle + 2 * da), r2 * sin(angle + 2 * da),
                width * 0.5f);
        glVertex3f(r1 * cos(angle + 3 * da), r1 * sin(angle + 3 * da),
                width * 0.5f);
    }
    glEnd();


    glNormal3f(0, 0, -1);

    /* draw back face */
    glBegin(GL_QUAD_STRIP);
    for (i = 0; i <= teeth; i++) {
        angle = i * 2 * FM_PI / teeth;
        glVertex3f(r1 * cos(angle), r1 * sin(angle), -width * 0.5f);
        glVertex3f(r0 * cos(angle), r0 * sin(angle), -width * 0.5f);
        glVertex3f(r1 * cos(angle + 3 * da), r1 * sin(angle + 3 * da),
                -width * 0.5f);
        glVertex3f(r0 * cos(angle), r0 * sin(angle), -width * 0.5f);
    }
    glEnd();

    /* draw back sides of teeth */
    glBegin(GL_QUADS);
    da = 2 * FM_PI / teeth / 4;
    for (i = 0; i < teeth; i++) {
        angle = i * 2 * FM_PI / teeth;

        glVertex3f(r1 * cos(angle + 3 * da), r1 * sin(angle + 3 * da),
                -width * 0.5f);
        glVertex3f(r2 * cos(angle + 2 * da), r2 * sin(angle + 2 * da),
                -width * 0.5f);
        glVertex3f(r2 * cos(angle + da), r2 * sin(angle + da), -width * 0.5f);
        glVertex3f(r1 * cos(angle), r1 * sin(angle), -width * 0.5f);
    }
    glEnd();


    /* draw outward faces of teeth */
    glBegin(GL_QUAD_STRIP);
    for (i = 0; i < teeth; i++) {
        angle = i * 2 * FM_PI / teeth;

        glVertex3f(r1 * cos(angle), r1 * sin(angle), width * 0.5f);
        glVertex3f(r1 * cos(angle), r1 * sin(angle), -width * 0.5f);
        u = r2 * cos(angle + da) - r1 * cos(angle);
        v = r2 * sin(angle + da) - r1 * sin(angle);
        len = sqrt(u * u + v * v);
        u /= len;
        v /= len;
        glNormal3f(v, -u, 0);
        glVertex3f(r2 * cos(angle + da), r2 * sin(angle + da), width * 0.5f);
        glVertex3f(r2 * cos(angle + da), r2 * sin(angle + da), -width * 0.5f);
        glNormal3f(cos(angle), sin(angle), 0);
        glVertex3f(r2 * cos(angle + 2 * da), r2 * sin(angle + 2 * da),
                width * 0.5f);
        glVertex3f(r2 * cos(angle + 2 * da), r2 * sin(angle + 2 * da),
                -width * 0.5f);
        u = r1 * cos(angle + 3 * da) - r2 * cos(angle + 2 * da);
        v = r1 * sin(angle + 3 * da) - r2 * sin(angle + 2 * da);
        glNormal3f(v, -u, 0);
        glVertex3f(r1 * cos(angle + 3 * da), r1 * sin(angle + 3 * da),
                width * 0.5f);
        glVertex3f(r1 * cos(angle + 3 * da), r1 * sin(angle + 3 * da),
                -width * 0.5f);
        glNormal3f(cos(angle), sin(angle), 0);
    }

    glVertex3f(r1 /* * cos(0) */, /* r1 * sin(0) */ 0, width * 0.5f);
    glVertex3f(r1 /* * cos(0) */, /* r1 * sin(0) */ 0, -width * 0.5f);

    glEnd();


    glShadeModel(GL_SMOOTH);

    /* draw inside radius cylinder */
    glBegin(GL_QUAD_STRIP);
    for (i = 0; i <= teeth; i++) {
        angle = i * 2 * FM_PI / teeth;
        glNormal3f(-cos(angle), -sin(angle), 0);
        glVertex3f(r0 * cos(angle), r0 * sin(angle), -width * 0.5f);
        glVertex3f(r0 * cos(angle), r0 * sin(angle), width * 0.5f);
    }
    glEnd();

    check();
#ifdef DEBUG_CODE
printf("GEAR done %s %d\n", __FILE__, __LINE__);
#endif
}



static PyObject *
visualizer_idle(PyObject *self, PyObject *args)
{
    WHIRLYGIZMO *Wg;
    Togl   *togl;
    PyObject *widget;
#ifdef DEBUG_CODE
//printf("IDLE %d\n", 1);
#endif

    if (!PyArg_ParseTuple(args, "O", &widget))
        return NULL;

    togl = getToglFromWidget(widget);

    Wg = (WHIRLYGIZMO *) Togl_GetClientData(togl);
    Wg->Angle += 2;
    Togl_PostRedisplay(togl);
    check();

#ifdef DEBUG_CODE
//printf("IDLE DONE %d\n", 1);
#endif
    return PyLong_FromLong(0);
}

static PyObject *
visualizer_init(PyObject *self, PyObject *args)
{
    WHIRLYGIZMO *Wg;
    static GLfloat red[4] = { 0.8f, 0.1f, 0, 1 };
    static GLfloat green[4] = { 0, 0.8f, 0.2f, 1 };
    static GLfloat blue[4] = { 0.2f, 0.2f, 1, 1 };
    static GLfloat pos[4] = { 5, 5, 10, 0 };
    Togl   *togl;
    PyObject *widget;

#ifdef DEBUG_CODE
printf("visualizer_init start %s %d\n", __FILE__, __LINE__);
#endif

#if 0
    //printf("SELF INIT = %p ARGS = %p\n", self, args);
    PyObject* foo = PyObject_Repr(args);
    //printf("ARGS INIT = %s\n", PyUnicode_AsUTF8(foo));
#endif

    if (!PyArg_ParseTuple(args, "O", &widget))
        return NULL;

#if 0
    //printf("WIDGET INIT = %p\n", widget);
    PyObject* foo = PyObject_Repr(widget);
    printf("WIDGET INIT= %s\n", PyUnicode_AsUTF8(foo));
#endif

#ifdef DEBUG_CODE
printf("visualizer_init before getTogl %s %d\n", __FILE__, __LINE__);
#endif
    togl = getToglFromWidget(widget);
#ifdef DEBUG_CODE
printf("visualizer_init after getTogl %s %d\n", __FILE__, __LINE__);
#endif

    glLightfv(GL_LIGHT0, GL_POSITION, pos);
    glEnable(GL_CULL_FACE);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_DEPTH_TEST);
    /* make the gears */
    Wg = (WHIRLYGIZMO *) malloc(sizeof (WHIRLYGIZMO));
    Wg->Gear1 = glGenLists(1);
    glNewList(Wg->Gear1, GL_COMPILE);
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, red);
    gear(1, 4, 1, 20, 0.7f);
    glEndList();

    Wg->Gear2 = glGenLists(1);
    glNewList(Wg->Gear2, GL_COMPILE);
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, green);
    gear(0.5f, 2, 2, 10, 0.7f);
    glEndList();


    Wg->Gear3 = glGenLists(1);
    glNewList(Wg->Gear3, GL_COMPILE);
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, blue);
    gear(1.3f, 2, 0.5f, 10, 0.7f);
    glEndList();

    glEnable(GL_NORMALIZE);
#if 1
    Wg->Height = Togl_Height(togl);
    Wg->Width = Togl_Width(togl);
#else
    Wg->Height = 200; 
    Wg->Width = 200; 
#endif
    Wg->Angle = 0;
    Wg->Rotx = 0;
    Wg->Roty = 0;
    Wg->Rotz = 0;
    //DSH4
    glFlush();
    Togl_SetClientData(togl, (ClientData) Wg);
    check();

    //printf("WIDGET INIT DONE= %s\n", PyUnicode_AsUTF8(foo));

#ifdef DEBUG_CODE
printf("visualizer_init done %s %d\n", __FILE__, __LINE__);
#endif
    return PyLong_FromLong(0);
}

/* 
 * static GLfloat view_rotx=20, view_roty=30, view_rotz=0; static GLint
 * gear1, gear2, gear3; static GLfloat angle = 0; */
// static GLuint limit;
// static GLuint count = 1;

// static GLubyte polycolor[4] = { 255, 255, 255, 255 };

static PyObject *
visualizer_draw(PyObject *self, PyObject *args)
{
    WHIRLYGIZMO *Wg;
    Togl   *togl;
    PyObject *widget;
#ifdef DEBUG_CODE
//printf("visualizer_draw start %s %d\n", __FILE__, __LINE__);
#endif

#if 0
    if (objc != 2) {
        Tcl_WrongNumArgs(interp, 1, objv, "pathName");
        return TCL_ERROR;
    }

    if (Togl_GetToglFromObj(interp, objv[1], &togl) != TCL_OK) {
        return TCL_ERROR;
    }
#endif

    if (!PyArg_ParseTuple(args, "O", &widget))
        return NULL;

#if 0
    PyObject* foo = PyObject_Repr(widget);
    printf("WIDGET DRAW= %s\n", PyUnicode_AsUTF8(foo));
#endif

    togl = getToglFromWidget(widget);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    Wg = (WHIRLYGIZMO *) Togl_GetClientData(togl);
    glDisable(GL_TEXTURE_2D);
    glPushMatrix();
    glRotatef((float) Wg->Rotx, 1, 0, 0);
    glRotatef((float) Wg->Roty, 0, 1, 0);
    glRotatef((float) Wg->Rotz, 0, 0, 1);

    glPushMatrix();
    glTranslatef(-3, -2, 0);
    glRotatef((float) Wg->Angle, 0, 0, 1);
    glEnable(GL_DEPTH_TEST);
    glCallList(Wg->Gear1);
    glEnable(GL_DEPTH_TEST);
    glPopMatrix();

    glPushMatrix();
    glTranslatef(3.1f, -2, 0);
    glRotatef(-2 * (float) Wg->Angle - 9, 0, 0, 1);
    glCallList(Wg->Gear2);
    glPopMatrix();

    glPushMatrix();
    glTranslatef(-3.1f, 4.2f, 0);
    glRotatef(-2 * (float) Wg->Angle - 25, 0, 0, 1);
    glCallList(Wg->Gear3);
    glPopMatrix();

    glPopMatrix();

    Togl_SwapBuffers(togl);
    //DSH4
    glFlush();
    check();

#ifdef DEBUG_CODE
//printf("visualizer_draw done %s %d\n", __FILE__, __LINE__);
#endif
    return PyLong_FromLong(0);
}

/* change view angle, exit upon ESC */
/* 
 * static GLenum key(int k, GLenum mask) { switch (k) { case TK_UP: view_rotx
 * += 5; return GL_TRUE; case TK_DOWN: view_rotx -= 5; return GL_TRUE; case 
 * TK_LEFT: view_roty += 5; return GL_TRUE; case TK_RIGHT: view_roty -= 5;
 * return GL_TRUE; case TK_z: view_rotz += 5; return GL_TRUE; case TK_Z:
 * view_rotz -= 5; return GL_TRUE; } return GL_FALSE; } */

/* new window size or exposure */
static PyObject *
visualizer_reshape(PyObject *self, PyObject *args)
{
    int     width, height;
    Togl   *togl;
    PyObject *widget;
#if 1
    // INIT code
#endif

#ifdef DEBUG_CODE
printf("visualizer_reshape start %s %d\n", __FILE__, __LINE__);
#endif
#if 0
    if (objc != 2) {
        Tcl_WrongNumArgs(interp, 1, objv, "pathName");
        return TCL_ERROR;
    }

    if (Togl_GetToglFromObj(interp, objv[1], &togl) != TCL_OK) {
        return TCL_ERROR;
    }
#endif

    if (!PyArg_ParseTuple(args, "O", &widget))
        return NULL;

    togl = getToglFromWidget(widget);

#if 1
    // INIT code

#endif

    width = Togl_Width(togl);
    height = Togl_Height(togl);
    glViewport(0, 0, (GLint) width, (GLint) height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    if (width > height) {
        GLfloat w = (GLfloat) width / (GLfloat) height;

        glFrustum(-w, w, -1, 1, 5, 60);
    } else {
        GLfloat h = (GLfloat) height / (GLfloat) width;

        glFrustum(-1, 1, -h, h, 5, 60);
    }

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslatef(0, 0, -40);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    //DSH4
    glFlush();
    check();
#ifdef DEBUG_CODE
printf("visualizer_reshape done %s %d\n", __FILE__, __LINE__);
#endif

    return PyLong_FromLong(0);
}


static PyObject *
visualizer_zap(PyObject *self, PyObject *args)
{
    WHIRLYGIZMO *Wg;
    Togl   *togl;
    PyObject *widget;

    if (!PyArg_ParseTuple(args, "O", &widget))
        return NULL;

    togl = getToglFromWidget(widget);

    Wg = (WHIRLYGIZMO *) Togl_GetClientData(togl);
    free(Wg);
    check();

    return PyLong_FromLong(0);
}



static PyObject *
visualizer_position(PyObject *self, PyObject *args)
{
    WHIRLYGIZMO *Wg;
    char    Result[100];
    Togl   *togl;
    PyObject *widget;

    if (!PyArg_ParseTuple(args, "O", &widget))
        return NULL;

    togl = getToglFromWidget(widget);

    Wg = (WHIRLYGIZMO *) Togl_GetClientData(togl);

    /* Let result string equal value */
    sprintf(Result, "%g %g", Wg->Roty, Wg->Rotx);
    check();

    return PyLong_FromLong(0);
}

static PyObject *
visualizer_rotate(PyObject *self, PyObject *args)
{
    WHIRLYGIZMO *Wg;
    Togl   *togl;
    PyObject *widget;

    if (!PyArg_ParseTuple(args, "O", &widget))
        return NULL;

    togl = getToglFromWidget(widget);

    Wg = (WHIRLYGIZMO *) Togl_GetClientData(togl);

#if 0
    if (Tcl_GetDoubleFromObj(interp, objv[2], &Wg->Roty) != TCL_OK) {
	    //DSH4
        return PyLong_FromLong(1);
    }
    if (Tcl_GetDoubleFromObj(interp, objv[3], &Wg->Rotx) != TCL_OK) {
	    //DSH4
        return PyLong_FromLong(1);
    }
#endif
    Togl_PostRedisplay(togl);
    check();

    return PyLong_FromLong(0);
}



static PyMethodDef visualizerMethods[] = {
    {"init",  visualizer_init, METH_VARARGS, "Init Visualizer."},
    {"idle",  visualizer_idle, METH_VARARGS, "Visualizer Idle."},
    {"draw",  visualizer_draw, METH_VARARGS, "Visualizer Draw."},
    {"reshape",  visualizer_reshape, METH_VARARGS, "Visualizer Reshape."},
    {"zap",  visualizer_zap, METH_VARARGS, "Visualizer Zap."},
    {"position",  visualizer_position, METH_VARARGS, "Visualizer Position."},
    {"rotate",  visualizer_rotate, METH_VARARGS, "Visualizer Rotate."},
    {NULL, NULL, 0, NULL}        /* Sentinel */
};

static struct PyModuleDef visualizermodule = {
    PyModuleDef_HEAD_INIT,
    "visualizer",   /* name of module */
    NULL, /* module documentation, may be NULL */
    -1,       /* size of per-interpreter state of the module,
                 or -1 if the module keeps state in global variables. */
    visualizerMethods
};

PyMODINIT_FUNC
PyInit_visualizer(void)
{
#ifdef DEBUG_CODE
printf("Py_Init start %s %d\n", __FILE__, __LINE__);
#endif
    return PyModule_Create(&visualizermodule);
}


