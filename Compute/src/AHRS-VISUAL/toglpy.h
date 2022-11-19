/* 
 * getToglFromWidget:
 *
 * Given a Python widget, get the corresponding Togl pointer.  <Python.h>
 * and <togl.h> should be included before this.  If included into a C file,
 * there should be a static keyword just before the include.
 * 
 * There should be one copy of getToglFromWidget per-shared libary so that
 * the library's Tcl/Tk/Togl stub pointers are properly initialized.
 *
 * Copyright (C) 2006  Greg Couch
 * See the LICENSE file for copyright details.
 */

//DSH4
// #define DEBUG_CODE
#include <stdexcept>

static Togl   *
getToglFromWidget(PyObject *widget)
{
    PyObject *cmdNameObj, *tk, *interpAddr;
    const char *cmdName;
    Tcl_Interp *interp;
    Togl   *curTogl;

#ifdef USE_TOGL_STUBS
    static int didOnce = 0;
#endif

#ifdef DEBUG_CODE
    printf("getTogl %s %d\n", __FILE__, __LINE__);
#endif

    /* Python: cmdName = widget._w */
    /* Python: interpAddr = widget.tk.interpaddr() */
    cmdNameObj = PyObject_GetAttrString(widget, "_w");
    tk = PyObject_GetAttrString(widget, "tk");
    if (cmdNameObj == NULL || !PyUnicode_Check(cmdNameObj) || tk == NULL) {
        Py_XDECREF(cmdNameObj);
        Py_XDECREF(tk);
#ifdef __cplusplus
        throw   std::invalid_argument("not a Tk widget");
#else
        return NULL;
#endif
    }
#ifdef DEBUG_CODE
    printf("getTogl %s %d\n", __FILE__, __LINE__);
#endif

    interpAddr = PyObject_CallMethod(tk, "interpaddr", "()");
    if (interpAddr == NULL || !PyLong_Check(interpAddr)) {
        Py_DECREF(cmdNameObj);
        Py_DECREF(tk);
        Py_XDECREF(interpAddr);
#ifdef __cplusplus
        throw   std::invalid_argument("not a Tk widget");
#else
        return NULL;
#endif
    }
#ifdef DEBUG_CODE
    printf("getTogl %s %d\n", __FILE__, __LINE__);
#endif

    cmdName = PyUnicode_AsUTF8(cmdNameObj);
    interp = (Tcl_Interp *) PyLong_AsLong(interpAddr);
#ifdef DEBUG_CODE
    printf("getTogl %s %d\n", __FILE__, __LINE__);
#endif

#ifdef USE_TOGL_STUBS
    if (!didOnce) {
        /* make sure stubs are initialized before calling a Togl function. */
        if (Tcl_InitStubs(interp, TCL_VERSION, 0) == NULL
                || Tk_InitStubs(interp, TK_VERSION, 0) == NULL
                || Togl_InitStubs(interp, TOGL_VERSION, 0) == NULL)
#  ifdef __cplusplus
            throw   std::runtime_error("unable to initialize Togl");
#  else
            return NULL;
#  endif
        didOnce = 1;
    }
#endif
#ifdef DEBUG_CODE
    printf("getTogl %s %d cmdName=%s\n", __FILE__, __LINE__, cmdName);
#endif

    if (Togl_GetToglFromName(interp, cmdName, &curTogl) != TCL_OK)
        curTogl = NULL;
#ifdef DEBUG_CODE
    printf("getTogl %s %d\n", __FILE__, __LINE__);
#endif
    Py_DECREF(cmdNameObj);
    Py_DECREF(tk);
    Py_DECREF(interpAddr);
#ifdef DEBUG_CODE
    printf("getTogl %s %d\n", __FILE__, __LINE__);
#endif
#ifdef __cplusplus
    if (curTogl == NULL)
        throw   std::invalid_argument("not a Togl widget");
#endif
#ifdef DEBUG_CODE
    printf("getTogl %s %d\n", __FILE__, __LINE__);
#endif
    return curTogl;
}
