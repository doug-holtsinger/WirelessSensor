
python3 setup.py build_ext --inplace
sudo python3 setup.py install

pxd file 
Cython uses .pxd files which work like C header files – they contain Cython declarations 
(and sometimes code sections) which are only meant for inclusion by Cython modules. A pxd file is 
imported into a pyx module by using the cimport keyword.

pyx
Cython source code

setup.py
Like a Makefile for Cython

