#!/bin/bash

set -x

sudo rm -rf build

if [[ ! -z $1 && $1 -eq 1 ]] ; then
    python3 setup.py  build
    rc=$?
    if [[ $rc -ne 0 ]] ; then
	     exit $rc
    fi
    sudo python3 setup.py  install
    ldd build/lib.linux-armv7l-3.9/visualizer.cpython-39-arm-linux-gnueabihf.so | sort
    exit $?
fi

# Rather than compile with user-permission libraries, compile with libs at /usr/lib and
# do a 'make install' from the DEBUG or non-debug directories.

#if [[ ! -z $1 && $1 -eq 2 ]] ; then
#    python3 setup-debug.py  build
#    rc=$?
#    if [[ $rc -ne 0 ]] ; then
#	     exit $rc
#    fi
#    sudo python3 setup-debug.py  install
#    ldd build/lib.linux-armv7l-3.9/visualizer.cpython-39-arm-linux-gnueabihf.so | sort
#    exit $?
#fi


mkdir -p build/temp.linux-armv7l-3.9/
mkdir -p build/lib.linux-armv7l-3.9/

arm-linux-gnueabihf-g++ -pthread -Wno-unused-result -Wsign-compare -DNDEBUG -g -fwrapv -O2 -Wall -g -ffile-prefix-map=/python3.9-3.9.2=. -fstack-protector-strong -Wformat -Werror=format-security -g -fwrapv -O2 -g -ffile-prefix-map=/python3.9-3.9.2=. -fstack-protector-strong -Wformat -Werror=format-security -Wdate-time -D_FORTIFY_SOURCE=2 -fPIC -DSTANDALONE=1 -D__STDC_CONSTANT_MACROS=1 -D__STDC_LIMIT_MACROS=1 -DTARGET_POSIX=1 -D_LINUX=1 -DPIC=1 -D_REENTRANT=1 -D_LARGEFILE64_SOURCE=1 -D_FILE_OFFSET_BITS=64=1 -DHAVE_LIBOPENMAX=2 -DOMX=1 -DOMX_SKIP64BIT=1 -DUSE_EXTERNAL_OMX=1 -DHAVE_LIBBCM_HOST=1 -DUSE_EXTERNAL_LIBBCM_HOST=1 -DUSE_VCHIQ_ARM=1 -U_FORTIFY_SOURCE -I/usr/include/python3.9 -I./ -I/opt/vc/include/ -I/opt/vc/include/interface/vcos/pthreads -I/opt/vc/include/interface/vmcs_host/linux -I/opt/vc/src/hello_pi/libs/ilclient -I/opt/vc/src/hello_pi/libs/vgfont -I/opt/vc/src/hello_pi/libs/revision -I/usr/include/tcl8.6 -I/usr/include/tcl8.6/tcl-private/generic -I/usr/include/tcl8.6/tcl-private/unix -I/usr/include/python3.9 -c visualizermodule.cpp -o build/temp.linux-armv7l-3.9/visualizermodule.o -Wno-psabi -fPIC -Wall -ftree-vectorize -pipe
rc=$?
if [[ $rc -ne 0 ]] ; then
	    exit $rc
fi

arm-linux-gnueabihf-g++ -pthread -shared -Wl,-O1 -Wl,-Bsymbolic-functions -Wl,-z,relro -g -fwrapv -O2 -Wl,-z,relro -g -fwrapv -O2 -g -ffile-prefix-map=/python3.9-3.9.2=. -fstack-protector-strong -Wformat -Werror=format-security -Wdate-time -D_FORTIFY_SOURCE=2 build/temp.linux-armv7l-3.9/visualizermodule.o -L/opt/vc/lib/ -L/opt/vc/src/hello_pi/libs/ilclient -L/opt/vc/src/hello_pi/libs/vgfont -L/opt/vc/src/hello_pi/libs/revision -L/usr/lib/Togl2.0/ -lbrcmGLESv2 -lbrcmEGL -lopenmaxil -lbcm_host -lvcos -lvchiq_arm -lpthread -lrt -lm -lGL -lTogl2.0 -o build/lib.linux-armv7l-3.9/visualizer.cpython-39-arm-linux-gnueabihf.so -Wl,--rpath=/usr/lib/Togl2.0
rc=$?
if [[ $rc -ne 0 ]] ; then
	    exit $rc
fi


sudo python3 setup.py  install
ldd build/lib.linux-armv7l-3.9/visualizer.cpython-39-arm-linux-gnueabihf.so | sort

