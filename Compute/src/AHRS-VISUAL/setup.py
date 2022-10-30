from distutils.core import setup, Extension

module1 = Extension('visualizer',
                    #define_macros = [('__STDC_CONSTANT_MACROS', 1), ('__STDC_LIMIT_MACROS', 1), ('TARGET_POSIX', 1), ('_LINUX', 1), ('PIC', 1), ('_REENTRANT', 1), ('_LARGEFILE64_SOURCE', 1), ('_FILE_OFFSET_BITS=64', 1) ],
                    define_macros = [('__STDC_CONSTANT_MACROS', 1), ('__STDC_LIMIT_MACROS', 1), ('TARGET_POSIX', 1), ('_LINUX', 1), ('PIC', 1), ('_REENTRANT', 1), ('_LARGEFILE64_SOURCE', 1), ('_FILE_OFFSET_BITS=64', 1), ('HAVE_LIBOPENMAX', 2), ('OMX', 1), ('OMX_SKIP64BIT', 1), ('USE_EXTERNAL_OMX', 1), ('HAVE_LIBBCM_HOST', 1), ('USE_EXTERNAL_LIBBCM_HOST', 1), ('USE_VCHIQ_ARM', 1)],
                    #undef_macros = ['_FORTIFY_SOURCE'],
                    extra_compile_args = ['-Wno-psabi', '-fPIC', '-Wall', '-ftree-vectorize' , '-pipe'],
                    include_dirs = ['/usr/include/python3.9', './' , '/opt/vc/include/', '/opt/vc/include/interface/vcos/pthreads', '/opt/vc/include/interface/vmcs_host/linux', '/opt/vc/src/hello_pi/libs/ilclient', '/opt/vc/src/hello_pi/libs/vgfont', '/opt/vc/src/hello_pi/libs/revision' , '/usr/include/tcl8.6', '/usr/include/tcl8.6/tcl-private/generic', '/usr/include/tcl8.6/tcl-private/unix'],
                    library_dirs = [ '/opt/vc/lib/', '/opt/vc/src/hello_pi/libs/ilclient', '/opt/vc/src/hello_pi/libs/vgfont', '/opt/vc/src/hello_pi/libs/revision', '/usr/lib/Togl2.0/'],
                    libraries = [ 'brcmGLESv2', 'brcmEGL', 'openmaxil', 'bcm_host', 'vcos', 'vchiq_arm', 'pthread', 'rt', 'm', 'GL', 'Togl2.0', 'stdc++'],
                    extra_link_args = ['-Wl,--rpath=/usr/lib/Togl2.0'],
                    language = ['c++'],
                    sources = ['visualizermodule.cpp'])

setup (name = 'visualizer',
       version = '1.0',
       description = 'Visualizer package',
       author = 'Douglas Holtsinger',
       ext_modules = [module1])

