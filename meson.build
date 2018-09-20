# Copyright © 2018, VideoLAN and dav1d authors
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

project('dav1d', ['c'],
    version: '0.0.1',
    default_options: ['c_std=c11'],
    meson_version: '>= 0.47.0')

dav1d_src_root = meson.current_source_dir()
cdata = configuration_data()
cc = meson.get_compiler('c')

if not meson.is_cross_build() 
    thread_dependency = dependency('threads')
else
    thread_dependency = cc.find_library('pthread', required: false)
endif
if thread_dependency.found()
    cdata.set('HAVE_PTHREAD_H', 1)
endif

dav1d_inc_dirs = include_directories(['include', 'include/dav1d'])

#
# Option handling
#
dav1d_bitdepths = get_option('bitdepths')
foreach bitdepth : dav1d_bitdepths
    cdata.set('CONFIG_@0@BPC'.format(bitdepth), 1)
endforeach

#
# OS/Compiler feature detection
#

feature_defines = [
    ['_REENTRANT',                  1], # Define so that reentrant versions of several functions get declared
    ['_THREAD_SAFE',                1], # Same as _REENTANT for some other OSes
    ['_GNU_SOURCE',                 1], # Enable GNU extensions on systems that have them
    ['_POSIX_PTHREAD_SEMANTICS',    1], # Enable threading extensions on Solaris
    ['__EXTENSIONS__',              1], # Enable general extensions on Solaris
    ['_FILE_OFFSET_BITS',           64], # Define to 64 for large files support
]

if host_machine.system() == 'windows'
    feature_defines += [
            ['_WIN32_WINNT',                0x0601],
            ['UNICODE',                     1], # Define to 1 for Unicode (Wide Chars) APIs
            ['_UNICODE',                    1], # Define to 1 for Unicode (Wide Chars) APIs
            ['__USE_MINGW_ANSI_STDIO',      1], # Define to force use of MinGW printf
            ['_ISOC99_SOURCE',              1], # Extensions to ISO C89 from ISO C99
            ['_ISOC11_SOURCE',              1], # Extensions to ISO C99 from ISO C11
            ['_POSIX_SOURCE',               1], # IEEE Std 1003.1
            ['_POSIX_C_SOURCE',             '200809L'], #IEEE Std 1003.1
            ['_XOPEN_SOURCE',               700], # POSIX and XPG 7th edition
            ['_XOPEN_SOURCE_EXTENDED',      1], # XPG things and X/Open Unix extensions
            ['_BSD_SOURCE',                 1], # ISO C, POSIX, and 4.3BSD things
            ['_SVID_SOURCE',                1], # ISO C, POSIX, and SVID things
    ]
endif

if not cc.check_header('stdatomic.h')
    error('Atomics not supported')
endif

if cc.has_argument('-mpreferred-stack-boundary=5')
    stackalign_flag = '-mpreferred-stack-boundary=5'
    stackrealign_flag = '-mincoming-stack-boundary=4'
# When cross compiling for win64 gcc refuses to use -mpreferred-stack-boundary
# with a value which isn't 3 or 4. However, when cross compiling with clang, 5 is
# accepted.
elif (host_machine.system() == 'windows' and host_machine.cpu_family() == 'x86_64'
and cc.has_argument('-mpreferred-stack-boundary=4'))
    stackalign_flag = '-mpreferred-stack-boundary=4'
    stackrealign_flag = '-mincoming-stack-boundary=4'
elif cc.has_argument('-mstack-alignment=32')
    stackalign_flag = '-mstack-alignment=32'
    stackrealign_flag = '-mstackrealign'
else
    error('Failed to specify stack alignment')
endif

if cc.has_argument('-fvisibility=hidden')
    add_project_arguments('-fvisibility=hidden', language: 'c')
else
    warning('Compiler does not support -fvisibility=hidden, all symbols will be public!')
endif

if cc.has_function('posix_memalign', prefix: '#include <stdlib.h>', args: ['-D_POSIX_C_SOURCE=200112'])
    feature_defines += [['POSIX_C_SOURCE', 200112]]
    cdata.set('HAVE_POSIX_MEMALIGN', 1)
elif cc.has_function('_aligned_malloc', prefix: '#include <malloc.h>')
    cdata.set('HAVE_ALIGNED_MALLOC', 1)
endif

add_project_arguments('-fomit-frame-pointer', '-ffast-math',
    language: 'c')

add_project_arguments('-Wall', '-Wundef',
    language: 'c')

foreach f : feature_defines
   cdata.set(f.get(0), f.get(1))
endforeach

#
# Generate config headers
#

config_h_target = configure_file(output: 'config.h', configuration: cdata)

subdir('include')

#
# dav1d library
#
libdav1d_tmpl_sources = files(
    'src/ipred.c',
    'src/itx.c',
    'src/ipred_prepare.c',
    'src/lf_apply.c',
    'src/loopfilter.c',
    'src/mc.c',
    'src/cdef_apply.c',
    'src/cdef.c',
    'src/lr_apply.c',
    'src/looprestoration.c',
    'src/recon.c'
)

# Build a helper library for each bitdepth
bitdepth_objs = []
foreach bitdepth : dav1d_bitdepths
    bitdepth_lib = static_library(
        'dav1d_bitdepth_@0@'.format(bitdepth),
        libdav1d_tmpl_sources, config_h_target,
        include_directories: dav1d_inc_dirs,
        c_args: ['-DBITDEPTH=@0@'.format(bitdepth), stackalign_flag],
        install: false,
        build_by_default: false,
    )
    bitdepth_objs += bitdepth_lib.extract_all_objects()
endforeach

entrypoints_src = files(
    'src/lib.c',
    'src/thread_task.c'
)
entrypoints_lib = static_library(
    'libdav1dentrypoint',
    entrypoints_src,
    include_directories: dav1d_inc_dirs,
    c_args: [stackrealign_flag],
    install: false,
)
entrypoints_objs = entrypoints_lib.extract_all_objects()

libdav1d_sources = files(
    'src/picture.c',
    'src/data.c',
    'src/ref.c',
    'src/getbits.c',
    'src/obu.c',
    'src/decode.c',
    'src/cdf.c',
    'src/msac.c',
    'src/tables.c',
    'src/scan.c',
    'src/dequant_tables.c',
    'src/intra_edge.c',
    'src/lf_mask.c',
    'src/ref_mvs.c',
    'src/warpmv.c',
    'src/wedge.c',
    'src/qm.c',
)

if host_machine.system() == 'windows'
    libdav1d_sources += files('src/win32/thread.c')
endif

libdav1d = library('dav1d',
    libdav1d_sources, rev_target,
    version: '0.0.1',
    objects: [bitdepth_objs, entrypoints_objs],
    include_directories: dav1d_inc_dirs,
    c_args: [stackalign_flag],
    dependencies: thread_dependency,
    install: true
)

install_subdir('include/dav1d/', install_dir: 'include')

#
# dav1d cli tool
#
dav1d_sources = files(
    'tools/dav1d.c',
    'tools/dav1d_cli_parse.c',
    'tools/input/input.c',
    'tools/input/ivf.c',
    'tools/output/md5.c',
    'tools/output/output.c',
    'tools/output/y4m2.c',
    'tools/output/yuv.c'
)

dav1d = executable('dav1d',
    dav1d_sources, rev_target,
    link_with: libdav1d,
    include_directories: [dav1d_inc_dirs, include_directories('tools')]
)

#
# pkg-config boilerplate
#
pkg_mod = import('pkgconfig')
pkg_mod.generate(libraries: libdav1d,
    version: '0.0.1',
    name: 'libdav1d',
    filebase: 'dav1d',
    description: 'AV1 decoding library'
)
