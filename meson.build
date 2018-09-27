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
    default_options: ['c_std=c99'],
    meson_version: '>= 0.47.0')

dav1d_src_root = meson.current_source_dir()
cdata = configuration_data()
cdata_asm = configuration_data()
cc = meson.get_compiler('c')

# On windows, we use a compatibility layer to emulate pthread
if host_machine.system() != 'windows'
    thread_dependency = dependency('threads')
else
    thread_dependency = declare_dependency(sources: ['src/win32/thread.c'])
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
    ['_POSIX_C_SOURCE',             '200112L'], # POSIX.1–2001 (IEEE Std 1003.1-2001)
]

if host_machine.system() == 'windows'
    feature_defines += [
            ['_WIN32_WINNT',                0x0601],
            ['UNICODE',                     1], # Define to 1 for Unicode (Wide Chars) APIs
            ['_UNICODE',                    1], # Define to 1 for Unicode (Wide Chars) APIs
            ['__USE_MINGW_ANSI_STDIO',      1], # Define to force use of MinGW printf
    ]
endif

if not cc.check_header('stdatomic.h')
    error('Atomics not supported')
endif

stackalign_flag = []
stackrealign_flag = []

if host_machine.cpu_family().startswith('x86')
    if cc.has_argument('-mpreferred-stack-boundary=5')
        stackalign_flag = ['-mpreferred-stack-boundary=5']
        stackrealign_flag = ['-mincoming-stack-boundary=4']
        cdata_asm.set('STACK_ALIGNMENT', 32)
        cdata.set('STACK_ALIGNMENT', 32)
    elif cc.has_argument('-mpreferred-stack-boundary=4')
        stackalign_flag = ['-mpreferred-stack-boundary=4']
        stackrealign_flag = ['-mincoming-stack-boundary=4']
        cdata_asm.set('STACK_ALIGNMENT', 16)
        cdata.set('STACK_ALIGNMENT', 16)
    elif cc.has_argument('-mstack-alignment=32')
        stackalign_flag = ['-mstack-alignment=32']
        stackrealign_flag = ['-mstackrealign']
        cdata_asm.set('STACK_ALIGNMENT', 32)
        cdata.set('STACK_ALIGNMENT', 32)
    else
        if host_machine.cpu_family() == 'x86_64'
            cdata_asm.set('STACK_ALIGNMENT', 16)
            cdata.set('STACK_ALIGNMENT', 16)
        else
            cdata_asm.set('STACK_ALIGNMENT', 4)
            cdata.set('STACK_ALIGNMENT', 4)
        endif
    endif
endif

if host_machine.cpu_family().startswith('x86')
    cdata.set10('ARCH_X86', true)
    if host_machine.cpu_family() == 'x86_64'
        cdata_asm.set10('ARCH_X86_64', true)
        cdata.set10('ARCH_X86_64', true)
        cdata_asm.set10('ARCH_X86_32', false)
        cdata.set10('ARCH_X86_32', false)

        cdata_asm.set10('PIC', true)
    else
        cdata_asm.set10('ARCH_X86_64', false)
        cdata.set10('ARCH_X86_64', false)
        cdata_asm.set10('ARCH_X86_32', true)
        cdata.set10('ARCH_X86_32', true)
    endif
else
    cdata.set10('ARCH_X86', false)
    cdata.set10('ARCH_X86_64', false)
    cdata.set10('ARCH_X86_32', false)
endif

if cc.symbols_have_underscore_prefix()
    cdata_asm.set10('PREFIX', true)
endif

if cc.has_argument('-fvisibility=hidden')
    add_project_arguments('-fvisibility=hidden', language: 'c')
else
    warning('Compiler does not support -fvisibility=hidden, all symbols will be public!')
endif

if cc.has_function('posix_memalign', prefix: '#include <stdlib.h>', args: ['-D_POSIX_C_SOURCE=200112L'])
    cdata.set('HAVE_POSIX_MEMALIGN', 1)
elif cc.has_function('_aligned_malloc', prefix: '#include <malloc.h>')
    cdata.set('HAVE_ALIGNED_MALLOC', 1)
endif

if cc.check_header('unistd.h')
    cdata.set('HAVE_UNISTD_H', 1)
endif

if (get_option('buildtype') != 'debug' and
    get_option('buildtype') != 'plain')
    add_project_arguments('-fomit-frame-pointer', '-ffast-math',
        language: 'c')
endif

warning_flags = [
  '-Wundef',
  '-Wvla', # should be '-Werror=vla
]

add_project_arguments(cc.get_supported_arguments(warning_flags), language: 'c')

foreach f : feature_defines
   cdata.set(f.get(0), f.get(1))
endforeach

is_asm_enabled = (get_option('build_asm') == true and
    host_machine.cpu_family().startswith('x86'))
cdata.set10('HAVE_ASM', is_asm_enabled)

#
# Generate config headers
#

config_h_target = configure_file(output: 'config.h', configuration: cdata)

if is_asm_enabled
    config_asm_target = configure_file(output: 'config.asm', output_format: 'nasm', configuration: cdata_asm)
endif

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

entrypoints_src = files(
    'src/lib.c',
    'src/thread_task.c'
)
entrypoints_lib = static_library(
    'libdav1dentrypoint',
    entrypoints_src, rev_target,
    include_directories: dav1d_inc_dirs,
    c_args: stackrealign_flag,
    install: false,
    build_by_default: false,
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

if is_asm_enabled
    libdav1d_sources += files(
        'src/x86/cpu.c',
    )
    libdav1d_tmpl_sources += files(
        'src/x86/mc_init.c',
    )
    libdav1d_sources_asm = files(
        'src/x86/cpuid.asm',
        'src/x86/mc.asm',
    )

    nasm = find_program('nasm')

    if host_machine.system() == 'windows'
        nasm_format = 'win'
    elif host_machine.system() == 'darwin'
        nasm_format = 'macho'
    else
        nasm_format = 'elf'
    endif
    if host_machine.cpu_family() == 'x86_64'
        nasm_format += '64'
    else
        nasm_format += '32'
    endif

    nasm_gen = generator(nasm,
        output: '@BASENAME@.obj',
        depfile: '@BASENAME@.obj.ndep',
        arguments: [
            '-f', nasm_format,
            '-I', '@CURRENT_SOURCE_DIR@/',
            '-MQ', '@OUTPUT@', '-MF', '@DEPFILE@',
            '@EXTRA_ARGS@',
            '@INPUT@',
            '-o', '@OUTPUT@'
        ])

    nasm_objs = nasm_gen.process(libdav1d_sources_asm)
else
    nasm_objs = []
endif

# Build a helper library for each bitdepth
bitdepth_objs = []
foreach bitdepth : dav1d_bitdepths
    bitdepth_lib = static_library(
        'dav1d_bitdepth_@0@'.format(bitdepth),
        libdav1d_tmpl_sources, config_h_target,
        include_directories: dav1d_inc_dirs,
        c_args: ['-DBITDEPTH=@0@'.format(bitdepth)] + stackalign_flag,
        install: false,
        build_by_default: false,
    )
    bitdepth_objs += bitdepth_lib.extract_all_objects()
endforeach

libdav1d = library('dav1d',
    libdav1d_sources, nasm_objs,
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

if cc.get_id() == 'msvc'
    dav1d_sources += files('tools/compat/getopt.c')
endif

dav1d = executable('dav1d',
    dav1d_sources, rev_target,
    link_with: libdav1d,
    include_directories: [dav1d_inc_dirs, include_directories('tools')],
    install: true,
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
