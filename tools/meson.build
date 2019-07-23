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

#
# Build definition for the dav1d tools
#

# Leave subdir if tools are disabled
if not get_option('enable_tools')
    subdir_done()
endif


# Configuratin data for cli_config.h
cli_cdata = configuration_data()

rt_dependency = []
if host_machine.system() != 'windows'
    if cc.has_function('clock_gettime', prefix : '#include <time.h>', args : test_args)
        cli_cdata.set('HAVE_CLOCK_GETTIME', 1)
    elif host_machine.system() == 'darwin'
        cli_cdata.set('HAVE_MACH_ABSOLUTE_TIME', 1)
    else
        rt_dependency = cc.find_library('rt', required: false)
        if not cc.has_function('clock_gettime', prefix : '#include <time.h>', args : test_args, dependencies : rt_dependency)
            error('clock_gettime not found')
        endif
        cli_cdata.set('HAVE_CLOCK_GETTIME', 1)
    endif
endif

cli_config_h_target = configure_file(output: 'cli_config.h', configuration: cli_cdata)

# dav1d cli tool sources
dav1d_sources = files(
    'dav1d.c',
    'dav1d_cli_parse.c',
    'input/input.c',
    'input/annexb.c',
    'input/ivf.c',
    'output/md5.c',
    'output/null.c',
    'output/output.c',
    'output/y4m2.c',
    'output/yuv.c',
)

dav1d = executable('dav1d',
    dav1d_sources,
    rev_target, cli_config_h_target,

    link_with : libdav1d,
    include_directories : [dav1d_inc_dirs],
    dependencies : [getopt_dependency, thread_dependency, rt_dependency],
    install : true,
)


# dav1d player sources
dav1dplay_sources = files(
    'dav1dplay.c',
    'input/input.c',
    'input/annexb.c',
    'input/ivf.c',
)

sdl2_dependency = dependency('sdl2', version: '>= 2.0.1', required: false)

if sdl2_dependency.found()
    placebo_dependency = dependency('libplacebo', version: '>= 1.18.0', required: false)
    vulkan_dependency = dependency('vulkan', required: false)
    sdl_has_vulkan = cc.has_header('SDL_vulkan.h', dependencies: [sdl2_dependency])
    cflag_placebo = []
    deps_placebo = []
    if placebo_dependency.found() and vulkan_dependency.found() and sdl_has_vulkan
        cflag_placebo += '-DHAVE_PLACEBO_VULKAN=1'
        deps_placebo = [vulkan_dependency, placebo_dependency]
    endif
    dav1dplay = executable('dav1dplay',
        dav1dplay_sources,
        rev_target,

        link_with : libdav1d,
        include_directories : [dav1d_inc_dirs],
        dependencies : [getopt_dependency, sdl2_dependency, deps_placebo],
        install : true,
        c_args : cflag_placebo,
    )
endif
