if is_asm_enabled
    checkasm_sources = files('checkasm/checkasm.c')

    checkasm_tmpl_sources = files('checkasm/mc.c')

    checkasm_bitdepth_objs = []
    foreach bitdepth : dav1d_bitdepths
        checkasm_bitdepth_lib = static_library(
            'checkasm_bitdepth_@0@'.format(bitdepth),
            checkasm_tmpl_sources,
            include_directories: dav1d_inc_dirs,
            c_args: ['-DBITDEPTH=@0@'.format(bitdepth), stackalign_flag],
            install: false,
            build_by_default: false,
        )
        checkasm_bitdepth_objs += checkasm_bitdepth_lib.extract_all_objects()
    endforeach

    checkasm_nasm_objs = nasm_gen.process(files('checkasm/x86/checkasm.asm'))

    checkasm = executable('checkasm', checkasm_sources, checkasm_nasm_objs,
        objects: [checkasm_bitdepth_objs],
        include_directories: dav1d_inc_dirs,
        link_with: libdav1d_private,
        c_args: [stackalign_flag, stackrealign_flag],
        build_by_default: false,
        )

    test('checkasm test', checkasm)
endif
