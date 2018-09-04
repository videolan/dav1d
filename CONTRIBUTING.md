# Dav1d contribution guide

## CoC
The [VideoLAN Code of Conduct](https://wiki.videolan.org/CoC) applies to this project.

## Codebase language

The codebase is developed with the following assumptions:

For the library:
- C language with C99 version, without the VLA or the Complex (*\_\_STDC_NO_COMPLEX__*) features, and without compiler extension,
- asm in .asm files, using the NASM syntax,
- no C++ is allowed, whatever the version.

For the tools and utils:
- C *(see above for restrictions)*
- Rust
- C++ only for the MFT.

If you want to use *Threads* or *Atomic* features, please conform to the **C11**/**POSIX** semantic and use a wrapper for older compilers/platforms *(like done in VLC)*.

Please use modern standard POSIX functions *(strscpy, asprintf, tdestroy)*, and provide a compatibility fallback *(like done in VLC)*.

We will make reasonable efforts for compilers that are a bit older, but we won't support gcc 3 or MSVC 2012.

## Authorship

Please provide a correct authorship for your commit logs, with a name and a valid email.

We will reject anonymous contributions for now. As an exception, known pseudonyms from the multimedia community are accepted.

This project is respecting **Copyright** and **Droit d'auteur**. There is no copyright attribution or CLA.

## Commit logs

Please read [How to Write a Git Commit Message](https://chris.beams.io/posts/git-commit/).

## Submit requests (WIP)

- Code
- Test
- Try
- Submit patches

## Patent license

You need to read and understand the [AV1 patents license](doc/PATENTS), before committing.
