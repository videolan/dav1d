#!/usr/bin/env bash

DAV1D="tools/dav1d"
ARGON_DIR='.'
FILMGRAIN=1
CPUMASK=-1
THREADS=0

usage() {
    NAME=$(basename "$0")
    {
        printf "Usage:   %s [-d dav1d] [-a argondir] [-g \$filmgrain] [-c \$cpumask] [-t threads] [DIRECTORY]...\n" "$NAME"
        printf "Example: %s -d /path/to/dav1d -a /path/to/argon/ -g 0 -c avx2 profile0_core\n" "$NAME"
        printf "Used to verify that dav1d can decode the Argon AV1 test vectors correctly.\n\n"
        printf " DIRECTORY one or more dirs in the argon folder to check against\n"
        printf "             (default: everything except large scale tiles and stress files)\n"
        printf " -d dav1d  path to dav1d executable (default: tools/dav1d)\n"
        printf " -a dir    path to argon dir (default: 'tests/argon' if found; '.' otherwise)\n"
        printf " -g \$num   enable filmgrain (default: 1)\n"
        printf " -c \$mask  use restricted cpumask (default: -1)\n"
        printf " -t \$num   number of threads per dav1d (default: 0)\n\n"
    } >&2
    exit 1
}

error() {
    printf "\033[1;91m%s\033[0m\n" "$*" >&2
    exit 1
}

# find tests/argon
tests_dir=$(dirname "$(readlink -f "$0")")
if [ -d "$tests_dir/argon" ]; then
    ARGON_DIR="$tests_dir/argon"
fi

while getopts ":d:a:g:c:t:" opt; do
    case "$opt" in
        d)
            DAV1D="$OPTARG"
            ;;
        a)
            ARGON_DIR="$OPTARG"
            ;;
        g)
            FILMGRAIN="$OPTARG"
            ;;
        c)
            CPUMASK="$OPTARG"
            ;;
        t)
            THREADS="$OPTARG"
            ;;
        \?)
            printf "Error! Invalid option: -%s\n" "$OPTARG" >&2
            usage
            ;;
        *)
            usage
            ;;
    esac
done
shift $((OPTIND-1))

if [ "$#" -eq 0 ]; then
    # Everything except large scale tiles and stress files.
    dirs=("$ARGON_DIR/profile0_core"       "$ARGON_DIR/profile0_core_special"
          "$ARGON_DIR/profile0_not_annexb" "$ARGON_DIR/profile0_not_annexb_special"
          "$ARGON_DIR/profile1_core"       "$ARGON_DIR/profile1_core_special"
          "$ARGON_DIR/profile1_not_annexb" "$ARGON_DIR/profile1_not_annexb_special"
          "$ARGON_DIR/profile2_core"       "$ARGON_DIR/profile2_core_special"
          "$ARGON_DIR/profile2_not_annexb" "$ARGON_DIR/profile2_not_annexb_special"
          "$ARGON_DIR/profile_switching")
else
    mapfile -t dirs < <(printf "${ARGON_DIR}/%s\n" "$@" | sort -u)
fi

ver_info="dav1d $("$DAV1D" -v 2>&1) filmgrain=$FILMGRAIN cpumask=$CPUMASK" || error "Error! Can't run $DAV1D"
files=()

for d in "${dirs[@]}"; do
    if [ -d "$d/streams" ]; then
        files+=("${d/%\//}"/streams/*.obu)
    fi
done

if [ ${#files[@]} -eq 0 ]; then
    error "Error! No files found at ${dirs[*]}"
fi

failed=0
for i in "${!files[@]}"; do
    f="${files[i]}"
    if [ "$FILMGRAIN" -eq 0 ]; then
        md5=${f/\/streams\//\/md5_no_film_grain\/}
    else
        md5=${f/\/streams\//\/md5_ref\/}
    fi
    md5=$(<"${md5/%obu/md5}") || error "Error! Can't read md5 ${md5} for file ${f}"
    md5=${md5/ */}

    printf "\033[1K\r[%3d%% %d/%d] Verifying %s" "$(((i+1)*100/${#files[@]}))" "$((i+1))" "${#files[@]}" "$f"
    if ! "$DAV1D" -i "$f" --filmgrain "$FILMGRAIN" --verify "$md5" --cpumask "$CPUMASK" --threads "$THREADS" -q 2>/dev/null; then
        printf "\033[1K\rMismatch in %s\n" "$f"
        (( failed++ ))
    fi
done

if [ "$failed" -ne 0 ]; then
    printf "\033[1K\r%d/%d files \033[1;91mfailed\033[0m to verify" "$failed" "${#files[@]}"
else
    printf "\033[1K\r%d files \033[1;92msuccessfully\033[0m verified" "${#files[@]}"
fi
printf " in %dm%ds (%s)\n" "$((SECONDS/60))" "$((SECONDS%60))" "$ver_info"

exit $failed
