#! /usr/bin/env zsh

# environment variables:
# BUILD_ALL (default: 0): Whether to build base ros2 packages.
# SKIP_OLD (default: 1): Whether to skip building packages that are already up-to-date.
# BASE_PATHS (default: ${ARENA_WS_DIR}/src): The base paths to search for packages.
# PATHS (default: all packages in BASE_PATHS): The specific packages to build.
# BUILD_PACKAGES (default: all packages in PATHS): The packages to build.
# BUILD_BASE (default: ${ARENA_WS_DIR}/build): The base directory for building packages.
# INSTALL_BASE (default: ${ARENA_WS_DIR}/install): The base directory for installing packages.

source "${ARENA_WS_DIR:-.}/src/arena/arena-rosnav/tools/source.zsh" || return $?

_SKIP_OLD=${SKIP_OLD:-$([[ "${*}" == *"--packages-"* ]] || echo 1 )}

if [ -n "${BUILD_BASE}" ]; then
    _BUILD_BASE="${BUILD_BASE}"
else
    _BUILD_BASE="${ARENA_WS_DIR}/build"
fi
if [ -n "${INSTALL_BASE}" ]; then
    _INSTALL_BASE="${INSTALL_BASE}"
else
    _INSTALL_BASE="${ARENA_WS_DIR}/install"
fi

if [ -n "${BASE_PATHS}" ]; then
    IFS=';' read -r -a _BASE_PATHS <<< "$BASE_PATHS"
else
    _BASE_PATHS=("${ARENA_WS_DIR}/src")
fi

if [ -n "${PATHS}" ]; then
    IFS=';' read -r -a _PATHS <<< "$PATHS"
elif [ "${BUILD_ALL}" = 1 ]; then
    _PATHS=("${ARENA_WS_DIR}/src/*")
else
    declare -a _PATHS=()
    for dir in "${ARENA_WS_DIR}/src"/*/; do
        name="$(basename "$dir")"
        case "$name" in
            ros2|tools)
                ;;
            *)
                _PATHS+=("$dir*")
                ;;
        esac
    done
fi

echo "Using base paths: ${_BASE_PATHS[*]}"
echo "Using build base: ${_BUILD_BASE}"
echo "Using install base: ${_INSTALL_BASE}"
echo "Building paths: ${_PATHS[*]}"

_BUILD_PACKAGES=()


if [ "${_SKIP_OLD}" = 1 ]; then
    echo "INDEXING: colcon list --base-paths ${_PATHS[*]}"

    function recursive_mtime(){
        find "$1" \! -type l -printf "%T@\n"  | sort | tail -1 | cut -d. -f1
    }

    while read -r target
    do
        package=$(echo "$target" | cut -f1)
        src_path=$(echo "$target" | cut -f2)

        # echo testing $package
        # test -n "$package" || echo "Package name is empty"
        # test -d "${_INSTALL_BASE}/${package}" || echo "Package ${package} not found in install base ${_INSTALL_BASE}"
        # test -d "${src_path}" || echo "Source path ${src_path} not found"
        # echo 

        if  [ -n "${package}" ] &&
            [ -d "${_INSTALL_BASE}/${package}" ] &&
            [ "$(recursive_mtime "${_INSTALL_BASE}/${package}")" -ge "$(recursive_mtime "${src_path}")" ];
        then
            # package is already built and newer than source
            :
        else
            # package is not built or source is newer than install
            _BUILD_PACKAGES+=("$package")
        fi

    done < <(colcon list --base-paths "${_PATHS[@]}"); #only check packages to be built
fi

ARGS=(
    "--build-base ${_BUILD_BASE}"
    "--install-base ${_INSTALL_BASE}"
    "--base-paths ${_BASE_PATHS[*]}"
    "--symlink-install"
    "--continue-on-error"
    "--packages-skip qt_gui_core rqt qt_gui_cpp rqt_gui_cpp"
    "--cmake-args '-DPython3_ROOT_DIR=$(cd "${ARENA_WS_DIR}/src/arena/arena-rosnav" && poetry env info -p) -DBUILD_TESTING=OFF'"
)

if [ ${#_BUILD_PACKAGES[@]} -ne 0 ]; then
    ARGS+=("--packages-select" "${_BUILD_PACKAGES[@]}")
fi

echo "BUILDING: colcon build ${ARGS[*]} $*"

eval "colcon build ${ARGS[*]} $*"

unset _SKIP_OLD
unset _BUILD_BASE
unset _INSTALL_BASE
unset _BASE_PATHS
unset _BUILD_PACKAGES

source "${ARENA_WS_DIR}/src/arena/arena-rosnav/tools/source.zsh"
