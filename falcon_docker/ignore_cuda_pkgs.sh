#!/bin/bash
# ============================================================
# ignore_cuda_pkgs.sh
#
# Seeds CATKIN_IGNORE on packages that hard-require CUDA, then
# cascades CATKIN_IGNORE to any package that declares a dependency
# (via package.xml) on something already ignored.
#
# This eliminates the whack-a-mole pattern of catkin_make failing
# on transitive dependents one at a time.
#
# Usage:
#   ./ignore_cuda_pkgs.sh /catkin_ws/src
# ============================================================
set -e

WS_SRC="${1:-/catkin_ws/src}"

# Seed: packages we know hard-require CUDA.
SEED_DIRS=(
    "${WS_SRC}/FALCON/uav_simulator/camera_sensing/pointcloud_render"
    "${WS_SRC}/FALCON/uav_simulator/map_render"
)

for d in "${SEED_DIRS[@]}"; do
    if [ -d "$d" ]; then
        touch "$d/CATKIN_IGNORE"
        echo "==> Seed ignored: $(basename "$d")"
    fi
done

# Cascade: keep ignoring packages whose package.xml lists a dependency
# on an already-ignored package, until no more changes happen.
ITER=0
while : ; do
    ITER=$((ITER + 1))
    CHANGED=0

    # Build current set of ignored package names (from their package.xml)
    IGNORED_NAMES=""
    while IFS= read -r ig_marker; do
        ig_dir=$(dirname "$ig_marker")
        if [ -f "$ig_dir/package.xml" ]; then
            n=$(grep -oP '<name>\K[^<]+' "$ig_dir/package.xml" | head -1)
            IGNORED_NAMES="$IGNORED_NAMES $n"
        fi
    done < <(find "$WS_SRC" -name CATKIN_IGNORE)

    # Walk every active (not-yet-ignored) package and check its deps
    while IFS= read -r pkg_xml; do
        pkg_dir=$(dirname "$pkg_xml")
        [ -f "$pkg_dir/CATKIN_IGNORE" ] && continue

        for ig_name in $IGNORED_NAMES; do
            # Match <build_depend>, <depend>, <exec_depend>, <run_depend>
            if grep -qE "<(build_depend|depend|exec_depend|run_depend)>${ig_name}</" "$pkg_xml"; then
                pkg_name=$(grep -oP '<name>\K[^<]+' "$pkg_xml" | head -1)
                echo "==> Iter ${ITER}: ignoring '${pkg_name}' (depends on '${ig_name}')"
                touch "$pkg_dir/CATKIN_IGNORE"
                CHANGED=1
                break
            fi
        done
    done < <(find "$WS_SRC" -name package.xml)

    [ "$CHANGED" = "0" ] && break
done

echo ""
echo "===== Final ignored packages ====="
find "$WS_SRC" -name CATKIN_IGNORE | while read -r m; do
    d=$(dirname "$m")
    if [ -f "$d/package.xml" ]; then
        grep -oP '<name>\K[^<]+' "$d/package.xml" | head -1
    fi
done | sort
echo "=================================="