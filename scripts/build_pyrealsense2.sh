#!/usr/bin/env bash
# Build pyrealsense2 from librealsense source for this Mac and install it into
# vendor/pyrealsense2. Needed because the PyPI pyrealsense2-macosx wheel segfaults
# when it opens the camera as root on macOS 26.
set -euo pipefail

VERSION="${1:-v2.58.4}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
WORK="${ROOT}/.build/librealsense"
SDK="$(xcrun --show-sdk-path)"
PY="${ROOT}/.venv/bin/python"
DEST="${ROOT}/vendor/pyrealsense2/src/pyrealsense2"
MAJOR_MINOR="$(echo "${VERSION#v}" | cut -d. -f1-2)"

if [ ! -d "${WORK}" ]; then
  git clone --depth 1 --branch "${VERSION}" https://github.com/realsenseai/librealsense.git "${WORK}"
fi

# This machine's Command Line Tools lack the toolchain libc++ headers, so point
# clang at the SDK's copy.
cmake -S "${WORK}" -B "${WORK}/build" -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_OSX_SYSROOT="${SDK}" "-DCMAKE_CXX_FLAGS=-isystem ${SDK}/usr/include/c++/v1" \
  -DBUILD_PYTHON_BINDINGS=ON -DPYTHON_EXECUTABLE="${PY}" -DPython_EXECUTABLE="${PY}" \
  -DBUILD_EXAMPLES=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF -DBUILD_TOOLS=OFF \
  -DBUILD_UNIT_TESTS=OFF -DCHECK_FOR_UPDATES=OFF -DBUILD_WITH_OPENMP=OFF \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5
cmake --build "${WORK}/build" -j"$(sysctl -n hw.ncpu)"

REL="${WORK}/build/Release"
SO="$(ls "${REL}"/pyrealsense2.*.*.*.cpython-312-darwin.so | head -1)"
DYLIB="$(ls "${REL}"/librealsense2.*.*.*.dylib | head -1)"
mkdir -p "${DEST}"
cp "${SO}" "${DEST}/pyrealsense2.cpython-312-darwin.so"
cp "${DYLIB}" "${DEST}/librealsense2.${MAJOR_MINOR}.dylib"
cp "${WORK}/wrappers/python/pyrealsense2/__init__.py" "${DEST}/"
install_name_tool -delete_rpath "${REL}" "${DEST}/pyrealsense2.cpython-312-darwin.so" || true
install_name_tool -add_rpath @loader_path "${DEST}/pyrealsense2.cpython-312-darwin.so"
codesign -f -s - "${DEST}"/*.so "${DEST}"/*.dylib
echo "Installed pyrealsense2 ${VERSION} into ${DEST}"
