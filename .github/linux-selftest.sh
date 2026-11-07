#!/bin/bash
# The packaged Linux app, run on a bare Ubuntu (the image given) with only the
# libraries the README lists, on a virtual X display: open a board, route it,
# export the session, take a screenshot (``self_test``). Run in a container by
# the build workflow, from the checkout with the built app in dist/.
set -u
version=$(. /etc/os-release && echo "$VERSION_ID")
mkdir -p selftest
apt-get update -qq
glib=libglib2.0-0t64
apt-cache show "$glib" > /dev/null 2>&1 || glib=libglib2.0-0   # (the name before Ubuntu 24.04)
DEBIAN_FRONTEND=noninteractive apt-get install -y -qq --no-install-recommends xvfb xauth \
  libegl1 libgl1 "$glib" libfontconfig1 libdbus-1-3 libxkbcommon-x11-0 \
  libxcb-cursor0 libxcb-icccm4 libxcb-keysyms1 libxcb-shape0 libxcb-randr0 libxcb-render-util0 \
  libxcb-image0 libxcb-xinerama0 libxcb-xkb1 > /dev/null || exit 1
echo "Ubuntu $version: libraries the app cannot find:"
find dist/WeaveEngine -name "*.so*" -type f -exec ldd {} \; 2>/dev/null | grep "not found" | sort -u
log="selftest/linux-$version.log"
timeout -k 10 600 xvfb-run -a dist/WeaveEngine/WeaveEngine --self-test "boards/Word of RAM.dsn" \
  --ses "selftest/linux-$version.ses" --screenshot "selftest/linux-$version.png" > "$log" 2>&1
status=$?
cat "$log"
if [ $status -ne 0 ]; then
  why="ended with $status"
  [ $status -eq 124 ] && why="did not finish in 10 minutes"
  # (an annotation, readable without signing in: the end of what the app printed)
  echo "::error title=Linux self-test on Ubuntu $version $why::$(tail -c 3000 "$log" | sed -e 's/%/%25/g' | sed -e ':a' -e 'N' -e '$!ba' -e 's/\n/%0A/g')"
fi
exit $status
