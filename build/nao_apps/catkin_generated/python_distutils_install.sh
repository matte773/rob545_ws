#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/matt/rob545_ws/src/nao_robot/nao_apps"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/matt/rob545_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/matt/rob545_ws/install/lib/python3/dist-packages:/home/matt/rob545_ws/build/nao_apps/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/matt/rob545_ws/build/nao_apps" \
    "/usr/bin/python3" \
    "/home/matt/rob545_ws/src/nao_robot/nao_apps/setup.py" \
     \
    build --build-base "/home/matt/rob545_ws/build/nao_apps" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/matt/rob545_ws/install" --install-scripts="/home/matt/rob545_ws/install/bin"
