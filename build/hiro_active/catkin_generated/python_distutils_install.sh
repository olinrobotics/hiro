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

echo_and_run cd "/home/ksuresh/hiro/src/hiro_active"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/ksuresh/hiro/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/ksuresh/hiro/install/lib/python3/dist-packages:/home/ksuresh/hiro/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/ksuresh/hiro/build" \
    "/usr/bin/python3" \
    "/home/ksuresh/hiro/src/hiro_active/setup.py" \
     \
    build --build-base "/home/ksuresh/hiro/build/hiro_active" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/ksuresh/hiro/install" --install-scripts="/home/ksuresh/hiro/install/bin"
