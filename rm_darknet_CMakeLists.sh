#!/bin/sh
SCRIPT_DIR=$(cd $(dirname $0); pwd)
rm $SCRIPT_DIR/darknet/CMakeLists.txt
rm $SCRIPT_DIR/darknet/src/csharp/CMakeLists.txt