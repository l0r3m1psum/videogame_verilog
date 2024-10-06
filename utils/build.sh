#!/bin/sh

set -e

cc -framework Cocoa -framework AVFoundation -g viewer.m -o viewer
mkdir -p viewer.app/Contents/MacOS
cp -f Info.plist viewer.app/Contents
cp -rf viewer viewer.dSYM viewer.app/Contents/MacOS
