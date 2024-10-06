#!/bin/sh

set -e

cc -framework Cocoa -framework AVFoundation -g main.m -o main
mkdir -p main.app/Contents/MacOS
cp -f Info.plist main.app/Contents
cp -rf main main.dSYM main.app/Contents/MacOS