#!/bin/sh

set -e

cc -framework Cocoa -framework AVFoundation main.m -o main
mkdir -p main.app/Contents/MacOS
cp -f Info.plist main.app/Contents
cp -f main/Contents/MacOS