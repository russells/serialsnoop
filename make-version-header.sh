#!/bin/sh

set -e
Version=`head -1 VERSION`
[ -n $Version ]

< version-template.h > version.h \
sed "s/@VERSION/$Version/"
