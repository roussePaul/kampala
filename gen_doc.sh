#!/bin/bash

echo "Execute this file in the kampala folder"

pack=`cat kampala/package.xml | sed 's/^.*<run_depend>\(.*\)<\/run_depend>$/\1/p' -n`
for f in $pack; do
	rosdoc_lite -o ../../doc/$f $f
done