#!/bin/bash

echo "Execute this file in the kampala folder"

pack=`cat kampala/package.xml | sed 's/^.*<run_depend>\(.*\)<\/run_depend>$/\1/p' -n`
for f in $pack; do
	echo -e "\033[31m\e[1mGenerate documentation for \e[30;48;5;82m$f\e[0m"
	rosdoc_lite -o ../../doc/$f $f
done
