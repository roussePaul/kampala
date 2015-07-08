#!/bin/sh


cat $1 | sed -e "s/iris1/$2/g" > $3

echo $3