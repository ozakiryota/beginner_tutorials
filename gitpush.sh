#!/bin/bash

if [ $# -ne 1 ]; then
	comment="$1"
else
	comment="updated"
fi

git add .
git commit -m $comment
git push origin master
