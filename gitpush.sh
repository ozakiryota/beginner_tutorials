#!/bin/bash

if [ $# -ne 1 ]; then
	comment="comment: $1"
	echo $1
else
	comment="updated"
fi

git add .
git commit -m $comment
git push origin master
