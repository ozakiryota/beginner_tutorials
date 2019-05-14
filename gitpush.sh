#!/bin/bash

if [ $# -ne 0 ]; then
	comment="$*"
else
	comment="updated"
fi

echo "comment: $comment"
git add .
git commit -m "$comment"
git push origin master
