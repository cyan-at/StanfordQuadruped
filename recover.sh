#!/bin/bash

rm -fr .git
git init
git remote add origin https://github.com/cyan-at/StanfordQuadruped.git
git fetch
git reset --mixed origin/master
git branch --set-upstream-to=origin/master master