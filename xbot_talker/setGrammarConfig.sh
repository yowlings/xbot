#!/bin/bash

echo "Start setting ..."

basepath=$(cd `dirname $0`; pwd)
sourcepath=$basepath"/userconfig"
savepath=$basepath"/cache/grammar_config"

if [ ! -d $savepath ]; then
  mkdir -p ${savepath}
else
  echo "Folder exist"
fi

echo "Your source files are from: "$sourcepath
echo "Your config files will save at: "$savepath

cd $sourcepath
for file in *.txt *.bnf; do
  cp -u $file $savepath
done
cd $basepath"/cache"
mkdir wav
mkdir pcm
mkdir log
echo "Setting finish!"
