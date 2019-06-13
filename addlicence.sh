#!/bin/bash

for i in $(find . -name '*.h')
do
  if ! grep -q Copyright $i
  then
    cat license.txt $i >$i.new && mv $i.new $i
  fi
done
