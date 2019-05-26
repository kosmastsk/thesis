#!/bin/bash

# File path
cp $1

for typ in "meander" "spiral"
do
	for speed in "slow" "normal" "fast"
	do
		for world in "warehouse" "box"
		do
			for i in 1 2 3 4 5
			do
				python evaluate_localization.py $1/$typ/$speed/$world/$i"_truth.csv" $1/$typ/$speed/$world/$i"_amcl.csv"
			done
		done
	done
done

for typ in "line"
do
	for speed in "slow" "normal" "fast"
	do
		for world in "corridor" "warehouse"
		do
			for i in 1 2 3 4 5
			do
				python evaluate_localization.py $1/$typ/$speed/$world/$i"_truth.csv" $1/$typ/$speed/$world/$i"_amcl.csv"
			done
		done
	done
done
