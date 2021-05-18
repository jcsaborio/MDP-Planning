#!/bin/sh

inputFile=../../Maze/maze.prob
outputFile=output.txt
minSims=1
maxSims=16
numSteps=50
runs=100
verbose=0

./uctMaze --inputFile $inputFile --outputFile $outputFile --minSims $minSims --maxSims $maxSims --numSteps $numSteps --runs $runs --verbose $verbose

