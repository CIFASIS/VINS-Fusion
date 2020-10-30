#!/bin/bash
#
# Run Rosario dataset (from sequence 01 to 06)
# Parameter:
#       - Path of folder containing rosbags (files must be named sequence0*.bag)

DATASET_DIR=$1

dt=$(date '+%Y%m%d_%H%M%S')
OUTPUT_DIR=rosario_${dt}
mkdir $OUTPUT_DIR

for i in `seq 1 6`; do
  ./run_rosario_sequence.sh -r -o $OUTPUT_DIR/trajectory_rosario_0$i.txt $DATASET_DIR/sequence0$i.bag
done