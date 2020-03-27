#!/bin/bash

if [ "$#" -ne 1 ]; then
  echo "usage: ./download_fuel_models.bash TXT_FILE"
  echo "  where TXT_FILE is a list of Ignition Fuel URLs"
  exit
fi

INPUT_FILE=`pwd`/$1
echo "Downloading Ignition Models listed on file: [$INPUT_FILE]"

NUM_MODELS=$(cat $INPUT_FILE | wc -l)
echo "Found [$NUM_MODELS] models"

while IFS= read -r url
do
  spaced_url=${url//%20/ }
  ign fuel download -u "$spaced_url" -v 4
  echo "----------------------------------------"
done < $INPUT_FILE
