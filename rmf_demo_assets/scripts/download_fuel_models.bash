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
echo "----------------------------------------"

RAW_FUEL_MODELS_DIR=$HOME/.gazebo/fuel_models

function download_extract_rm {
  zip_filename=$(basename "$1")
  filename="${zip_filename%.*}"
  spaced_filename=${filename//%20/ }

  wget $1 -O /tmp/tmp.zip
  unzip /tmp/tmp.zip -d $RAW_FUEL_MODELS_DIR/"$spaced_filename"
  rm /tmp/tmp.zip
}

mkdir -p $RAW_FUEL_MODELS_DIR

while IFS= read -r url
do
  download_extract_rm $url
  echo "----------------------------------------"
done < $INPUT_FILE
