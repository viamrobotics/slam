#!/bin/bash
set -o errexit

if [ "$(uname)" == "Linux" ]; then
    cd $(dirname $(readlink -f "${BASH_SOURCE}"))/..
elif [ "$(uname)" == "Darwin" ]; then
    readlinkorreal() { readlink "$1" || echo "$1"; }
    cd $(dirname string readlinkorreal "${BASH_SOURCE}")/..
else
    echo ERROR: your OS is not handled yet
    exit 1
fi

# ---- Edit based on your needs:
DATE="Oct17"

DATA_BASE_DIRECTORY="$HOME/viam/lidar_data"
MAPPING_DATA_DIRECTORY="$DATA_BASE_DIRECTORY/data_Feb_11_2022_small"
LOCALIZATION_DATA_DIRECTORY="$DATA_BASE_DIRECTORY/data_Feb_24_2022_printer_room"
UPDATE_DATA_DIRECTORY="$DATA_BASE_DIRECTORY/data_Feb_24_2022_printer_room"

DESCRIPTION="_refactor"
OUTPUT_DIRECTORY="pics${DESCRIPTION}_${DATE}"
MAP_OUTPUT_NAME="map${DESCRIPTION}_${DATE}.pbstream"

# ----

DATA_DIR=$MAPPING_DATA_DIRECTORY

mkdir -p output
cd output
rm -rf ${OUTPUT_DIRECTORY}
mkdir ${OUTPUT_DIRECTORY}

../build/main  \
    -data_dir=${DATA_DIR}  \
    -config_param="{mode=2D,}"  \
    -port=localhost:8083  \
    -sensors=[]  \
    -data_rate_ms=200 \
    -map_rate_sec=60 
