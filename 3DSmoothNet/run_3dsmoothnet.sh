#!/bin/bash
ENV="./env"
cd "/home/tavu/workspace/ICS-Fusion/3DSmoothNet"
# cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 
source ${ENV}/bin/activate

LOGS="./data/logs/main_cnn.log"

python ./python/main_cnn.py --run_mode=test --evaluate_input_file="$1" --evaluate_output_file="$2" >"$LOGS" 2>&1 
cd -