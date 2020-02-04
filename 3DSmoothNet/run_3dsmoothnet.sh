#!/bin/bash
ENV="./env"
cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 
source ${ENV}/bin/activate
python ./python/main_cnn.py --run_mode=test --evaluate_input_file="$1" --evaluate_output_file="$2"
cd -