#!/bin/bash

# Get base.sh funcs
source "$(dirname "$0")/base.sh"

stop_docker

# declare mode, use cpu by default
mode="cpu"

# declare sim, use sim by default
sim="True"

while getopts 'ch' opt; do
    case "$opt" in
        c)
            mode="cpu"
            ;;
        ?|h)
            echo "Usage: $(basename $0) [-c]"
            exit 1
            ;;
    esac
done
shift "$(($OPTIND -1))"

# Set DISPLAY environment variable
export DISPLAY=10.255.255.254:0

# If the mode is GPU, run Docker with NVIDIA runtime
if [ "$mode" == "gpu" ]; then
    run_docker --runtime=nvidia \
    -e DISPLAY=10.255.255.254:0 \
    -v $(dirname "$0")/../../workspace/:/root/workspace/src \
    limo_bot:sim bash
else
    run_docker \
    -e DISPLAY=10.255.255.254:0 \
    -v $(dirname "$0")/../../workspace/:/root/workspace/src \
    limo_bot:sim bash
fi
