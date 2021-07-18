#!/bin/bash

PARTITION=$1
shift
MAX=$1
shift

used=$(df -Ph | grep '${PARTITION}' | awk {'print $5'})
if [ ${used%?} -ge ${MAX%?} ]; then
    echo "The Mount Point ${PARITION} on $(hostname) has used $used at $(date). Purging docker objects" 

    # Remove all unused containers, networks, images (both dangling and unreferenced), and volumes.
    docker system prune -af --volumes

    # Remove all buildx images (both dangling and unreferences)
    docker buildx prune -af
    
    # Remove all build cache (both unused and dangling)
    docker builder prune -af
fi

