if [ "$(docker inspect -f '{{.State.Running}}' orbslam3 2>/dev/null)" != "true" ]; then
    docker start orbslam3 2>/dev/null || (echo "Container not found. Run run_container.sh first." && exit 1)
fi

docker exec -it orbslam3 /bin/bash
