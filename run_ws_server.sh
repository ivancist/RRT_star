#!/bin/bash

# Find the container ID of the running oppi-path_planning image
container_id=$(docker ps -qf "ancestor=oppi-path_planning")

# Check if the container ID was found
if [ -z "$container_id" ]; then
    # No containers running

    # Run the Docker container with specific settings
  echo "Running the container..."
#    docker run -it --rm --net host --ipc host -e DISPLAY=$ip:0 -v /tmp/.X11-unix:/tmp/.X11-unix:rw ros-test-path_planning
    docker compose run --rm -p 9002:9002 path_planning /bin/bash -c "source /opt/ros/humble/setup.bash && /bin/bash"
    exit 1
fi

# Execute /bin/bash inside the container
echo "Container ID: $container_id"
docker exec -it $container_id /bin/bash -c "source /opt/ros/humble/setup.bash && /bin/bash"
