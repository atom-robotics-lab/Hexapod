#!/bin/bash

# Map host's display socket to docker
DOCKER_ARGS+=("-v /tmp/.X11-unix:/tmp/.X11-unix")
DOCKER_ARGS+=("-v $HOME/.Xauthority:/home/admin/.Xauthority:rw")
# DOCKER_ARGS+=("-e DISPLAY")
DOCKER_ARGS+=("-e NVIDIA_VISIBLE_DEVICES=all")
DOCKER_ARGS+=("-e NVIDIA_DRIVER_CAPABILITIES=all")
gpu=$(lspci | grep -i '.* vga .* nvidia .*')

# DOCKER_ARGS+=("-e FASTRTPS_DEFAULT_PROFILES_FILE=/usr/local/share/middleware_profiles/rtps_udp_profile.xml")

image_name="hexapod_image"
container_name="hexapod_atom"

# Initialize variables
force_option=false 
clean_option=false


# Parse options
while [[ $# -gt 0 ]]; do
  case "$1" in
    --force)
      force_option=true
      shift
      ;;

    --clean)
      clean_option=true
      shift
      ;;
      
    *)
      echo "Invalid option: $1"
      exit 1
      ;;
  esac
done


if $force_option; then
  echo "Buidling Existing Docker Image: $image_name"
  docker build -f Dockerfile -t "$image_name":1.0 .
  # ./build_image.sh

else
  run_command='source /opt/ros/humble/setup.bash && colcon build --symlink-install && source install/setup.bash && exit'

  if $clean_option; then
    run_command='source /opt/ros/humble/setup.bash && rm -rf build log install && colcon build --symlink-install && source install/setup.bash && exit'
    echo "Force Command Enabled"

  else
    echo "Force Command Disabled"
  fi


  if docker images --format '{{.Repository}}' | grep -q "$image_name"; then

      echo "Found Docker Image: $image_name:1.0"

      echo "Updating the existing Docker image: $image_name:1.0"

    if [[ $gpu == *'NVIDIA'* ]]; then
    printf 'Nvidia GPU is present:  %s\n' "$gpu"
    docker run -it --rm \
        ${DOCKER_ARGS[@]} \
        -e DISPLAY=$DISPLAY \
        -v $PWD/build_files:/workspaces/hexapod_ws/ \
        -v $PWD:/workspaces/hexapod_ws/src \
        -v /var/run/docker.sock:/var/run/docker.sock \
        --name "$container_name" \
        --workdir /workspaces/hexapod_ws \
        -v $PWD/ddsconfig.xml:/ddsconfig.xml \
        --env CYCLONEDDS_URI=/ddsconfig.xml \
        --runtime nvidia \
        --network host \
        $@ \
        "$image_name":1.0 \
        bash -c "$run_command"

    else
    printf 'Nvidia GPU is not present: %s\n' "$gpu"
    docker run -it --rm \
        -e DISPLAY=$DISPLAY \
        -v $PWD/build_files:/workspaces/hexapod_ws/ \
        -v $PWD:/workspaces/hexapod_ws/src \
        -v /var/run/docker.sock:/var/run/docker.sock \
        --name "$container_name" \
        --workdir /workspaces/hexapod_ws \
        -v $PWD/ddsconfig.xml:/ddsconfig.xml \
        --env CYCLONEDDS_URI=/ddsconfig.xml \
        --network host \
        $@ \
        "$image_name":1.0 \
          bash -c "$run_command"
    fi
            
  else
      echo "Building a new Docker image: $image_name"
      
      docker build -f Dockerfile -t "$image_name":1.0 .
      ./build.sh
  fi

fi