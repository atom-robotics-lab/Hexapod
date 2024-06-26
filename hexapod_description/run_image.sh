x   #!/bin/bash

DOCKER_ARGS+=("-e NVIDIA_VISIBLE_DEVICES=all")
DOCKER_ARGS+=("-e NVIDIA_DRIVER_CAPABILITIES=all")
gpu=$(lspci | grep -i '.* vga .* nvidia .*')


xhost +local:root

image_name="hexapod_image"
container_name="hexapod_atom"

if docker ps --format '{{.Names}}' | grep -q "$container_name"; then
    docker exec -it robocon_atom /bin/bash
else
    if [[ $gpu == *'NVIDIA'* ]]; then
    sudo docker run hello-world
    printf 'Nvidia GPU is present:  %s\n' "$gpu"
    docker run -it --rm \
        ${DOCKER_ARGS[@]} \
        -e DISPLAY=$DISPLAY \
        -v $PWD/build_files:/workspaces/robocon_ws/ \
        -v $PWD:/workspaces/robocon_ws/src \
        -v /var/run/docker.sock:/var/run/docker.sock \
        --name "$container_name" \
        --workdir /workspaces/robocon_ws/src \
        -v $PWD/ddsconfig.xml:/ddsconfig.xml \
        # --env CYCLONEDDS_URI=/ddsconfig.xml \
        --runtime nvidia \
        --network host \
        $@ \
        "$image_name":1.0 \
        bash -c "./welcome.sh"
    else
    printf 'Nvidia GPU is not present: %s\n' "$gpu"
    docker run -it --rm \
        -e DISPLAY=$DISPLAY \
        -v $PWD/build_files:/workspaces/robocon_ws/ \
        -v $PWD:/workspaces/robocon_ws/src \
        -v /var/run/docker.sock:/var/run/docker.sock \
        --name "$container_name" \
        --workdir /workspaces/robocon_ws/src \
        -v $PWD/ddsconfig.xml:/ddsconfig.xml \
        --env CYCLONEDDS_URI=/ddsconfig.xml \
        --network host \
        $@ \
        "$image_name":1.0 \
        bash -c "./welcome.sh"
    fi
    
fi