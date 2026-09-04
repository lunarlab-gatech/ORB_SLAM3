# One mount for the whole repo (matches Dockerfile's WORKDIR), one for the
# external datasets. No --gpus: nothing here uses CUDA (README: RGB-D loaders
# run headless, Pangolin viewer disabled) — X11 forwarding is kept (via the mounted
# XAUTHORITY cookie below) in case you enable the viewer.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
DATA_DIR="/media/$USER/T73/"

docker run --init -it \
    --name="orbslam3" \
    --shm-size=2gb \
    --net="host" \
    --privileged \
    --workdir="/home/$USER/orb_slam3_ws" \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XAUTHORITY=/tmp/.Xauthority" \
    --env="XDG_RUNTIME_DIR=/tmp/runtime-$USER" \
    --env="USER_ID=$(id -u)" \
    --env="GROUP_ID=$(id -g)" \
    --volume="$WS_ROOT:/home/$USER/orb_slam3_ws" \
    --volume="$DATA_DIR:/home/$USER/data" \
    --volume="/home/$USER/.bash_aliases:/home/$USER/.bash_aliases" \
    --volume="/home/$USER/.ssh:/home/$USER/.ssh:ro" \
    --volume="/home/$USER/.gnupg:/home/$USER/.gnupg" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$XAUTHORITY:/tmp/.Xauthority:ro" \
    --volume="/etc/localtime:/etc/localtime:ro" \
    --volume="/etc/timezone:/etc/timezone:ro" \
    --volume /tmp/runtime-$USER:/tmp/runtime-$USER \
    orbslam3-ubuntu22 \
    /bin/bash
