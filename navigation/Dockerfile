FROM ros:jazzy-ros-base@sha256:da725acf8b0f9f30c683e33ffbdcd6482d077af96d6fdc7688c5f4f280b7d923

ARG DEBIAN_FRONTEND=noninteractive
ARG VIZANTI_COMMIT=ab43643b2e12b6b7b66f0d53d6ba2ef3eacd3b42

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        git \
        python3-flask \
        python3-pytest \
        python3-waitress \
        ros-jazzy-nav2-bringup \
        ros-jazzy-nav2-loopback-sim \
        ros-jazzy-rosbridge-suite \
        ros-jazzy-rtabmap-ros \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/house_bot

RUN git clone --branch ros2 --single-branch https://github.com/MoffKalast/vizanti.git src/vizanti \
    && git -C src/vizanti checkout --detach "${VIZANTI_COMMIT}" \
    && test "$(git -C src/vizanti rev-parse HEAD)" = "${VIZANTI_COMMIT}"

COPY ros_ws/src/house_bot_navigation src/house_bot_navigation

RUN source /opt/ros/jazzy/setup.bash \
    && colcon build --symlink-install \
        --packages-up-to house_bot_navigation vizanti_server vizanti_cpp

COPY navigation/entrypoint.sh /opt/house_bot/entrypoint.sh
RUN chmod 0755 /opt/house_bot/entrypoint.sh

EXPOSE 5000 5001

ENTRYPOINT ["/opt/house_bot/entrypoint.sh"]
CMD ["ros2", "launch", "house_bot_navigation", "mock_navigation.launch.py"]
