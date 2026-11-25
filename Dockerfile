ARG RUNTIME_IMAGE=t2-runtime
FROM ros:humble-ros-base-jammy AS dependencies

ARG UNITREE_REVISION=668d1ec5a05d1c38d3306bdca7d59f2ba3581a88
ARG SIMULATOR_REVISION=abce1d8afe39e72f5287318e0e538b1cdfd909a8

RUN apt-get update \
    && apt-get install -y --no-install-recommends curl python3-pip build-essential \
        ros-humble-rosidl-default-generators ros-humble-rosidl-generator-dds-idl \
    && rm -rf /var/lib/apt/lists/*

RUN curl -fsSL --retry 3 -o /tmp/unitree.tar.gz "https://codeload.github.com/unitreerobotics/unitree_ros2/tar.gz/${UNITREE_REVISION}" \
    && mkdir /tmp/unitree_go \
    && tar -xzf /tmp/unitree.tar.gz -C /tmp/unitree_go --strip-components=5 \
        "unitree_ros2-${UNITREE_REVISION}/cyclonedds_ws/src/unitree/unitree_go" \
    && . /opt/ros/humble/setup.sh \
    && MAKEFLAGS=-j2 colcon build --base-paths /tmp/unitree_go --merge-install --install-base /opt/unitree \
        --packages-select unitree_go

RUN curl -fsSL --retry 3 -o /tmp/simulator.tar.gz "https://codeload.github.com/Urika000/go1-simulator-public/tar.gz/${SIMULATOR_REVISION}" \
    && mkdir -p /opt/go1/model \
    && tar -xzf /tmp/simulator.tar.gz -C /opt/go1/model --strip-components=4 \
        --exclude='*/go1.png' "go1-simulator-public-${SIMULATOR_REVISION}/models/mujoco/unitree_go1" \
    && python3 -m pip install --no-cache-dir --target /opt/go1/python 'numpy<2' mujoco==3.3.7

RUN tar -xOf /tmp/unitree.tar.gz "unitree_ros2-${UNITREE_REVISION}/LICENSE" \
    > /opt/unitree/share/unitree_go/LICENSE

FROM ros:humble-ros-base-jammy AS t2-runtime

RUN apt-get update \
    && apt-get install -y --no-install-recommends libgl1 libglfw3 ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/* \
    && useradd --create-home --uid 10001 participant

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    PYTHONPATH=/opt/go1/python

COPY --from=dependencies /opt/unitree /opt/unitree
COPY --from=dependencies /opt/go1 /opt/go1
COPY t2_node.py t2_safety.py /opt/go1/

FROM ${RUNTIME_IMAGE}
COPY t3_node.py t3_policy.py t3_weights.npz /opt/go1/
COPY LICENSE-WALK-THESE-WAYS /opt/go1/
COPY --chmod=0755 entrypoint.sh /entrypoint.sh

USER participant
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ping"]
