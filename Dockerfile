FROM ros:humble-ros-base-jammy

RUN apt-get update \
    && apt-get install -y --no-install-recommends ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/* \
    && useradd --create-home --uid 10001 participant

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

COPY --chmod=0755 entrypoint.sh /entrypoint.sh

USER participant
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ping"]
