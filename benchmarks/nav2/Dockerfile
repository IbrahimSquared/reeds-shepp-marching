FROM ros:humble-ros-base-jammy@sha256:09da889006b4d4f120ada9b788394d566818f9e451f59a3a9246a1f9eecf849b

ARG NAV2_SMAC_DEB_VERSION=1.1.20-1jammy.20260804.211106
ARG NAV2_COSTMAP_DEB_VERSION=1.1.20-1jammy.20260804.204611

RUN apt-get update && apt-get install -y --no-install-recommends \
      libpng-dev \
      nlohmann-json3-dev \
      python3-colcon-common-extensions \
      ros-humble-nav2-costmap-2d=${NAV2_COSTMAP_DEB_VERSION} \
      ros-humble-nav2-smac-planner=${NAV2_SMAC_DEB_VERSION} \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/rs_marching_benchmark
COPY prepare_nav2.py prefix_observer.hpp prefix_observer.cpp ./
RUN python3 prepare_nav2.py \
    && . /opt/ros/humble/setup.sh \
    && cmake -S /opt/nav2_source -B /opt/nav2_build \
      -DBUILD_TESTING=OFF -DCMAKE_INSTALL_PREFIX=/opt/nav2_observed \
    && cmake --build /opt/nav2_build -j2 \
    && cmake --install /opt/nav2_build \
    && rm -rf /opt/nav2_source /opt/nav2_build

COPY CMakeLists.txt package.xml runner.cpp prefix_observer_test.cpp ./
RUN . /opt/ros/humble/setup.sh \
    && export LD_LIBRARY_PATH=/opt/nav2_observed/lib:$LD_LIBRARY_PATH \
    && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
      -Dnav2_smac_planner_DIR=/opt/nav2_observed/share/nav2_smac_planner/cmake \
    && cmake --build build -j2 \
    && ctest --test-dir build --output-on-failure \
    && install -m 0755 build/nav2_benchmark_runner /usr/local/bin/nav2_benchmark_runner

LABEL org.rs-marching.benchmark="nav2-smac-hybrid-lattice" \
      org.ros-distro="humble" \
      org.nav2-version="1.1.20"

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && export LD_LIBRARY_PATH=/opt/nav2_observed/lib:$LD_LIBRARY_PATH && exec \"$@\"", "--"]
CMD ["/usr/local/bin/nav2_benchmark_runner"]
