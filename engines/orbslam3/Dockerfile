# ORB-SLAM3 (commit 4452a3c, GPLv3) with Pangolin 0.9.2 on Ubuntu 22.04; RGB-D example, headless.
# Build:  docker build -t mv2/orbslam3:4452a3c engines/orbslam3      (20-60 min, ~4 GB)
FROM ubuntu:22.04
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake git ca-certificates pkg-config python3 \
      libeigen3-dev libopencv-dev libboost-serialization-dev libssl-dev \
      libglew-dev libepoxy-dev libgl1-mesa-dev \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /opt
RUN git clone --depth 1 --branch v0.9.2 https://github.com/stevenlovegrove/Pangolin.git \
 && cmake -S Pangolin -B Pangolin/build -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=OFF \
          -DBUILD_PANGOLIN_PYTHON=OFF -DBUILD_TESTS=OFF \
 && cmake --build Pangolin/build -j"$(nproc)" && cmake --install Pangolin/build && ldconfig \
 && rm -rf Pangolin
RUN git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git \
 && git -C ORB_SLAM3 checkout 4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4
COPY patch_orbslam3.sh build_orbslam3.sh /opt/mv2/
RUN bash /opt/mv2/build_orbslam3.sh /opt/ORB_SLAM3 && rm -rf /opt/ORB_SLAM3/build /opt/ORB_SLAM3/Thirdparty/*/build/CMakeFiles
ENV LD_LIBRARY_PATH=/opt/ORB_SLAM3/lib:/opt/ORB_SLAM3/Thirdparty/DBoW2/lib:/opt/ORB_SLAM3/Thirdparty/g2o/lib:/usr/local/lib
