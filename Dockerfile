FROM ubuntu:22.04
RUN apt-get update -y && apt-get install -y \
    build-essential \
    cmake \
    git \
    libboost-all-dev \
    libssl-dev \
    libzmq3-dev \
    pkg-config \
    python3-dev \
    python3-pip
WORKDIR /workspace
RUN git clone -b release/v1.0 https://github.com/autowarefoundation/autoware.git
WORKDIR /workspace/autoware
RUN apt-get install -y python3-venv
RUN git config --global user.email all4dich@gmail.com
RUN git config --global user.name "Sunjoo Park"
RUN git revert 896fd14
RUN patch -p1 << 'EOF' 
diff --git a/ansible/roles/tensorrt/tasks/main.yaml b/ansible/roles/tensorrt/tasks/main.yaml
index df85ae7..388484b 100644
--- a/ansible/roles/tensorrt/tasks/main.yaml
+++ b/ansible/roles/tensorrt/tasks/main.yaml
@@ -20,6 +20,8 @@
       - libnvinfer-plugin-dev={{ tensorrt_version }}
       - libnvparsers-dev={{ tensorrt_version }}
       - libnvonnxparsers-dev={{ tensorrt_version }}
+      - libnvinfer-headers-dev={{ tensorrt_version }}
+      - libnvinfer-headers-plugin-dev={{ tensorrt_version }}
     allow_change_held_packages: true
     allow_downgrade: true
     update_cache: true
EOF
RUN ./setup-dev-env.sh -y
RUN mkdir src
RUN vcs import src < autoware.repos
RUN bash -c '. /opt/ros/humble/setup.bash && sudo apt update && sudo apt upgrade && rosdep update && rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO'
RUN sed -i 's/<angles\/angles\/angles/<angles\/angles/g' ./src/sensor_component/external/nebula/nebula_decoders/include/nebula_decoders/nebula_decoders_velodyne/decoders/velodyne_scan_decoder.hpp
RUN patch -p1 << 'EOF' 
diff --git a/src/universe/autoware.universe/perception/multi_object_tracker/CMakeLists.txt b/perception/multi_object_tracker/CMakeLists.txt
index 3e379bcfd1..055e414790 100644
--- a/src/universe/autoware.universe/perception/multi_object_tracker/CMakeLists.txt
+++ b/src/universe/autoware.universe/perception/multi_object_tracker/CMakeLists.txt
@@ -2,6 +2,7 @@ cmake_minimum_required(VERSION 3.14)
 project(multi_object_tracker)

 find_package(autoware_cmake REQUIRED)
+ament_auto_find_build_dependencies()
 autoware_package()

 # Ignore -Wnonportable-include-path in Clang for mussp
diff --git a/src/universe/autoware.universe/perception/multi_object_tracker/package.xml b/perception/multi_object_tracker/package.xml
index e3172dfd22..f343492b2c 100644
--- a/src/universe/autoware.universe/perception/multi_object_tracker/package.xml
+++ b/perception/multi_object_tracker/package.xml
@@ -24,7 +24,7 @@
   <depend>tier4_autoware_utils</depend>
   <depend>tier4_perception_msgs</depend>
   <depend>unique_identifier_msgs</depend>
-
+  <depend>diagnostic_updater</depend>
   <test_depend>ament_lint_auto</test_depend>
   <test_depend>autoware_lint_common</test_depend>
EOF
RUN apt-get install -y ros-humble-grid-map ros-humble-grid-map-core ros-humble-grid-map-cv ros-humble-grid-map-msgs ros-humble-grid-map-ros ros-humble-grid-map-rviz-plugin
RUN mkdir -p /opt/ros/humble/include/grid_map_core/eigen_plugins
RUN ln -s /opt/ros/humble/include/grid_map_core/grid_map_core/eigen_plugins/FunctorsPlugin.hpp /opt/ros/humble/include/grid_map_core/eigen_plugins/FunctorsPlugin.hpp
RUN ln -s /opt/ros/humble/include/grid_map_core/grid_map_core/eigen_plugins/Functors.hpp /opt/ros/humble/include/grid_map_core/eigen_plugins/Functors.hpp
RUN ln -s /opt/ros/humble/include/grid_map_core/grid_map_core/eigen_plugins/DenseBasePlugin.hpp /opt/ros/humble/include/grid_map_core/eigen_plugins/DenseBasePlugin.hpp
RUN bash -c '. /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'
