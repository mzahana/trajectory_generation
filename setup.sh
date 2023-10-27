#!/bin/bash -e


if [ -z "${ROS2_WS}" ]; then
  echo "Error: ROS2_WS environment variable is not set. Set it using export ROS2_WS=<deirectory_of_ros2_ws>"
  exit 1
fi
echo "Path of ROS2_WS is $ROS2_WS" echo

if [ -z "${OSQP_SRC}" ]; then
  echo "Error: OSQP_SRC environment variable is not set. Set it using export OSQP_SRC=<deirectory_that_will_osqp_source_code>"
  exit 1
fi
echo "Path of OSQP_SRC is  $OSQP_SRC" && echo  


if [ ! -d "$ROS2_WS/src/custom_trajectory_msgs" ];then
    cd $ROS2_WS/src/
    git clone https://github.com/mzahana/custom_trajectory_msgs.git -b ros2_humble
else
    cd $ROS2_WS/src/custom_trajectory_msgs
    git checkout ros2_humble && git pull origin ros2_humble
fi

# osqp
if [ ! -d "$OSQP_SRC" ];then
    mkdir -p $OSQP_SRC
fi

if [ ! -d "$OSQP_SRC/osqp" ]; then
    cd $OSQP_SRC
    git clone --recursive https://github.com/osqp/osqp.git
    cd osqp && git checkout 25b6b39
    git submodule update --recursive
else
    cd $OSQP_SRC/osqp && git checkout 25b6b39
    git submodule update --recursive
fi
cd $OSQP_SRC/osqp && rm -rf build
mkdir build
cd build
cmake -G "Unix Makefiles" ..
cmake --build .
if [ -n "$SUDO_PASSWORD" ]; then
    echo $SUDO_PASSWORD | sudo -S cmake --build . --target install
else
    sudo cmake --build . --target install
fi

# osqqp-eigen: https://github.com/robotology/osqp-eigen
if [ ! -d "$OSQP_SRC/osqp-eigen" ]; then
    cd $OSQP_SRC
    git clone https://github.com/robotology/osqp-eigen.git
    cd $OSQP_SRC/osqp-eigen && git checkout v0.8.0
else
    cd $OSQP_SRC/osqp-eigen && git checkout v0.8.0
fi
cd $OSQP_SRC/osqp-eigen
rm -rf build install
mkdir build && mkdir install
cd build
# cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX:PATH=$HOME/src/osqp-eigen/install ../
cmake -DCMAKE_BUILD_TYPE=Release ../
make
if [ -n "$SUDO_PASSWORD" ]; then
    echo $SUDO_PASSWORD | sudo -S make install
else
    sudo make install
fi

# bashrc_file="$HOME/.bashrc"
# line_to_check="export OsqpEigen_DIR=$OSQP_SRC/osqp-eigen/install"

# if ! grep -qF "$line_to_check" "$bashrc_file"; then
#     echo "$line_to_check" >> "$bashrc_file"
#     echo "OsqpEigen_DIR is added to .bashrc file."
# else
#     echo "OsqpEigen_DIR already exists in .bashrc file. No changes made."
# fi

cd $ROS2_WS && colcon build --packages-up-to trajectory_generation