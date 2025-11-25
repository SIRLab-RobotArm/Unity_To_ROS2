#!/bin/bash

# ===== 색상 설정 =====
GREEN="\e[32m"
CYAN="\e[36m"
RESET="\e[0m"

echo -e "${GREEN}"
echo "=============================================="
echo "   ROS2 + MoveIt + Unity IK Bridge Launcher"
echo "=============================================="
echo -e "${RESET}"

# ===== 워크스페이스 경로 자동 설정 =====
WS_PATH=~/Dev/ROS2/vr_ik_test_ws

echo -e "${CYAN}▶ Sourcing ROS2 Humble...${RESET}"
source /opt/ros/humble/setup.bash

echo -e "${CYAN}▶ Sourcing Workspace: $WS_PATH${RESET}"
source $WS_PATH/install/setup.bash


# ===== TERMINAL 1 ====================================
echo -e "${GREEN}Launching Terminal 1: ros_tcp_endpoint${RESET}"
gnome-terminal -- bash -c "echo '[TCP ENDPOINT STARTED]'; ros2 run ros_tcp_endpoint default_server_endpoint; exec bash"


# ===== TERMINAL 2 ====================================
echo -e "${GREEN}Launching Terminal 2: IK Solver Node${RESET}"
gnome-terminal -- bash -c "echo '[IK SOLVER NODE STARTED]'; ros2 run ik_bridge_pkg dual_ik_solver; exec bash"


# ===== TERMINAL 3 ====================================
echo -e "${GREEN}Launching Terminal 3: robot_state_publisher (rsp.launch.py)${RESET}"
gnome-terminal -- bash -c "echo '[RSP LAUNCH STARTED]'; ros2 launch open_manipulator_x_moveit_config rsp.launch.py; exec bash"


# ===== TERMINAL 4 ====================================
echo -e "${GREEN}Launching Terminal 4: MoveIt move_group${RESET}"
gnome-terminal -- bash -c "echo '[MOVE_GROUP STARTED]'; ros2 launch open_manipulator_x_moveit_config move_group.launch.py; exec bash"


# ===== TERMINAL 5 ====================================
echo -e "${GREEN}Launching Terminal 5: Debug Terminal (topic echo)${RESET}"
gnome-terminal -- bash -c "echo '[DEBUG TERMINAL]'; ros2 topic echo /ik_joint_commands; exec bash"


echo -e "${GREEN}"
echo "=============================================="
echo "        🚀 All systems have been launched!"
echo "=============================================="
echo -e "${RESET}"

