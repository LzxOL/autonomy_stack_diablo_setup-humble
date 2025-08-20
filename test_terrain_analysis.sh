#!/bin/bash

# 显示彩色输出
GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}测试terrain_analysis节点${NC}"

# 检查ROS2环境
echo -e "${BLUE}[1/3] 检查ROS2环境...${NC}"
if ! command -v ros2 &> /dev/null; then
    echo -e "${RED}ROS2未安装或未在PATH中${NC}"
    exit 1
fi

# 检查terrain_analysis包
echo -e "${BLUE}[2/3] 检查terrain_analysis包...${NC}"
cd ~/autonomy_stack_diablo_setup-humble
source install/setup.bash

if ! ros2 pkg list | grep -q "terrain_analysis"; then
    echo -e "${RED}terrain_analysis包未找到${NC}"
    exit 1
fi

# 检查可执行文件
if ! ros2 run terrain_analysis terrainAnalysis --help &> /dev/null; then
    echo -e "${RED}terrainAnalysis可执行文件有问题${NC}"
    exit 1
fi

echo -e "${GREEN}terrain_analysis包检查通过${NC}"

# 测试运行（短暂运行以检查是否崩溃）
echo -e "${BLUE}[3/3] 测试运行terrain_analysis节点...${NC}"
echo -e "${YELLOW}注意：这将启动节点5秒钟来检查是否崩溃${NC}"

timeout 5s ros2 run terrain_analysis terrainAnalysis &
TERRAIN_PID=$!

# 等待5秒
sleep 5

# 检查进程是否还在运行
if kill -0 $TERRAIN_PID 2>/dev/null; then
    echo -e "${GREEN}✓ terrain_analysis节点运行正常，没有崩溃${NC}"
    kill $TERRAIN_PID 2>/dev/null
    echo -e "${GREEN}测试完成！节点可以正常运行。${NC}"
else
    echo -e "${RED}✗ terrain_analysis节点在5秒内崩溃了${NC}"
    echo -e "${YELLOW}建议检查：${NC}"
    echo -e "${YELLOW}1. 确保相机和SLAM节点正在运行${NC}"
    echo -e "${YELLOW}2. 检查点云话题是否正常发布${NC}"
    echo -e "${YELLOW}3. 检查odometry话题是否正常发布${NC}"
    exit 1
fi 