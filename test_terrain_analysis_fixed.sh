#!/bin/bash

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}=== terrain_analysis 修复验证测试 ===${NC}"

# 1. 检查ROS2环境
echo -e "${YELLOW}[1/4] 检查ROS2环境...${NC}"
if ! command -v ros2 &> /dev/null; then
    echo -e "${RED}✗ ROS2未安装或未在PATH中${NC}"
    exit 1
fi
echo -e "${GREEN}✓ ROS2环境正常${NC}"

# 2. 检查terrain_analysis包
echo -e "${YELLOW}[2/4] 检查terrain_analysis包...${NC}"
if ! ros2 pkg list | grep -q terrain_analysis; then
    echo -e "${RED}✗ terrain_analysis包未找到${NC}"
    exit 1
fi
echo -e "${GREEN}✓ terrain_analysis包存在${NC}"

# 3. 检查可执行文件
echo -e "${YELLOW}[3/4] 检查terrainAnalysis可执行文件...${NC}"
if ! ros2 run terrain_analysis terrainAnalysis --help &> /dev/null; then
    echo -e "${RED}✗ terrainAnalysis可执行文件有问题${NC}"
    exit 1
fi
echo -e "${GREEN}✓ terrainAnalysis可执行文件正常${NC}"

# 4. 运行测试（5秒）
echo -e "${YELLOW}[4/4] 运行terrainAnalysis节点测试（5秒）...${NC}"
echo -e "${BLUE}启动terrainAnalysis节点...${NC}"

# 使用timeout运行5秒
if timeout 5s ros2 run terrain_analysis terrainAnalysis; then
    echo -e "${GREEN}✓ terrainAnalysis节点运行正常，未发生segmentation fault${NC}"
    echo -e "${GREEN}✓ 修复验证成功！${NC}"
else
    EXIT_CODE=$?
    if [ $EXIT_CODE -eq 124 ]; then
        echo -e "${GREEN}✓ terrainAnalysis节点运行5秒后正常退出（timeout）${NC}"
        echo -e "${GREEN}✓ 修复验证成功！${NC}"
    elif [ $EXIT_CODE -eq -11 ]; then
        echo -e "${RED}✗ terrainAnalysis节点发生segmentation fault${NC}"
        echo -e "${RED}✗ 修复验证失败${NC}"
        exit 1
    else
        echo -e "${YELLOW}⚠ terrainAnalysis节点以退出码 $EXIT_CODE 退出${NC}"
        echo -e "${YELLOW}⚠ 需要进一步检查${NC}"
    fi
fi

echo -e "${BLUE}=== 测试完成 ===${NC}" 