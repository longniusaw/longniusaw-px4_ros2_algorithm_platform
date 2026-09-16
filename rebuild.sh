#!/bin/bash
echo "🔨 开始编译 algorithm_platform..."
cd ~/algorithm_platform_ws || exit 1

CLEAN_BUILD=true
if [ "$1" == "--no-clean" ]; then
    CLEAN_BUILD=false
    echo "⚠️  跳过清理缓存（使用 --no-clean）"
fi

if [ "$CLEAN_BUILD" = true ]; then
    echo "🧹 清理旧的编译缓存（build/ install/ log/）..."
    rm -rf build/ install/ log/
    echo "✅ 清理完成"
fi

echo "📡 加载 px4_msgs 环境..."
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
    echo "✅ px4_msgs 环境已加载"
else
    echo "❌ 错误: 未找到 ~/ros2_ws/install/setup.bash"
    echo "    请先编译ros2_ws中的px4_msgs"
    return 1
fi

echo "📦 开始编译 algorithm_platform..."
colcon build \
    --packages-select algorithm_platform \
    --symlink-install

if [ $? -eq 0 ]; then
    echo "✅ 编译成功，正在加载当前工作空间环境到终端..."
    source install/setup.bash
    echo "🚀 环境就绪！直接启动launch即可"
    echo ""
    echo "📌 启动命令："
    echo "  ros2 launch algorithm_platform algws_alg_launch.py"
else
    echo "❌ 编译失败，请检查错误信息。"
    return 1
fi
