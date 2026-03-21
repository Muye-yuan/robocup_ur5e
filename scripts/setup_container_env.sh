#!/bin/bash
# ============================================================================
# ROS Container Environment Setup Script
# ============================================================================
# This script configures the ROS environment inside Docker containers.
# Usage: source /workspace/scripts/setup_container_env.sh
# ============================================================================

set -e

echo "================================================"
echo "🚀 Setting up ROS Environment"
echo "================================================"

# Source ROS Noetic
if [ -f /opt/ros/noetic/setup.bash ]; then
    source /opt/ros/noetic/setup.bash
    echo "✅ ROS Noetic sourced"
else
    echo "❌ ERROR: ROS Noetic not found!"
    return 1
fi

# Source workspace
if [ -f /workspace/devel/setup.bash ]; then
    source /workspace/devel/setup.bash
    echo "✅ Workspace sourced"
else
    echo "⚠️  WARNING: Workspace not built yet (run 'catkin build' first)"
fi

# Mounted robot description packages live outside the generated workspace
# metadata, so expose them explicitly for rospack and MoveIt.
for extra_pkg in /workspace/src/ur_description /workspace/src/robotiq_description; do
    if [ -d "$extra_pkg" ]; then
        export ROS_PACKAGE_PATH="${extra_pkg}:${ROS_PACKAGE_PATH}"
        echo "✅ Added to ROS_PACKAGE_PATH: $extra_pkg"
    fi
done

# Configure ROS network (for host network mode)
export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=127.0.0.1

echo ""
echo "📡 ROS Network Configuration:"
echo "   ROS_MASTER_URI: $ROS_MASTER_URI"
echo "   ROS_IP:         $ROS_IP"
echo ""

# Test ROS Master connection
echo "🔍 Testing ROS Master connection..."
if timeout 2 rostopic list &>/dev/null; then
    echo "✅ ROS Master is reachable!"
    echo ""
    echo "📋 Available topics:"
    rostopic list | head -n 10
    TOPIC_COUNT=$(rostopic list | wc -l)
    if [ $TOPIC_COUNT -gt 10 ]; then
        echo "   ... and $((TOPIC_COUNT - 10)) more topics"
    fi
else
    echo "⚠️  WARNING: Cannot connect to ROS Master"
    echo "   Make sure Gazebo is running: docker-compose up gazebo"
fi

echo ""
echo "================================================"
echo "✅ Environment setup complete!"
echo "================================================"
echo ""
echo "💡 Quick commands:"
echo "   rostopic list          - List all ROS topics"
echo "   rosnode list           - List all ROS nodes"
echo "   rosrun robocup_brain test_ur5e_control.py - Run arm test"
echo ""
