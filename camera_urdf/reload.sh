#!/bin/bash
# 改完 robot.urdf 后，运行本脚本重新生成 SDF 并重新 spawn 到 Gazebo

DIR="/home/dev/XTDrone/camera_urdf"
URDF="$DIR/robot.urdf"
SDF="$DIR/robot.sdf"
MODEL="my_gimbal"

# 1. URDF -> SDF（这一步也会顺带检查 URDF 语法是否合法）
echo "[1/4] 转换 robot.urdf -> robot.sdf ..."
cd "$DIR"
if ! gz sdf -p "$URDF" > "$SDF" 2>/tmp/reload_err.log; then
    echo "❌ URDF 转换失败，请检查语法："
    cat /tmp/reload_err.log
    exit 1
fi

# 2. mesh 相对路径 -> 绝对路径（否则 Gazebo 找不到 STL）
sed -i 's#<uri>meshes/#<uri>file:///home/dev/XTDrone/camera_urdf/meshes/#g' "$SDF"
echo "[2/4] mesh 路径已改为绝对路径"

# 3. 删除旧模型（不存在就忽略）
echo "[3/4] 删除旧模型 $MODEL ..."
rosservice call /gazebo/delete_model "model_name: '$MODEL'" >/dev/null 2>&1
sleep 1

# 4. 重新 spawn
echo "[4/4] 重新 spawn $MODEL ..."
rosrun gazebo_ros spawn_model -file "$SDF" -sdf -model "$MODEL"

echo ""
echo "✅ 完成。验证命令："
echo "   rostopic list | grep gimbal_camera"
echo "   rosrun image_view image_view image:=/gimbal_camera/image_raw"
