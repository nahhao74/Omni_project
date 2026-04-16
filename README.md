# Autonomous Omni-directional Hospital Navigation Robot

**Tác giả:** Nguyễn Anh Hào & Trần Minh Cương

**Email:** nahhao74@gmail.com

Dự án này triển khai hệ thống điều hướng tự hành toàn diện cho một robot đa hướng (omni-directional robot) hoạt động trong môi trường mô phỏng bệnh viện (AWS Hospital World). Hệ thống được xây dựng trên nền tảng ROS 2 Jazzy và thư viện Nav2 stack trong môi trường mô phỏng Gazebo.

Điểm nhấn của dự án nằm ở việc giải quyết bài toán động học phức tạp của xe omni trong không gian hẹp. Hệ thống sử dụng Smac A* làm Global Planner để tạo ra các đường đi an toàn, có tính toán đến footprint của robot. Ở cấp độ điều khiển, dự án kết hợp Rotation Shim Controller để xử lý các góc quay ưu tiên, và MPPI Controller để dự đoán và lấy mẫu quỹ đạo trong thời gian thực, giúp robot lách qua các vật cản động một cách mượt mà nhất.

# ⚙️ Setup

1. Yêu cầu hệ thống:

    Ubuntu 24.04 (Noble Numbat)

    ROS 2 Jazzy (Desktop Install)

    Gazebo Simulator (tương thích với Jazzy)

    AWS Robotic Hospital World packages

2. Tạo Workspace và cài đặt thư viện:
Bash

- Tạo ROS 2 workspace
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src

- Clone repository của dự án
git clone https://github.com/nahhao74/Omni_project.git
