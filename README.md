# Autonomous Omni-directional Hospital Navigation Robot

**Tác giả:** Nguyễn Anh Hào & Trần Minh Cương

**Email:** nahhao74@gmail.com

Dự án này triển khai hệ thống điều hướng tự hành toàn diện cho một robot đa hướng (omni-directional robot) hoạt động trong môi trường mô phỏng bệnh viện (AWS Hospital World). Hệ thống được xây dựng trên nền tảng ROS 2 Jazzy và thư viện Nav2 stack trong môi trường mô phỏng Gazebo.

Điểm nhấn của dự án nằm ở việc giải quyết bài toán động học phức tạp của xe omni trong không gian hẹp. Hệ thống sử dụng Smac A* làm Global Planner để tạo ra các đường đi an toàn, có tính toán đến footprint của robot. Ở cấp độ điều khiển, dự án kết hợp Rotation Shim Controller để xử lý các góc quay ưu tiên, và MPPI Controller để dự đoán và lấy mẫu quỹ đạo trong thời gian thực, giúp robot lách qua các vật cản động một cách mượt mà nhất.

# ⚙️ Setup
1. Yêu cầu hệ thống:
    Ubuntu 24.04
    ROS 2 Jazzy
    Gazebo Simulator
    AWS Robotic Hospital World packages 
2. Tạo Workspace và cài đặt thư viện:
- Tạo ROS 2 workspace
```bash
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src
```
- Clone repository của dự án
```bash
git clone https://github.com/nahhao74/Omni_project.git
```
- Build
```bash
colcon build
```
- Khai báo đường dẫn
```bash
source install/setup.bash
```
3. Khởi chạy file launch
```bash
ros2 launch robot_omni localization_launch.py
```
# ✨ Tính năng & Demo
1. Xây dựng bản đồ trong môi trường phức tạp (SLAM & Mapping)
Sử dụng các thuật toán SLAM hiện đại trên ROS 2 để quét và xây dựng bản đồ Occupancy Grid 2D từ môi trường AWS Hospital World, xử lý nhiễu từ các hành lang dài và các phòng bệnh có cấu trúc giống nhau.
2. Lập kế hoạch đường đi toàn cục với Smac A* (Global Planning)
Smac A* Planner tính toán đường đi tối ưu nhất từ vị trí hiện tại đến phòng bệnh mục tiêu. Thuật toán cân nhắc chi phí về khoảng cách cũng như vùng rủi ro, đảm bảo quỹ đạo không bo góc quá gắt gây va chạm tường.
3. Điều hướng mượt mà với MPPI & Rotation Shim Controller (Local Control)
Rotation Shim giúp robot xoay hướng mặt về phía mục tiêu trước khi tiến. Ngay sau đó, MPPI Controller sẽ liên tục lấy mẫu hàng ngàn quỹ đạo để chọn ra hướng đi linh hoạt nhất, giúp đạt tới vị trí đích với đường đi tối ưu nhất và tránh va chạm.

## 🙏 Acknowledgments

Dự án này được phát triển dựa trên nền tảng của nhiều thư viện mã nguồn mở và tài liệu tham khảo từ cộng đồng ROS. Xin gửi lời cảm ơn đặc biệt đến:

* **Cartographer ROS / TurtleBot3:** Các file cấu hình Launch và SLAM (như `cartographer_node`) được xây dựng dựa trên mã nguồn gốc của tác giả Darby Lim (Bản quyền 2019 thuộc Open Source Robotics Foundation, Inc. - Giấy phép Apache 2.0). Cấu trúc gốc đã được tùy biến lại (remapping) để có thể hợp nhất dữ liệu từ hệ thống Lidar kép và Odometry cho robot di chuyển đa hướng.
* **Nav2 & MPPI:** https://docs.nav2.org/configuration/packages/configuring-mppic.html
* **AWS Hospital World:** https://github.com/aws-robotics/aws-robomaker-hospital-world.git
