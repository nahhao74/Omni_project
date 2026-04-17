# PROJECT ROBOT OMNI
# ROS 2 Nav2: Autonomous Omni-directional Hospital Navigation Robot

Tác giả: Nguyễn Anh Hào & Trần Minh Cương
Email: nahhao74@gmail.com , tmcuong0507@gmail.com

Dự án này triển khai hệ thống điều hướng tự hành toàn diện cho một robot đa hướng (omni-directional robot) hoạt động trong môi trường mô phỏng bệnh viện (AWS Hospital World). Hệ thống được xây dựng trên nền tảng ROS 2 Jazzy và thư viện Nav2 stack trong môi trường mô phỏng Gazebo.

Điểm nhấn của dự án nằm ở việc giải quyết bài toán động học phức tạp của xe omni trong không gian hẹp. Hệ thống sử dụng Smac A* làm Global Planner để tạo ra các đường đi an toàn, có tính toán đến footprint của robot. Ở cấp độ điều khiển, dự án kết hợp Rotation Shim Controller để xử lý các góc quay ưu tiên, và MPPI Controller để dự đoán và lấy mẫu quỹ đạo trong thời gian thực, giúp robot lách qua các vật cản động một cách mượt mà nhất.
**Phạm vi của dự án bao gồm:**
1. Xây dựng mô hình robot và môi trường mô phỏng vật lý có độ tin cậy cao.
2. Xử lý tín hiệu đầu vào từ cảm biến (LiDAR, IMU, Camera) và dữ liệu Odometry.
3. Thiết lập hệ thống TF tree chuẩn xác.
4. Xây dựng bản đồ trong môi trường phức tạp (SLAM & Mapping) Sử dụng các thuật toán SLAM hiện đại trên ROS 2 để quét và xây dựng bản đồ Occupancy Grid 2D từ môi trường AWS Hospital World, xử lý nhiễu từ các hành lang      dài và các phòng bệnh có cấu trúc giống nhau.
5. Tinh chỉnh các tham số cấu hình của Nav2 để tối ưu hóa quỹ đạo di chuyển và khả năng tránh vật cản động/tĩnh.

# Setup
**1. Yêu cầu hệ thống:**

Ubuntu 24.04

ROS 2 Jazzy

Gazebo Simulator

AWS Robotic Hospital World packages

**2. Tạo Workspace và cài đặt thư viện:**

    Tạo ROS 2 workspace

mkdir -p ~/ros_ws/src
cd ~/ros_ws/src

    Clone repository của dự án

git clone https://github.com/nahhao74/Omni_project.git

    Build

colcon build

    Khai báo đường dẫn

source install/setup.bash

**3. Khởi chạy file launch**

ros2 launch robot_omni localization_launch.py

✨ Tính năng & Demo

    Xây dựng bản đồ trong môi trường phức tạp (SLAM & Mapping) Sử dụng các thuật toán SLAM hiện đại trên ROS 2 để quét và xây dựng bản đồ Occupancy Grid 2D từ môi trường AWS Hospital World, xử lý nhiễu từ các hành lang dài và các phòng bệnh có cấu trúc giống nhau.
    Lập kế hoạch đường đi toàn cục với Smac A* (Global Planning) Smac A* Planner tính toán đường đi tối ưu nhất từ vị trí hiện tại đến phòng bệnh mục tiêu. Thuật toán cân nhắc chi phí về khoảng cách cũng như vùng rủi ro, đảm bảo quỹ đạo không bo góc quá gắt gây va chạm tường.
    Điều hướng mượt mà với MPPI & Rotation Shim Controller (Local Control) Rotation Shim giúp robot xoay hướng mặt về phía mục tiêu trước khi tiến. Ngay sau đó, MPPI Controller sẽ liên tục lấy mẫu hàng ngàn quỹ đạo để chọn ra hướng đi linh hoạt nhất, giúp đạt tới vị trí đích với đường đi tối ưu nhất và tránh va chạm.

# Acknowledgments

Dự án này được phát triển dựa trên nền tảng của nhiều thư viện mã nguồn mở và tài liệu tham khảo từ cộng đồng ROS. Xin gửi lời cảm ơn đặc biệt đến:

    Cartographer ROS / TurtleBot3: Các file cấu hình Launch và SLAM (như cartographer_node) được xây dựng dựa trên mã nguồn gốc của tác giả Darby Lim (Bản quyền 2019 thuộc Open Source Robotics Foundation, Inc. - Giấy phép Apache 2.0). Cấu trúc gốc đã được tùy biến lại (remapping) để có thể hợp nhất dữ liệu từ hệ thống Lidar kép và Odometry cho robot di chuyển đa hướng.
    Nav2 & MPPI: https://docs.nav2.org/configuration/packages/configuring-mppic.html
    AWS Hospital World: https://github.com/aws-robotics/aws-robomaker-hospital-world.git

# Xây dưng mô hình robot
1. Hình dạng kích thước .
    Loại robot: Xe 4 bánh đa hướng (Omni/Mecanum).
    Trọng lượng và Kích thước: Phần thân chính (base_link) có khối lượng hơn 34 kg, kích thước bao phủ khoảng 0.71m x 0.49m x 0.16m. Hệ thống bánh xe: 4 bánh (trước-phải, trước-trái, sau-phải, sau-trái) được cấp lệnh vận tốc độc lập thông qua plugin gz_ros2_control.
2. Hệ thống Cảm biến (Sensors)
    Mẫu robot này được cấu hình hình để xây dựng bản đồ (SLAM) và điều hướng (Nav2), vì vậy cần phải có các cảm biến phù hợp:
    Hệ thống quét LiDAR kép (SICK TiM551): Xe được trang bị hai cảm biến quét laser, một đặt ở góc phía trước (base_front_laser) và một ở góc phía sau (base_rear_laser). Mỗi LiDAR có góc quét rộng lên tới 270 độ và tầm nhìn xa 25 mét (với noise tiêu chuẩn). Việc bố trí chéo nhau như thế này sinh ra là để tạo ra góc nhìn 360 độ hoàn hảo xung quanh xe, loại bỏ hoàn toàn điểm mù khi đưa vào Costmap.
    Hệ thống Camera Chiều sâu (RealSense D435): Xe có sẵn 2 camera RGBD. Một chiếc gắn ngay trên thân base và một chiếc gắn trên phần giá đỡ mở rộng (cameras_add_on_link) ở trên nóc. Rất phù hợp để nhận dạng vật thể 3D phía trước.
    Cảm biến quán tính (IMU): Cập nhật dữ liệu gia tốc và góc quay liên tục ở tần số cao 100Hz, giúp bộ lọc nhiễu odometry tính toán vị trí chuẩn xác hơn khi xe thực hiện các thao tác xoay phức tạp.

3. Tương thích Hệ thống
    ROS 2 & Gazebo: File này được viết để chạy trơn tru trên các phiên bản ROS 2 mới (có tham chiếu đến đường dẫn của bản Jazzy) và sử dụng các plugin hiện đại của Gazebo mới (Ignition/gz-sim) như **gz_ros2_control::GazeboSimSystem** hay **gz-sim-imu-system**.
<img width="927" height="681" alt="image" src="https://github.com/user-attachments/assets/f4e3c76a-80d7-4ed1-9f35-0749ab168c47" />


# Sensor Processing & Odometry Pipeline

Hệ thống robot xử lý dữ liệu từ cảm biến theo pipeline:


Gazebo Sensors
↓
ROS-GZ Bridge
↓
ROS2 Topics (/scan, /imu, /odom)
↓
EKF (robot_localization)
↓
/odometry/filtered
↓
SLAM / Navigation (Nav2)


---

## 1. Sensor Inputs

### LiDAR

**Topics:**
- `/scan_front_raw`
- `/scan_rear_raw`

**Data:**
- Khoảng cách đến vật cản theo từng góc

**Usage:**
- Phát hiện vật cản  
- Xây dựng bản đồ (SLAM)  
- Tránh va chạm  

---

### IMU

**Topic:**
- `/base_imu`

**Data:**
- Góc quay (yaw)  
- Vận tốc góc  
- Gia tốc  

**Usage:**
- Xác định hướng robot  
- Giữ ổn định góc quay  
- Hỗ trợ EKF  

---

### RGB-D Camera

**Topics:**
- `/camera/depth_image`
- `/camera/points`

**Data:**
- Ảnh độ sâu  
- PointCloud 3D  

**Usage:**
- Nhận diện vật thể  
- Phát hiện vật cản 3D  
- Bổ sung cho LiDAR  

---

### Wheel Odometry

**Topic:**
- `/mobile_base_controller/odometry`

**Data:**
- Vận tốc tiến (Vx), ngang (Vy)  
- Vận tốc quay  

**Usage:**
- Ước lượng chuyển động robot  

---

## 2. Gazebo → ROS2 Bridge

Sử dụng:

```bash
ros2 run ros_gz_bridge parameter_bridge
Launch config
bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    parameters=[{'config_file': bridge_config}, {'use_sim_time': True}],
)```bash
Chức năng
Chuyển đổi message:
LaserScan
Imu
Image
Chuyển topic từ Gazebo sang ROS2
3. Time Synchronization
use_sim_time: true
Tất cả node sử dụng /clock
Đảm bảo dữ liệu đồng bộ giữa các sensor
4. Sensor Fusion (EKF)
Node
ekf_node = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node'
)
Input
Odometry (vận tốc)
IMU (góc quay)
Process
Lọc nhiễu
Kết hợp nhiều nguồn dữ liệu
Ước lượng trạng thái robot
Output
/odometry/filtered
Vai trò

Dữ liệu chính cho:

SLAM
Navigation
RViz
5. SLAM Integration
Config (slam.lua)
use_odometry = true
use_imu_data = true
num_laser_scans = 2
Input topics
('/scan_1', '/scan_front_raw'),
('/scan_2', '/scan_rear_raw'),
('/imu', '/base_imu'),
('/odom', '/odometry/filtered')
Chức năng
Xây dựng bản đồ
Định vị robot
6. Navigation (Nav2)
Input
/odometry/filtered
/scan
/camera/points
Map
Chức năng
Xác định vị trí robot
Lập kế hoạch đường đi
Tránh vật cản
Summary
LiDAR + Camera → môi trường
IMU            → hướng
Odometry       → vận tốc
                     ↓
                   EKF
                     ↓
         /odometry/filtered
                     ↓
            SLAM / Nav2


    

