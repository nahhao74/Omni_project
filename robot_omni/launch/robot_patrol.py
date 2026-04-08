import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from ga_path_optimizer import optimize_route # Kéo file GA vào
import math

def calculate_path_length(path):
    """Hàm phụ trợ: Tính tổng độ dài của một quỹ đạo (Path) do Nav2 trả về"""
    length = 0.0
    for i in range(len(path.poses) - 1):
        p1 = path.poses[i].pose.position
        p2 = path.poses[i+1].pose.position
        length += math.hypot(p2.x - p1.x, p2.y - p1.y)
    return length

def main():
    rclpy.init()
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()

    # 1. DANH SÁCH TỌA ĐỘ CÁC PHÒNG (Ví dụ 4 phòng)
    raw_rooms = [
    {"name": "home", "x": 0.0, "y": 0.0},
    {"name": "room1", "x": -4.847009658813477, "y": 5.1401872634887695},
    {"name": "room2", "x": -4.732115745544434, "y": -5.227284908294678},
    {"name": "room3", "x": -1.5093536376953125, "y": 6.668848037719727},
    {"name": "room4", "x": -1.6213617324829102, "y": -6.714361190795898},
    {"name": "room5", "x": -0.3830866813659668, "y": 9.61686897277832},
    {"name": "room6", "x": -0.4712810516357422, "y": -9.2749662399292},
    {"name": "room7", "x": 5.951502799987793, "y": 9.720064163208008},
    {"name": "room8", "x": 5.901442050933838, "y": -9.54945182800293},
    {"name": "room9", "x": 11.099164962768555, "y": 2.731727123260498},
    {"name": "room10", "x": 11.125873565673828, "y": -2.8554491996765137},
    {"name": "room11", "x": 19.81332778930664, "y": 7.038923740386963},
    {"name": "room12", "x": 19.8038330078125, "y": -7.406303405761719},
    {"name": "room13", "x": 10.860511779785156, "y": 7.144526958465576},
    {"name": "room14", "x": 10.804434776306152, "y": -7.254854202270508},
    {"name": "room15", "x": 21.157136917114258, "y": 2.845792770385742},
    {"name": "room16", "x": 21.085874557495117, "y": -2.863096237182617},
    {"name": "room17", "x": 26.743011474609375, "y": 2.8824472427368164},
    {"name": "room18", "x": 26.931209564208984, "y": -2.749497890472412},
    {"name": "room19", "x": 33.63111114501953, "y": 7.249409198760986},
    {"name": "room20", "x": 33.47489547729492, "y": -7.031137943267822},
    {"name": "room21", "x": 24.762149810791016, "y": 7.218897342681885},
    {"name": "room22", "x": 24.684158325195312, "y": -7.177097797393799},
    {"name": "room23", "x": 37.21979522705078, "y": 5.565120220184326},
    {"name": "room24", "x": 41.921417236328125, "y": -2.3477907180786133},
    {"name": "room25", "x": 42.04990768432617, "y": -7.275146961212158},
    {"name": "room26", "x": 35.6135368347168, "y": -6.739638805389404}
]
    num_rooms = len(raw_rooms)

    # 2. XÂY DỰNG MA TRẬN KHOẢNG CÁCH THỰC TẾ (ĐO BẰNG NAV2)
    print("Đang quét bản đồ để lập Ma trận khoảng cách...")
    distance_matrix = [[0.0 for _ in range(num_rooms)] for _ in range(num_rooms)]
    
    for i in range(num_rooms):
        for j in range(num_rooms):
            if i != j:
                # Tạo pose bắt đầu và kết thúc
                start_pose = PoseStamped()
                start_pose.header.frame_id = 'map'
                start_pose.pose.position.x = raw_rooms[i]["x"]
                start_pose.pose.position.y = raw_rooms[i]["y"]
                
                goal_pose = PoseStamped()
                goal_pose.header.frame_id = 'map'
                goal_pose.pose.position.x = raw_rooms[j]["x"]
                goal_pose.pose.position.y = raw_rooms[j]["y"]

                # GỌI NAV2 ĐỂ TÍNH ĐƯỜNG ĐI TRÁNH VẬT CẢN
                path = navigator.getPath(start_pose, goal_pose)
                if path:
                    real_length = calculate_path_length(path)
                    distance_matrix[i][j] = real_length
                else:
                    distance_matrix[i][j] = 9999.0 # Phạt nặng nếu không có đường tới

    # 3. ĐƯA MA TRẬN CHO THUẬT TOÁN GA
    print("Bắt đầu chạy Thuật toán GA...")
    best_route_indices = optimize_route(distance_matrix)
    print("Thứ tự đi tối ưu:", [raw_rooms[i]["name"] for i in best_route_indices])

    # 4. CHUYỂN ĐỔI THÀNH DANH SÁCH POSE VÀ CHO XE CHẠY
    goal_poses = []
    for idx in best_route_indices:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position.x = raw_rooms[idx]["x"]
        pose.pose.position.y = raw_rooms[idx]["y"]
        pose.pose.orientation.w = 1.0 
        goal_poses.append(pose)

    print("Xe bắt đầu chạy theo lộ trình GA...")
    navigator.goThroughPoses(goal_poses)

    while not navigator.isTaskComplete():
        pass 

    print("Hoàn thành nhiệm vụ!")
    rclpy.shutdown()

if __name__ == '__main__':
    main()