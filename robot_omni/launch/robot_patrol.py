import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# Thư viện cho thông số và TF
from rclpy.parameter import Parameter
from tf2_ros import Buffer, TransformListener

# Thư viện cho thuật toán và Vẽ biểu đồ
from ga_path_optimizer import optimize_route, get_greedy_route
import math
import matplotlib.pyplot as plt
import time

def main():
    rclpy.init()
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()

    # --- BƯỚC 1: LẤY TỌA ĐỘ HIỆN TẠI (ĐÃ FIX LỖI 0,0) ---
    print("\n" + "="*60)
    print("🛑 BƯỚC QUAN TRỌNG: HỆ THỐNG ĐANG TẠM DỪNG!")
    print("💡 MẸO: AMCL đang mặc định xe ở tọa độ (0,0).")
    print("👉 Hãy sang RViz, dùng nút '2D Pose Estimate' để chấm chính xác vị trí xe!")
    print("="*60)
    
    # Bắt file Python đứng im ở đây cho đến khi Quý bấm Enter
    input("✅ SAU KHI CHẤM XONG TRÊN RVIZ, BẤM PHÍM [ENTER] TẠI ĐÂY ĐỂ TIẾP TỤC...")

    print("\nĐang định vị vị trí hiện tại của robot...")
    tf_node = rclpy.create_node('tf_getter_node', parameter_overrides=[Parameter('use_sim_time', value=True)])
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, tf_node)
    
    # Cho TF Buffer vài giây để hứng dữ liệu mới sau khi Quý vừa chấm trên RViz
    for _ in range(20):
        rclpy.spin_once(tf_node, timeout_sec=0.1)
        
    current_x, current_y = 0.0, 0.0
    
    while rclpy.ok():
        try:
            trans = tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            current_x = trans.transform.translation.x
            current_y = trans.transform.translation.y
            print(f"📍 THÀNH CÔNG! Đã bắt được vị trí thực tế: X={current_x:.2f}, Y={current_y:.2f}")
            break
        except Exception:
            rclpy.spin_once(tf_node, timeout_sec=0.5)

    tf_node.destroy_node()

    # --- BƯỚC 2: DANH SÁCH PHÒNG TỌA ĐỘ ---
    raw_rooms = [
        {"name": "START_POS", "x": current_x, "y": current_y},
        {"name": "room1", "x": 4.512959957122803, "y": 6.124974727630615},
        {"name": "room2", "x": -4.434882164001465, "y": 6.329789161682129},
        {"name": "room3", "x": 7.715358734130859, "y": 3.4496376514434814},
        {"name": "room4", "x": -7.528562545776367, "y": 3.5376126766204834},
        {"name": "room5", "x": 11.14523983001709, "y": 0.10691706091165543},
        {"name": "room6", "x": -10.60666561126709, "y": -0.8900203704833984},
        {"name": "room7", "x": 10.764137268066406, "y": -5.083093166351318},
        {"name": "room8", "x": -9.977296829223633, "y": -4.6399760246276855},
        {"name": "room9", "x": 1.3793208599090576, "y": -13.007988929748535},
        {"name": "room10", "x": -1.1569912433624268, "y": -13.083712577819824},
        {"name": "room11", "x": 8.283170700073242, "y": -17.805253982543945},
        {"name": "room12", "x": -7.983156681060791, "y": -18.167951583862305},
        {"name": "room13", "x": 7.140707969665527, "y": -10.434832572937012},
        {"name": "room14", "x": -7.237728118896484, "y": -10.21239185333252},
        {"name": "room15", "x": 1.7844094038009644, "y": -22.217103958129883},
        {"name": "room16", "x": -1.6766775846481323, "y": -22.29205894470215},
        {"name": "room17", "x": 2.165343761444092, "y": -28.470705032348633},
        {"name": "room18", "x": -2.4039149284362793, "y": -27.779836654663086},
        {"name": "room19", "x": 8.326717376708984, "y": -31.874582290649414},
        {"name": "room20", "x": -8.169065475463867, "y": -32.08611297607422},
        {"name": "room21", "x": 7.0930633544921875, "y": -24.400388717651367},
        {"name": "room22", "x": -7.323123455047607, "y": -24.400379180908203},
        {"name": "room23", "x": 1.431821346282959, "y": -38.35205078125},
        {"name": "room24", "x": -2.21226167678833, "y": -40.58604049682617},
        {"name": "room25", "x": -7.972261428833008, "y": -39.52783203125},
        {"name": "room26", "x": -8.579231262207031, "y": -35.845523834228516}
    ]

    num_rooms = len(raw_rooms)

    # --- BƯỚC 3: LẬP MA TRẬN KHOẢNG CÁCH (Toán học) ---
    print("\nĐang quét bản đồ để lập Ma trận khoảng cách (Euclidean)...")
    distance_matrix = [[0.0 for _ in range(num_rooms)] for _ in range(num_rooms)]
    
    for i in range(num_rooms):
        for j in range(num_rooms):
            if i != j:
                euclidean_dist = math.hypot(raw_rooms[j]["x"] - raw_rooms[i]["x"], 
                                            raw_rooms[j]["y"] - raw_rooms[i]["y"])
                distance_matrix[i][j] = euclidean_dist

    # --- BƯỚC 4: SO SÁNH THUẬT TOÁN ---
    print("\n" + "="*60)
    print("🥊 BẮT ĐẦU TRẬN CHIẾN: THAM LAM vs DI TRUYỀN (GA)")
    print("="*60)

    greedy_indices, greedy_dist = get_greedy_route(distance_matrix)
    greedy_names = [raw_rooms[i]["name"] for i in greedy_indices]
    print(f"🏃 [THAM LAM] Quãng đường: {greedy_dist:.2f} m")

    print("\n🧠 [GA + 2-OPT] Đang phân tích chiến lược toàn cục...")
    best_route_indices = optimize_route(distance_matrix) 
    
    ga_dist = 0.0
    for i in range(len(best_route_indices) - 1):
        ga_dist += distance_matrix[best_route_indices[i]][best_route_indices[i+1]]
    
    ga_names = [raw_rooms[i]["name"] for i in best_route_indices]
    print(f"🧠 [GA + 2-OPT] Quãng đường: {ga_dist:.2f} m")
    print(f"Lộ trình: {' -> '.join(ga_names)}")

    # --- BƯỚC 5: HIỂN THỊ BẢN ĐỒ QUỸ ĐẠO BẰNG MATPLOTLIB ---
    print("\n📈 Đang hiển thị bản đồ kiểm tra tọa độ (Cửa sổ tự đóng sau 5 giây)...")
    plt.figure(figsize=(10, 6))
    
    # 1. Vẽ các điểm phòng
    for room in raw_rooms:
        if room["name"] == "START_POS":
            plt.scatter(room["x"], room["y"], color='red', s=150, zorder=5, label='Vị trí Xe')
            plt.text(room["x"] + 0.3, room["y"] + 0.3, "START", color='red', fontweight='bold')
        else:
            plt.scatter(room["x"], room["y"], color='blue', s=80, zorder=5)
            plt.text(room["x"] + 0.3, room["y"] + 0.3, room["name"], color='blue')

    # 2. Vẽ đường nối nối quỹ đạo GA
    route_x = [raw_rooms[i]["x"] for i in best_route_indices]
    route_y = [raw_rooms[i]["y"] for i in best_route_indices]
    plt.plot(route_x, route_y, color='green', linestyle='--', linewidth=2, label='Lộ trình GA')

    plt.title('Bản đồ Tọa độ các phòng và Quỹ đạo Tối ưu', fontsize=14)
    plt.xlabel('Trục X (mét)')
    plt.ylabel('Trục Y (mét)')
    plt.legend()
    plt.grid(True, linestyle=':', alpha=0.6)
    
    # Hiển thị biểu đồ trong 5 giây rồi tự đi tiếp
    plt.show(block=False)
    plt.pause(5)
    plt.close()

    # --- BƯỚC 6: DỊCH TỌA ĐỘ VÀ CHO XE CHẠY THỰC TẾ ---
    final_route_indices = best_route_indices[1:] # Bỏ điểm START để xe không xoay vòng tại chỗ

    # === KHU VỰC ĐỂ CHẠY XE THỰC TẾ (MỞ COMMENT KHI MUỐN CHẠY) ===
    goal_poses = []
    for i, idx in enumerate(final_route_indices):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        
        target_x = raw_rooms[idx]["x"]
        target_y = raw_rooms[idx]["y"]
        pose.pose.position.x = target_x
        pose.pose.position.y = target_y
        
        # Tính góc nhìn về điểm tiếp theo để xe chạy êm
        if i == 0:
            angle = math.atan2(target_y - current_y, target_x - current_x)
        else:
            prev_x = raw_rooms[final_route_indices[i-1]]["x"]
            prev_y = raw_rooms[final_route_indices[i-1]]["y"]
            angle = math.atan2(target_y - prev_y, target_x - prev_x)
            
        pose.pose.orientation.z = math.sin(angle / 2.0)
        pose.pose.orientation.w = math.cos(angle / 2.0)
        goal_poses.append(pose)

    route_names = [raw_rooms[i]["name"] for i in final_route_indices]
    print(f"\n🚀 BẮT ĐẦU CHẠY THỰC TẾ THEO LỘ TRÌNH: Vị trí hiện tại -> {' -> '.join(route_names)}")

    for i, pose in enumerate(goal_poses):
        current_room = route_names[i]
        print(f"\n---> Đang di chuyển đến trạm [{i+1}/{len(route_names)}]: {current_room} ...")
        
        navigator.goToPose(pose)

        while not navigator.isTaskComplete():
            pass 

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f"✅ [THÀNH CÔNG] Đã đến {current_room}!")
        elif result == TaskResult.CANCELED:
            print(f"⚠️ [HỦY BỎ] Lệnh đi tới {current_room} bị hủy!")
            break
        elif result == TaskResult.FAILED:
            print(f"❌ [THẤT BẠI] Không thể tìm đường tới {current_room}!")
            break

    print("\n🎉 HOÀN THÀNH TOÀN BỘ NHIỆM VỤ!")
    rclpy.shutdown()

if __name__ == '__main__':
    main()