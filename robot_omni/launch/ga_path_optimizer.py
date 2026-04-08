import random
import matplotlib.pyplot as plt

# --- CẤU HÌNH SỨC MẠNH GA ---
POPULATION_SIZE = 300   
GENERATIONS = 1500      
MUTATION_RATE = 0.15    
# =========================================================
# THUẬT TOÁN THAM LAM (GREEDY / NEAREST NEIGHBOR)
# Luôn chọn điểm gần nhất tính từ vị trí hiện tại
# =========================================================
def get_greedy_route(distance_matrix):
    num_rooms = len(distance_matrix)
    # Danh sách các phòng chưa đi (từ 1 đến 26, bỏ qua 0 là START_POS)
    unvisited = list(range(1, num_rooms)) 
    
    current_node = 0
    route = [0]
    total_distance = 0.0

    while unvisited:
        next_node = None
        min_dist = float('inf')
        
        # Tìm phòng chưa đi có khoảng cách ngắn nhất so với chỗ đang đứng
        for candidate in unvisited:
            if distance_matrix[current_node][candidate] < min_dist:
                min_dist = distance_matrix[current_node][candidate]
                next_node = candidate
                
        # Di chuyển đến phòng đó
        route.append(next_node)
        total_distance += min_dist
        current_node = next_node
        unvisited.remove(next_node)

    return route, total_distance

def create_individual(num_rooms):
    # KHÓA CHẶT: Tạo mảng từ 1 đến N, xáo trộn, rồi ghép số 0 lên đầu
    rooms = list(range(1, num_rooms))
    random.shuffle(rooms)
    return [0] + rooms

def calculate_fitness(individual, distance_matrix):
    total_distance = 0
    # Tính đường mở (không quay về chỗ cũ)
    for i in range(len(individual) - 1):
        total_distance += distance_matrix[individual[i]][individual[i+1]]
    return total_distance

def crossover(parent1, parent2):
    # LAI GHÉP: Chỉ cắt phần lõi (từ vị trí 1 trở đi), chừa vị trí 0 ra
    p1_core = parent1[1:]
    p2_core = parent2[1:]
    
    child_core = [-1] * len(p1_core)
    start, end = sorted([random.randint(0, len(p1_core) - 1) for _ in range(2)])
    child_core[start:end+1] = p1_core[start:end+1]
    
    p2_idx = 0
    for i in range(len(child_core)):
        if child_core[i] == -1:
            while p2_core[p2_idx] in child_core:
                p2_idx += 1
            child_core[i] = p2_core[p2_idx]
            
    # Gắn lại số 0 lên đầu
    return [0] + child_core

def mutate(individual):
    if random.random() < MUTATION_RATE:
        # ĐỘT BIẾN: Chỉ bốc ngẫu nhiên từ vị trí 1 đến cuối, không đụng số 0
        idx1, idx2 = random.sample(range(1, len(individual)), 2)
        individual[idx1], individual[idx2] = individual[idx2], individual[idx1]

def apply_2opt(route, distance_matrix):
    best_route = route[:]
    improved = True
    while improved:
        improved = False
        # 2-OPT: Bắt đầu gỡ rối từ vị trí 1, không được lật ngược số 0
        for i in range(1, len(best_route) - 1):
            for j in range(i + 2, len(best_route) + 1):
                new_route = best_route[:]
                new_route[i:j] = best_route[i:j][::-1]
                
                if calculate_fitness(new_route, distance_matrix) < calculate_fitness(best_route, distance_matrix):
                    best_route = new_route
                    improved = True
    return best_route

def optimize_route(distance_matrix):
    num_rooms = len(distance_matrix)
    population = [create_individual(num_rooms) for _ in range(POPULATION_SIZE)]
    history_best_distances = []

    for gen in range(GENERATIONS):
        population = sorted(population, key=lambda ind: calculate_fitness(ind, distance_matrix))
        history_best_distances.append(calculate_fitness(population[0], distance_matrix))

        parents = population[:POPULATION_SIZE // 2]
        next_generation = parents[:] 
        while len(next_generation) < POPULATION_SIZE:
            p1, p2 = random.sample(parents, 2)
            child = crossover(p1, p2)
            mutate(child)
            next_generation.append(child)
            
        population = next_generation

    # 1. Lấy kết quả thô của GA
    best_route_ga = min(population, key=lambda ind: calculate_fitness(ind, distance_matrix))
    ga_distance = calculate_fitness(best_route_ga, distance_matrix)
    
    # 2. Đưa qua 2-Opt để "Gỡ Rối Zig-Zag"
    print("🔧 Đang dùng 2-Opt để nắn lại đường đi dọc hành lang...")
    final_best_route = apply_2opt(best_route_ga, distance_matrix)
    final_best_distance = calculate_fitness(final_best_route, distance_matrix)
    
    print(f"🌟 GA Thô: {ga_distance:.2f}m | Sau khi 2-Opt nắn đường: {final_best_distance:.2f}m")

    # Vẽ biểu đồ
    plt.figure(figsize=(10, 5))
    plt.plot(history_best_distances, color='blue', label='GA Tiến hóa')
    plt.axhline(y=final_best_distance, color='green', linestyle='--', label='Sau khi 2-Opt nắn đường')
    plt.title('Biểu đồ Hội tụ: GA + 2-Opt (Đã khóa điểm xuất phát)', fontsize=14)
    plt.xlabel('Thế hệ', fontsize=12)
    plt.ylabel('Quãng đường (m)', fontsize=12)
    plt.legend()
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.show(block=False) 
    plt.pause(2)
    plt.close()

    return final_best_route