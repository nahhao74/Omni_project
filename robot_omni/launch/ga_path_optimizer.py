import random
import time

# Hàm tính tổng chi phí (dựa vào tra bảng ma trận)
def total_cost(route, distance_matrix):
    cost = 0
    num_points = len(route)
    for i in range(num_points - 1):
        # Tra bảng lấy khoảng cách thực tế từ phòng này sang phòng kế tiếp
        cost += distance_matrix[route[i]][route[i+1]]
    return cost

def crossover(parent1, parent2, num_points):
    a, b = sorted(random.sample(range(num_points), 2))
    child = [-1] * num_points
    child[a:b] = parent1[a:b]
    fill = [gene for gene in parent2 if gene not in child]
    idx = 0
    for i in range(num_points):
        if child[i] == -1:
            child[i] = fill[idx]
            idx += 1
    return child

def mutate(route, num_points):
    i, j = random.sample(range(num_points), 2)
    route[i], route[j] = route[j], route[i]
    return route

def optimize_route(distance_matrix, num_generations=50, pop_size=60):
    num_points = len(distance_matrix)
    
    # Khởi tạo quần thể ngẫu nhiên
    population = [random.sample(range(num_points), num_points) for _ in range(pop_size)]
    
    start_time = time.time()
    for gen in range(num_generations):
        # Sắp xếp quần thể dựa trên cost
        population.sort(key=lambda r: total_cost(r, distance_matrix))
        
        new_pop = population[:10]  # Giữ lại 10 tinh hoa
        
        # Crossover
        for _ in range(32):
            p1, p2 = random.sample(population[:25], 2)
            new_pop.append(crossover(p1, p2, num_points))
            
        # Mutation
        for _ in range(14):
            p = random.choice(population[:20])
            new_pop.append(mutate(p.copy(), num_points))
            
        # Thêm mới
        for _ in range(4):
            new_pop.append(random.sample(range(num_points), num_points))
            
        population = new_pop

    best_route = min(population, key=lambda r: total_cost(r, distance_matrix))
    print(f"Thời gian GA: {time.time() - start_time:.4f} giây")
    return best_route