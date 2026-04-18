import random

POPULATION_SIZE = 100   
GENERATIONS = 500       
MUTATION_RATE = 0.15    

def get_greedy_route(distance_matrix):
    num_rooms = len(distance_matrix)
    unvisited = list(range(1, num_rooms)) 
    current_node = 0
    route = [0]
    while unvisited:
        next_node = min(unvisited, key=lambda x: distance_matrix[current_node][x])
        route.append(next_node)
        current_node = next_node
        unvisited.remove(next_node)
    return route

def create_individual(num_rooms):
    rooms = list(range(1, num_rooms))
    random.shuffle(rooms)
    return [0] + rooms

def calculate_fitness(individual, distance_matrix):
    return sum(distance_matrix[individual[i]][individual[i+1]] for i in range(len(individual)-1))

def crossover(parent1, parent2):
    p1_core, p2_core = parent1[1:], parent2[1:]
    child_core = [-1] * len(p1_core)
    start, end = sorted([random.randint(0, len(p1_core) - 1) for _ in range(2)])
    child_core[start:end+1] = p1_core[start:end+1]
    
    p2_idx = 0
    for i in range(len(child_core)):
        if child_core[i] == -1:
            while p2_core[p2_idx] in child_core:
                p2_idx += 1
            child_core[i] = p2_core[p2_idx]
    return [0] + child_core

def mutate(individual):
    if random.random() < MUTATION_RATE and len(individual) > 2:
        idx1, idx2 = random.sample(range(1, len(individual)), 2)
        individual[idx1], individual[idx2] = individual[idx2], individual[idx1]

def apply_2opt(route, distance_matrix):
    best_route = route[:]
    improved = True
    while improved:
        improved = False
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

    # 🔥 MỚM LỘ TRÌNH THÔNG MINH ĐỂ CHỐNG "ZIGZAG"
    population[0] = get_greedy_route(distance_matrix)

    for gen in range(GENERATIONS):
        population = sorted(population, key=lambda ind: calculate_fitness(ind, distance_matrix))
        parents = population[:POPULATION_SIZE // 2]
        next_generation = parents[:] 
        
        while len(next_generation) < POPULATION_SIZE:
            p1, p2 = random.sample(parents, 2)
            child = crossover(p1, p2)
            mutate(child)
            next_generation.append(child)
            
        population = next_generation

    best_route_ga = min(population, key=lambda ind: calculate_fitness(ind, distance_matrix))
    final_best_route = apply_2opt(best_route_ga, distance_matrix)
    return final_best_route