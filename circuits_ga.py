import random
import json
import matplotlib.pyplot as plt
import pandas as pd
import math

df = pd.read_csv("janikhurd.csv")

def haversine(lat1, lon1, lat2, lon2):
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])

    # Haversine formula
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.asin(math.sqrt(a))
    r = 6371  # Radius of earth in kilometers
  
    return c * r

'''Get JSON data'''
def get_data():
    with open("gps_dist.json", "r") as fileg:
        gps_dist = json.loads(fileg.read())
    for key, val in gps_dist.items():
        gps_dist[key] = tuple([tuple(v) for v in val])

    with open("block_dist.json", "r") as fileb:
        block_dist = json.loads(fileb.read())
    for key, val in block_dist.items():
        block_dist[key] = tuple(val)

    with open("code_map.json", "r") as filec:
        code_map = json.loads(filec.read())
    for key, val in code_map.items():
        code_map[key] = tuple(val)

    return gps_dist, block_dist, code_map

'''Rings'''
class Ring():
    def __init__(self, ring):
        self.ring = ring
        self.cost = self.get_cost()
        self.nodes = self.get_nodes()

    def get_cost(self):
        c = 0
        for r in range(len(self.ring)):
            c += self.ring[r][1]
        c += block_dist[self.ring[0][0]][1]
        c += block_dist[self.ring[-1][0]][1]
        if len(self.ring) != len(set(self.ring)):
            c += 1000
        return c

    def get_nodes(self):
        nodes = []
        for r in self.ring:
            nodes.append(r[0])
        return nodes

'''Group of rings'''
class Ring_Group():
    def __init__(self, rings, n):
        self.rings = rings
        self.n = n
        self.cost = self.get_cost()

    def get_cost(self):
        c = 0
        all = []
        for r in self.rings:
            c += r.cost
            all.extend(r.nodes)
        if len(set(all)) != 44:
            c += 1000 * (44 - len(set(all)))
        return c

'''Populate pool'''
def populate(total, pop_size, gps_dist, block_dist):
    population = []
    for i in range(pop_size):
        all_rings = []
        n = total
        while n != 0:
            split = random.randint(1, 8)
            n = n-split if n-split >= 0 else n
            ring = []
            init_gp = block_dist[random.choice(list(block_dist.keys()))][0]
            for j in range(split):
                sel_gp = random.choice(gps_dist[init_gp])
                ring.append(sel_gp)
                init_gp = sel_gp[0]
            all_rings.append(Ring(ring))
        population.append(Ring_Group(all_rings, total))
    return population

gps_dist, block_dist, code_map = get_data()

'''Plot data points'''
def plot_rings(rings):
    print(rings.cost)
    plt.scatter(df["LONG"][0], df["LAT"][0])
    plt.scatter(df["LONG"][1:], df["LAT"][1:])
    for ring in rings.rings:
        lat = [df["LAT"][0]]
        long = [df["LONG"][0]]
        for r in ring.ring:
            lat.append(code_map[r[0]][1])
            long.append(code_map[r[0]][2])
        lat.append(df["LAT"][0])
        long.append(df["LONG"][0])
        plt.plot(long, lat)
    plt.show()

'''Mutation and crossing'''
def mutate_group(rings):
    cross_or_mut = random.random()
    if cross_or_mut < 0.4:
        all_rings = []
        n = 44
        while n != 0:
            split = random.randint(1, 8)
            n = n-split if n-split >= 0 else n
            ring = []
            init_gp = block_dist[random.choice(list(block_dist.keys()))][0]
            for j in range(split):
                sel_gp = random.choice(gps_dist[init_gp])
                ring.append(sel_gp)
                init_gp = sel_gp[0]
            all_rings.append(Ring(ring))
        return Ring_Group(all_rings, n)
    else:
        parents = random.sample(rings.rings, 2)
        parent1, parent2 = sorted(parents, key=lambda x: len(x.nodes))
        indices = (rings.rings.index(parent1), rings.rings.index(parent2))
        split = random.randint(0, len(parent1.ring)-1)

        if haversine(code_map[parent1.ring[split][0]][1], code_map[parent1.ring[split][0]][2], code_map[parent2.ring[split-1][0]][1], code_map[parent2.ring[split-1][0]][1]) <= 8.5:
            child1 = parent1.ring[:split] + parent2.ring[split:]
            child2 = parent2.ring[:split] + parent1.ring[split:]
        else:
            child1 = parent1.ring
            child2 = parent2.ring
        
        new = []
        c = 1
        for r in range(len(rings.rings)):
            if r in indices:
                if c == 1:
                    new.append(Ring(child1))
                    c += 1
                elif c == 2:
                    new.append(Ring(child2))
            new.append(rings.rings[r])
        return Ring_Group(new, 44)


pool = populate(44, 400, gps_dist, block_dist)
sorted_pool = []

epochs = 5000

for gen in range(epochs):
    sorted_pool = sorted(pool, key=lambda x: x.cost)
    new_pool = sorted_pool[:40]
    for _ in range(360):
        selected = random.choice(sorted_pool)
        child = mutate_group(selected)
        new_pool.append(child)
    pool = new_pool

    if gen%100 == 0:
        print(f"Gen {gen}: {sorted_pool[0].cost}")

plot_rings(sorted_pool[0])
