import random
import json
import matplotlib.pyplot as plt
import pandas as pd

df = pd.read_csv("janikhurd.csv")

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
    prob = random.random()
    pass

'''
pool = populate(44, 200, gps_dist, block_dist)
plot_rings(sorted(pool, key=lambda x: x.cost)[0])

epochs = 1000

for gen in range(epochs):
    sorted_pool = sorted(pool, key=lambda x: x.cost)
    new_pool = sorted_pool[:20]
    for _ in range(180):
        parent1 = random.choice(sorted_pool[20:])
'''
