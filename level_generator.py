import argparse
import heapq
import random
import string


class PriorityQueue:
    def __init__(self):
        self.elements = []

    def empty(self):
        return len(self.elements) == 0

    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)[1]


class Cube:
    def __init__(self, edge_length, num_faces):
        self.edge_length = edge_length
        self.num_faces = num_faces
        self.height = 4 * edge_length
        self.width = 3 * edge_length
        self.walls = []
        if num_faces == 3:
            self.walls.extend([(x, -1) for x in range(0, self.edge_length)])
            self.walls.extend([(-1, y) for y in range(0, self.edge_length)])
            self.walls.extend([(-self.edge_length, y) for y in range(0, self.edge_length)])
            self.walls.extend([(x, -2*self.edge_length)] for x in range(0, self.edge_length))

    def passable(self, loc):
        x,y = loc
        if loc in self.walls:
            return False
        if x >= self.edge_length and y >= self.edge_length:
            return False
        if x <= 0 and y >= self.edge_length:
            return False
        if x <= 0 and y < 0:
            return False
        if x >= self.edge_length and y < 0:
            return False
        return True

    def cost(self, from_node, to_node):
        return 1

    # directions refer to a flat "T" representation of a cube
    def neighbors(self, loc):
        (x, y) = loc

        right, down, left, up = (x + 1, y), (x, y - 1), (x - 1, y), (x, y + 1)

        if x == 0:
            if y >= self.edge_length:
                left = (-y + self.edge_length - 1, self.edge_length - 1)
            if y < 0 and y >= -self.edge_length:
                left = (y, 0)
            if y < -self.edge_length:
                left = (-self.edge_length, -y - self.edge_length)
        if x == self.edge_length - 1:
            if y >= self.edge_length:
                right = (y, self.edge_length - 1)
            if y < 0 and y >= -self.edge_length:
                right = (-y + self.edge_length - 1, 0)
            if y < -self.edge_length:
                right = (2 * self.edge_length - 1, -y - self.edge_length - 1)
        if y == 0:
            if x < 0:
                down = (0, x)
            if x > self.edge_length - 1:
                down = (self.edge_length - 1, -x + self.edge_length - 1)
        if y == self.edge_length - 1:
            if x < 0:
                up = (0, -x + self.edge_length - 1)
            if x > self.edge_length - 1:
                up = (self.edge_length - 1, x)
        if y == 2 * self.edge_length - 1:
            up = (x, -2 * self.edge_length)
        if y == -2 * self.edge_length:
            down = (x, 2 * self.edge_length - 1)
        if x == -self.edge_length:
            left = (2 * self.edge_length - 1, y)
        if x == 2 * self.edge_length - 1:
            right = (-self.edge_length, y)

        results = [right, down, left, up]

        results = filter(self.passable, results)
        return results


def draw_level(cube, paths):
    csv_string = ""
    new_paths = []
    for path in paths:
        route = path[1]
        reroute = []
        for loc in route:
            x, y = loc
            x += cube.edge_length
            y = 2 * cube.edge_length - y
            reroute.append((x, y))
        new_path = (path[0], reroute)
        new_paths.append(new_path)

    for y in range(cube.edge_length * 4 + 1):
        line = []
        for x in range(cube.edge_length * 3 + 1):
            if (x < cube.edge_length and y < cube.edge_length) or \
                    (x > 2 * cube.edge_length and y < cube.edge_length) or \
                    (x < cube.edge_length and y > 2 * cube.edge_length) or \
                    (x > 2 * cube.edge_length and y > 2 * cube.edge_length):
                line.append("  ")
                continue
            if x % cube.edge_length == 0 and y % cube.edge_length == 0:
                line.append(".")
            elif x % cube.edge_length == 0:
                line.append("|")
            elif y % cube.edge_length == 0:
                line.append("_")
            else:
                line.append(" ")
            has_path = False
            for symbol, route in new_paths:
                if (x, y) == route[0] or (x, y) == route[-1]:
                    csv_string += symbol
                    line.append(symbol)
                    has_path = True
                elif (x, y) in route:
                    line.append(" ")
                    csv_string += "0"
                    has_path = True
            if not has_path:
                if (x-cube.edge_length, -y+2*cube.edge_length) in cube.walls:
                    line.append("X")
                    csv_string += "X"
                else:
                    if y != 0 and not (y == cube.edge_length and x < cube.edge_length) and not (y == cube.edge_length and x > 2*cube.edge_length):
                        csv_string += "0"
                    line.append(" ")
        csv_string = csv_string[:len(csv_string)-1]

        line_str = ""
        for item in line:
            line_str += item
        print line_str
    chunks = [csv_string[i: i + cube.edge_length] for i in range(0, len(csv_string) + 1 - cube.edge_length, cube.edge_length)]
    faces = [[] for i in range(6)]
    if cube.edge_length == 5:
        faces[0] = chunks[:5]
        faces[1] = [chunks[5], chunks[8], chunks[11], chunks[14], chunks[17]]
        faces[2] = [chunks[6], chunks[9], chunks[12], chunks[15], chunks[18]]
        faces[3] = [chunks[7], chunks[10], chunks[13], chunks[16], chunks[19]]
        faces[4] = chunks[20:25]
        faces[5] = chunks[25:30]
    elif cube.edge_length == 4:
        faces[0] = chunks[:4]
        faces[1] = [chunks[4], chunks[7], chunks[10], chunks[13]]
        faces[2] = [chunks[5], chunks[8], chunks[11], chunks[14]]
        faces[3] = [chunks[6], chunks[9], chunks[12], chunks[15]]
        faces[4] = chunks[15:19]
        faces[5] = chunks[19:23]
    result = "{0}:".format(cube.edge_length)
    for face in faces:
        for i, row in enumerate(face):
            result += row
            if cube.edge_length-i == 1:
                result += ','
            else:
                result += '.'
    print result[:len(result)-1]



def heuristic(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)


def a_star_search(graph, start, goal):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    current = None
    success = False

    while not frontier.empty():
        current = frontier.get()

        if current == goal:
            success = True
            break

        for next in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(next, priority)
                came_from[next] = current

    #if current != goal:
    #    print "failed to find path!"


    return came_from, cost_so_far, success


def reconstruct_path(came_from, start, goal):
    current = goal
    path = [current]
    while current != start:
        current = came_from[current]
        path.append(current)
    path.reverse()  # optional
    return path


def add_path(cube, start, goal, test=False):
    came_from, cost_so_far, success = a_star_search(cube, start, goal)
    path = None
    if success:
        path = reconstruct_path(came_from, start, goal)
        if not test:
            cube.walls.extend(path)
        else:
            return len(path)
    return cube, path, success

def get_random_spot(num_faces=6):
    x_bound_low = -cube.edge_length
    x_bound_high = 2 * cube.edge_length - 1
    y_bound_low = -2 * cube.edge_length
    y_bound_high = 2 * cube.edge_length - 1
    return (random.randint(x_bound_low, x_bound_high), random.randint(y_bound_low, y_bound_high))

def generate_paths(cube, num_colors, min_distance, max_walls, max_tries=100, stop_at=1e5):
    start_cube = cube
    paths = []
    pairs = []
    tries = 0
    overall_tries = 0

    for i in range(num_colors):
        start = get_random_spot()
        while cube.passable(start) is False:
            start = get_random_spot()
        end = get_random_spot()
        success = False
        while success is False:
            while cube.passable(end) is False or add_path(start_cube, start, end, True) < min_distance:
                end = get_random_spot()
            cube, path, success = add_path(cube, start, end)
            if success:
                pairs.append((start,end))
                paths.append((string.ascii_lowercase[i], path))
            else:
                tries += 1
                overall_tries += 1
                if tries >= max_tries:
                    tries = 0
                    start = get_random_spot()
                    #while cube.passable(start) is False:
                    #    start = get_random_spot()
                    return cube, paths
                end = get_random_spot()
    return cube, paths

def generate_level(cube, num_colors, min_distance=4, max_walls=10, max_tries=100):
    # init
    start_cube = cube

    cube, paths = generate_paths(cube, num_colors, min_distance, max_walls, max_tries)

    # add obstacles! TODO

    draw_level(cube, paths)

parser = argparse.ArgumentParser()
parser.add_argument("edge_length", type=int)
parser.add_argument("max_colors", type=int)
parser.add_argument("min_distance", type=int)
args = parser.parse_args()

cube = Cube(args.edge_length, 6)
cube.walls = []
for i in range(1,args.edge_length-1):
    for j in range(1,args.edge_length-1):
        cube.walls.append((i,j-2*cube.edge_length))

generate_level(cube, args.max_colors, min_distance=args.min_distance, max_tries=100000)