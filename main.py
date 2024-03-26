import math
import random
import sys
import time


"""The code implementation below contains a number of construction heuristics such as Nearest Neighbor and Random Tours
 for finding a solution to berlin52.tsp as well as improvement heuristics
  (Local Search, Local Search with Random Start)"""

def dist(cord1, cord2):
    x1, y1 = cord1
    x2, y2 = cord2
    xdiff = x2 - x1
    ydiff = y2 - y1
    return int(math.sqrt(xdiff*xdiff + ydiff*ydiff) + .5)


def mk_matrix(coord, dist):
    n = len(coord)
    D = {}
    for i in range(n):
        for j in range(i+1,n):
            (x1,y1) = coord[i]
            (x2,y2) = coord[j]
            D[i,j] = dist((x1,y1), (x2,y2))
            D[j,i] = D[i,j]
    return n,D


# parse file
def read_tsplib(berlin52):
    f = open(berlin52, 'r')
    line = f.readline()
    while line.find("NODE_COORD_SECTION") == -1:
        line = f.readline()

    xy_positions = []
    while 1:
        line = f.readline()
        if line.find("EOF") != -1:
            break
        (i,x,y) = line.split()
        x = float(x)
        y = float(y)
        xy_positions.append((x,y))

    n,D = mk_matrix(xy_positions, dist)
    return n, xy_positions, D


def mk_closest(D, n):
    C = []
    for i in range(n):
        dlist = [(D[i,j], j) for j in range(n) if j != i]
        dlist.sort()
        C.append(dlist)
    return C


def length(tour, D):
    # Calculates the duration of a tour according to table 'D'
    z = D[tour[-1], tour[0]]    # edge from the last to the first edge of the path
    for i in range(1,len(tour)):
        z += D[tour[i], tour[i-1]]     # adds length to edges of cities i-1 with i
    return z


def randtour(n):
    sol = list(range(n))
    random.shuffle(sol)
    return sol


def nearest(last, unvisited, D):
    near = unvisited[0]
    min_dist = D[last, near]
    for i in unvisited[1:]:
        if D[last,i] < min_dist:
            near = i
            min_dist = D[last, near]
    return near


def nearest_neighbor(n, i, D):
    unvisited = list(range(n))
    unvisited.remove(i)
    last = i
    tour = [i]
    while unvisited != []:
        next = nearest(last, unvisited, D)
        tour.append(next)
        unvisited.remove(next)
        last = next
    return tour



def exchange_cost(tour, i, j, D):
    n = len(tour)
    a,b = tour[i],tour[(i+1)%n]
    c,d = tour[j],tour[(j+1)%n]
    return (D[a,c] + D[b,d]) - (D[a,b] + D[c,d])


def exchange(tour, tinv, i, j):
    n = len(tour)
    if i>j:
        i,j = j,i
    assert 0 <= i < j - 1 and j < n
    path = tour[i+1:j+1]
    path.reverse()
    tour[i+1:j+1] = path
    for k in range(i+1,j+1):
        tinv[tour[k]] = k


def improve(tour, z, D, C):
# Improves tour 't' by swapping edges ; returns optimized path length

    n = len(tour)
    tinv = [0 for i in tour]
    for k in range(n):
        tinv[tour[k]] = k  # location of each city on route 't'
    for i in range(n):
        a,b = tour[i],tour[(i+1)%n]
        dist_ab = D[a,b]
        improved = False
        for dist_ac,c in C[a]:
            if dist_ac >= dist_ab:
                break
            j = tinv[c]
            d = tour[(j+1)%n]
            dist_cd = D[c,d]
            dist_bd = D[b,d]
            delta = (dist_ac + dist_bd) - (dist_ab + dist_cd)
            if delta < 0:      # swap reduces the length
                exchange(tour, tinv, i, j)
                z += delta
                improved = True
                break
        if improved:
            continue
        for dist_bd,d in C[b]:
            if dist_bd >= dist_ab:
                break
            j = tinv[d]-1
            if j==-1:
                j=n-1
            c = tour[j]
            dist_cd = D[c,d]
            dist_ac = D[a,c]
            delta = (dist_ac + dist_bd) - (dist_ab + dist_cd)
            if delta < 0:       # swap reduces the length
                exchange(tour, tinv, i, j)
                z += delta
                break
    return z


def localsearch(tour, z, D, C=None):
    n = len(tour)
    if C == None:
        C = mk_closest(D, n)
    while 1:
        newz = improve(tour, z, D, C)
        if newz < z:
            z = newz
        else:
            break
    return z


def multistart_localsearch(k, n, D, report=None):
    """Does k iterations of local search, starting from random solutions.
        Parameters:
        k-- number of iterations
        D-- distance table
        report-- if none, called to print detailed results
        Returns the best solution and its cost"""

    C = mk_closest(D, n) # creates a sorted list of distances to each node
    bestt=None
    bestz=None
    for i in range(0,k):
        tour = randtour(n)
        z = length(tour, D)
        z = localsearch(tour, z, D, C)
        if bestz == None or z < bestz:
            bestz = z
            bestt = list(tour)
            if report:
                report(z, tour)

    return bestt, bestz


if __name__ == "__main__":
    if len(sys.argv) == 1:
        n, coord, D = read_tsplib("berlin52.tsp")

    init = time.process_time()
    def report_sol(obj, s=""):
        print(f"CPUtime: {round(time.process_time(),4)}sec,", f"z = {obj},", f"tour: {s}")
        print("\t\t   \u2193")

    def print_route(tour):
        for i in range(len(tour)):
            if not i == len(tour) - 1:
                print(tour[i] + 1, "\u2192 ", end="")
            else:
                print(tour[i] + 1, end="")

    print ("*** Applying Construction Heuristics and Improvement Heuristics for 52 locations (coordinates) of Berlin.\n"
           "The concept is to find the minimum z (length) passing by all locations on a single route ***","\n")
 
    # random construction
    print("Applying the Local Search Algorithm for a Random Construction:\n")
    tour = randtour(n)
 
    # Creates a random tour
    z = length(tour, D)  # Calculates the length of the tour
    print("Random Route: ", end="")
    print_route(tour)
    print(f", z = {z}", '  --->  ', end="")
    z = localsearch(tour, z, D)  # local search starting from a random tour
    print("Local Search: ", end="")
    print_route(tour)
    print(f", z = {z}", "\n")

    # greedy construction
    print("Applying the Local Search Algorithm for a Greedy Construction with Nearest Neighbor:\n")
    for i in range(n):
        tour = nearest_neighbor(n, i, D)    # create a greedy route, going to city 'i' first
        z = length(tour, D)
        print("nneigh: ", end="")
        print_route(tour)
        print(f", z = {z}", '  --->  ', end="")
        z = localsearch(tour, z, D)
        print("Local Search: ", end="")
        print_route(tour)
        print(f", z = {z}")
    print("")


    # multi-start local search
    print ("Applying Multi-Start Local Search (100 iterations):\n")
    niter = 100
    tour,z = multistart_localsearch(niter, n, D, report_sol)
    assert z == length(tour, D)
    print (f"Best Found Solution: z = {z}", f" tour: {tour}")
