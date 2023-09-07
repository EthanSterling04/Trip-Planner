#lang dssl2

let eight_principles = ["Know your rights.",
    "Acknowledge your sources.",
    "Protect your work.",
    "Avoid suspicion.",
    "Do your own work.",
    "Never falsify a record or permit another person to do so.",
    "Never fabricate data, citations, or experimental results.",
    "Always tell the truth when discussing your work with your instructor."]

# Final project: Trip Planner

import cons
import 'project-lib/dictionaries.rkt' 
import 'project-lib/graph.rkt' 
import 'project-lib/binheap.rkt'
import 'project-lib/stack-queue.rkt'
import sbox_hash

### Basic Types ###

#  - Latitudes and longitudes are numbers:
let Lat?  = num?
let Lon?  = num?

#  - Point-of-interest categories and names are strings:
let Cat?  = str?
let Name? = str?

### Raw Item Types ###

#  - Raw positions are 2-element vectors with a latitude and a longitude
let RawPos? = TupC[Lat?, Lon?]

#  - Raw road segments are 4-element vectors with the latitude and
#    longitude of their first endpoint, then the latitude and longitude
#    of their second endpoint
let RawSeg? = TupC[Lat?, Lon?, Lat?, Lon?]

#  - Raw points-of-interest are 4-element vectors with a latitude, a
#    longitude, a point-of-interest category, and a name
let RawPOI? = TupC[Lat?, Lon?, Cat?, Name?]

### Contract Helpers ###

# ListC[T] is a list of `T`s (linear time):
let ListC = Cons.ListC
# List of unspecified element type (constant time):
let List? = Cons.list?


interface TRIP_PLANNER:

    # Returns the positions of all the points-of-interest that belong to
    # the given category.
    def locate_all(
            self,
            dst_cat:  Cat?           # point-of-interest category
        )   ->        ListC[RawPos?] # positions of the POIs

    # Returns the shortest route, if any, from the given source position
    # to the point-of-interest with the given name.
    def plan_route(
            self,
            src_lat:  Lat?,          # starting latitude
            src_lon:  Lon?,          # starting longitude
            dst_name: Name?          # name of goal
        )   ->        ListC[RawPos?] # path to goal

    # Finds no more than `n` points-of-interest of the given category
    # nearest to the source position.
    def find_nearby(
            self,
            src_lat:  Lat?,          # starting latitude
            src_lon:  Lon?,          # starting longitude
            dst_cat:  Cat?,          # point-of-interest category
            n:        nat?           # maximum number of results
        )   ->        ListC[RawPOI?] # list of nearby POIs

#TUP STRUCT
struct tup:
    let dst
    let preds

#DIJKSTRA'S  
def dijkstra(graph, start_pos):
    # Initialize distances, predecessors, and visited dictionary
    let distances = [2**64 for node in range(graph.len())]
    distances[start_pos] = 0
    let predecessors = [None for node in range(graph.len())]
    let visited = [False for node in range(graph.len())]

    # Initialize the priority queue with the start position
    let pq = BinHeap(graph.len(), lambda x, y: distances[x] < distances[y])
    pq.insert(start_pos)

    # Loop through the priority queue until it's empty
    while pq.len() != 0:
        # Get the node with the minimum distance from the start position
        let v = pq.find_min()
        pq.remove_min()

        # Skip visited nodes
        if not visited[v]:
            # Mark the current node as visited
            visited[v] = True

            # Update the distances and predecessors of neighboring nodes
            for neighbor in Cons.to_vec(graph.get_adjacent(v)):
                if distances[v] + graph.get_edge(neighbor, v) < distances[neighbor]:
                    distances[neighbor] = distances[v] + graph.get_edge(neighbor, v)
                    predecessors[neighbor] = v
                    pq.insert(neighbor)

    # Return the distances and predecessors dictionaries
    return tup(distances, predecessors)
    
    
#TRIP PLANNER CLASS                        
class TripPlanner (TRIP_PLANNER):
    #DATA FIELDS
    let _data
    let _node_to_position
    let _position_to_node
    let _cat_to_positions
    let _name_to_position
    let _position_to_POIs
    
    def __init__(self, segments, POIs):
        let n = segments.len()
        
        #GRAPH CREATION
        self._position_to_node = HashTable(n, make_sbox_hash())        
        self._node_to_position = vec(n*2)
        self._data = WuGraph(n*2)
        
        #LOOP THROUGH ROAD SEGMENTS
        let n_positions = 0
        for seg in segments:
            let lat1 = seg[0]
            let lon1 = seg[1]
            let lat2 = seg[2]
            let lon2 = seg[3]
            if not self._position_to_node.mem?([lat1, lon1]):
                self._position_to_node.put([lat1, lon1], n_positions)
                self._node_to_position[n_positions] = [lat1, lon1]
                n_positions = n_positions + 1
            if not self._position_to_node.mem?([lat2, lon2]):
                self._position_to_node.put([lat2, lon2], n_positions)
                self._node_to_position[n_positions] = [lat2, lon2]
                n_positions = n_positions + 1
            self._data.set_edge(self._position_to_node.get([lat1, lon1]), self._position_to_node.get([lat2, lon2]), ((lat1-lat2)**2 + (lon1-lon2)**2).sqrt() )
        
        #DICTS        
        self._name_to_position = HashTable(n_positions, make_sbox_hash())
        self._position_to_POIs = HashTable(n_positions, make_sbox_hash())
        self._cat_to_positions = HashTable(n_positions, make_sbox_hash())
        
        #LOOP THROUGH POIs
        for POI in POIs:
            let lat = POI[0]
            let lon = POI[1]
            let cat = POI[2]
            let name = POI[3]
            self._name_to_position.put(name, [lat, lon])
            
            if not self._position_to_POIs.mem?([lat, lon]):
                self._position_to_POIs.put([lat, lon], cons(POI, None))
            else:
                self._position_to_POIs.put([lat, lon], cons(POI, self._position_to_POIs.get([lat, lon])))
            
            if not self._cat_to_positions.mem?(cat):
                self._cat_to_positions.put(cat, cons([lat, lon], None))
            elif not Cons.ormap(lambda x: x == [lat, lon], self._cat_to_positions.get(cat)):
                self._cat_to_positions.put(cat, cons([lat, lon], self._cat_to_positions.get(cat)))
            else:
                pass
        
    
    
    #LOCATE ALL
    def locate_all(self, dst_cat: Cat?):
        if not self._cat_to_positions.mem?(dst_cat):
            return None
        return self._cat_to_positions.get(dst_cat)
        
    #PLAN ROUTE
    def plan_route(self, src_lat: Lat?, src_lon: Lon?, dst_name: Name?):
        let start = self._position_to_node.get([src_lat, src_lon])
        if not self._name_to_position.mem?(dst_name):
            return None
        let end = self._position_to_node.get(self._name_to_position.get(dst_name))
        let path = dijkstra(self._data, start).preds
        let output = cons(self._node_to_position[end], None)
        while end != start:
            if path[end] is None:
                return None
            output = cons(self._node_to_position[path[end]], output)
            end = path[end]
        return output
        
    #FIND NEARBY
    def find_nearby(self, src_lat: Lat?, src_lon: Lon?, dst_cat: Cat?, n: nat?):
        let start = self._position_to_node.get([src_lat, src_lon])
        let distances = dijkstra(self._data, start).dst
        let destinations = self.locate_all(dst_cat)
        let pq = BinHeap(distances.len(), lambda x, y: distances[self._position_to_node.get(x)] < distances[self._position_to_node.get(y)])
        for pos in Cons.to_vec(destinations):
            if distances[self._position_to_node.get(pos)] != 2**64:
                pq.insert(pos)
        let output = None
        let output_len = 0
        while output_len < n:
            if pq.len() == 0:
                return output
            for POI in Cons.to_vec(self._position_to_POIs.get(pq.find_min())):
                if POI[2] == dst_cat and output_len < n:
                    output = cons(POI, output)
                    output_len = output_len + 1
            pq.remove_min()
        return output

        
        
        
#TESTS        
def my_first_example():
    return TripPlanner([[0,0, 0,1], [0,0, 1,0]],
                       [[0,0, "bar", "The Empty Bottle"],
                        [0,1, "food", "Pierogi"]])

test 'My first locate_all test':
    assert my_first_example().locate_all("food") == \
        cons([0,1], None)

test 'My first plan_route test':
   assert my_first_example().plan_route(0, 0, "Pierogi") == \
       cons([0,0], cons([0,1], None))

test 'My first find_nearby test':
   assert my_first_example().find_nearby(0, 0, "food", 1) == \
      cons([0,1, "food", "Pierogi"], None)
      
      
      
def eg():
    return TripPlanner([[0,0, 0,1], [0,0, 1,0], [0,1, 0,2], [0,2, 1,2], [0,1, 1,1], [1,0, 1,1], [1,1, 1,2], [1,2, 1,3], [1,3, -0.2, 3.3]],
                       [[0,0, "food", "Sandwiches"],
                        [0,1, "food", "Pasta"],
                        [0,1, "clothes", "Pants"],
                        [1,1, "bank", "Local Credit Union"],
                        [1,3, "bar", "Bar None"],
                        [1,3, "bar", "H bar"],
                        [-0.2,3.3, "food", "Burritos"]])
                        
def noPOI():
    return TripPlanner(
      [[0, 0, 1, 0]],
      [])

def lt(vec1, vec2):
    if vec1[0] == vec2[0]:
        if vec1[1] < vec2[1]:
            True
        else:
            False
    if vec1[0] < vec2[0]:
        True
    else:
        False                     
                                                
test 'locate-all':
    assert Cons.sort(lt, eg().locate_all("food")) == \
        Cons.sort(lt, cons([0,1], cons([0,0], cons([-0.2, 3.3], None))))
    assert eg().locate_all("bank") == \
        cons([1,1], None)
    assert eg().locate_all("bar") == \
        cons([1,3], None)
    assert eg().locate_all("barber") == \
        None
    assert noPOI().locate_all("bank") == None
    
test 'adv locate-all':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0],
       [3, 0, 4, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [5, 0, 'barber', 'Judy'],
       [5, 0, 'barber', 'Lily']])
    let result = tp.locate_all('barber')
    assert Cons.to_vec(result) \
      == [[5, 0], [3, 0]]
        
test 'plan_route':
    assert eg().plan_route(0, 0, "Sandwiches") == \
       cons([0,0], None)
    assert eg().plan_route(0, 1, "Sandwiches") == \
       cons([0,1], cons([0,0], None))
    assert eg().plan_route(1, 1, "Sandwiches") == \
       cons([1,1], cons([0,1], cons([0,0], None)))
    assert eg().plan_route(1, 1, "Burritos") == \
       cons([1,1], cons([1,2], cons([1,3], cons([-0.2, 3.3], None))))
    assert eg().plan_route(1, 1, "Sushi") == \
       None
       
test 'adv plan-route':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony']])
    let result = tp.plan_route(0, 0, 'Judy')
    assert Cons.to_vec(result) \
      == []
    tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [5, 0, 'barber', 'Judy']])
    result = tp.plan_route(0, 0, 'Judy')
    assert Cons.to_vec(result) \
      == []

test 'find_nearby':
    assert eg().find_nearby(1, 3, "food", 1) == \
      cons([-0.2,3.3, "food", "Burritos"], None)
    assert eg().find_nearby(0, 2, "food", 1) == \
      cons([0,1, "food", "Pasta"], None)
    assert eg().find_nearby(0, 2, "food", 2) == \
      cons([0,0, "food", "Sandwiches"], cons([0,1, "food", "Pasta"], None))
    assert eg().find_nearby(0, 2, "food", 3) == \
      cons([-0.2,3.3, "food", "Burritos"], cons([0,0, "food", "Sandwiches"], cons([0,1, "food", "Pasta"], None)))
    assert eg().find_nearby(0, 2, "food", 4) == \
      cons([-0.2,3.3, "food", "Burritos"], cons([0,0, "food", "Sandwiches"], cons([0,1, "food", "Pasta"], None)))
    assert eg().find_nearby(0, 2, "bar", 1) == \
      cons([1,3, "bar", "H bar"], None)
    assert eg().find_nearby(0, 2, "bar", 2) == \
      cons([1,3, "bar", "Bar None"], cons([1,3, "bar", "H bar"], None))
    assert eg().find_nearby(0, 2, "school", 5) == \
      None
      
test 'adv find-nearby':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [4, 0, 'food', 'Jollibee'],
       [5, 0, 'barber', 'Judy']])
    let result = tp.find_nearby(0, 0, 'barber', 2)
    assert Cons.to_vec(result) \
      == [[3, 0, 'barber', 'Tony']]
    tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [5, 0, 'barber', 'Judy']])
    result = tp.find_nearby(0, 0, 'food', 1)
    assert Cons.to_vec(result) \
      == []
    tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [4, 0, 'food', 'Jollibee'],
       [5, 0, 'barber', 'Judy']])
    result = tp.find_nearby(0, 0, 'food', 1)
    assert Cons.to_vec(result) \
      == []
    tp = TripPlanner(
      [[0, 0, 0, 9],
       [0, 9, 9, 9],
       [0, 0, 1, 1],
       [1, 1, 2, 2],
       [2, 2, 3, 3],
       [3, 3, 4, 4],
       [4, 4, 5, 5],
       [5, 5, 6, 6],
       [6, 6, 7, 7],
       [7, 7, 8, 8],
       [8, 8, 9, 9]],
      [[7, 7, 'haberdasher', 'Archit'],
       [8, 8, 'haberdasher', 'Braden'],
       [9, 9, 'haberdasher', 'Cem']])
    result = tp.find_nearby(0, 0, 'haberdasher', 2)
    assert Cons.to_vec(result) \
      == [[8, 8, 'haberdasher', 'Braden'], [7, 7, 'haberdasher', 'Archit']]
    tp = TripPlanner(
      [[-1.1, -1.1, 0, 0],
       [0, 0, 3, 0],
       [3, 0, 3, 3],
       [3, 3, 3, 4],
       [0, 0, 3, 4]],
      [[0, 0, 'food', 'Sandwiches'],
       [3, 0, 'bank', 'Union'],
       [3, 3, 'barber', 'Judy'],
       [3, 4, 'barber', 'Tony']])
    result = tp.find_nearby(-1.1, -1.1, 'barber', 1)
    assert Cons.to_vec(result) \
      == [[3, 4, 'barber', 'Tony']]
    tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0],
       [3, 0, 4, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [4, 0, 'food', 'Jollibee'],
       [5, 0, 'barber', 'Judy']])
    result = tp.find_nearby(0, 0, 'barber', 3)
    assert Cons.to_vec(result) \
      == [[5, 0, 'barber', 'Judy'], [3, 0, 'barber', 'Tony']]
    tp = TripPlanner(
      [[-1, -1, 0, 0],
       [0, 0, 3.5, 0],
       [0, 0, 0, 3.5],
       [3.5, 0, 0, 3.5]],
      [[-1, -1, 'food', 'Jollibee'],
       [0, 0, 'bank', 'Union'],
       [3.5, 0, 'barber', 'Tony'],
       [0, 3.5, 'barber', 'Judy']])
    result = tp.find_nearby(-1, -1, 'barber', 1)
    assert Cons.to_vec(result) \
      == [[3.5, 0, 'barber', 'Tony']] or [[0, 3.5, 'barber', 'Judy']]
    tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0],
       [3, 0, 4, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [5, 0, 'barber', 'Judy'],
       [5, 0, 'barber', 'Lily']])
    result = tp.find_nearby(0, 0, 'barber', 2)
    assert Cons.to_vec(result) \
      == [[5, 0, 'barber', 'Judy'], [3, 0, 'barber', 'Tony']] or [[5, 0, 'barber', 'Lily'], [3, 0, 'barber', 'Tony']]
    tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0],
       [3, 0, 4, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [0, 0, 'barber', 'Lily'],
       [3, 0, 'barber', 'Tony'],
       [5, 0, 'barber', 'Judy']])
    result = tp.find_nearby(2.5, 0, 'barber', 2)
    assert Cons.to_vec(result) \
      == [[5, 0, 'barber', 'Judy'], [3, 0, 'barber', 'Tony']] or [[0, 0, 'barber', 'Lily'], [3, 0, 'barber', 'Tony']]
    tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0],
       [3, 0, 4, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [5, 0, 'food', 'Jollibee'],
       [5, 0, 'barber', 'Judy'],
       [5, 0, 'bar', 'Pasta']])
    result = tp.find_nearby(0, 0, 'barber', 2)
    assert Cons.to_vec(result) \
      == [[5, 0, 'barber', 'Judy'], [3, 0, 'barber', 'Tony']]