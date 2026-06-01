/*	
 * Title:       	Movhex: it calculates the minimum cost between two hexagons in a grid
 * Author:       	Enrico Bernard
 * Date:         	July 2025
 * Course:       	Algoritmi e Principi dell'Informatica
 *               	Politecnico di Milano
 *
 * Description:  	This program implements Dijkstra's algorithm to compute the minimum-cost path 
 *					between two hexagons on a grid, considering the costs associated with both nodes and edges.
 *
 * Compilation:  	gcc -Wall -Werror -std=gnu11 -O2  -lm  movhex.c   -o movhex
 * Use:          	./movhex < input.txt > output.txt
 *
 * Note:		 	Time of execution: 11,784 sec
 *				 	Memory used: 65,6 MiB
 *				 	Vote: 27
 */
 
#include <stdio.h> 
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <limits.h>
#include <math.h>

#define INF INT_MAX
#define CACHE_SIZE 512

typedef struct Hexagon{
	int x;
	int y;
	int z;
}Hexagon_t;

typedef struct Edge{
	int destination_x, destination_y;
	unsigned char edge_cost;
	unsigned char is_air_route;
	struct Edge* next;
} __attribute__((packed)) Edge_t;

typedef struct Node{
	unsigned char node_cost;
	unsigned char air_routes;
	Edge_t* head;
} __attribute__((packed)) Node_t;

typedef struct EdgePool{
	Edge_t* pool;
	int capacity;
	int current_size;
	Edge_t* free_head;
} EdgePool_t;

typedef struct Graph{
	int rows;
	int cols;
	Node_t* grid;
	EdgePool_t* edge_pool;
} Graph_t;

typedef struct MinHeapNode{
	int id;
	int dist;
} MinHeapNode_t;

typedef struct MinHeap{
	int size; 
	int capacity;
	int* pos;
	MinHeapNode_t* array;
} MinHeap_t;

typedef struct PathKey{
    int from_x;
    int from_y;
    int to_x;
    int to_y;
} PathKey_t;

typedef struct CacheEntry{
    PathKey_t key;
    int cost;
    struct CacheEntry* next;
} CacheEntry_t;

EdgePool_t* create_edgePool(int);
Edge_t* get_new_edge(EdgePool_t*);
void free_edgePool(EdgePool_t*);

void handle_change_cost(int, int, int, int);
void change_cost(int, int, int, int);
int distance(Hexagon_t, Hexagon_t);
Hexagon_t create_hexagon(int, int, int);
Hexagon_t to_hexagon(int, int);

void handle_init(int, int);
Graph_t* create_graph(int, int);
void all_neighbors(Graph_t*);
void add_edge(Graph_t*, int, int, int, int, int, int);
void print_graph(Graph_t*);
void free_graph(Graph_t*);

void handle_toggle_air_route(int, int, int, int);
int air_cost_calc(int, int);
int check_existing_route(int, int, int, int);
void remove_air_route(Graph_t*, int, int, int, int);

void handle_travel_cost(int, int, int, int);
MinHeap_t* create_MinHeap(int);
void switch_MinHeapNode(MinHeapNode_t*, MinHeapNode_t*);
void min_Heapify(MinHeap_t*, int);
MinHeapNode_t extract_first(MinHeap_t*);
int is_empty(MinHeap_t*);
void decreaseKey(MinHeap_t*, int, int);
int is_in_MinHeap(MinHeap_t*, int);
int dijkstra(Graph_t*, int, int, int, int, int*, MinHeap_t*);
int coord_to_id(int, int, int);
void free_MinHeap(MinHeap_t*);

unsigned int hash_path_key(int, int, int, int);
void clear_travel_cost_cache();


Graph_t* graph = NULL;

int* dijkstra_dist = NULL;
MinHeap_t* dijkstra_minHeap = NULL;
int allocated_nodes_count = 0;

CacheEntry_t* travel_cost_cache[CACHE_SIZE];    

int main(){
    char line[32];
    int rows, cols;
    int from_x, from_y, to_x, to_y;
    int center_x, center_y, v, radius;

    for(int i = 0; i < CACHE_SIZE; i++){
        travel_cost_cache[i] = NULL;
    }

    while (scanf("%s", line) == 1) {

        if (strcmp(line, "init") == 0) {
            if (scanf("%d %d", &cols, &rows) == 2) 
                handle_init(rows, cols);
            else 
                printf("Error while reading init\n");
            
        } else if (strcmp(line, "travel_cost") == 0) {
            if (scanf("%d %d %d %d", &from_y, &from_x, &to_y, &to_x) == 4) 
                handle_travel_cost(from_x, from_y, to_x, to_y);
            else 
                printf("Errore while reading travel_cost\n");

        } else if (strcmp(line, "toggle_air_route") == 0) {
            if (scanf("%d %d %d %d", &from_y, &from_x, &to_y, &to_x) == 4) 
                handle_toggle_air_route(from_x, from_y, to_x, to_y);
            else 
                printf("Errore while reading toggle_air_route\n");
            
        } else if (strcmp(line, "change_cost") == 0) {
            if (scanf("%d %d %d %d", &center_y, &center_x, &v, &radius) == 4) 
                handle_change_cost(center_x, center_y, v, radius);
            else 
                printf("Errore while reading change_cost\n");

		} else if (strcmp(line, "exit") == 0){
			printf("Exiting...\n");
			break;

        } else 
            printf("Invalid command: %s\n", line);
        
        // print_graph(graph);
    }

	if(graph != NULL){
		free_graph(graph);
        graph = NULL;
    }

    if(dijkstra_dist != NULL){
        free(dijkstra_dist);
        dijkstra_dist = NULL;
    }

    if(dijkstra_minHeap != NULL){
        free_MinHeap(dijkstra_minHeap);
        dijkstra_minHeap = NULL;
    }
    clear_travel_cost_cache();

	return 0;
}

EdgePool_t* create_edgePool(int inital_capacity){
	EdgePool_t* pool = (EdgePool_t*)malloc(sizeof(EdgePool_t));
	if(pool == NULL){
		printf("Error with malloc in create_edgePool\n");
		return NULL;
	}
	pool -> pool = (Edge_t*)malloc(inital_capacity * sizeof(Edge_t));
	if(pool -> pool == NULL){
		printf("Error with malloc in create_edgePool\n");
		free(pool);
		return NULL;
	}
	pool -> capacity = inital_capacity;
	pool -> current_size = 0;
	pool -> free_head = NULL;
	return pool;
}

Edge_t* get_new_edge(EdgePool_t* pool){
	if(pool == NULL){
		printf("Edge pool is NULL\n");
		return NULL;
	}
	if(pool -> free_head != NULL){
		Edge_t* reusable_edge = pool -> free_head;
		pool -> free_head = reusable_edge -> next;
		return reusable_edge; 
	}

	if(pool -> current_size >= pool -> capacity){
		int new_capacity = pool -> capacity * 2;
		Edge_t* new_pool = (Edge_t*) realloc(pool -> pool, new_capacity * sizeof(Edge_t));
		if(new_pool == NULL){
			printf("Error with realloc in get_new_edge\n");
			return NULL;
		}
		pool -> pool = new_pool;
		pool -> capacity = new_capacity;
	}
	return &pool -> pool[pool -> current_size++];
}

void return_edge_to_pool(EdgePool_t* pool, Edge_t* edge){
	if(pool == NULL || edge == NULL){
		return;
	}
	edge -> next = pool -> free_head;
	pool -> free_head = edge;
}

void free_edgePool(EdgePool_t* pool){
	if(pool == NULL)
		return;
	free(pool -> pool);
	free(pool);
}

void handle_change_cost(int x, int y, int v, int radius){
	
	if(graph == NULL || x < 0 || x >= graph -> rows || y < 0 || y >= graph -> cols || radius <= 0 || v < -10 || v > 10){
		printf("KO\n");
		return;
	}
		
	change_cost(x, y, v, radius);
	printf("OK\n");

    clear_travel_cost_cache();
}

void change_cost(int x, int y, int v, int radius){
	Hexagon_t center = to_hexagon(x, y);

	int min_row = (x - radius < 0) ? 0 : x - radius;
	int max_row = (x + radius >= graph -> rows) ? graph -> rows - 1 : x + radius;
	int min_col = (y - radius < 0) ? 0 : y - radius;
	int max_col = (y + radius >= graph -> cols) ? graph -> cols - 1 : y + radius;

	for(int i = min_row; i <= max_row; i++){
		for(int j = min_col; j <= max_col; j++){
			Hexagon_t curr = to_hexagon(i, j);

			int dist = distance(center, curr);

			if(dist < radius){
				long long numerator = (long long)v * (radius - dist);
                int change_cost_val = numerator / radius;
				
                if(numerator < 0 && numerator % radius != 0)
                    change_cost_val--;
				
				int new_node_cost = graph -> grid[i * graph -> cols + j].node_cost + change_cost_val;

				if(new_node_cost < 0)
					new_node_cost = 0;
				if(new_node_cost > 100)
					new_node_cost = 100;
				graph -> grid[i * graph -> cols + j].node_cost = new_node_cost;

				Edge_t* head = graph -> grid[i * graph -> cols + j].head;
				while(head != NULL){
					int new_edge_cost = head -> edge_cost + change_cost_val;

					if(new_edge_cost < 0)
						new_edge_cost = 0;
					if(new_edge_cost > 100)
						new_edge_cost = 100;

					head -> edge_cost = new_edge_cost;	
						
					head = head -> next;
				}
			} // cost remains invariant
		}
	}
}

int distance(Hexagon_t a, Hexagon_t b){
	return (abs(a.x - b.x) + abs(a.y - b.y) + abs(a.z - b.z)) / 2;
}

Hexagon_t create_hexagon(int x, int y, int z){
	Hexagon_t hexagon = {x, y, z};
	return hexagon;
}

Hexagon_t to_hexagon(int row, int col){
	int x = col - (row - (row & 1)) / 2;
	int y = row;
	int z = -x -y;
	
	return create_hexagon(x, y, z);
}


void handle_init(int x, int y){
	
    if(x <= 0 || y <= 0){
        printf("KO\n");
        return;
    }

	if(graph != NULL) // if there is an already existing graph
		free_graph(graph);

	graph = create_graph(x,y);
	printf("OK\n");

    clear_travel_cost_cache();
}

Graph_t* create_graph(int rows, int cols){
	Graph_t* graph = (Graph_t*) malloc(sizeof(Graph_t));
	if(graph == NULL){
		printf("Error with malloc in create_Graph\n");
		return NULL;
	}
	 graph -> rows = rows;
	 graph -> cols = cols;

	int initial_edge_capacity = rows * cols * 8;

	graph -> edge_pool = create_edgePool(initial_edge_capacity);
	if(graph -> edge_pool == NULL){
		free(graph);
		return NULL;
	}

	 graph -> grid = (Node_t*) malloc(sizeof(Node_t) * rows * cols);
	 if(graph -> grid == NULL){
		 printf("Error with malloc in create_Graph\n");
         free_edgePool(graph -> edge_pool);
		 free(graph);
		 return NULL;
	}
	for(int i = rows - 1; i >= 0; i--){
		for(int j = 0; j < cols; j++){
            Node_t* node = &graph -> grid[i * cols + j];
            node -> node_cost= 1;
			node -> air_routes = 0;
			node -> head = NULL;
        }
	}	
	all_neighbors(graph);

	return graph;
}

void all_neighbors(Graph_t* graph){
	int curr_x, curr_y;
	const int even_y_changes[] = {-1, 0, 1, 0, -1, -1};
	const int odd_y_changes[] = {0, 1, 1, 1, 0, -1};
	const int x_changes[] = {1, 1, 0, -1, -1, 0};
	
	for(int i = 0; i < graph -> rows; i++){
		for(int j = 0; j < graph -> cols; j++){ // for every hexagon
			for(int k = 0; k < 6; k++){ // for every neighbor
				if(i % 2 == 0) // even row
					curr_y = j + even_y_changes[k];
				else // odd row
					curr_y = j + odd_y_changes[k];

				curr_x = i + x_changes[k]; // x changes doesn't depend on parity of row

				if(curr_x >= 0 && curr_x < graph -> rows && curr_y >= 0 && curr_y < graph -> cols) // in bounds
					add_edge(graph, i, j, curr_x, curr_y, 1, 0);
			}
		}
	}
}

void add_edge(Graph_t* graph, int from_x, int from_y, int to_x, int to_y, int cost, int air_route){
	if(from_x < 0 || from_x >= graph -> rows || from_y < 0 || from_y >= graph -> cols || to_x < 0 ||
		       	to_x >= graph -> rows || to_y < 0 || to_y >= graph -> cols){
		printf("Invalid coordinates in add_edge!\n");
		return;
	}
	
	Edge_t* edge = get_new_edge(graph -> edge_pool);
	if(edge == NULL){
		printf("Error with malloc in add_edge\n");
		return;
	}
	edge -> destination_x = to_x;
	edge -> destination_y = to_y;
	edge -> edge_cost = cost;
	edge -> is_air_route = air_route;

	Node_t* from_node = &(graph -> grid[from_x * graph -> cols + from_y]);
	edge -> next = from_node -> head;
	from_node -> head = edge;

	if(air_route == 1)
		from_node -> air_routes++;
}

void print_graph(Graph_t* graph){
	for(int i = 0; i < graph -> rows; i++){
		for(int j = 0; j < graph -> cols; j++){
			Node_t curr_node = graph -> grid[i * graph -> cols + j];
			printf("L'esagono %d,%d [w: %d] è collegato con: ", i, j, curr_node.node_cost);

			Edge_t* curr_edge = curr_node.head;
			if(curr_edge == NULL)
				printf("nessuno!\n");
			else{
				while(curr_edge != NULL){
					printf("[%d,%d t:%d c:%d] ", curr_edge -> destination_x, curr_edge -> destination_y, 
							curr_edge -> is_air_route, curr_edge -> edge_cost);
					curr_edge = curr_edge -> next;
				}
			}
		printf("\n");
		}
	}
}

void free_graph(Graph_t* gr){
	if(gr == NULL)
		return;
	
	free(gr -> grid);
	free_edgePool(gr -> edge_pool);
	free(gr);
}

void handle_toggle_air_route(int from_x, int from_y, int to_x, int to_y){

	int already_route;

	int route_cost;

	if(graph == NULL || from_x < 0 || from_x >= graph -> rows || from_y < 0 || from_y >= graph -> cols || to_x < 0 ||
		    to_x >= graph -> rows || to_y < 0 || to_y >= graph -> cols){
		printf("KO\n");
		return;
	}
	printf("OK\n");
	if(graph -> grid[from_x * graph -> cols + from_y].air_routes < 5){
		already_route = check_existing_route(from_x, from_y, to_x, to_y);
		if(already_route == 0){ // route not already existing
			route_cost = air_cost_calc(from_x, from_y);
			add_edge(graph, from_x, from_y, to_x, to_y, route_cost, 1);

            clear_travel_cost_cache();
		}else if(already_route == 1) // route already existing
			remove_air_route(graph, from_x, from_y, to_x, to_y);
	}else
		printf("Too much air routes from this hexagon!\n");

}

int check_existing_route(int from_x, int from_y, int to_x, int to_y){

	

	if(from_x < 0 || from_x >= graph -> rows || from_y < 0 || from_y >= graph -> cols || to_x < 0 ||
		       	to_x >= graph -> rows || to_y < 0 || to_y >= graph -> cols){
		printf("Invalid coordinates in check_existing_route!\n");
		return -1;
	}

	Node_t* curr_node = &graph -> grid[from_x * graph -> cols + from_y];
	Edge_t* curr_edge = curr_node -> head;

	while(curr_edge != NULL){
		if(curr_edge -> destination_x == to_x && curr_edge -> destination_y == to_y && curr_edge -> is_air_route == 1)
			return 1;
		curr_edge = curr_edge -> next;
	}
	return 0;
}

int air_cost_calc(int i, int j){
	int route_cost;
	int sum_aereal_cost = 0;
	Node_t* curr_node = &graph -> grid[i * graph -> cols + j];
	Edge_t* curr_edge = curr_node -> head;

	while(curr_edge != NULL){
		if(curr_edge -> is_air_route == 1)
			sum_aereal_cost += curr_edge -> edge_cost;
		curr_edge = curr_edge -> next;
	}
	route_cost = (int)(sum_aereal_cost + curr_node -> node_cost) / (curr_node -> air_routes + 1);
	return route_cost;
}

void remove_air_route(Graph_t* graph, int from_x, int from_y, int to_x, int to_y){
	if(from_x < 0 || from_x >= graph -> rows || from_y < 0 || from_y >= graph -> cols || to_x < 0 ||
		to_x >= graph -> rows || to_y < 0 || to_y >= graph -> cols){
			printf("Invalid coordinates in remove_air_route!\n");
			return;
	}

	Node_t* curr_node = &(graph -> grid[from_x * graph -> cols + from_y]);
	Edge_t* curr_edge = curr_node -> head;
	Edge_t* prev_edge = NULL;

	while(curr_edge != NULL && 
		!(curr_edge -> destination_x == to_x && curr_edge -> destination_y == to_y && curr_edge -> is_air_route == 1)){
			prev_edge = curr_edge;
			curr_edge = curr_edge -> next;
		}

	if(curr_edge == NULL){
		printf("Air Route not found\n");
		return;
	}

	if(prev_edge == NULL) // it was the first edge
		curr_node -> head = curr_edge -> next;
	else // it wasn't the first edge
		prev_edge -> next = curr_edge -> next;

	return_edge_to_pool(graph -> edge_pool, curr_edge);

	curr_node -> air_routes --; 
}

void handle_travel_cost(int from_x, int from_y, int to_x, int to_y){

	int travel_cost;

    if(graph == NULL || from_x < 0 || from_x >= graph -> rows || from_y < 0 || from_y >= graph -> cols || to_x < 0 ||
		    to_x >= graph -> rows || to_y < 0 || to_y >= graph -> cols){
		printf("-1\n");
		return;
	}

    unsigned int hash_val = hash_path_key(from_x, from_y, to_x, to_y);
    CacheEntry_t* current_entry = travel_cost_cache[hash_val];

    while(current_entry != NULL){
        if(current_entry -> key.from_x == from_x && current_entry -> key.from_y == from_y &&
            current_entry -> key.to_x == to_x && current_entry -> key.to_y == to_y){
            printf("%d\n", current_entry -> cost);
            return;
        }
        current_entry = current_entry -> next;
    }

    int current_nodes = graph -> rows * graph -> cols;

    if(current_nodes != allocated_nodes_count){
        if(dijkstra_dist != NULL){
            free(dijkstra_dist);
            dijkstra_dist = NULL;
        }
        if(dijkstra_minHeap != NULL){
            free_MinHeap(dijkstra_minHeap);
            dijkstra_minHeap = NULL;
        }

        dijkstra_dist = malloc(current_nodes * sizeof(int));
        dijkstra_minHeap = create_MinHeap(current_nodes);

        if(dijkstra_dist == NULL || dijkstra_minHeap == NULL){
            printf("Error with malloc in handle_travel_cost\n");
            return;
        }

        allocated_nodes_count = current_nodes;
    }

	travel_cost = dijkstra(graph, from_x, from_y, to_x, to_y, dijkstra_dist, dijkstra_minHeap);

    if(travel_cost != -1){
        CacheEntry_t* new_entry = (CacheEntry_t*) malloc(sizeof(CacheEntry_t));
        if(new_entry == NULL){
            printf("Error with malloc in handle_trave_cost!\n");
            return;
        }else{
            new_entry -> key.from_x = from_x;
            new_entry -> key.from_y = from_y;
            new_entry -> key.to_x = to_x;
            new_entry -> key.to_y = to_y;
            new_entry -> cost = travel_cost;
            new_entry -> next = travel_cost_cache[hash_val];
            travel_cost_cache[hash_val] = new_entry;
        }
    }
	printf("%d\n", travel_cost);
}

MinHeap_t* create_MinHeap(int capacity){
	MinHeap_t* minHeap = (MinHeap_t*) malloc(sizeof(MinHeap_t));
	if(minHeap == NULL){
		printf("Error with malloc in create_MinHeap\n");
		return NULL;
	}

    minHeap -> pos = NULL;
    minHeap -> array = NULL;
	
    minHeap -> capacity = capacity;
    minHeap -> size = 0;
	minHeap -> pos = (int*) malloc(capacity * sizeof(int)); 
	minHeap -> array = (MinHeapNode_t*) malloc(capacity * sizeof(MinHeapNode_t));

	if(minHeap -> array == NULL || minHeap -> pos == NULL){
		free_MinHeap(minHeap);
		
		printf("Error with malloc in create_MinHeap\n");
		return NULL;
	}
	
	return minHeap;
}

void switch_MinHeapNode(MinHeapNode_t* a, MinHeapNode_t* b){
	MinHeapNode_t tmp = *a;
	*a = *b;
	*b = tmp;
}

void min_Heapify(MinHeap_t* minHeap, int index){
	int lowest, left, right;

	while(1){
		lowest = index;
		left = (2 * index) + 1;
		right = (2 * index) + 2;

		if(left < minHeap -> size && 
		minHeap -> array[left].dist < minHeap -> array[lowest].dist)
		lowest = left;

		if(right < minHeap -> size &&
		minHeap -> array[right].dist < minHeap -> array[lowest].dist)
		lowest = right;

		if(lowest != index){

			minHeap -> pos[minHeap -> array[lowest].id] = index;
			minHeap -> pos[minHeap -> array[index].id] = lowest;
		
			switch_MinHeapNode(&minHeap -> array[lowest], &minHeap -> array[index]);
		
			index = lowest;

		}else {
			break;
		}
	}

}

MinHeapNode_t extract_first(MinHeap_t* minHeap){
	if(is_empty(minHeap)){
		MinHeapNode_t empty_node = {-1, -1};
		return empty_node;
	}
	
	
	MinHeapNode_t root = minHeap -> array[0];
	MinHeapNode_t* last = &minHeap -> array[minHeap -> size - 1];
	
	minHeap -> array[0] = *last;

	minHeap -> pos[root.id] = minHeap -> size - 1;
	minHeap -> pos[last -> id] = 0;

	--minHeap -> size;

	min_Heapify(minHeap, 0);

	return root;
}

int is_empty(MinHeap_t* minHeap){
	return minHeap -> size == 0;
}

void decreaseKey(MinHeap_t* minHeap, int v, int new_dist){
	int i;

	i = minHeap -> pos[v];
	minHeap -> array[i].dist = new_dist;

	while(i && minHeap -> array[i].dist < minHeap -> array[(i - 1) / 2].dist){
		int parent = (i - 1) / 2;

		minHeap -> pos[minHeap -> array[i].id] = parent;
		minHeap -> pos[minHeap -> array[parent].id] = i;
		switch_MinHeapNode(&minHeap -> array[i], &minHeap -> array[parent]);

		i = parent;
		}
}

int is_in_MinHeap(MinHeap_t* minHeap, int v){
	return minHeap -> pos[v] < minHeap -> size;
}

int dijkstra(Graph_t* graph, int from_x, int from_y, int to_x, int to_y, int* dist, MinHeap_t* minHeap){
	if(graph == NULL || dist == NULL || minHeap == NULL || graph -> rows <= 0 || graph -> cols <= 0)
		return -1;
	
	int rows = graph -> rows;
	int cols = graph -> cols;
	int num_nodes = rows * cols;

	int from_id = coord_to_id(from_x, from_y, cols);
	int to_id = coord_to_id(to_x, to_y, cols);

    if(from_id == to_id)
        return 0;

    for(int i = 0; i < num_nodes; i++){
		dist[i] = INF;
	}
	minHeap -> size = 0;

    dist[from_id] = 0;

	int h_val = 0;
	
	minHeap -> pos[from_id] = 0;
	minHeap -> array[0].id = from_id;
	minHeap -> array[0].dist = dist[from_id] + h_val;
	minHeap -> size = 1;

    int travel_cost = -1;

    while(!is_empty(minHeap)){
        MinHeapNode_t curr_HeapNode = extract_first(minHeap);
        int curr_id = curr_HeapNode.id;

        if(curr_HeapNode.dist > dist[curr_id])
            continue;

        if(curr_id == to_id){
            travel_cost = curr_HeapNode.dist;
            break;
        }

		Node_t* curr_node = &graph -> grid[curr_id];
		Edge_t* curr_edge = curr_node -> head;

		while(curr_edge != NULL){
			int next_x = curr_edge -> destination_x;
			int next_y = curr_edge -> destination_y;
			int next_id = coord_to_id(next_x, next_y, cols);

			int weight = curr_edge -> edge_cost;

			if(weight > 0 && dist[curr_id] != INF && dist[curr_id] + weight < dist[next_id]){
				int old_dist = dist[next_id];
				dist[next_id] = dist[curr_id] + weight;

				int f_new = dist[next_id];

				if(old_dist == INF){
					int i = minHeap -> size;
					minHeap -> array[i].id = next_id;
					minHeap -> array[i]. dist = f_new;
					minHeap -> pos[next_id] = i;
					minHeap -> size++;

					while(i && minHeap -> array[i].dist < minHeap -> array[(i - 1) / 2].dist){
						int parent = (i - 1) / 2;
						minHeap -> pos[minHeap -> array[i].id] = parent;
						minHeap -> pos[minHeap -> array[parent].id] = i;

						switch_MinHeapNode(&minHeap -> array[i], &minHeap -> array[parent]);

						i = parent;
					}
				}else{
					decreaseKey(minHeap, next_id, dist[next_id]);
				}
			}
			curr_edge = curr_edge -> next;
		}
	}

	return travel_cost;
}

int coord_to_id(int x, int y, int cols){
	return x * cols + y;
}

void free_MinHeap(MinHeap_t* minHeap){
	if(minHeap == NULL)
		return;

	free(minHeap -> array);
	free(minHeap -> pos);
	free(minHeap);
}

unsigned int hash_path_key(int from_x, int from_y, int to_x, int to_y){
    unsigned int hash_val = 0;
    hash_val = (hash_val * 31 + from_x);
    hash_val = (hash_val * 31 + from_y);
    hash_val = (hash_val * 31 + to_x);
    hash_val = (hash_val * 31 + to_y);
    return hash_val % CACHE_SIZE;
}

void clear_travel_cost_cache(){
    for(int i = 0; i < CACHE_SIZE; i++){
        CacheEntry_t* current = travel_cost_cache[i];
        while(current != NULL){
            CacheEntry_t* tmp = current;
            current = current -> next;
            free(tmp);
        }
        travel_cost_cache[i] = NULL;
    }
}
