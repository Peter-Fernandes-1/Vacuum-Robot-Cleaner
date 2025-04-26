#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <set>
#include <queue>
#include <stack>
#include <unordered_set>
#include <functional>

//Compile: g++ -std=c++11 -o planner planner.cpp
//Run: ./planner depth-first sample-5x7.txt 


// Struct for robot position
struct Position {
    int row;
    int col;
    
    Position(int r, int c) : row(r), col(c) {}
    
    bool operator==(const Position& other) const {
        return row == other.row && col == other.col;
    }
    
    bool operator!=(const Position& other) const {
        return !(*this == other);
    }
};

// Hash function for Position
namespace std {
    template <>
    struct hash<Position> {
        std::size_t operator()(const Position& pos) const {
            return std::hash<int>()(pos.row) ^ std::hash<int>()(pos.col);
        }
    };
}


// Struct to represent dirty cells as a set of positions
struct DirtyCells {
    std::set<std::pair<int, int> > cells; // Note space between > >
    
    bool operator==(const DirtyCells& other) const {
        return cells == other.cells;
    }
};



// Hash function for DirtyCells
namespace std {
    template <>
    struct hash<DirtyCells> {
        std::size_t operator()(const DirtyCells& dirt) const {
            std::size_t hash_value = 0;
            std::set<std::pair<int, int> >::const_iterator pos;
            for (pos = dirt.cells.begin(); pos != dirt.cells.end(); ++pos) {
                hash_value ^= std::hash<int>()(pos->first) ^ std::hash<int>()(pos->second);
            }
            return hash_value;
        }
    };
}

// State representation for the search
struct State {
    Position robot;
    DirtyCells dirty;
    std::vector<char> actions;
    int cost;
    
    State(Position r, DirtyCells d, const std::vector<char>& a, int c) 
        : robot(r), dirty(d), actions(a), cost(c) {}
};



// Comparison function for priority queue in UCS
struct CompareStates {
    bool operator()(const State& a, const State& b) const {
        return a.cost > b.cost;  // Lower cost has higher priority
    }
};

// Hash function for state (for explored set)
struct StateHash {
    std::size_t operator()(const std::pair<Position, DirtyCells>& state) const {
        std::size_t robot_hash = std::hash<Position>()(state.first);
        std::size_t dirty_hash = std::hash<DirtyCells>()(state.second);
        return robot_hash ^ dirty_hash;
    }
};

// Equality function for state (for explored set)
struct StateEqual {
    bool operator()(const std::pair<Position, DirtyCells>& a, 
                   const std::pair<Position, DirtyCells>& b) const {
        return a.first == b.first && a.second == b.second;
    }
};

class VacuumWorld {
private:
    int rows;
    int cols;
    std::vector<std::string> grid;
    Position robot;
    DirtyCells dirty_cells;



public:
    // Constructor to load world from file
    VacuumWorld(const std::string& filename) : robot(0, 0) {  // Initialize robot with default position
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Error opening file: " << filename << std::endl;
            exit(1);
        }
        
        // Read dimensions
        file >> cols >> rows;
        file.ignore(); // Consume newline
        
        // Read grid
        grid.resize(rows);
        for (int r = 0; r < rows; r++) {
            std::getline(file, grid[r]);
        }
        
        // Find robot and dirty cells
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                if (c < grid[r].length()) {
                    char cell = grid[r][c];
                    if (cell == '@') {
                        robot = Position(r, c);
                    } else if (cell == '*') {
                        std::pair<int, int> dirtyPos(r, c);
                        dirty_cells.cells.insert(dirtyPos);
                    }
                }
            }
        }
        
        file.close();
    }
    
    // Check if a move is valid
    bool isValidMove(int r, int c) const {
        return r >= 0 && r < rows && c >= 0 && c < cols && 
               c < grid[r].length() && grid[r][c] != '#';
    }
    


    // Getters
    Position getRobotPosition() const {
        return robot;
    }
    
    DirtyCells getDirtyCells() const {
        return dirty_cells;
    }
};




// Uniform-Cost Search implementation
std::pair<std::vector<char>, std::pair<int, int> > uniformCostSearch(const VacuumWorld& world) {
    // Initial state
    State initial(world.getRobotPosition(), world.getDirtyCells(), std::vector<char>(), 0);
    
    // Priority queue for frontier
    std::priority_queue<State, std::vector<State>, CompareStates> frontier;
    frontier.push(initial);
    
    // Explored set
    std::unordered_set<std::pair<Position, DirtyCells>, StateHash, StateEqual> explored;
    



    // Statistics
    int nodes_generated = 1;
    int nodes_expanded = 0;
    
    while (!frontier.empty()) {
        // Get state with lowest cost
        State current = frontier.top();
        frontier.pop();
        
        // Check if goal state
        if (current.dirty.cells.empty()) {
            return std::make_pair(current.actions, std::make_pair(nodes_generated, nodes_expanded));
        }
        


        // Create a key for the explored set
        std::pair<Position, DirtyCells> state_key(current.robot, current.dirty);
        
        // Skip if already explored
        if (explored.find(state_key) != explored.end()) {
            continue;
        }
        
        // Mark as explored
        explored.insert(state_key);
        nodes_expanded++;
        


        // Try to vacuum if on dirty cell
        std::pair<int, int> robot_pos(current.robot.row, current.robot.col);
        if (current.dirty.cells.find(robot_pos) != current.dirty.cells.end()) {
            DirtyCells new_dirty = current.dirty;
            new_dirty.cells.erase(robot_pos);
            
            std::vector<char> new_actions = current.actions;
            new_actions.push_back('V');
            
            State new_state(current.robot, new_dirty, new_actions, current.cost + 1);
            frontier.push(new_state);
            nodes_generated++;
        }
        
        // Try to move in four directions
        const int dr[] = {-1, 0, 1, 0};  // N, E, S, W
        const int dc[] = {0, 1, 0, -1};
        const char actions[] = {'N', 'E', 'S', 'W'};
        
        for (int i = 0; i < 4; i++) {
            int new_r = current.robot.row + dr[i];
            int new_c = current.robot.col + dc[i];
            
            if (world.isValidMove(new_r, new_c)) {
                Position new_pos(new_r, new_c);
                
                std::vector<char> new_actions = current.actions;
                new_actions.push_back(actions[i]);
                
                State new_state(new_pos, current.dirty, new_actions, current.cost + 1);
                frontier.push(new_state);
                nodes_generated++;
            }
        }
    }
    


    // No solution found
    std::vector<char> empty_vector;
    return std::make_pair(empty_vector, std::make_pair(nodes_generated, nodes_expanded));
}

// Depth-First Search implementation
std::pair<std::vector<char>, std::pair<int, int> > depthFirstSearch(const VacuumWorld& world) {

    // Initial state
    State initial(world.getRobotPosition(), world.getDirtyCells(), std::vector<char>(), 0);
    
    // Stack for frontier
    std::stack<State> frontier;
    frontier.push(initial);
    


    // Explored set
    std::unordered_set<std::pair<Position, DirtyCells>, StateHash, StateEqual> explored;
    
    // Statistics
    int nodes_generated = 1;
    int nodes_expanded = 0;
    



    while (!frontier.empty()) {
        // Get the top state from stack
        State current = frontier.top();
        frontier.pop();
        
        // Check if goal state
        if (current.dirty.cells.empty()) {
            return std::make_pair(current.actions, std::make_pair(nodes_generated, nodes_expanded));
        }
        


        // Create a key for the explored set
        std::pair<Position, DirtyCells> state_key(current.robot, current.dirty);
        
        // Skip if already explored
        if (explored.find(state_key) != explored.end()) {
            continue;
        }
        
        // Mark as explored
        explored.insert(state_key);
        nodes_expanded++;
        
        // Store possible actions and their resulting states
        std::vector<State> next_states;
        
        // Try to move in four directions
        const int dr[] = {-1, 0, 1, 0};  // N, E, S, W
        const int dc[] = {0, 1, 0, -1};
        const char actions[] = {'N', 'E', 'S', 'W'};
        

        for (int i = 0; i < 4; i++) {
            int new_r = current.robot.row + dr[i];
            int new_c = current.robot.col + dc[i];
            
            if (world.isValidMove(new_r, new_c)) {
                Position new_pos(new_r, new_c);
                
                std::vector<char> new_actions = current.actions;
                new_actions.push_back(actions[i]);
                
                next_states.push_back(State(new_pos, current.dirty, new_actions, current.cost + 1));
            }
        }
        


        // Try to vacuum if on dirty cell
        std::pair<int, int> robot_pos(current.robot.row, current.robot.col);
        if (current.dirty.cells.find(robot_pos) != current.dirty.cells.end()) {
            DirtyCells new_dirty = current.dirty;
            new_dirty.cells.erase(robot_pos);
            
            std::vector<char> new_actions = current.actions;
            new_actions.push_back('V');
            
            next_states.push_back(State(current.robot, new_dirty, new_actions, current.cost + 1));
        }
        
        // Add states to frontier in reverse order (so we explore N, W, S, E from stack - DFS)
        for (std::vector<State>::reverse_iterator it = next_states.rbegin(); it != next_states.rend(); ++it) {
            frontier.push(*it);
            nodes_generated++;
        }
    }
    

    // No solution found
    std::vector<char> empty_vector;
    return std::make_pair(empty_vector, std::make_pair(nodes_generated, nodes_expanded));
}




int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " [algorithm] [world-file]" << std::endl;
        return 1;
    }
    
    std::string algorithm = argv[1];
    std::string world_file = argv[2];
    
    VacuumWorld world(world_file);
    std::pair<std::vector<char>, std::pair<int, int> > result;
    
    if (algorithm == "uniform-cost") {
        result = uniformCostSearch(world);
    } else if (algorithm == "depth-first") {
        result = depthFirstSearch(world);
    } else {
        std::cerr << "Unknown algorithm: " << algorithm << std::endl;
        std::cerr << "Please use 'uniform-cost' or 'depth-first'" << std::endl;
        return 1;
    }
    
    std::vector<char> plan = result.first;
    int nodes_generated = result.second.first;
    int nodes_expanded = result.second.second;
    
    if (!plan.empty()) {
        for (std::vector<char>::const_iterator it = plan.begin(); it != plan.end(); ++it) {
            std::cout << *it << std::endl;
        }
        std::cout << nodes_generated << " nodes generated" << std::endl;
        std::cout << nodes_expanded << " nodes expanded" << std::endl;
    } else {
        std::cout << "No solution found!" << std::endl;
    }
    
    return 0;
}
