//
// Created by juan-diego on 3/29/24.
//

#ifndef HOMEWORK_GRAPH_PATH_FINDING_MANAGER_H
#define HOMEWORK_GRAPH_PATH_FINDING_MANAGER_H


#include "window_manager.h"
#include "graph.h"
#include <unordered_map>
#include <set>
#include <unordered_set>


// Este enum sirve para identificar el algoritmo que el usuario desea simular
enum Algorithm {
    None,
    Dijkstra,
    AStar,
    GBFS
};


//* --- PathFindingManager ---
//
// Esta clase sirve para realizar las simulaciones de nuestro grafo.
//
// Variables miembro
//     - path           : Contiene el camino resultante del algoritmo que se desea simular
//     - visited_edges  : Contiene todas las aristas que se visitaron en el algoritmo, notar que 'path'
//                        es un subconjunto de 'visited_edges'.
//     - window_manager : Instancia del manejador de ventana, es utilizado para dibujar cada paso del algoritmo
//     - src            : Nodo incial del que se parte en el algoritmo seleccionado
//     - dest           : Nodo al que se quiere llegar desde 'src'
//*
class PathFindingManager {
    WindowManager *window_manager;
    std::vector<sfLine> path;
    std::vector<sfLine> visited_edges;

    struct Entry {
        Node* node;
        double dist;

        bool operator < (const Entry& other) const {
            return dist < other.dist;
        }
    };

    void dijkstra(Graph &graph) {
        std::unordered_map<Node *, Node *> parent;
        std::unordered_map<Node *, double> dist;
        std::set<Entry> pq;
        
        // Inicializar distancias
        for (auto& [id, node] : graph.nodes) {
            dist[node] = std::numeric_limits<double>::infinity();
        }
        dist[src] = 0.0;
        pq.insert({src, 0.0});
        
        int iteration = 0;  // Contador para renderizar cada N iteraciones
        
        while (!pq.empty()) {
            Node* current = pq.begin()->node;
            pq.erase(pq.begin());
            
            if (current == dest) break;
            
            for (Edge* edge : current->edges) {
                Node* neighbor = (edge->src == current) ? edge->dest : edge->src;
                
                // Tiempo como peso de la arista
                double weight = edge->length / edge->max_speed;
                double new_dist = dist[current] + weight;
                
                if (new_dist < dist[neighbor]) {
                    pq.erase({neighbor, dist[neighbor]});
                    dist[neighbor] = new_dist;
                    parent[neighbor] = current;
                    pq.insert({neighbor, new_dist});
                    
                    // Visualización
                    visited_edges.push_back(sfLine(
                        current->coord, neighbor->coord,
                        sf::Color::White, 1.0f
                    ));
                    
                    // Renderizar cada N iteraciones
                    if (++iteration % timmer == 0) {
                        render();
                    }
                }
            }
        }
        
        render();
        
        set_final_path(parent);
    }
    
    void a_star(Graph &graph) {
        std::unordered_map<Node *, Node *> parent;
        std::unordered_map<Node *, double> g_score;  // Costo real desde src
        std::unordered_map<Node *, double> f_score;  // g_score + heurística
        std::set<Entry> pq;
        
        // Función heurística -> distancia euclidiana
        auto heuristic = [](Node* a, Node* b) -> double {
            float dx = a->coord.x - b->coord.x;
            float dy = a->coord.y - b->coord.y;
            double euclidean_dist = std::sqrt(dx * dx + dy * dy);
            return euclidean_dist;
        };
        
        // Inicializar costos
        for (auto& [id, node] : graph.nodes) {
            g_score[node] = std::numeric_limits<double>::infinity();
            f_score[node] = std::numeric_limits<double>::infinity();
        }
        g_score[src] = 0.0;
        f_score[src] = heuristic(src, dest);
        pq.insert({src, f_score[src]});
        
        int iteration = 0;  // Contador para renderizar
        
        while (!pq.empty()) {
            Node* current = pq.begin()->node;
            pq.erase(pq.begin());
            
            if (current == dest) break;
            
            for (Edge* edge : current->edges) {
                Node* neighbor = (edge->src == current) ? edge->dest : edge->src;
                
                // Tiempo como peso de la arista
                double weight = edge->length / edge->max_speed;
                double tentative_g_score = g_score[current] + weight;
                
                if (tentative_g_score < g_score[neighbor]) {
                    // Actualizar el camino
                    parent[neighbor] = current;
                    g_score[neighbor] = tentative_g_score;
                    f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, dest);

                    pq.erase({neighbor, f_score[neighbor]});
                    pq.insert({neighbor, f_score[neighbor]});
                    
                    // Visualización
                    visited_edges.push_back(sfLine(
                        current->coord, neighbor->coord,
                        sf::Color::White, 1.0f
                    ));
                    
                    // Renderizar cada N iteraciones
                    if (++iteration % timmer == 0) {
                        render();
                    }
                }
            }
        }

        render();
        
        set_final_path(parent);
    }
    void best_first_search(Graph &graph) {
        std::unordered_map<Node *, Node *> parent;
        std::unordered_map<Node *, double> h_score;
        std::set<Entry> pq;
        
        // Función heurística: distancia euclidiana
        auto heuristic = [](Node* a, Node* b) -> double {
            float dx = a->coord.x - b->coord.x;
            float dy = a->coord.y - b->coord.y;
            double euclidean_dist = std::sqrt(dx * dx + dy * dy);
            return euclidean_dist;
        };
        
        // Inicializar scores
        for (auto& [id, node] : graph.nodes) {
            h_score[node] = std::numeric_limits<double>::infinity();
        }
        h_score[src] = heuristic(src, dest);
        pq.insert({src, h_score[src]});
        
        // Set para rastrear nodos visitados
        std::unordered_set<Node*> visited;
        
        int iteration = 0;
        
        while (!pq.empty()) {
            Node* current = pq.begin()->node;
            pq.erase(pq.begin());
            
            // Marcar como visitado
            if (visited.find(current) != visited.end()) {
                continue;
            }
            visited.insert(current);
            
            if (current == dest) break;
            
            for (Edge* edge : current->edges) {
                Node* neighbor = (edge->src == current) ? edge->dest : edge->src;
                
                // Si ya fue visitado, saltar
                if (visited.find(neighbor) != visited.end()) {
                    continue;
                }
                
                // solo se usa la heurística
                if (parent.find(neighbor) == parent.end()) {
                    parent[neighbor] = current;
                    h_score[neighbor] = heuristic(neighbor, dest);
                    pq.insert({neighbor, h_score[neighbor]});
                    
                    // Visualización
                    visited_edges.push_back(sfLine(
                        current->coord, neighbor->coord,
                        sf::Color::White, 1.0f
                    ));

                    if (++iteration % timmer == 0) {
                        render();
                    }
                }
            }
        }
        
        render();
        
        set_final_path(parent);
    }
    //* --- render ---
    // En cada iteración de los algoritmos esta función es llamada para dibujar los cambios en el 'window_manager'
    void render() {
        sf::sleep(sf::milliseconds(10));

        window_manager->clear();
        if (graph_ptr != nullptr) {
            graph_ptr->draw();
        }
        
        // Dibujar las aristas visitadas hasta ahora
        for (sfLine &line : visited_edges) {
            line.draw(window_manager->get_window(), sf::RenderStates::Default);
        }
        
        // Dibujar el nodo inicial (verde)
        if (src != nullptr) {
            src->draw(window_manager->get_window());
        }
        
        // Dibujar el nodo final (celeste)
        if (dest != nullptr) {
            dest->draw(window_manager->get_window());
        }
        
        // Mostrar el frame actual
        window_manager->display();
    }

    //* --- set_final_path ---
    // Esta función se usa para asignarle un valor a 'this->path' al final de la simulación del algoritmo.
    // 'parent' es un std::unordered_map que recibe un puntero a un vértice y devuelve el vértice anterior a el,
    // formando así el 'path'.
    //
    // ej.
    //     parent(a): b
    //     parent(b): c
    //     parent(c): d
    //     parent(d): NULL
    //
    // Luego, this->path = [Line(a.coord, b.coord), Line(b.coord, c.coord), Line(c.coord, d.coord)]
    //
    // Este path será utilizado para hacer el 'draw()' del 'path' entre 'src' y 'dest'.
    //*
    void set_final_path(std::unordered_map<Node *, Node *> &parent) {
        Node* current = dest;

        // Verificar que existe un camino
        if (parent.find(dest) == parent.end()) {
            return;
        }
        
        // Camino desde dest hasta src
        while (current != nullptr && parent.find(current) != parent.end()) {
            Node* prev = parent[current];
            
            path.push_back(sfLine(
                prev->coord, 
                current->coord,
                sf::Color(230, 5, 203),
                3.0f
            ));
            
            current = prev;
        }
    }

public:
    Node *src = nullptr;
    Node *dest = nullptr;
    Graph *graph_ptr = nullptr; // referencia al grafo
    unsigned timmer = 400;

    explicit PathFindingManager(WindowManager *window_manager) : window_manager(window_manager) {}

    void exec(Graph &graph, Algorithm algorithm) {
        if (src == nullptr || dest == nullptr) {
            return;
        }

        graph_ptr = &graph;

        path.clear();
        visited_edges.clear();
        
        // Ejecutar algoritmo seleccionado
        switch (algorithm) {
            case Dijkstra:
                dijkstra(graph);
                break;
            case AStar:
                a_star(graph);
                break;
            case GBFS:
                best_first_search(graph);
                break;
            case None:
            default:
                break;
        }

        graph_ptr = nullptr;
    }

    void reset() {
        path.clear();
        visited_edges.clear();

        if (src) {
            src->reset();
            src = nullptr;
            // ^^^ Pierde la referencia luego de restaurarlo a sus valores por defecto
        }
        if (dest) {
            dest->reset();
            dest = nullptr;
            // ^^^ Pierde la referencia luego de restaurarlo a sus valores por defecto
        }
    }

    void draw(bool draw_extra_lines) {
        // Dibujar todas las aristas visitadas
        if (draw_extra_lines) {
            for (sfLine &line: visited_edges) {
                line.draw(window_manager->get_window(), sf::RenderStates::Default);
            }
        }

        // Dibujar el camino resultante entre 'str' y 'dest'
        for (sfLine &line: path) {
            line.draw(window_manager->get_window(), sf::RenderStates::Default);
        }

        // Dibujar el nodo inicial
        if (src != nullptr) {
            src->draw(window_manager->get_window());
        }

        // Dibujar el nodo final
        if (dest != nullptr) {
            dest->draw(window_manager->get_window());
        }
    }
};


#endif //HOMEWORK_GRAPH_PATH_FINDING_MANAGER_H
