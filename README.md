[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/5zgGDtf4)
[![Open in Visual Studio Code](https://classroom.github.com/assets/open-in-vscode-2e0aaae1b6195c2367325f4f02e2d04e9abb55f0b24a779b69b11b9e10269abc.svg)](https://classroom.github.com/online_ide?assignment_repo_id=21626637&assignment_repo_type=AssignmentRepo)
# Tarea de Grafos

## Integrantes: 
- 1 Fabricio Alonso Lanche Pacsi
- 2 Jhogan Haldo Pachacutec Aguilar
- 3 Juan Carlos Ticlia Maqui

## Objetivo: 
El objetivo de esta tarea es implementar un **Path Finder** para la ciudad de Lima. 

<p align="center">
    <img src=https://github.com/utec-cs-aed/homework_graph/assets/79115974/b63f69db-17eb-417a-8aa1-8483d8dcdaf0 / >
</p>

## Dependencias

Para esta tarea se solicita utilizar ```C++17``` y la librería ```SFML 2.5```

- Para instalar ```SFML 2.5```:

    - [Windows](https://www.youtube.com/watch?v=HkPRG0vfObc)
    - [MacOS y Linux](https://www.youtube.com/playlist?list=PLvv0ScY6vfd95GMoMe2zc4ZgGxWYj3vua)

Cuando se instale la librería, probar que las siguientes líneas del ```CMakeLists.txt``` encuentren la librería adecuadamente.
```cmake
find_package(SFML 2.5 COMPONENTS graphics window REQUIRED)
if(SFML_FOUND)
    target_link_libraries(${PROJECT_NAME} PRIVATE sfml-graphics sfml-window)
else()
    message("SFML not found")
endif()
```

## Dataset
El dataset consiste de dos csv:

- *nodes.csv*

    ![image](https://github.com/utec-cs-aed/homework_graph/assets/79115974/6a68cf06-196a-4605-83a7-3183e9a3f0ec)


- *edges.csv*

    ![image](https://github.com/utec-cs-aed/homework_graph/assets/79115974/247bbbd7-6203-45f4-8196-fcb0434b0f1d)


## Algoritmos

Se les solicita implementar tres algoritmos para busqueda en grafos

- *Dijkstra*

- *Best First Search*

- *A**

Además:
- Analice la complejidad computacional de los tres algoritmos de acuerdo a su propia implementación.
- Puede considere como heuristica la distancia en linea recta.
- **Debe realizar un pequeño video (2 min) mostrando la funcionalidad visual de cada algoritmo**

## Diagrama de clases UML 

![image](https://github.com/utec-cs-aed/homework_graph/assets/79115974/f5a3d89e-cb48-4715-b172-a17e6e27ee24)
 
## Análisis de complejidad

En esta sección se presenta el análisis de complejidad de los algoritmos implementados: Dijkstra, A* y Best-First Search. Se incluyen las fórmulas relevantes en formato matemático (KaTeX) y descripciones por partes.

**1. Dijkstra**

1.1 Peso (tiempo) por arista

La métrica de costo usada en la implementación es el tiempo:

$$
    {tiempo} = \frac{\text{longitud}}{\text{velocidad maxima}}
$$

1.2 Complejidad por partes

- Inicialización

```cpp
for (auto& [id, node] : graph.nodes) dist[node] = infinity;
dist[src] = 0;
pq.insert({src, 0});
```

Recorre $V$ nodos → $O(V)$ y cada inserción en la cola/set → $O(\log V)$.\
Complejidad: $O(V)$ (operación de inicialización general con inserciones en estructuras).

- Bucle principal

```cpp
while (!pq.empty()) {
    current = pq.begin()->node;
    pq.erase(pq.begin());
    // ...
}
```

Cada nodo se extrae una vez → $V$ veces.\
`erase` en set/cola prioridad → $O(\log V)$.\
Complejidad: $O(V\log V)$.

- Recorrido de aristas

```cpp
for (Edge* edge : current->edges) {
    if (new_dist < dist[neighbor]) {
        pq.erase(...);
        pq.insert(...);
    }
}
```

Cada arista se recorre como máximo una vez → $E$ veces.\
Cada `erase`/`insert` en la estructura de prioridad → $O(\log V)$.\
Complejidad: $O(E\log V)$.

1.3 Cálculo total (Dijkstra sin render)

$$
T_{total} = O\big((V+E)\log V\big)
$$

Espacio: estructuras `dist`, `parent`, `pq` → $O(V)$.

1.4 Complejidad adicional con renderizado

La función `render()` dibuja el grafo y las aristas visitadas;
por simplicidad consideramos que dibujar todo el grafo cuesta $O(V+E)$ y dibujar las aristas visitadas hasta el momento cuesta $O(m)$ con $m\le E$.

Costo por llamada a `render`:

$$
T_{render} = O(V+E)
$$

Si la función `render()` se llama cada `timmer` relajaciones y en total se relajan $L$ aristas (peor caso $L=E$), el número de renders es

$$
R = \left\lfloor\frac{L}{timmer}\right\rfloor + 1
$$

Por tanto, el costo total de render es

$$
T_{render\_total} = O\left(\frac{L}{timmer}(V+E)\right)
$$

En el peor caso ($L=E$):

$$\boxed{T_{render\_total} = O\left(\frac{E}{timmer}(V+E)\right)}$$

1.5 Complejidad total de Dijkstra con render

$$\boxed{T_{total} = O\big((V+E)\log V\big) \; + \; O\left(\frac{E}{timmer}(V+E)\right)}$$

**2. A* — Análisis de complejidad**

2.1 Idea general

A* es similar a Dijkstra pero añade una heurística $h(n)$ que empuja la búsqueda hacia el destino, reduciendo en la práctica la cantidad de nodos explorados.

2.2 Heurística

Usamos la distancia euclidiana como heurística (coste $O(1)$ por evaluación):

```cpp
float dx = neighbor->coord.x - dest->coord.x;
float dy = neighbor->coord.y - dest->coord.y;
return sqrt(dx*dx + dy*dy);
```

2.3 Complejidad por partes

- Inicialización: igual que Dijkstra pero con dos mapas (`g_score`, `f_score`) → $O(V)$.

- Bucle principal: en el peor caso explora casi todo → $O(V\log V)$, en la práctica explora $V' \ll V$ y cuesta $O(V'\log V)$.

- Recorrido de aristas: similar a Dijkstra, con coste heurístico $O(1)$ adicional por vecino → $O(E\log V)$ peor caso.

2.4 Complejidad final de A*

Peor caso (heurística mala):

$$\boxed{O\big((V+E)\log V\big)}$$

Caso práctico con heurística buena (expande menos nodos):

$$O\big((V'+E')\log V\big) \quad \text{con } V' \ll V$$

Espacio: similar a Dijkstra pero con mapas extra → $O(V)$.

2.5 Complejidad adicional por render (A*)

Idéntica a la formulación de Dijkstra, reemplazando $E$ por $E'$ (aristas realmente exploradas):

$$T_{render\_total} = O\left(\frac{E'}{timmer}(V+E)\right)$$

2.6 Complejidad total (A*) con render

$$\boxed{T_{total} = O\big((V+E)\log V\big) \; + \; O\left(\frac{E'}{timmer}(V+E)\right)}$$ 

**3. Best-First Search**

3.1 Idea general

Best-First Search (Greedy) selecciona siempre el nodo más prometedor según la heurística $h(n)$ (distancia euclidiana al destino). No acumula el costo real (no usa $g$), por lo que no garantiza optimalidad, pero a menudo explora menos nodos en mapas geométricos.

3.2 Heurística

Se usa la misma distancia euclidiana (costo $O(1)$ por evaluación).

3.3 Complejidad por partes

- Inicialización: $O(V)$ por asignaciones y $O(\log V)$ por inserciones en la cola.

```cpp 
for (...) h_score[node] = infinity;
h_score[src] = heuristic(src, dest);
pq.insert(...);
```

- Bucle principal: cada extracción cuesta $O(\log V)$. En total $O(V\log V)$ en el peor caso.
```cpp 
while (!pq.empty()) {
    current = pq.begin()->node;
    pq.erase(pq.begin());
    if (visited.contains(current)) continue;
    visited.insert(current);
}
```

- Recorrido de aristas: cada arista se examina a lo más una vez → $O(E)$, y cada inserción en la cola cuesta $O(\log V)$ → $O(E\log V)$.
```cpp 
for (Edge* edge : current->edges) {
    if (!visited[neighbor]) {
        parent[neighbor] = current;
        h_score[neighbor] = heuristic(neighbor);
        pq.insert({...});
    }
}
```

3.4 Complejidad final de Best-First

$$\boxed{O\big((V+E)\log V\big)}$$

Espacio: `parent`, `h_score`, `visited`, `pq` → $O(V)$.

3.5 Complejidad adicional con `render()`

La formulación es la misma que en los apartados anteriores: si $L$ aristas son efectivamente relajadas/examinadas,

$$T_{render\_total} = O\left(\frac{L}{timmer}(V+E)\right)$$

3.6 Complejidad total (Best-First) con `render()`

$$\boxed{T_{total} = O\big((V+E)\log V\big) \; + \; O\left(\frac{L}{timmer}(V+E)\right)}$$

**Comparación general (peor caso y renderizado)**

Dijkstra, A* y Best-First comparten el mismo peor caso asintótico:

$$
O\big((V+E)\log V\big)
$$

La diferencia práctica está en cuántos nodos exploran:

- Dijkstra puede explorar todo el grafo si es necesario.\
- A* explora mucho menos cuando la heurística es buena, pudiendo reducir el número de nodos explorados a $V' \ll V$.\
- Best-First Search suele ser el más rápido en exploración porque solo usa la heurística (no acumula costo real), pero no garantiza encontrar la ruta óptima.

Respecto al renderizado, para los tres algoritmos su costo adicional (si se renderiza cada `timmer` relajaciones/visitas) es:

$$
T_{render\_total} = O\left(\frac{L}{timmer}(V+E)\right)
$$

Este término puede dominar el coste total cuando `timmer` es pequeño o el grafo es grande (L grande).

----------
> **Créditos:** Juan Diego Castro Padilla [juan.castro.p@utec.edu.pe](mailto:juan.castro.p@utec.edu.pe)




