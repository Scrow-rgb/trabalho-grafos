import heapq
from collections import defaultdict, deque

from exporter import gerar_dot



class Graph:
    def __init__(self, num_vertices, is_directed=False):
        self.num_vertices = num_vertices
        self.is_directed = is_directed
        self.vertices = [chr(65 + i) for i in range(num_vertices)]  # Gera letras A, B, C...
        self.adj_matrix = [[0] * num_vertices for _ in range(num_vertices)]
        self.adj_list = defaultdict(list)
        self.vertex_map = {v: i for i, v in enumerate(self.vertices)}
        self.vertex_reverse_map = {i: v for i, v in enumerate(self.vertices)}
        self.edge_weights = {}

    def add_edge(self, u, v, weight=1):
        """Adiciona uma aresta entre os vértices u e v."""
        u_idx, v_idx = self.vertex_map[u], self.vertex_map[v]
        self.adj_matrix[u_idx][v_idx] = weight
        self.adj_list[u].append((v, weight))
        if not self.is_directed:
            self.adj_matrix[v_idx][u_idx] = weight
            self.adj_list[v].append((u, weight))
        self.edge_weights[(u, v)] = weight
        if not self.is_directed:
            self.edge_weights[(v, u)] = weight  # Aresta bidirecional para grafos não direcionados

    def remove_edge(self, u, v):
        """Remove a aresta entre os vértices u e v."""
        u_idx, v_idx = self.vertex_map[u], self.vertex_map[v]
        self.adj_matrix[u_idx][v_idx] = 0
        self.adj_list[u] = [(vertex, w) for vertex, w in self.adj_list[u] if vertex != v]
        if not self.is_directed:
            self.adj_matrix[v_idx][u_idx] = 0
            self.adj_list[v] = [(vertex, w) for vertex, w in self.adj_list[v] if vertex != u]
        self.edge_weights.pop((u, v), None)
        if not self.is_directed:
            self.edge_weights.pop((v, u), None)

    def print_adj_matrix(self):
        """Exibe a matriz de adjacência."""
        print("  ", "  ".join(self.vertices))
        for i, row in enumerate(self.adj_matrix):
            print(f"{self.vertices[i]} {row}")
    

    def print_adj_list(self):
        """Exibe a lista de adjacência."""
        for key, value in self.adj_list.items():
            print(f"{key}: {value}")

    def dijkstra(self, start):
        """Calcula a menor distância de um vértice para todos os outros."""
        distances = {v: float('inf') for v in self.vertices}
        distances[start] = 0
        priority_queue = [(0, start)]

        while priority_queue:
            current_distance, current_vertex = heapq.heappop(priority_queue)
            if current_distance > distances[current_vertex]:
                continue

            for neighbor, weight in self.adj_list[current_vertex]:
                distance = current_distance + weight
                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    heapq.heappush(priority_queue, (distance, neighbor))

        return distances

    def grau_do_vertice(self, vertice):
        """Retorna o grau do vértice (número de arestas incidentes)."""
        return len(self.adj_list[vertice])

    def grafo_conexo(self):
        """Verifica se o grafo é conexo (somente para grafos não direcionados)."""
        def dfs(v, visited):
            visited.add(v)
            for neighbor, _ in self.adj_list[v]:
                if neighbor not in visited:
                    dfs(neighbor, visited)

        visited = set()
        dfs(self.vertices[0], visited)
        return len(visited) == self.num_vertices

    def grafo_completo(self):
        """Verifica se o grafo é completo (se todos os vértices são conectados)."""
        for i in range(self.num_vertices):
            for j in range(self.num_vertices):
                if i != j and self.adj_matrix[i][j] == 0:
                    return False
        return True

    def grafo_regular(self):
        """Verifica se o grafo é regular (todos os vértices têm o mesmo grau)."""
        grau = self.grau_do_vertice(self.vertices[0])
        for v in self.vertices:
            if self.grau_do_vertice(v) != grau:
                return False
        return True

    def grafo_aciclico(self):
        """Verifica se o grafo é acíclico (somente para grafos direcionados)."""
        def dfs(v, visited, rec_stack):
            visited.add(v)
            rec_stack.add(v)

            for neighbor, _ in self.adj_list[v]:
                if neighbor not in visited and dfs(neighbor, visited, rec_stack):
                    return True
                elif neighbor in rec_stack:
                    return True
            rec_stack.remove(v)
            return False

        visited = set()
        rec_stack = set()

        for vertex in self.vertices:
            if vertex not in visited:
                if dfs(vertex, visited, rec_stack):
                    return False
        return True

    def busca_em_profundidade(self, start):
        """Realiza uma busca em profundidade (DFS) a partir do vértice 'start'."""
        visited = set()
        def dfs(v):
            visited.add(v)
            print(v, end=" ")
            for neighbor, _ in self.adj_list[v]:
                if neighbor not in visited:
                    dfs(neighbor)

        print("Busca em profundidade a partir de", start)
        dfs(start)
        print()

    def busca_em_largura(self, start):
        """Realiza uma busca em largura (BFS) a partir do vértice 'start'."""
        visited = set()
        queue = deque([start])
        visited.add(start)

        while queue:
            vertex = queue.popleft()
            print(vertex, end=" ")

            for neighbor, _ in self.adj_list[vertex]:
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(neighbor)
                    
                    
    def grafo_euleriano(self):
        """Verifica se o grafo é euleriano (caminho ou circuito euleriano)."""
        if not self.grafo_conexo():
            return False  # O grafo precisa ser conexo

        odd_degree_vertices = 0
        for v in self.vertices:
            if self.grau_do_vertice(v) % 2 != 0:
                odd_degree_vertices += 1

        # Para caminho euleriano, o grafo deve ter exatamente dois vértices de grau ímpar
        # Para circuito euleriano, o grafo deve ter zero vértices de grau ímpar
        return odd_degree_vertices == 0 or odd_degree_vertices == 2
    
    def floyd_warshall(self):
        """Calcula as menores distâncias entre todos os pares de vértices."""
        dist = [row[:] for row in self.adj_matrix]  # Cria uma cópia da matriz de adjacência

        # Aplica o algoritmo de Floyd-Warshall
        for k in range(self.num_vertices):
            for i in range(self.num_vertices):
                for j in range(self.num_vertices):
                    if dist[i][j] > dist[i][k] + dist[k][j]:
                        dist[i][j] = dist[i][k] + dist[k][j]

        return dist

    def print_all_pairs_shortest_paths(self):
        """Exibe as menores distâncias entre todos os pares de vértices."""
        dist = self.floyd_warshall()
        print("Menores distâncias entre todos os pares de vértices:")
        for i in range(self.num_vertices):
            for j in range(self.num_vertices):
                if dist[i][j] == float('inf'):
                    print(f"De {self.vertices[i]} para {self.vertices[j]}: Infinito")
                else:
                    print(f"De {self.vertices[i]} para {self.vertices[j]}: {dist[i][j]}")





def main():
    print("Bem-vindo ao gerenciador de grafos!")
    num_vertices = int(input("Digite o número de vértices do grafo: "))
    is_directed = input("O grafo é direcionado? (s/n): ").strip().lower() == 's'
    graph = Graph(num_vertices, is_directed)

    print("\nOs vértices do grafo serão identificados automaticamente como:")
    print(" ".join(graph.vertices))

    while True:
        print("\nMenu:")
        print("1. Adicionar aresta")
        print("2. Remover aresta")
        print("3. Mostrar matriz de adjacência")
        print("4. Mostrar lista de adjacência")
        print("5. Calcular menor distância (Dijkstra)")
        print("6. Verificar se o grafo é conexo")
        print("7. Grau do vértice")
        print("8. Verificar se o grafo é completo")
        print("9. Verificar se o grafo é regular")
        print("10. Verificar se o grafo é acíclico")
        print("11. Busca em profundidade")
        print("12. Busca em largura")
        print("13. Grafo Euleriano")
        print("14. Calcular menor distância (Floyd Warshall)")
        print("15. Sair")

        choice = input("Escolha uma opção: ").strip()

        if choice == "1":
            u = input(f"Digite o vértice de origem ({' '.join(graph.vertices)}): ").strip().upper()
            v = input(f"Digite o vértice de destino ({' '.join(graph.vertices)}): ").strip().upper()
            weight = int(input("Digite o peso da aresta (ou 1 para padrão): "))
            graph.add_edge(u, v, weight)
            print("Aresta adicionada com sucesso!")
        elif choice == "2":
            u = input(f"Digite o vértice de origem ({' '.join(graph.vertices)}): ").strip().upper()
            v = input(f"Digite o vértice de destino ({' '.join(graph.vertices)}): ").strip().upper()
            graph.remove_edge(u, v)
            print("Aresta removida com sucesso!")
        elif choice == "3":
            print("Matriz de Adjacência:")
            gerar_dot(graph)
            graph.print_adj_matrix()
        elif choice == "4":
            print("Lista de Adjacência:")
            graph.print_adj_list()
        elif choice == "5":
            start = input(f"Digite o vértice inicial ({' '.join(graph.vertices)}): ").strip().upper()
            distances = graph.dijkstra(start)
            print("Menor distância do vértice", start, "para os outros vértices:")
            for vertex, distance in distances.items():
                print(f"Vértice {vertex}: {distance}")
        elif choice == "6":
            if graph.grafo_conexo():
                print("O grafo é conexo.")
            else:
                print("O grafo não é conexo.")
        elif choice == "7":
            v = input(f"Digite o vértice ({' '.join(graph.vertices)}): ").strip().upper()
            print(f"Grau do vértice {v}: {graph.grau_do_vertice(v)}")
        elif choice == "8":
            if graph.grafo_completo():
                print("O grafo é completo.")
            else:
                print("O grafo não é completo.")
        elif choice == "9":
            if graph.grafo_regular():
                print("O grafo é regular.")
            else:
                print("O grafo não é regular.")
        elif choice == "10":
            if graph.grafo_aciclico():
                print("O grafo é acíclico.")
            else:
                print("O grafo não é acíclico.")
        elif choice == "11":
            start = input(f"Digite o vértice inicial ({' '.join(graph.vertices)}): ").strip().upper()
            graph.busca_em_profundidade(start)
        elif choice == "12":
            start = input(f"Digite o vértice inicial ({' '.join(graph.vertices)}): ").strip().upper()
            graph.busca_em_largura(start)
        
        elif choice == "13":
            if graph.grafo_euleriano():
                print("O grafo é euleriano.")
            else:
                print("O grafo não é euleriano.")
        
        elif choice == "14":
            graph.print_all_pairs_shortest_paths()


        
        elif choice == "15":
            print("Saindo do programa. Até logo!")
            break
        else:
            print("Opção inválida. Tente novamente.")

if __name__ == "__main__":
    main()
