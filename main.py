import heapq
from collections import defaultdict, deque
from exporter import gerar_dot



class Graph:
    def __init__(self, num_vertices, is_directed=False):
        self.num_vertices = num_vertices
        self.is_directed = is_directed
        self.vertices = [
            chr(65 + i) if i < 26 else str((i+1)) for i in range(num_vertices)
        ]  # Gera letras A-Z e números para índices >= 26
        self.adj_matrix = [[0] * num_vertices for _ in range(num_vertices)]
        self.adj_list = defaultdict(list)
        self.vertex_map = {v: i for i, v in enumerate(self.vertices)}
        self.vertex_reverse_map = {i: v for i, v in enumerate(self.vertices)}
        self.edge_weights = {}
        self.weights = {}

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
    
    
    def set_vertex_weight(self, vertex, weight):
        """Define o peso de um vértice"""
        if vertex in self.vertices:
            self.weights[vertex] = weight
        else:
            print(f"Vértice {vertex} não encontrado!")

    def get_vertex_weight(self, vertex):
        """Obtém o peso de um vértice."""
        return self.vertex_weights.get(vertex, 1)  # Retorna 1 se o peso não for definido
    
    # def set_vertex_label(self, vertex, label):
    #     """Define a etiqueta (rótulo) do vértice"""
    #     # Aqui podemos usar um dicionário para armazenar a rotulação
    #     if vertex in self.vertices:
    #         self.vertex_labels[vertex] = label
    #     else:
    #         print(f"Vértice {vertex} não encontrado!")

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

    def _grafo_direcionado_aciclico(self):
        """Verifica se um grafo direcionado é acíclico."""
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

    def _grafo_nao_direcionado_aciclico(self):
        """Verifica se um grafo não direcionado é acíclico."""
        def dfs(v, visited, parent):
            visited.add(v)

            for neighbor, _ in self.adj_list[v]:
                if neighbor not in visited:
                    if dfs(neighbor, visited, v):
                        return True
                elif neighbor != parent:
                    return True
            return False

        visited = set()

        for vertex in self.vertices:
            if vertex not in visited:
                if dfs(vertex, visited, None):
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
        
        
        # Verificar se o grafo é conexo
        if not self.grafo_conexo():
            return 'Grafo precisa ser conexo'

        # Contar vértices com grau ímpar
        odd_degree_vertices = sum(1 for v in self.vertices if self.grau_do_vertice(v) % 2 != 0)

        if odd_degree_vertices == 0:
            return "\nÉ Euleriano"  # Todos os vértices têm grau par
        
        else:
            return '\nNão é Euleriano'  # Não é euleriano
        
    def adjacent(self, u, v):
        """
        Verifica se os vértices u e v são adjacentes.
        u e v são índices ou rótulos de vértices.
        """
        u_index = self.vertices.index(u)
        v_index = self.vertices.index(v)
        return self.adj_matrix[u_index][v_index] != 0 and self.adj_matrix[u_index][v_index] != float('inf')

    # Printing the solution
    def print_solution(self, distance, INF=999):
        # Labels dos vértices
        labels = [chr(65 + i) for i in range(self.num_vertices)]  # Gera letras A, B, C...
        
        # Cabeçalho da tabela
        print("   ", "  ".join(labels))  # Espaço inicial para alinhar com as linhas
        for i in range(self.num_vertices):
            # Imprime o rótulo da linha
            print(f"{labels[i]} [", end="")
            # Imprime os valores da matriz para a linha i
            for j in range(self.num_vertices):
                if distance[i][j] == INF:
                    print("INF", end=", " if j < self.num_vertices - 1 else "")
                else:
                    print(distance[i][j], end=", " if j < self.num_vertices - 1 else "")
            print("]")
       
    def convert_matrix_to_inf(self, matrix, INF=999):
        num_vertices = len(matrix)
        converted_matrix = [row[:] for row in matrix]  # Cria uma cópia da matriz
        for i in range(num_vertices):
            for j in range(num_vertices):
                if i != j and converted_matrix[i][j] == 0:
                    converted_matrix[i][j] = INF
        return converted_matrix 
         
    # Floyd Algorithm implementation
    def floyd_warshall(self):
        distance = list(map(lambda i: list(map(lambda j: j, i)), self.convert_matrix_to_inf(self.adj_matrix)))

        # Adding vertices individually
        for k in range(self.num_vertices):
            for i in range(self.num_vertices):
                for j in range(self.num_vertices):
                    distance[i][j] = min(distance[i][j], distance[i][k] + distance[k][j])
        self.print_solution(distance)
    

def main():
    print("Bem-vindo ao gerenciador de grafos!")
    num_vertices = int(input("Digite o número de vértices do grafo: "))
    is_directed = input("O grafo é direcionado? (s/n): ").strip().lower() == 's'
    edges_are_directed = input("Arestas possuem peso?(s) Vertices possuem peso?(n): ").strip().lower() == 's'
    graph = Graph(num_vertices, is_directed)
    # gerar_dot(graph)
    print("\nOs vértices do grafo serão identificados automaticamente como:")
    print(" ".join(graph.vertices))
    gerar_dot(graph)

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
        print("15. Verificar adjacência entre vértices")
        print("16. Sair")

        choice = input("Escolha uma opção: ").strip()

        if choice == "1":
            u = input(f"Digite o vértice de origem ({' '.join(graph.vertices)}): ").strip().upper()
            v = input(f"Digite o vértice de destino ({' '.join(graph.vertices)}): ").strip().upper()
            
            if not edges_are_directed:
                # Definindo o peso do vértice de origem com validação
                while True:
                    try:
                        peso1 = input("Digite o peso do vértice de origem (ou 1 para padrão): ").strip()
                        if peso1 == "":
                            peso1 = 1  # Usa o padrão
                        else:
                            peso1 = int(peso1)
                        break
                    except ValueError:
                        print("Entrada inválida! Por favor, digite um número inteiro.")

                # Definindo o peso do vértice de destino com validação
                while True:
                    try:
                        peso2 = input("Digite o peso do vértice de destino (ou 1 para padrão): ").strip()
                        if peso2 == "":
                            peso2 = 1  # Usa o padrão
                        else:
                            peso2 = int(peso2)
                        break
                    except ValueError:
                        print("Entrada inválida! Por favor, digite um número inteiro.")

                # Definindo o peso da aresta com validação
                while True:
                    try:
                        pesoA = input("Digite o peso da aresta (ou 1 para padrão): ").strip()
                        if pesoA == "":
                            pesoA = 1  # Usa o padrão
                        else:
                            pesoA = int(pesoA)
                        break
                    except ValueError:
                        print("Entrada inválida! Por favor, digite um número inteiro.")

                # Atribuindo os pesos dos vértices e da aresta
                graph.set_vertex_weight(u, peso1)
                graph.set_vertex_weight(v, peso2)
                weight = pesoA

            else:
                # Definindo o peso da aresta com validação em grafos direcionados
                while True:
                    try:
                        weight = input("Digite o peso da aresta (ou 1 para padrão): ").strip()
                        if weight == "":
                            weight = 1
                        else:
                            weight = int(weight)
                        break
                    except ValueError:
                        print("Entrada inválida! Por favor, digite um número inteiro.")
            
            # Adicionando a aresta ao grafo
            graph.add_edge(u, v, weight)
            print("Aresta adicionada com sucesso!")

        elif choice == "2":
            u = input(f"Digite o vértice de origem ({' '.join(graph.vertices)}): ").strip().upper()
            v = input(f"Digite o vértice de destino ({' '.join(graph.vertices)}): ").strip().upper()
            graph.remove_edge(u, v)
            print("Aresta removida com sucesso!")

        elif choice == "3":
            print("\nMatriz de Adjacência:")
            graph.print_adj_matrix()

        elif choice == "4":
            print("\nLista de Adjacência:")
            graph.print_adj_list()

        elif choice == "5":
            start = input(f"\nDigite o vértice inicial ({' '.join(graph.vertices)}): ").strip().upper()
            distances = graph.dijkstra(start)
            print("Menor distância do vértice", start, "para os outros vértices:")
            for vertex, distance in distances.items():
                print(f"Vértice {vertex}: {distance}")

        elif choice == "6":
            if graph.grafo_conexo():
                print("\nO grafo é conexo.")
            else:
                print("\nO grafo não é conexo.")

        elif choice == "7":
            v = input(f"\nDigite o vértice ({' '.join(graph.vertices)}): ").strip().upper()
            print(f"Grau do vértice {v}: {graph.grau_do_vertice(v)}")

        elif choice == "8":
            if graph.grafo_completo():
                print("\nO grafo é completo.")
            else:
                print("\nO grafo não é completo.")

        elif choice == "9":
            if graph.grafo_regular():
                print("\nO grafo é regular.")
            else:
                print("\nO grafo não é regular.")

        elif choice == "10":
            if not is_directed :
                if graph._grafo_nao_direcionado_aciclico():
                    print("\nO grafo é acíclico.")
                else:
                    print("\nO grafo não é acíclico.")
            else:
                if graph._grafo_direcionado_aciclico():
                    print("\nO grafo é acíclico.")
                else:
                    print("\nO grafo não é acíclico.")

        elif choice == "11":
            start = input(f"\nDigite o vértice inicial ({' '.join(graph.vertices)}): ").strip().upper()
            graph.busca_em_profundidade(start)
        elif choice == "12":
            start = input(f"\nDigite o vértice inicial ({' '.join(graph.vertices)}): ").strip().upper()
            graph.busca_em_largura(start)
        
        elif choice == "13":
            print(graph.grafo_euleriano())
        
        elif choice == "14":
            #graph.print_all_pairs_shortest_paths()
            graph.floyd_warshall()
        
        elif choice == "15":
            u = input("Digite o primeiro vértice: ").strip().upper()
            v = input("Digite o segundo vértice: ").strip().upper()
            if u not in graph.vertices or v not in graph.vertices:
                print("Um ou ambos os vértices não existem no grafo.")
            elif graph.adjacent(u, v):
                print(f"Os vértices {u} e {v} são adjacentes.")
            else:
                print(f"Os vértices {u} e {v} não são adjacentes.")

        elif choice == "16":    
            print("\nSaindo do programa. Até logo!")
            break
        else:
                print("Opção inválida. Tente novamente.")

        gerar_dot(graph)
        endVerificator = int(input("\n1 Voltar para o Menu \n2 Encerrar\n"))
        if endVerificator == 2:
            print("Saindo do programa. Até logo!")
            break


if __name__ == "__main__":
    main()
