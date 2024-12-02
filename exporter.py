def gerar_dot(grafo_obj):
    with open("grafo.dot", 'w') as arquivo:
        # Escolher o tipo de grafo (direcionado ou não)
        if grafo_obj.is_directed:
            arquivo.write("digraph G {\n")
            operador = "->"
        else:
            arquivo.write("graph G {\n")
            operador = "--"

        # Estilização dos nós
        for i in range(len(grafo_obj.adj_matrix)):
            peso = grafo_obj.weights.get(grafo_obj.vertices[i], 0)
            arquivo.write(f"    {i} [label=\"{grafo_obj.vertices[i]} (Peso: {peso})\", penwidth=2, shape=\"circle\"];\n")

        # Estilização das arestas
        for i in range(len(grafo_obj.adj_matrix)):
            for j in range(len(grafo_obj.adj_matrix[i])):
                # Verifica se há uma aresta entre i e j
                if grafo_obj.adj_matrix[i][j] != float('inf') and grafo_obj.adj_matrix[i][j] != 0:  
                    # Para grafos não direcionados, evita duplicar arestas
                    if not grafo_obj.is_directed and j <= i:
                        continue
                    
                    # Obtém o peso da aresta
                    peso = grafo_obj.adj_matrix[i][j]
                    
                    # Se for um grafo ponderado, inclui o peso na aresta
                    if peso != 1:  # Se o peso não for o valor padrão (1), considera ponderado
                        arquivo.write(f"    {i} {operador} {j} [penwidth=2, color=black, label=\"{peso}\"];\n")
                    else:
                        arquivo.write(f"    {i} {operador} {j} [penwidth=2, color=black];\n")
        
        arquivo.write("}")
