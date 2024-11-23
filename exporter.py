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
            arquivo.write(f"    {i} [label=\"{grafo_obj.vertices[i]}\", penwidth=2, shape=\"circle\"];\n")

        # Estilização das arestas
        for i in range(len(grafo_obj.adj_matrix)):
            for j in range(len(grafo_obj.adj_matrix[i])):
                if grafo_obj.adj_matrix[i][j] != float('inf') and grafo_obj.adj_matrix[i][j] != 0:  # Aresta com peso
                    # Para grafos não direcionados, evitar duplicar arestas
                    if not grafo_obj.is_directed and j <= i:
                        continue
                    
                    # Inclui o peso se o grafo for ponderado
                    if grafo_obj.adj_matrix[i][j] != 1:  # Peso diferente de 1, considera ponderado
                        peso = grafo_obj.adj_matrix[i][j]
                        arquivo.write(f"    {i} {operador} {j} [penwidth=3, color=black, label=\"{peso}\"];\n")
                    else:
                        arquivo.write(f"    {i} {operador} {j} [penwidth=3, color=black];\n")
        
        arquivo.write("}")
