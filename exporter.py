from main import Grafo  # Importando a classe Grafo do arquivo main

def gerar_dot(grafo_obj):
    with open("grafo.dot", 'w') as arquivo:
        # Escolher o tipo de grafo (direcionado ou não)
        if grafo_obj.direcionado:
            arquivo.write("digraph G {\n")
            operador = "->"
        else:
            arquivo.write("graph G {\n")
            operador = "--"

        # Estilização dos nós
        for i in range(len(grafo_obj.matriz_adjacencia)):
            arquivo.write(f"    {i + 1} [label=\"{i + 1}\", penwidth=2, shape=\"circle\"];\n")

        # Estilização das arestas
        for i in range(len(grafo_obj.matriz_adjacencia)):
            for j in range(len(grafo_obj.matriz_adjacencia[i])):
                if grafo_obj.matriz_adjacencia[i][j] != 0:  # Verifica se há uma aresta (ou peso diferente de zero)
                    # Para grafos não direcionados, evitar duplicar arestas
                    if not grafo_obj.direcionado and j <= i:
                        continue
                    
                    # Inclui o peso se o grafo for ponderado
                    if grafo_obj.ponderado:
                        peso = grafo_obj.matriz_adjacencia[i][j]
                        arquivo.write(f"    {i + 1} {operador} {j + 1} [penwidth=3, color=black, label=\"{peso}\"];\n")
                    else:
                        arquivo.write(f"    {i + 1} {operador} {j + 1} [penwidth=3, color=black];\n")
        arquivo.write("}")
