# **Biblioteca de Manipulação de Grafos**

Esta biblioteca permite a criação, manipulação e análise de grafos utilizando **Matriz de Adjacência** e **Lista de Adjacência**. Além disso, ela inclui funções para adicionar e remover arestas, ponderar e rotular vértices e arestas, e implementar diversos algoritmos de análise de grafos.

---

## **Funcionalidades da Biblioteca**

### **1. Criação de Grafos**
A biblioteca permite a criação de grafos com **X vértices**, onde o número de vértices é inserido pelo usuário no momento da criação do grafo.

- **Representação do grafo**: Você pode escolher entre usar uma **Matriz de Adjacência** ou uma **Lista de Adjacência** para representar o grafo.

---

### **2. Inserção e Remoção de Arestas**
A biblioteca permite adicionar e remover arestas entre dois vértices.

- **Inserção de aresta**: Adicione uma aresta entre dois vértices, com a possibilidade de definir um peso para a aresta.
- **Remoção de aresta**: Remova uma aresta entre dois vértices.

---

### **3. Ponderação e Rotulação de Vértices**
Você pode atribuir um peso e um rótulo (nome) a cada vértice.

- **Ponderação de vértices**: Atribua um peso a um vértice específico.
- **Rotulação de vértices**: Dê um nome ou identificador para o vértice.

---

### **4. Ponderação e Rotulação de Arestas**
Atribua pesos e rótulos (nomes) para as arestas que conectam os vértices.

- **Ponderação de arestas**: Atribua um peso a uma aresta específica.
- **Rotulação de arestas**: Dê um nome ou identificador para a aresta.

---

### **5. Funções de Checagem no Menu**

A biblioteca oferece várias funções para checar propriedades do grafo, como:

- **Checagem de Adjacência entre Vértices**: Verifique se dois vértices são adjacentes, ou seja, se existe uma aresta entre eles.
- **Vizinhança de um Vértice**: Retorne os vértices vizinhos de um vértice específico.
- **Grau de um Vértice**: Calcule o grau de um vértice, que é o número de arestas conectadas a ele.
- **Grafo Completo**: Verifique se o grafo é completo, ou seja, se todos os vértices estão conectados entre si.
- **Grafo Regular**: Verifique se o grafo é regular, ou seja, se todos os vértices possuem o mesmo grau.
- **Grafo Conexo**: Verifique se o grafo é conexo, ou seja, se há um caminho entre qualquer par de vértices.
- **Grafo Acíclico**: Verifique se o grafo é acíclico, ou seja, se não contém ciclos.
- **Grafo Euleriano**: Verifique se o grafo é euleriano, ou seja, se existe um caminho que passa por todas as arestas exatamente uma vez.

---

### **6. Algoritmos de Análise de Grafos no Menu**

A biblioteca implementa os seguintes algoritmos para análise de grafos:

- **Busca em Profundidade (DFS)**: Realiza uma busca recursiva pelo grafo a partir de um vértice inicial.
- **Busca em Largura (BFS)**: Realiza uma busca por níveis pelo grafo a partir de um vértice inicial.
- **Dijkstra**: Algoritmo para calcular a menor distância de um vértice de origem para todos os outros vértices do grafo.
- **Floyd-Warshall**: Algoritmo para calcular a menor distância entre todos os pares de vértices do grafo.

---
### **Criação do Grafo**

Para criar um grafo, basta informar o número de vértices desejado. Você pode escolher a representação do grafo no menu que deseja usar (Matriz de Adjacência ou Lista de Adjacência).

