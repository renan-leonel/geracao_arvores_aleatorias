#Renan Augusto Leonel ra: 115138
#Pedro Henrique de Melo Costa ra: 112653
from collections import deque
from random import randint

#classe para inicializarmos o grafo
class Grafo:
  def __init__(self, V, Adj, num_vertex):
    self.V = V
    self.Adj = Adj
    self.num_vertex = num_vertex

#classe para inicializarmos o vértice
class Vertice:
  def __init__(self, indice, d, pai, cor):
    self.indice = indice
    self.d = d
    self.pai = pai
    self.cor = cor

# função que enfileira um vértice v na fila criada em tempo O(1) 
def enqueue(Q, v):
    Q.append(v)

# função que desenfileira um vértice v da fila também em tempo O(1) 
def dequeue(Q):
    v = Q[0]
    Q.popleft() #utilizamos popleft pois precisamos manter o algoritmo em tempo 0(1), e a função pop() tem tempo O(n)
    return v

#função para encontrarmos o vértice do grafo que possui o maior valor do atributo .d, para isso utilizamos o algoritmo do BFS com pequenas modificações
#bfs implementado com base no algoritmo visto em aula
def bfsMax(G, s):
    s.d = 0 
    s.cor = 'cinza'
    s.pai = None

    maxValue = s #poderia ser inicializada com um vértice aleatório, então escolhemos o primeiro

    Q = deque([])
    enqueue(Q, s)

    while (len(Q) != 0):
        u = dequeue(Q)
        for v in G.Adj[u.indice]:
            # G é um grafo, V é uma lista do tipo Vertex, v tem as propriedades .indice, .d, .pai, .cor
            if G.V[v].cor == 'branco': 
                G.V[v].cor = 'cinza' 
                G.V[v].d = u.d + 1
                G.V[v].pai = u
                enqueue(Q, G.V[v])
                if G.V[v].d > maxValue.d:
                    maxValue = G.V[v]

        u.cor = 'preto'
    return maxValue

# função para verificar se um grafo é uma árvore, utilizando DFS
# retornando true caso sim, e false caso não seja.
def verifica_arvore(G):
    global tempo
    tempo = 0
    
    #DFSVisit retorna True se o grafo não possui ciclos, falso caso possua
    x = DFSVisit(G, G.V[0])

    #a variável tempo será responsável por verificar se o grafo é conexo
    #como temos o tempo de chegada e saída do vértice, ao dividirmos tempo/2 devemos obter a quantidade de vértices do grafo
    #caso tempo/2 e a quantidade de vértices sejam diferentes, temos que o grafo não é conexo
    if x and tempo/2 == G.num_vertex:
        return True
    else:
        return False
        
    
def DFSVisit(G, u):
    global tempo

    u.cor = "cinza"
    tempo = tempo + 1
    u.d = tempo

    for v in G.Adj[u.indice]:
        if G.V[v].cor == "branco":
            # se no meio da chamada DFSVisit retornar que o grafo não é uma árvore
            if not DFSVisit(G, G.V[v]):
                return False
        elif G.V[v].cor == 'preto':
            return False

    u.cor = "preto"
    tempo += 1
    u.f = tempo

    return True

# função que calcula o diâmetro de uma árvore T, para isso calculamos o comprimento do maior caminho em T, retornando este valor ao final da execução
def diameter(T):
    s = T.V[0] # s = vértice qualquer de T
    a = bfsMax(T, s)
    #devemos resetar os valores dos vértices, pois até o momento o .d de todos os vértices possuem a distância em relação à s
    for i in range(T.num_vertex):
        T.V[i].d = None
        T.V[i].pai = None
        T.V[i].cor = 'branco'

    b = bfsMax(T, a)

    return b.d

def random_tree_random_walk(n):
    #inicializa o grafo com n vértices; inicializa uma lista de adjacência para cada execução
    G = Grafo([Vertice(i, None, None, 'branco') for i in range(n)], [[] for i in range(n)] , n) 

    u = G.V[0] # s = vértice qualquer de V

    arestas = 0

    #lista para todos os vértices que forem visitados
    visitados = [False for i in range(n)]

    #marca que o primeiro vértice foi visitado
    visitados[u.indice] = True

    #como toda árvore tem n-1 arestas, sendo n o número de vértices.
    while arestas < n-1:
        v = G.V[randint(0, n-1)] # gera um vértice aleatório

        # se o vértice v não tiver sido visitado
        #adicionamos a aresta (u,v) na lista de adjacências
        #aumentamos a quantidade de arestas
        #ao final, marca que o vértice v foi visitado
        if visitados[v.indice] == False:
            G.Adj[u.indice].append(v.indice) #adiciona v na lista de adjacência de u
            G.Adj[v.indice].append(u.indice) #adiciona u na lista de adjacência de v

            arestas += 1
            visitados[v.indice] = True
        u = v
        
    if verifica_arvore(G) == True:
        return G
    else:
        return None


def main():
    testes = [250, 500, 750, 1000, 1250, 1500, 1750, 2000]

    #abertura de arquivo para escrita
    file = open("random_tree_random_walk.txt", "w")

    for n in testes:
        soma = 0
        for x in range(500):
            soma = soma + diameter(random_tree_random_walk(n))
        media = soma/500
        #escreve no arquivo
        file.write('{} {}\n'.format(n, media))

if __name__ == "__main__":
    main()
    
#testes automatizados para a função diameter, onde se o algoritmo executa sem erros de compilação, todos os testes foram um sucesso
assert diameter(Grafo([Vertice(i, None, None, 'branco') for i in range(5)], [[3], [2], [3, 1], [0, 4, 2], [3]], 5)) == 3
assert diameter(Grafo([Vertice(i, None, None, 'branco') for i in range(5)], [[4, 2], [4], [0, 3], [2], [0, 1]], 5)) == 4
assert diameter(Grafo([Vertice(i, None, None, 'branco') for i in range(5)], [[2], [4], [0, 3, 4], [2], [2, 1]], 5)) == 3
assert diameter(Grafo([Vertice(i, None, None, 'branco') for i in range(5)], [[1, 4], [0], [4, 3], [2], [0, 2]], 5)) == 4
assert diameter(Grafo([Vertice(i, None, None, 'branco') for i in range(5)], [[4, 3, 2], [4], [0], [0], [0, 1]], 5)) == 3
assert diameter(Grafo([Vertice(i, None, None, 'branco') for i in range(5)], [[3, 1], [0, 4], [3], [0, 2], [1]], 5)) == 4
assert diameter(Grafo([Vertice(i, None, None, 'branco') for i in range(5)], [[4], [2, 3], [4, 1], [1], [0, 2]], 5)) == 4
assert diameter(Grafo([Vertice(i, None, None, 'branco') for i in range(5)], [[2, 1, 4], [0], [0, 3], [2], [0]], 5)) == 3
assert diameter(Grafo([Vertice(i, None, None, 'branco') for i in range(5)], [[3], [4], [3], [0, 2, 4], [3, 1]], 5)) == 3
assert diameter(Grafo([Vertice(i, None, None, 'branco') for i in range(5)], [[1, 3], [0, 2], [1], [0, 4], [3]], 5)) == 4