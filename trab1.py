class Grafo:
  def __init__(self, V, Adj, num_vertex):
    self.V = V
    self.Adj = Adj
    self.num_vertex = num_vertex

class Vertice:
  def __init__(self, indice, d, pai, cor):
    self.indice = indice
    self.d = d
    self.pai = pai
    self.cor = cor

def enqueue(Q, v):
    Q.append(v)

def dequeue(Q):
    v = Q[0]
    Q.pop(0)
    return v

def bfs(G, s): #BFS alterado, onde fará a busca pelo grafo e retornará o vértice com o maior .d
    s.d = 0 
    s.cor = 'cinza'
    s.pai = None

    maxValue = s #variável que armazenará o vértice de maior .d

    Q = []
    enqueue(Q, s)

    while (len(Q) != 0):
        u = dequeue(Q)
        for v in G.Adj[u.indice]:
            if G.V[v].cor == 'branco': # G é um grafo, V é uma lista do tipo Vertex, v tem as propriedades .indice, .d, .pai. cor
                G.V[v].cor = 'cinza' 
                G.V[v].d = u.d + 1
                G.V[v].pai = u
                enqueue(Q, G.V[v])
                if G.V[v].d > maxValue.d:
                    maxValue = G.V[v]

        u.cor = 'preto'
    return maxValue

def diameter(T):
    s = T.V[0] # 𝑠 = vértice qualquer de 𝑇 
    a = bfs(T, s)
    #devemos resetar os valores dos vértices, pois até o momento o .d de todos os vértices possuem a distância em relação à s
    for i in range(T.num_vertex):
        T.V[i].d = None
        T.V[i].pai = None
        T.V[i].cor = 'branco'

    b = bfs(T, a)

    return b.d

#testes automatizados    
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