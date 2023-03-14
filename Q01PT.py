import networkx as nx
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Vetor de grafo padrao
graph = [('CHICAGO', 'PEORIA', 220), ('CHICAGO', 'SPRINGFIELDI', 255), ('CHICAGO', 'DESMOINES', 460),
         ('PEORIA', 'CHICAGO', 220), ('PEORIA', 'HANNIBAL', 240), ('PEORIA', 'OMAHA', 510),
         ('PEORIA', 'DESMOINES', 360), ('PEORIA', 'SPRINGFIELDI', 105), ('SPRINGFIELDI', 'CHICAGO', 255),
         ('SPRINGFIELDI', 'PEORIA', 105), ('SPRINGFIELDI', 'STLOUIS', 130), ('SPRINGFIELDI', 'HANNIBAL', 165),
         ('STLOUIS', 'SPRINGFIELDI', 130), ('STLOUIS', 'POPLARB', 240), ('STLOUIS', 'HANNIBAL', 180),
         ('STLOUIS', 'KANSASCITY', 340), ('STLOUIS', 'JEFFERSON', 195), ('STLOUIS', 'SPRINGFIELDM', 315),
         ('POPLARB', 'STLOUIS', 240), ('POPLARB', 'SPRINGFIELDM', 310), ('POPLARB', 'LITTLEROCK', 300),
         ('HANNIBAL', 'PEORIA', 240), ('HANNIBAL', 'SPRINGFIELDI', 165), ('HANNIBAL', 'DESMOINES', 360),
         ('HANNIBAL', 'STLOUIS', 180), ('HANNIBAL', 'JEFFERSON', 165), ('HANNIBAL', 'KANSASCITY', 285),
         ('HANNIBAL', 'STJOSEPH', 285), ('JEFFERSON', 'HANNIBAL', 165), ('JEFFERSON', 'STLOUIS', 195),
         ('JEFFERSON', 'SPRINGFIELDM', 225), ('JEFFERSON', 'FTSCOTT', 255), ('LITTLEROCK', 'POPLARB', 300),
         ('LITTLEROCK', 'SPRINGFIELDM', 330), ('LITTLEROCK', 'FTSMITH', 225), ('SPRINGFIELDM', 'STLOUIS', 130),
         ('SPRINGFIELDM', 'POPLARB', 310), ('SPRINGFIELDM', 'JEFFERSON', 225), ('SPRINGFIELDM', 'LITTLEROCK', 330),
         ('SPRINGFIELDM', 'KANSASCITY', 255), ('SPRINGFIELDM', 'TULSA', 220), ('SPRINGFIELDM', 'WICHITA', 360),
         ('SPRINGFIELDM', 'DESMOINES', 505), ('DESMOINES', 'CHICAGO', 460), ('DESMOINES', 'PEORIA', 360),
         ('DESMOINES', 'OMAHA', 200), ('DESMOINES', 'HANNIBAL', 360), ('DESMOINES', 'SPRINGFIELDM', 505),
         ('DESMOINES', 'KANSASCITY', 280), ('FTSMITH', 'LITTLEROCK', 225), ('FTSMITH', 'FTSCOTT', 330),
         ('FTSMITH', 'TULSA', 210), ('FTSMITH', 'OKLAHOMA', 270), ('KANSASCITY', 'HANNIBAL', 285),
         ('KANSASCITY', 'SPRINGFIELDM', 255), ('KANSASCITY', 'DESMOINES', 280), ('KANSASCITY', 'FTSCOTT', 150),
         ('KANSASCITY', 'STJOSEPH', 70), ('KANSASCITY', 'WICHITA', 200), ('KANSASCITY', 'GREATBEND', 270),
         ('KANSASCITY', 'STLOUIS', 340), ('KANSASCITY', 'KITCARSON', 285), ('FTSCOTT', 'JEFFERSON', 255),
         ('FTSCOTT', 'FTSMITH', 330), ('FTSCOTT', 'KANSASCITY', 150), ('FTSCOTT', 'TULSA', 195),
         ('FTSCOTT', 'WICHITA', 210), ('STJOSEPH', 'HANNIBAL', 285), ('STJOSEPH', 'KANSASCITY', 285),
         ('STJOSEPH', 'OMAHA', 255), ('STJOSEPH', 'BEATRICE', 195), ('TULSA', 'SPRINGFIELDM', 220),
         ('TULSA', 'FTSMITH', 210), ('TULSA', 'FTSCOTT', 195), ('TULSA', 'OMAHA', 590), ('TULSA', 'WICHITA', 230),
         ('TULSA', 'OKLAHOMA', 105), ('OMAHA', 'PEORIA', 510), ('OMAHA', 'DESMOINES', 200), ('OMAHA', 'STJOSEPH', 255),
         ('OMAHA', 'TULSA', 590), ('OMAHA', 'BEATRICE', 110), ('BEATRICE', 'STJOSEPH', 195), ('BEATRICE', 'OMAHA', 110),
         ('BEATRICE', 'GRANDISLAND', 170), ('BEATRICE', 'WICHITA', 275), ('WICHITA', 'SPRINGFIELDM', 360),
         ('WICHITA', 'KANSASCITY', 200), ('WICHITA', 'FTSCOTT', 210), ('WICHITA', 'TULSA', 230),
         ('WICHITA', 'GUYMON', 345), ('WICHITA', 'BEATRICE', 275), ('WICHITA', 'OKLAHOMA', 195),
         ('WICHITA', 'GREATBEND', 170), ('WICHITA', 'DOGDE', 210), ('OKLAHOMA', 'FTSMITH', 270),
         ('OKLAHOMA', 'TULSA', 105), ('OKLAHOMA', 'WICHITA', 195), ('OKLAHOMA', 'DOGDE', 170),
         ('OKLAHOMA', 'AMARILLO', 170), ('GREATBEND', 'KANSASCITY', 270), ('GREATBEND', 'WICHITA', 170),
         ('GREATBEND', 'PHILLIPSBURG', 170), ('GREATBEND', 'PUEBLO', 170), ('GREATBEND', 'DOGDE', 120),
         ('PHILLIPSBURG', 'STJOSEPH', 300), ('PHILLIPSBURG', 'GREATBEND', 165), ('PHILLIPSBURG', 'GRANDISLAND', 160),
         ('PHILLIPSBURG', 'DENVER', 390), ('GRANDISLAND', 'OMAHA', 190), ('GRANDISLAND', 'BEATRICE', 170),
         ('GRANDISLAND', 'PHILLIPSBURG', 160), ('GRANDISLAND', 'DOGDE', 375), ('GRANDISLAND', 'NORTHPLATTE', 190),
         ('DOGDE', 'WICHITA', 210), ('DOGDE', 'OKLAHOMA', 170), ('DOGDE', 'GREATBEND', 120),
         ('DOGDE', 'GRANDISLAND', 375), ('DOGDE', 'GUYMON', 150), ('DOGDE', 'NORTHPLATTE', 340),
         ('DOGDE', 'PUEBLO', 360), ('GUYMON', 'WICHITA', 345), ('GUYMON', 'DOGDE', 250), ('GUYMON', 'AMARILLO', 150),
         ('GUYMON', 'TUCUMCARI', 260), ('GUYMON', 'RATON', 255), ('NORTHPLATTE', 'GRANDISLAND', 190),
         ('NORTHPLATTE', 'DOGDE', 340), ('NORTHPLATTE', 'DENVER', 360), ('AMARILLO', 'OKLAHOMA', 170),
         ('AMARILLO', 'GUYMON', 150), ('AMARILLO', 'KITCARSON', 360), ('AMARILLO', 'TUCUMCARI', 175),
         ('AMARILLO', 'RATON', 285), ('KITCARSON', 'KANSASCITY', 285), ('KITCARSON', 'AMARILLO', 360),
         ('KITCARSON', 'DENVER', 210), ('TUCUMCARI', 'GUYMON', 345), ('TUCUMCARI', 'AMARILLO', 175),
         ('TUCUMCARI', 'SANTAFE', 260), ('TUCUMCARI', 'ALBUQUERQUE', 230), ('RATON', 'GUYMON', 255),
         ('RATON', 'AMARILLO', 285), ('RATON', 'PUEBLO', 150), ('RATON', 'SANTAFE', 255), ('RATON', 'DURANGO', 435),
         ('PUEBLO', 'DOGDE', 360), ('PUEBLO', 'RATON', 150), ('PUEBLO', 'GREATBEND', 170), ('PUEBLO', 'DENVER', 180),
         ('PUEBLO', 'DURANGO', 435), ('PUEBLO', 'GRANDJUNCTION', 480), ('DENVER', 'PHILLIPSBURG', 390),
         ('DENVER', 'NORTHPLATTE', 360), ('DENVER', 'KITCARSON', 210), ('DENVER', 'PUEBLO', 180),
         ('DENVER', 'DURANGO', 580), ('DENVER', 'GRANDJUNCTION', 420), ('SANTAFE', 'TUCUMCARI', 260),
         ('SANTAFE', 'RATON', 255), ('SANTAFE', 'ALBUQUERQUE', 75), ('SANTAFE', 'DURANGO', 330),
         ('ALBUQUERQUE', 'TUCUMCARI', 230), ('ALBUQUERQUE', 'SANTAFE', 75), ('ALBUQUERQUE', 'DURANGO', 300),
         ('ALBUQUERQUE', 'GALLUP', 185), ('DURANGO', 'RATON', 435), ('DURANGO', 'PUEBLO', 435),
         ('DURANGO', 'DENVER', 580), ('DURANGO', 'SANTAFE', 330), ('DURANGO', 'ALBUQUERQUE', 300),
         ('DURANGO', 'GRANDJUNCTION', 300), ('DURANGO', 'GALLUP', 260), ('DURANGO', 'GREENRIVER', 320),
         ('GRANDJUNCTION', 'PUEBLO', 480), ('GRANDJUNCTION', 'DENVER', 420), ('GRANDJUNCTION', 'DURANGO', 300),
         ('GRANDJUNCTION', 'GREENRIVER', 180), ('GALLUP', 'ALBUQUERQUE', 185), ('GALLUP', 'DURANGO', 260),
         ('GALLUP', 'GREENRIVER', 450), ('GALLUP', 'FLAGSTAFF', 260), ('GREENRIVER', 'DURANGO', 320),
         ('GREENRIVER', 'GRANDJUNCTION', 180), ('GREENRIVER', 'GALLUP', 450), ('GREENRIVER', 'MTCARMELJUNCTION', 480),
         ('FLAGSTAFF', 'GALLUP', 260), ('FLAGSTAFF', 'MTCARMELJUNCTION', 330), ('FLAGSTAFF', 'GRANDCANYON', 110),
         ('MTCARMELJUNCTION', 'GREENRIVER', 480), ('MTCARMELJUNCTION', 'FLAGSTAFF', 330),
         ('MTCARMELJUNCTION', 'GRANDCANYON', 150), ('GRANDCANYON', 'FLAGSTAFF', 110),
         ('GRANDCANYON', 'MTCARMELJUNCTION', 150)]


# Crie um gráfico em vazio
G = nx.Graph()

# Transfira os dados do vetor (grafo) para o grafo G
for i in range(len(graph)):
    G.add_edge(graph[i][0], graph[i][1], weight=graph[i][2])


def dijkstra_path(graph, start, end):
    # inicializa a distância como infinito e o nó inicial como 0
    distances = {node: float('inf') for node in graph}
    distances[start] = 0

    # Cria um conjunto vazio para nós visitados
    visited_nodes = set()

    # enquanto não houver nós visitados
    while len(visited_nodes) != len(graph):
        # Encontre o nó não visitado com a menor distância atual e marque-o como visitado
        current_node = min(
            [node for node in graph if node not in visited_nodes],
            key=lambda node: distances[node]
        )
        visited_nodes.add(current_node)

        # Para cada nó vizinho não visitado do nó atual, atualize sua distância se, somente se a distância através
        # do nó atual é menor que a distância atual
        for neighbor, weight in graph[current_node].items():
            if neighbor not in visited_nodes:
                current_distance = distances[current_node] + weight['weight']  # Peso do nó
                if current_distance < distances[neighbor]:
                    distances[neighbor] = current_distance

    # Retorna o caminho mais curto encontrado
    path = []
    current = end
    while current != start:
        path.insert(0, current)
        for neighbor, weight in graph[current].items():
            if distances[current] == distances[neighbor] + weight['weight']:  # Peso do nó
                current = neighbor
                break
    path.insert(0, start)

    # Calcula o peso total do caminho
    finalweight = sum([G[path[i]][path[i + 1]]['weight'] for i in range(len(path) - 1)])
    return path, finalweight



# executa o algoritmo de Dijkstra
path, finalweight = dijkstra_path(G, 'CHICAGO', 'GRANDCANYON')


# definir função de animação
def update_animation(frame):
    # limpa o gráfico anterior
    plt.clf()

    # traça o gráfico com as cores atualizadas
    pos = nx.spring_layout(G, seed=220)
    nx.draw_networkx(G, pos)
    nx.draw_networkx_nodes(G, pos, nodelist=path[:frame + 1], node_color='r')
    nx.draw_networkx_edges(G, pos, edgelist=[(path[i], path[i + 1]) for i in range(frame)], edge_color='r',
                           width=3)


# traça o gráfico com o caminho inicial
fig, ax = plt.subplots()
pos = nx.spring_layout(G, seed=220)
nx.draw_networkx(G, pos)

# configura a animacao
ani = FuncAnimation(fig, update_animation, frames=len(path), interval=1000, repeat=True, )

plt.show()

# saida para o graphviz
print("digraph G {")
for i in range(len(path) - 1):
    print(("{} -> {}".format(path[i], path[i + 1])))
    print("[color=red]")
    print(path[i], "[color=blue];")
print(path[i + 1], "[color=blue];")

edgelist_generator = nx.generate_edgelist(G)
edgelist = list(edgelist_generator)

string = "\n".join([f"{x.split()[0]} -> {x.split()[1]}" for x in edgelist])

print(string)
print("}")

print("----------")

print("weight", finalweight)
