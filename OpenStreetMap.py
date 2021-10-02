import osmnx as ox
import matplotlib.pyplot as plt

place_name = "The University of North Carolina at Charlotte"

graph = ox.graph_from_place(place_name)

type(graph)
fig, ax = ox.plot_graph(graph)
