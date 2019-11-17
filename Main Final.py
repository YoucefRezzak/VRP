"""Capacited Vehicles Routing Problem (CVRP)."""

from __future__ import print_function
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import random
import time
import numpy as np
import statistics
from collections import Counter
from matplotlib import pyplot as plt


def create_data_model():
    """Contient les données pour le probleme"""
    data = {}
    exe_times = []

    start = time.time()
    """ Entrée utilisateur pour parametrer l'instance, afin de definir le nombre de clients, distance maximum
    entre deux clients"""
    nClients = int(input("Donnez le nombre de clients : "))
    print()
    randDistance = int(input("Donnez la distance maximale entre 2 clients : "))
    print()
    """generation des matrices de distances pour les clients de l'instance"""
    A = [[round(random.random() * randDistance) for i in range(nClients)] for j in range(nClients)]
    np.linalg.det(A)
    end = time.time()
    exe_times.append(end - start)
    i = 0
    while (i < nClients):
        j = 0
        while (j < nClients):
            if ((A[i][j] == 0) & (i != j)):
                A[i][j] = random.randint(1, randDistance)
            if (i == j):
                A[i][j] = 0
            if (i > j):
                A[i][j] = A[j][i]
            j += 1
        i += 1
    #   print(A)
    data['distance_matrix'] = A
    print("Matrice des distances : ", data['distance_matrix'])
    print()
    """ Entrée utilisateur pour parametrer l'instance, afin de definir la demande maximale pour les clients générés"""
    randDemande = int(input("Donnez la capacité maximale demandé par un client : "))
    print()
    """génération des demandes clients en consequence de l'entrée utilisateur"""
    B = [round(random.random() * randDemande) for i in range(nClients)]
    i = 0
    s=0
    while (i < nClients):
        if (i == 0):
            B[i] = 0
        if (B[i] == 0 & i != 0):
            B[i] = random.randint(1, randDemande)
        s+=B[i]
        i += 1
    data['demands'] = B
    print("La somme des demandes : ",s)
    print("Les demandes des clients : ", B)
    i = 0
    print()
    """ Entrée utilisateur pour parametrer le nombre de camion, ainsi que leur charge minimale, avec differentes 
     possbilitées de paramétrage"""
    nCamions = int(input("Donnez le nombre de camion :"))
    print()
    C = [0] * nCamions
    print("Voulez vous des capacité aléatoire différentes pour les camion tapez 1.")
    print("Si vous voullez donnez la capacité de chaque camion tapez 2. ")
    oui = int(input("Sinon une meme capacité pour tous les camions tapez 3 :"))
    print()
    if (oui == 1):
        print("Donnez la capacité minimale pour chaque camions : ")
        minDemande = int(input())
        print()

        maxcap = int(input("Donnez la capacité maximale pour chaque camion : "))

        print()
        i = 0
        while (i < nCamions):
            C[i] = random.randint(minDemande, maxcap)
            i += 1
    else:
        if (oui == 2):
            i = 0
            while (i < nCamions):
                print("Donnez la capacité du camion ", i, " : ")
                cap = int(input())
                print()
                C[i] = cap
                i += 1
        else:
            if (oui == 3):
                cap = int(input("Donnez la capacité des camions : "))
                print()
                i = 0
                while (i < nCamions):
                    C[i] = cap
                    i += 1
    print("Les capacité des camions : ", C)
    print()
    print("Calcule de la solution optimale en cours ...")
    data['vehicle_capacities'] = C
    data['num_vehicles'] = nCamions
    data['depot'] = 0
    return data


def print_solution(data, manager, routing, assignment):
    """Affichage les informations de solutions sur la console."""

    """generation des variables """
    total_distance = 0
    total_load = 0
    """vecteurs pour stocker les distances et les charges"""
    med_distance = []
    med_charge = []
    """affichage des informations de chaque vehicules"""

    """pour chaque vehicules, on boucles sur chaque noeud qu'il visite pour afficher a la fin 
       de l'éxécution un suivi complet de son chemin, en partant du depot et en effectuant un cycle afin d'y retourner"""
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output = 'La route du camion {}:\n'.format(vehicle_id)
        route_distance = 0
        route_load = 0
        decharge = 0
        while not routing.IsEnd(index):
            """on boucle tant que le camion n'a pas totalement efféctué son cheminement et actualisons le noeud actuel"""
            node_index = manager.IndexToNode(index)
            """on selectione la demande client du noeud actuel"""
            route_load += data['demands'][node_index]
            d = route_load - decharge
            plan_output += ' {0} Livraison de :({1}) -> '.format(node_index, d)
            decharge = route_load
            previous_index = index
            index = assignment.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id)
        plan_output += ' {0} Livraison de :({1})\n'.format(
            manager.IndexToNode(index), route_load)
        plan_output += 'Distance de la route: {}m\n'.format(route_distance)
        plan_output += 'Charge de la route : {}\n'.format(route_load)
        print(plan_output)
        total_distance += route_distance
        total_load += route_load
        med_distance.append(route_distance)
        med_charge.append(route_load)

    print('Distance totale de toutes les routes : {}m'.format(total_distance))
    print('Charge totale de toutes les routes : {}'.format(total_load))
    """"La moyenne de la distance parcourue"""
    moyenne_distance = total_distance / data['num_vehicles']
    print("La moyenne de la distance parcourue des camions est :", moyenne_distance, "")

    """"La moyenne de la charge """
    moyenne_charge = total_load / data['num_vehicles']
    print("La moyenne de la charge des camions est :", moyenne_charge, "")

    """"La Median de la distance """
    print("Tableau des distances: ", med_distance, "")
    print("La Median de la distance parcourue est : ", statistics.median(med_distance), "")

    """"La Median de la charge """
    print("Tableau des charges: ", med_distance, "")
    print("La Median de la charge des camions est : ", statistics.median(med_charge), "")

    """"Le Mode de la distance """
    # statistics.mode(med-distance)

    """"Le MAX et le MIN de la distance """
    print("Le MIN des distances parcourues des camions est : ", min(med_distance), "")
    print("Le MAX des distances parcourues des camions est : ", max(med_distance), "")

    """"Le MAX et le MIN de la charge """
    print("Le MIN des charges  des camions est : ", min(med_charge), "")
    print("Le MAX des charges  des camions est : ", max(med_charge), "")

    """"L'ecart type """

    print("L'ecart type de des distances est : ", statistics.stdev(med_distance), "")
    print("L'ecart type des charges est : ", statistics.stdev(med_charge), "")

    """"Les quartiles de la distance """
    print('Premier quartile de la distance 25 %: ', np.percentile(med_distance, 25))
    print('Deuxième quartile de la distance 50 %: ', np.percentile(med_distance, 50))
    print('Troisième quartile de la distance 75 %: ', np.percentile(med_distance, 75))
    print("L'écart interquartile de la distance est : ",
          np.percentile(med_distance, 75) - np.percentile(med_distance, 25))

    """"Les quartiles de la charge """
    print('Premier quartile de la charge 25 %: ', np.percentile(med_charge, 25))
    print('Deuxième quartile de la charge 50 %: ', np.percentile(med_charge, 50))
    print('Troisième quartile de la charge 75 %: ', np.percentile(med_charge, 75))
    print("L'écart interquartile de la charge est : ", np.percentile(med_charge, 75) - np.percentile(med_charge, 25))
    x = np.arange(data['num_vehicles'])
    plt.suptitle('Histogramme de la distance parcourue par chaque vehicule',
                 fontweight='bold',
                 fontsize=18)

    plt.bar(x, med_distance)

    plt.xticks(x, fontsize=8)
    plt.yticks(fontsize=8)
    plt.xlabel('Véhicules');
    plt.ylabel('Distances');
    for x, y in zip(x, med_distance):
        plt.text(x, y - 2.55, '%.1f' % y,
                 ha='center',
                 va='bottom',
                 fontsize=6,
                 color='white',
                 fontweight='bold')

    plt.show()

    x1 = np.arange(data['num_vehicles'])
    plt.suptitle('Histogramme de la charge totale pour chaque vehicule',
                 fontweight='bold',
                 fontsize=18)

    plt.bar(x1, med_charge)

    plt.xticks(x1, fontsize=8)
    plt.yticks(fontsize=8)
    plt.xlabel('Véhicules');
    plt.ylabel('Charges');
    for x1, y in zip(x1, med_charge):
        plt.text(x1, y - 2.55, '%.1f' % y,
                 ha='center',
                 va='bottom',
                 fontsize=6,
                 color='white',
                 fontweight='bold')

    plt.show()

def main():
    """résout le probleme du CVRP."""
    import os
    import psutil
    process = psutil.Process(os.getpid())


    # Instantiation des données du probleme grace a la methode "create_data_model()
    data = create_data_model()
    start_time = time.time()

    # Cree le manager de routage des noeuds d'ortools
    manager = pywrapcp.RoutingIndexManager(
        len(data['distance_matrix']), data['num_vehicles'], data['depot'])

    # Cree le modele de routage
    routing = pywrapcp.RoutingModel(manager)

    # Cree et enregistre un callback de transit
    def distance_callback(from_index, to_index):
        """renvoie la distance entre doeux noeuds"""
        # Convertis la variable de routage de noeud en matrice de distance des noeuds.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Definis un cout pour chaque arc
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Ajoute les contraintes de capacitées
    def demand_callback(from_index):
        """Renvoie la demande du noeud."""
        # Convertis la variable de routage des noeuds en demandes d'indexe de noeuds.
        from_node = manager.IndexToNode(from_index)
        return data['demands'][from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(
        demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # capacité nulle lâche
        data['vehicle_capacities'],  # Capacités maximum des véhicules
        True,  # Commence le cumul a zero
        'Capacity')

    # Definis la premiere solution heuristique utilisées
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.CHRISTOFIDES)
    #print(search_parameters.first_solution_strategy)
    #Definis la solutions meta-heuristique utilisé pour optimiser les solutions obtenus
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.TABU_SEARCH)
    #print(search_parameters.local_search_metaheuristic)
    search_parameters.time_limit.seconds = 30
    search_parameters.log_search = False
    # Resous le probleme
    assignment = routing.SolveWithParameters(search_parameters)

    # Affiche les solutions sur la console.
    if assignment:
        print_solution(data, manager, routing, assignment)
    else:
        print("Aucune solution possible")
    print("Temps d'execution du programme : %s seconds " % (time.time() - start_time))
    print("Mémoire utilisé : ",process.memory_info().rss," Bytes")

if __name__ == '__main__':
    main()
