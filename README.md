# Projet de Travail Pratique #3 - GLO-2100: Algorithmes de Plus Courts Chemins Efficaces pour Itinéraires sur le Réseau de la RTC

## Description

Ce projet implémente un algorithme de plus court chemin efficace pour le réseau de transport de la capitale (RTC) de la ville de Québec. Ce TP est la suite des deux premiers travaux portant sur l'implémentation d'accès aux données GTFS et la construction d'un graphe pour exécuter l'algorithme de Dijkstra.

Dans ce travail, l’objectif est de modifier la méthode `plusCourtChemin()` de la classe `Graphe` pour la rendre plus efficace que l'implémentation de Dijkstra fournie, et ce, afin d'améliorer le temps d'exécution pour les itinéraires.

### Objectif
- Implémenter un algorithme de plus court chemin plus rapide et plus efficace que celui fourni.
- Comparer les performances de votre algorithme avec celle de Dijkstra et fournir un facteur d'accélération.
  
### Structure du Projet
- **graphe.h** : Définition de la classe `Graphe` avec la méthode à optimiser.
- **graphe.cpp** : Implémentation de la classe `Graphe` et modification de la méthode `plusCourtChemin()`.
- **main.cpp** : Fournit une méthode principale pour tester les différents itinéraires sur le réseau RTC.
- **rapport.pdf** : Rapport détaillant le choix de l'algorithme, les résultats obtenus et les conclusions.

## Prérequis

- C++11 ou supérieur
- CMake pour la gestion du projet
- Accès à la machine virtuelle du cours ou à un environnement compatible avec le CMakeLists.txt fourni

## Instructions

1. **Cloner le Repository**
   Clonez ce dépôt en utilisant la commande suivante:
   ```bash
   git clone https://github.com/votre_nom/nom_du_projet.git
   ```

2. **Compilation et Exécution**
   Utilisez CMake pour compiler le projet:
   ```bash
   mkdir build
   cd build
   cmake ..
   make
   ```

3. **Exécuter les Tests**
   Exécutez le programme avec la méthode `main()` pour tester les itinéraires. Vous pouvez ajuster la variable `nbDeTests` pour choisir le nombre de tests à effectuer.

4. **Analyse des Résultats**
   Analysez les résultats de l'exécution et comparez les performances de votre algorithme avec celle de l'algorithme de Dijkstra fourni.

## Critères de Remise

- **Fichiers à Soumettre** : 
  - `graphe.h`
  - `graphe.cpp`
  - `rapport.pdf`

- **Dépôt** : Zippez les fichiers en suivant la convention de nommage `NomDeFamille_Prénom.zip` et soumettez via le portail du cours.

## Avertissement

Assurez-vous que tous les fichiers sont bien compilables et exécutables dans l'environnement virtuel fourni avec le CMakeLists.txt. Toute modification du code fourni qui n'est pas autorisée entraînera une pénalité.
