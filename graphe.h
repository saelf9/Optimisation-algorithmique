//
//  Graphe.h
//  Classe pour graphes orientés pondérés (non négativement) avec listes d'adjacence
//


#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include <list>
#include <set>
#include <stack>
#include <queue>
#include <limits>
#include <iostream>
#include <algorithm>

//! \brief  Classe pour graphes orientés pondérés (non négativement) avec listes d'adjacence
class Graphe
{
public:

    explicit Graphe(size_t = 0);
    void resize(size_t);
    void ajouterArc(size_t i, size_t j, unsigned int poids);
    void enleverArc(size_t i, size_t j);
    unsigned int getPoids(size_t i, size_t j) const;
    size_t getNbSommets() const;
    size_t getNbArcs() const;

    unsigned int plusCourtChemin(size_t p_origine, size_t p_destination,
                                 std::vector<size_t> & p_chemin) const;

    // Binary heap implementation
    struct BinaryHeapNode {
        unsigned int distance;
        size_t vertex;

        BinaryHeapNode(unsigned int d, size_t v) : distance(d), vertex(v) {}
    };
private:

    struct Arc
    {
        Arc(size_t dest, unsigned int p) :
                destination(dest), poids(p)
        {
        }
        size_t destination;
        unsigned int poids;
    };

    void pushToBinaryHeap(std::vector<BinaryHeapNode> &binaryHeap, const BinaryHeapNode &node) const;
    BinaryHeapNode popFromBinaryHeap(std::vector<BinaryHeapNode> &binaryHeap) const;
    std::vector<std::vector<Arc>> m_listesAdj;
    size_t m_nbArcs;  // Declare m_nbArcs
};


#endif  //GRAPH_H
