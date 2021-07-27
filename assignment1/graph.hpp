#include <iostream>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <queue>
#include <stack>
#include <limits>
#include <utility>
#include <algorithm>
#include <string>
#include <cstdlib>

using namespace std;

template <typename T> // the template allows the weight of an edge to take any numeric data type (denoted by T).
class Graph
{

private:
    typedef pair<string, T> vertex;
    std::map<string, vertex> adjList;
    size_t countVertex, countEdge;

public:
    /* define your data structure to represent a weighted undirected graph */
    /* test1 */
    Graph();               // the contructor function.
    ~Graph();              // the destructor function.
    size_t num_vertices(); // returns the total number of vertices in the graph.
    size_t num_edges();    // returns the total number of edges in the graph.

    /* test2 */
    void add_vertex(const string &); // adds a vertex to the graph -- every vertex uses a string as its unique identifier.
    bool contains(const string &);   // checks if a vertex is in the graph -- returns true if the graph contains the given vertex; otherwise, returns false.

    /* test3 */
    vector<string> get_vertices(); // returns a vector of all the vertices in the graph.

    /* test4 */
    void add_edge(const string &, const string &, const T &); // adds a weighted edge to the graph -- the two strings represent the incident vertices; the third parameter represents the edge's weight.
    bool adjacent(const string &, const string &);            // check if there is an edge between the two vertices in the graph -- returns true if the edge exists; otherwise, returns false.

    /* test5 */
    vector<pair<string, string>> get_edges(); // returns a vector of all the edges in the graph -- each edge is represented by a pair of vertices incident to the edge.

    /* test6 */
    vector<string> get_neighbours(const string &); // returns a vector of all the vertices, each of which is directly connected with the given vertex by an edge.
    size_t degree(const string &);                 // returns the degree of a vertex.

    /* test7 */
    void remove_edge(const string &, const string &); // removes the edge between two vertices, if it exists.

    /* test8 */
    void remove_vertex(const string &); // delete the given vertex from the graph -- note that, all incident edges of the vertex should be deleted as well.

    /* test9 */
    vector<string> depth_first_traversal(const string &); // returns a vector of all the vertices in the visiting order of a depth-first traversal from the given vertex.

    /* test10 */
    vector<string> breadth_first_traversal(const string &); // returns a vector of all the vertices in the visiting order of a breadth-first traversal from the given vertex.

    /* test11 */
    bool contain_cycles(); // check if the graph contains any cycle -- returns true if there exists a path from a vertex to itself; otherwise, return false.

    /* test12 */
    Graph<T> minimum_spanning_tree(); // returns a spanning tree of the graph -- the returned tree is preferably a minimum spanning tree.
};

/* test1 */

template <typename T>
Graph<T>::Graph()
{
    countEdge = 0;
    countVertex = 0;
}

template <typename T>
Graph<T>::~Graph() {}

template <typename T>
size_t Graph<T>::num_vertices()
{
    return countVertex;
}

template <typename T>
size_t Graph<T>::num_edges()
{
    return countEdge;
}

/* test2 */

template <typename T>
void Graph<T>::add_vertex(const string &u)
{
    adjList[u] = pair<string, T>();
    countVertex++;
}

template <typename T>
bool Graph<T>::contains(const string &u)
{
    if (adjList.find(u) == adjList.end())
    {
        return false;
    }
    return true;
}

/* test3 */

template <typename T>
vector<string> Graph<T>::get_vertices()
{
    std::vector<string> vertexList;

    for (const auto &vertex : adjList)
    {
        vertexList.push_back(vertex.first);
    }
    return vertexList;
}

/* test4 */

template <typename T>
void Graph<T>::add_edge(const string &u, const string &v, const T &weight)
{
    if (contains(u) && contains(v))
    {
        adjList[u].second = weight;
        adjList[v].second = weight;
        countEdge++;
    }
}
template <typename T>
bool Graph<T>::adjacent(const string &u, const string &v)
{
    if (adjList[u].first == v)
    {
        return true;
    }
    return false;
}

/* test5 */

template <typename T>
vector<pair<string, string>> Graph<T>::get_edges()
{
    return vector<pair<string, string>>();
}

/* test6 */

template <typename T>
vector<string> Graph<T>::get_neighbours(const string &u)
{
    return vector<string>();
}

template <typename T>
size_t Graph<T>::degree(const string &u)
{
    return 0;
}

/* test7 */

template <typename T>
void Graph<T>::remove_edge(const string &u, const string &v)
{
}

/* test8 */

template <typename T>
void Graph<T>::remove_vertex(const string &u)
{
}

/* test9 */

template <typename T>
vector<string> Graph<T>::depth_first_traversal(const string &u)
{
    return vector<string>();
}

/* test10 */

template <typename T>
vector<string> Graph<T>::breadth_first_traversal(const string &u)
{
    return vector<string>();
}

/* test11 */

template <typename T>
bool Graph<T>::contain_cycles()
{
    return false;
}

/* test12 */

template <typename T>
Graph<T> Graph<T>::minimum_spanning_tree()
{
    return Graph<T>();
}