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
#include <climits>

using namespace std;

template <typename T> // the template allows the weight of an edge to take any numeric data type (denoted by T).
class Graph
{

private:
    typedef map<string, T> vertex;
    std::map<string, vertex> adjList;

    typename std::map<string, vertex>::iterator mIterator;
    typename std::map<string, T>::iterator vIterator;
    size_t countEdge;

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
}

template <typename T>
Graph<T>::~Graph() {}

template <typename T>
size_t Graph<T>::num_vertices()
{
    return adjList.size();
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
    adjList[u] = map<string, T>();
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
    //Iterate through the map of vertices and add the key(string:vertex) to the vertexList vector
    for (mIterator = adjList.begin(); mIterator != adjList.end(); ++mIterator)
    {

        vertexList.push_back(mIterator->first);
    }
    return vertexList;
}

/* test4 */

template <typename T>
void Graph<T>::add_edge(const string &u, const string &v, const T &weight)
{
    //Check if the adjacent list contains the given vectors
    if (contains(u) && contains(v))
    {
        //If found, assign the given weight to the value of both keys in their corresponding maps
        adjList[u][v] = weight;
        adjList[v][u] = weight;
        countEdge++;
    }
}
template <typename T>
bool Graph<T>::adjacent(const string &u, const string &v)
{
    //Check if the one of the given vertices is found in the map of the the other
    if (adjList[u].find(v) != adjList[u].end())
    {
        return true;
    }
    return false;
}

/* test5 */

template <typename T>
vector<pair<string, string>> Graph<T>::get_edges()
{
    std::vector<string> vertexList; //Keeps track of which outer keys have been iterated through
    std::vector<pair<string, string>> edgeList;

    //Iterate through the outer map of the adjacency list
    for (auto mIterator = adjList.begin(); mIterator != adjList.end(); mIterator++)
    {
        //Iterate through the inner map of of the adjacency list
        for (auto vIterator = mIterator->second.begin(); vIterator != mIterator->second.end(); vIterator++)
        {
            //If the key of the inner map is found in vertexList
            if (find(vertexList.begin(), vertexList.end(), vIterator->first) != vertexList.end())
            {
                //create a pair of the two vertices that form an edge
                edgeList.push_back(make_pair(mIterator->first, vIterator->first));
            }
        }
        //Add the key of the outer map to vertexList
        vertexList.push_back(mIterator->first);
    }
    return edgeList;
}

/* test6 */

template <typename T>
vector<string> Graph<T>::get_neighbours(const string &u)
{
    std::vector<string> neighbourList;
    for (auto mIterator = adjList[u].begin(); mIterator != adjList[u].end(); mIterator++)
    {
        //Record the key of the inner map according map according to which key is provided for the outer map
        neighbourList.push_back(mIterator->first);
    }
    return neighbourList;
}

template <typename T>
size_t Graph<T>::degree(const string &u)
{
    //Return the number of elements of the inner map of the given key
    return adjList[u].size();
}

/* test7 */

template <typename T>
void Graph<T>::remove_edge(const string &u, const string &v)
{
    //Check if the according inner key 'v' has been removed for the respective outer key 'u'
    if (adjList[u].erase(v))
    {
        countEdge--;
    }
    //Remove inner key 'u' in the map of outer key 'v'
    adjList[v].erase(u);
}

/* test8 */

template <typename T>
void Graph<T>::remove_vertex(const string &u)
{
    //Iterate through the map of key 'u'
    for (auto mIterator = adjList[u].begin(); mIterator != adjList[u].end(); mIterator++)
    {
        //Remove key 'u' in the map of any vertex that contains it
        adjList[mIterator->first].erase(u);
        countEdge--;
    }
    adjList.erase(u);
}

/* test9 */

template <typename T>
vector<string> Graph<T>::depth_first_traversal(const string &u)
{
    stack<string> visiting; //Using a stack due to backtrack nature of dft
    vector<string> visited;

    visiting.push(u); //Add vertex 'u' to the stack of vertices to be visited
    while (!visiting.empty())
    {
        string currentVertex = visiting.top();
        visiting.pop();
        if (find(visited.begin(), visited.end(), currentVertex) == visited.end())
        {                                     //if 'u' hasn't been visited before
            visited.push_back(currentVertex); //Add 'u' to the list of visited vertices
            for (auto mIterator = adjList[currentVertex].begin(); mIterator != adjList[currentVertex].end(); mIterator++)
            {
                //Add all vertices adjacent to 'u' to the list of vertices that have yet to be visited
                visiting.push(mIterator->first);
            }
        }
    }
    return visited;
}

/* test10 */

template <typename T>
vector<string> Graph<T>::breadth_first_traversal(const string &u)
{
    queue<string> visiting;
    vector<string> visited;

    visiting.push(u); //Add vertex 'u' to the queue of vertices to be visited
    while (!visiting.empty())
    {
        string currentVertex = visiting.front();
        visiting.pop();
        if (find(visited.begin(), visited.end(), currentVertex) == visited.end())
        {                                     //if 'u' hasn't been visited before
            visited.push_back(currentVertex); //Add 'u' to the list of visited vertices
            for (auto mIterator = adjList[currentVertex].begin(); mIterator != adjList[currentVertex].end(); mIterator++)
            {
                //Add all vertices adjacent to 'u' to the list of vertices that have yet to be visited
                visiting.push(mIterator->first);
            }
        }
    }
    return visited;
}

/* test11 */

template <typename T>
bool Graph<T>::contain_cycles()
{
    stack<pair<string, string>> visiting;
    vector<string> visited;
    visiting.push(make_pair(adjList.begin()->first, "-1"));

    while (!visiting.empty())
    {
        string currentVertex = visiting.top().first;
        string parentVertex = visiting.top().second;
        visiting.pop();

        for (auto mIterator = adjList[currentVertex].begin(); mIterator != adjList[currentVertex].end(); mIterator++)
        {
            visited.push_back(currentVertex);
            if (find(visited.begin(), visited.end(), mIterator->first) == visited.end())
            {
                //If a vertex in the map of the current vertex isn't found in the stack of visited vertices
                //Create a pair of the vertex that has been found and current vertex,
                //Where currentVertex then becomes the parentVertex and the found becomes current
                visiting.push(make_pair(mIterator->first, currentVertex));
            }
            else if (mIterator->first != parentVertex)
            {
                //If the vertex found is the same as the parent vertex,
                //the graph contains a cycle
                return true;
            }
        }
    }
    return false;
}

/* test12 */

template <typename T>
Graph<T> Graph<T>::minimum_spanning_tree()
{
    Graph<T> msT;
    vector<string> visited;
    visited.push_back(adjList.begin()->first); //Insert the first vertex of our graph
    msT.add_vertex(adjList.begin()->first);

    while (num_vertices() > msT.num_vertices())
    {
        int minWeight = INT_MAX;         //Set the minimum weight of an edge that has yet to be traversed
        pair<string, string> commonEdge; //Variable to store the values an edge that is common to both our graph and the MST

        for (int i = 0; i < visited.size(); i++)
        { //Using the index of the vector instead instead of our iterator
            for (auto mIterator = adjList[visited[i]].begin(); mIterator != adjList[visited[i]].end(); mIterator++)
            {
                if ((find(visited.begin(), visited.end(), mIterator->first) == visited.end()) && mIterator->second < minWeight)
                {
                    minWeight = mIterator->second;
                    commonEdge = make_pair(visited[i], mIterator->first);
                }
            }
        }
        visited.push_back(commonEdge.second);
        msT.add_vertex(commonEdge.second);
        msT.add_edge(commonEdge.first, commonEdge.second, adjList[commonEdge.first][commonEdge.second]);
    }
    return msT;
}