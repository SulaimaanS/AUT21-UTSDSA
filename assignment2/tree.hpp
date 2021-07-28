#include <iostream>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <map>
#include <set>
#include <queue>
#include <stack>
#include <limits>
#include <utility>
#include <algorithm>
#include <string>
#include <cstdlib>
#include <cmath>
#include <climits>

using namespace std;

template <typename T> // the template allows the weight of vertex to take any numeric data type (denoted by T).
class Node
{
public:
    string vertex_name;
    T weight;
    Node *left, *right, *parent;

    Node(const string &name, const T &weight);
};

template <typename T>
class BST
{
public:
    /* define your data structure to represent a binary search tree (bst) */
    size_t vertex_count, edge_count;
    T total_weight;
    Node<T> *root;

    /* test1 */
    BST();                 // the contructor function.
    ~BST();                // the destructor function.
    size_t num_vertices(); // returns the total number of vertices in the bst.
    size_t num_edges();    // returns the total number of edges in the bst.
    T sum_weight();        // return the total weight of all the vertices in the bst.
    Node<T> *find_vertex(const string &);
    /* test2 */
    void add_vertex(const string &, const T &); // adds a vertex, which has a weight, to the tree -- every vertex uses a string as its unique identifier.
    bool contains(const string &);              // checks if a vertex is in the bst -- returns true if the bst contains the given vertex; otherwise, returns false.

    /* test3 */
    vector<string> get_vertices(); // returns a vector of all the vertices in the bst.
    vector<string> get_leaves();   // returns a vector of all the leaves in the bst.
                                   //     Leaves are the vertices that do not have any children in the bst.

    /* test4 */
    bool adjacent(const string &, const string &); // check if there is an edge between the two vertices in the bst -- returns true if the edge exists; otherwise, returns false.

    /* test5 */
    vector<pair<string, string>> get_edges(); // returns a vector of all the edges in the bst -- each edge is represented by a pair of vertices incident to the edge.

    /* test6 */
    vector<string> get_neighbours(const string &); // returns a vector of all the vertices, each of which is directly connected with the given vertex via an edge.
    size_t degree(const string &);                 // returns the degree of a vertex.

    /* test7 */
    vector<string> preorder_traversal(); // returns a vector of all the vertices in the visiting order of a preorder traversal over the bst.

    /* test8 */
    vector<string> inorder_traversal(); // returns a vector of all the vertices in the visiting order of an inorder traversal over the bst.

    /* test9 */
    vector<string> postorder_traversal(); // returns a vector of all the vertices in the visiting order of a postorder traversal over the bst.

    /* test10 */
    vector<string> breadth_first_traversal(); // returns a vector of all the vertices in the visiting order of a breadth first traversal over the bst.

    /* test11 */
    Node<T> *lowest_common_ancestor(Node<T> *, Node<T> *);
    vector<string> path(const string &, const string &); // returns a vector of all the vertices in the path from the first vertex to the second vertex.
                                                         //     A path should include the source and destination vertices at the begining and the end, respectively.

    /* test12 */
    // T findMaxPathSum (Node<T>*, T &);
    // vector<string> heaviest_path(Node<T>*, vector<int>& ,int);
    vector<string> path_with_largest_weight(); // returns a path that has the largest weight in the bst.
                                               //    The weight of a path is the sum of the weights of all the vertices (including the source and destination vertices) in the path.

    /* test13 */
    size_t height(); // returns the height of bst. Height of a tree is the number of edges that form the longest path from root to any leaf.

    /* test14 */
    void remove_vertex(const string &); // delete the given vertex from bst -- note that, all incident edges of the vertex should be deleted as well.
};

template <typename T>
Node<T>::Node(const string &vertex_name, const T &vertex_weight)
{ //constructor to initialise a new node with the given params
    this->vertex_name = vertex_name;
    weight = vertex_weight;
    parent = left = right = NULL;
}

/* test1 */
template <typename T>
BST<T>::BST()
{
    vertex_count = edge_count = 0;
    total_weight = 0;
    root = NULL;
}

template <typename T>
BST<T>::~BST() {}

template <typename T>
size_t BST<T>::num_vertices()
{
    return vertex_count;
}

template <typename T>
size_t BST<T>::num_edges()
{
    return edge_count;
}

template <typename T>
T BST<T>::sum_weight()
{
    return total_weight;
}

template <typename T>
Node<T> *BST<T>::find_vertex(const string &u)
{                               //helper function that returns a node corresponding with the vertex of the same name
    stack<Node<T> *> traversal; //Preorder traversal that returns a node if found in the tree
    traversal.push(root);

    while (!traversal.empty())
    {
        Node<T> *curr_node = traversal.top();
        traversal.pop();
        if (curr_node->vertex_name == u)
            return curr_node;
        if (curr_node->right)
            traversal.push(curr_node->right);
        if (curr_node->left)
            traversal.push(curr_node->left);
    }
    return NULL;
}

/* test2 */
template <typename T>
void BST<T>::add_vertex(const string &u, const T &w)
{
    Node<T> *new_node = new Node(u, w);
    Node<T> *curr_node = root;
    Node<T> *prev_node;

    while (curr_node != NULL)
    {
        prev_node = curr_node;
        if (w < curr_node->weight)
            curr_node = curr_node->left;
        else
            curr_node = curr_node->right;
    }
    if (!root)
    { //If root is null, create a new node and make it root
        prev_node = root = new_node;
        vertex_count++;
        total_weight += w; //Increment vertex count and add total weight of the tree
    }
    else if (w < prev_node->weight)
    {                                        //If new node is has a weight less than the weight of the node before it
        prev_node->left = new_node;          //Insert the node to the left of the node before it
        prev_node->left->parent = prev_node; //Set the node before the new node to be be its parent
        vertex_count++;
        edge_count++;
        total_weight += w; //Increment edge count as well because for a node to be inserted after another, an edge has to exist
    }
    else
    { //If the weight is greater than the node before it, insert the node on the right of the node before it
        prev_node->right = new_node;
        prev_node->right->parent = prev_node;
        vertex_count++;
        edge_count++;
        total_weight += w;
    }
}
template <typename T>
bool BST<T>::contains(const string &u)
{
    if (!root)          //If there is no root node, the graph doesnt exist and therefore cannot contain the a node
        return false;   //with the given name
    if (find_vertex(u)) //If the helper function returns something, the node exists
        return true;
    return false;
}

/* test3 */
template <typename T>
vector<string> BST<T>::get_vertices()
{
    return preorder_traversal();
}
template <typename T>
vector<string> BST<T>::get_leaves()
{ //Preorder traversal that inserts leaves into a vector for each iteration of the while loop
    stack<Node<T> *> traversal;
    vector<string> leaves;
    traversal.push(root);

    while (!traversal.empty())
    {
        Node<T> *curr_node = traversal.top();
        traversal.pop();

        if (curr_node->left)
            traversal.push(curr_node->left);
        if (curr_node->right)
            traversal.push(curr_node->right);
        else if (curr_node->left == NULL && curr_node->right == NULL) //If the current node's left and right nodes do not exist, it is a leaf node
            leaves.push_back(curr_node->vertex_name);
    }
    return leaves;
}

/* test4 */
template <typename T>
bool BST<T>::adjacent(const string &u, const string &v)
{
    if (!root)
        return false;
    vector<string> neighbours = get_neighbours(u);                         //Insert all neighbours of the given node into a vector
    if (find(neighbours.begin(), neighbours.end(), v) != neighbours.end()) //If the target node name is found in the neighbours of the source node, the two nodes are adjacent
        return true;
    return false;
}

/* test5 */
template <typename T>
vector<pair<string, string>> BST<T>::get_edges()
{ //Implements preorder traversal to insert edges into the edges vector
    stack<Node<T> *> traversal;
    vector<pair<string, string>> edges;
    traversal.push(root);

    while (!traversal.empty())
    {
        Node<T> *curr_node = traversal.top();
        traversal.pop();

        if (curr_node->right)
        {
            traversal.push(curr_node->right); //If a node is adjacent to another, an edge exists between the two nodes
            if (adjacent(curr_node->vertex_name, curr_node->right->vertex_name))
                edges.push_back(make_pair(curr_node->vertex_name, curr_node->right->vertex_name));
        }
        if (curr_node->left)
        {
            traversal.push(curr_node->left);
            if (adjacent(curr_node->vertex_name, curr_node->left->vertex_name))
                edges.push_back(make_pair(curr_node->vertex_name, curr_node->left->vertex_name));
        }
    }
    return edges;
}

/* test6 */
template <typename T>
vector<string> BST<T>::get_neighbours(const string &u)
{
    vector<string> neighbours;
    Node<T> *curr_node = find_vertex(u);
    if (curr_node->left) //If the node to the left of the main node exists, it is a neighbour of the target node
        neighbours.push_back(curr_node->left->vertex_name);
    if (curr_node->right) //If the node to the right of the main node exists, it is a neighbour of the target node
        neighbours.push_back(curr_node->right->vertex_name);
    if (curr_node->parent) //If parent node of the main node exists, it is a neighbour of the target node
        neighbours.push_back(curr_node->parent->vertex_name);
    return neighbours;
}
template <typename T>
size_t BST<T>::degree(const string &u)
{
    return get_neighbours(u).size(); //The number of neighbours of a node == the degree of the node
}

/* test7 */
template <typename T>
vector<string> BST<T>::preorder_traversal()
{                               //Traverses root, left branch and then right branch
    stack<Node<T> *> traversal; //LIFO container for traversal
    vector<string> preorder;
    traversal.push(root);

    while (!traversal.empty())
    {
        Node<T> *curr_node = traversal.top();
        preorder.push_back(curr_node->vertex_name);
        traversal.pop();
        if (curr_node->right) //pushes all right nodes into the stack first so they are added to the vector last
            traversal.push(curr_node->right);
        if (curr_node->left)
            traversal.push(curr_node->left);
    }
    return preorder;
}

/* test8 */
template <typename T>
vector<string> BST<T>::inorder_traversal()
{ //Traverses left branch, root and then right branch
    stack<Node<T> *> traversal;
    vector<string> inorder;
    Node<T> *curr_node = root; //traversal starts from root but doesnt push root into the vector first

    while (!traversal.empty() || curr_node != NULL)
    {
        if (curr_node)
        { //Pushes left of the current node until it reaches a left leaf node
            traversal.push(curr_node);
            curr_node = curr_node->left;
        }
        else
        { //starts pushing from the left leaf, up to the root of the tree
            curr_node = traversal.top();
            traversal.pop();
            inorder.push_back(curr_node->vertex_name);
            curr_node = curr_node->right; //starts traversal of the right branches/sub branches
        }
    }
    return inorder;
}

/* test9 */
template <typename T>
vector<string> BST<T>::postorder_traversal()
{ //Traverses left branch, right branch and then root
    stack<Node<T> *> traversal_left, traversal_right;
    traversal_left.push(root); //Start traversing from root but don't add it to the vector
    vector<string> postorder;
    Node<T> *curr_node;
    Node<T> *prev_node = NULL;

    while (!traversal_left.empty())
    {
        curr_node = traversal_left.top();
        traversal_left.pop();
        traversal_right.push(curr_node); //Pushes the left node into the right traversal stack

        if (curr_node->left)
            traversal_left.push(curr_node->left);
        if (curr_node->right)
            traversal_left.push(curr_node->right);
    }
    while (!traversal_right.empty())
    {
        curr_node = traversal_right.top();
        traversal_right.pop();
        postorder.push_back(curr_node->vertex_name); //Adds the left nodes into the stack first
    }                                                //Adds the right nodes and root into the stack after all left nodes have been added
    return postorder;
}

/* test10 */
template <typename T>
vector<string> BST<T>::breadth_first_traversal()
{                               //Traverses top to bottom, left to right
    queue<Node<T> *> traversal; //FIFO container for traversal
    vector<string> breadth_first;
    traversal.push(root); //Starts traversal with root and pushes into the vector at the beginning
    Node<T> *curr_node;

    while (!traversal.empty())
    {
        curr_node = traversal.front();
        traversal.pop();
        breadth_first.push_back(curr_node->vertex_name);
        if (curr_node->left)
            traversal.push(curr_node->left); //Push left first
        if (curr_node->right)
            traversal.push(curr_node->right); //Push right after to maintain the left to right approach
    }                                         //This also maintains the top to bottom approach
    return breadth_first;
}

/* test11 */
template <typename T>
Node<T> *BST<T>::lowest_common_ancestor(Node<T> *source, Node<T> *dest)
{ //Helper function that finds the lowest common ancestor between two nodes
    if (!root)
        return NULL;
    Node<T> *curr_node = root;

    while (curr_node)
    { //If the two target nodes have a lower weight than the current node, the LCA is in the left subtree
        if (curr_node->weight > max(source->weight, dest->weight))
            curr_node = curr_node->left;
        else if (curr_node->weight < min(source->weight, dest->weight)) //If the two target nodes have a higher weight than the current node, the LCA is in the left subtree
            curr_node = curr_node->right;
        else
            return curr_node; //If the two nodes lie on different sides of the current node, the current node is the LCA
    }
    return NULL;
}
template <typename T>
vector<string> BST<T>::path(const string &u, const string &v)
{
    vector<string> path, dest_path;
    Node<T> *source_node, *dest_node;

    if (find_vertex(u))
        source_node = find_vertex(u);
    if (find_vertex(v))
        dest_node = find_vertex(v);

    while (source_node->vertex_name != lowest_common_ancestor(source_node, dest_node)->vertex_name)
    { //For all nodes in the first half of the path of the LCA
        if (source_node->parent)
        { //Find the lowest common ancestor of the source and destination node from the bottom up
            path.push_back(source_node->vertex_name);
            source_node = source_node->parent;
        }
    }
    path.push_back(lowest_common_ancestor(source_node, dest_node)->vertex_name); //Add the LCA of the 2 nodes to the vector
    while (dest_node->vertex_name != lowest_common_ancestor(source_node, dest_node)->vertex_name)
    { //For all nodes in the second half of the path of the LCA
        if (dest_node->parent)
        {
            dest_path.push_back(dest_node->vertex_name);
            dest_node = dest_node->parent;
        }
    }
    reverse(dest_path.begin(), dest_path.end());                   //Reverse the path of the second loop because we found it in a bottom to top order
    copy(dest_path.begin(), dest_path.end(), back_inserter(path)); //Merge the two vectors together
    return path;
}

/* test12 */
template <typename T>
vector<string> BST<T>::path_with_largest_weight()
{
    vector<string> max_path;
    T max_path_sum = INT_MIN;

    for (auto source_vertex : get_vertices())
    { //Calculating paths between all non-leaf vertices and leaf vertices
        for (auto dest_vertex : get_leaves())
        {
            T temp_max_path_sum = 0;
            for (auto vertex : path(source_vertex, dest_vertex))
            {                                                     //For each node in the path of vertex to leaf
                temp_max_path_sum += find_vertex(vertex)->weight; //Calculate the total weight of the path
                if (temp_max_path_sum > max_path_sum)
                { //If the weight of any path is greater than the actual max path
                    max_path_sum = temp_max_path_sum;
                    max_path = path(source_vertex, dest_vertex); //Replace the path with the new max path
                }
            }
        }
    }
    return max_path;
}

/* test13 */
template <typename T>
size_t BST<T>::height()
{
    if (!root || vertex_count == 0)
        return -1;
    queue<Node<T> *> traversal;
    traversal.push(root);
    size_t tree_height = 0;
    Node<T> *curr_node;

    while (!traversal.empty())
    { //Traverses the tree from top to bottom and increments height everytime it traverses one level of the tree
        int queue_size = traversal.size();

        while (queue_size--)
        {
            curr_node = traversal.front();
            traversal.pop();
            if (curr_node->right)
                traversal.push(curr_node->right);
            if (curr_node->left)
                traversal.push(curr_node->left);
        }
        tree_height++;
    }
    return tree_height - 1;
}

/* test14 */
template <typename T>
void BST<T>::remove_vertex(const string &u)
{
    Node<T> *curr_node = root;
    Node<T> *prev_node = NULL;
    Node<T> *deletion_node = find_vertex(u);

    while (curr_node && curr_node->vertex_name != u)
    { //Checks if the key of the node to be delete is in the BST or not
        prev_node = curr_node;
        if (deletion_node->weight < curr_node->weight)
            curr_node = curr_node->left;
        else
            curr_node = curr_node->right;
    }

    if (!curr_node->left || !curr_node->right)
    { //If there is only one child of the node to delete
        Node<T> *new_node;
        if (!curr_node->left) //If the left child doesn't exist, copy the right node into the new node
            new_node = curr_node->right;
        else
            new_node = curr_node->left;

        total_weight -= curr_node->weight;
        vertex_count--;
        edge_count--;
        free(curr_node); //Free the memory the node from the tree
    }
    else
    { //If there are two children of the node to delete
        Node<T> *parent_of_inorder_node = NULL;
        Node<T> *inorder_node = curr_node->right;

        while (inorder_node->left)
        { //Inorder traversal to find which node will replace the node to be deleted
            parent_of_inorder_node = inorder_node;
            inorder_node = inorder_node->left;
        }
        if (parent_of_inorder_node)                             //If the parent node isn't the current node,then make the left child of the parent
            parent_of_inorder_node->left = inorder_node->right; //equal to the right child of the node from the inorder traversal
        else
            curr_node->right = inorder_node->right;

        curr_node->vertex_name = inorder_node->vertex_name;
        total_weight -= inorder_node->weight; //Update all counts and the total weight of the tree
        vertex_count--;
        edge_count--;
        free(inorder_node);
    }
}