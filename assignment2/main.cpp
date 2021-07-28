#include "tree.hpp"

int main()
{

    BST<int> t;

    t.add_vertex("C", 5);
    t.add_vertex("A", 1);
    t.add_vertex("D", 3);
    t.add_vertex("H", 10);
    t.add_vertex("G", 20);
    t.add_vertex("F", 15);
    t.add_vertex("B", 30);
    t.add_vertex("E", 17);

    // for(auto x : t.get_leaves()){
    //     cout << x << " ";
    // }
    // cout << endl;
    // cout << endl;
    for (auto x : t.inorder_traversal())
    {
        cout << x << "";
    }
    cout << endl;
    cout << t.inorder_traversal().size() << endl;
    cout << t.num_vertices() << ", " << t.num_edges() << ", " << t.sum_weight() << endl;
    t.remove_vertex("H");
    cout << endl;
    for (auto x : t.inorder_traversal())
    {
        cout << x << "";
    }
    cout << endl;
    cout << t.inorder_traversal().size() << endl;
    cout << t.num_vertices() << ", " << t.num_edges() << ", " << t.sum_weight() << endl;

    // while(curr_node != NULL || !traversal_left.empty()){
    //     if(curr_node != NULL && find(postorder.begin(), postorder.end(), curr_node->vertex_name) == postorder.end()){
    //     traversal_left.push(curr_node);
    //     curr_node = curr_node->left;
    //     }
    //     else{
    //         if(traversal_left.top()->right != NULL && traversal_left.top()->right != prev_node){
    //         curr_node = traversal_left.top()->right;
    //         }
    //         else{
    //             curr_node = traversal_left.top();
    //             traversal_left.pop();
    //             postorder.push_back(curr_node->vertex_name);
    //             prev_node = curr_node;
    //         }
    //     }
    // }
    // for(auto i = 0; i < postorder.size(); i++){
    //     cout << postorder[i] << endl;
    // }
    // template <typename T>
    // T BST<T>::findMaxPathSum (Node<T>* node, T &globalMax){
    //     // base case: empty tree
    //     if (node == nullptr) {
    //         return 0;
    //     }

    //     // find maximum path sum "starting" from the left child
    //     int left = findMaxPathSum(node->left, globalMax);

    //     // find maximum path sum "starting" from the right child
    //     int right = findMaxPathSum(node->right, globalMax);

    //     // Try all possible combinations to get the optimal result
    //     globalMax = max(globalMax, node->weight);
    //     globalMax = max(globalMax, node->weight + left);
    //     globalMax = max(globalMax, node->weight + right);
    //     globalMax = max(globalMax, node->weight + left + right);

    //     // return the maximum path sum "starting" from the given node
    //     return max(node->weight, node->weight + max(left, right));
    // }

    // template <typename T>
    // vector<string> heaviest_path(Node<T> *root, vector<int>& path ,int path_sum){
    //     vector<string> heaviest_path;
    //     heaviest_path.push_back(root->weight);

    //     // check if there's any k sum path
    //     // in the left sub-tree.
    //     heaviest_path(root->left, heaviest_path, path_sum);

    //     // check if there's any k sum path
    //     // in the right sub-tree.
    //     heaviest_path(root->right, heaviest_path, path_sum);

    //     // check if there's any k sum path that
    //     // terminates at this node
    //     // Traverse the entire path as
    //     // there can be negative elements too
    //     int f = 0;
    //     // for (int j=path.size()-1; j>=0; j--)
    //     // {
    //     //     f += path[j];

    //     //     // If path sum is k, print the path
    //     //     if (f == path_sum)
    //     //         printVector(path, j);
    //     // }

    //     // Remove the current element from the path
    //     heaviest_path.pop_back();
    //     return heaviest_path;
    // }
    // vector<int> abc;
    // int y = INT_MIN;
    // T max = findMaxPathSum(root, y);
    // //cout << findMaxPathSum(root, y);
    // heaviest_path(root, abc, max);
}