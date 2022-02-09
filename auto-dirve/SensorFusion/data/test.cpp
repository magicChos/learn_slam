

#include <iostream>
#include <vector>
using namespace std;


int bfs(vector<vector<int>> &matrix , int i , int j)
{
    int m = matrix.size();
    int n = matrix[0].size();
    if (i >= 0 && i < m && j >= 0 && j < n && matrix[i][j] == 1)
    {
        matrix[i][j] = 0;
        return 1 + bfs(matrix , i + 1 , j) + bfs(matrix , i - 1 , j) + bfs(matrix , i , j + 1) + bfs(matrix , i , j - 1);
    }
    return 0;
}

int findIslandNumber(vector<vector<int>> &matrix)
{
    int nrow = matrix.size();
    int ncol = matrix[0].size();
    int count = 0;
    for(int i = 0 ; i < nrow ; i++)
    {
        for(int j = 0 ; j < ncol ; j++)
        {
            if (matrix[i][j] == 1)
            {
                count += bfs(matrix , i , j);
            }
        }
    }
    return count;
}