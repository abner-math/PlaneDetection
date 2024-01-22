#ifndef UNIONFIND_H
#define UNIONFIND_H

#include <vector>
#include "string.h"

class UnionFind
{
public:
    UnionFind(size_t n)
        : mGroups(std::vector<int>(n, -1))
    {

    }

    int root(int x)
    {
        int r = x;

        /* Find root */
        while(mGroups[r] >= 0)
            r = mGroups[r];

        /* Compress path to root */
        while(mGroups[x] >= 0) {
            int tmp = mGroups[x];
            mGroups[x] = r;
            x = tmp;
        }

        return r;
    }

    void join(int x, int y)
    {
        x = root(x);
        y = root(y);

        if(x != y) {
            if(mGroups[x] < mGroups[y]) {
                mGroups[x] += mGroups[y];
                mGroups[y] = x;
            }
            else {
                mGroups[y] += mGroups[x];
                mGroups[x] = y;
            }
        }
    }

    bool connected(int x, int y)
    {
        return root(x) == root(y);
    }

private:
    std::vector<int> mGroups;

};

#endif // UNIONFIND_H
