//#pragma GCC optimize ("Ofast")
//#pragma GCC target ("string"...)
//#pragma GCC push_options
//#pragma GCC pop_options
//#pragma GCC reset_options
//#pragma GCC optimize ("O3")
#include<cstdio>
#include<cstring>
#include<cstdlib>
#include<cmath>
#include<algorithm>
#include<vector>
using namespace std;
typedef long long LL;
#define INF (LL)1e15
#define MAXV 105

template<typename T>
T getNum(){
    T res=0;
    char c;
    while(1) {
        c=getchar_unlocked();
        if(c==' ' || c=='\n') continue;
        else break;
    }
    res=c-'0';
    while(1) {
        c=getchar_unlocked();
        if(c>='0' && c<='9') res=10*res+c-'0';
        else break;
    }
    return res;
}

void GetStr(char* res){
    char c;
    while(1){
        c=getchar_unlocked();
        if(c==' ' || c=='\n') continue;
        else break;
    }
    *res=c; res++;
    while(1){
        c=getchar_unlocked();
        if (c==' ' || c=='\n' || c==EOF) break;
        *res=c; res++;
    }
    *res='\0';
}

struct Hungarian {

    int org_n, org_m, n;
    vector<int> mateL, mateR, p;
    vector<LL> lx, ly, slack;
    vector<vector<LL> > W;
    vector<bool> m;
    LL inf;

    Hungarian(int _n, int _m) {
        org_n = _n;
        org_m = _m;
        n = max(_n, _m);
        inf = numeric_limits<LL>::max();
        W = vector<vector<LL> >(n, vector<LL>(n));
        mateL = vector<int>(n, -1);
        mateR = vector<int>(n, -1);
        p = vector<int>(n);
        lx = vector<LL>(n, -inf);
        ly = vector<LL>(n);
        slack = vector<LL>(n);
    }


    void augment(int j) {
        int i, next;
        do {
            i = p[j];
            mateR[j] = i;
            next = mateL[i];
            mateL[i] = j;
            if (next != -1) j = next;
        } while (next != -1);
    }

    LL solve_hungarian() {
        int vrex = 0;
        for (int i = 0; i < n; i++) if (mateL[i] == -1) vrex++;
        while (vrex > 0) {
            for (int i = 0; i < n; i++) {
                m[i] = false;
                p[i] = -1;
                slack[i] = INF;
            }
            bool aug = false, Q[MAXV];
            int numQ = 0;
            memset(Q, 0, sizeof(Q));
            for (int i = 0; i < n; i++)
                if (mateL[i] == -1) {
                    Q[i] = true;
                    numQ++;
                }

            do {
                int i, j;
                for (int k = 0; k < n; k++)
                    if (Q[k]) {
                        i = k;
                        numQ--;
                        Q[i] = false;
                        m[i] = true;
                        j = 0;
                        break;
                    }

                while (aug == false && j < n) {
                    if (mateL[i] != j) {
                        if (lx[i] + ly[j] - W[i][j] < slack[j]) {
                            slack[j] = lx[i] + ly[j] - W[i][j];
                            p[j] = i;
                            if (slack[j] == 0) {
                                if (mateR[j] == -1) {
                                    augment(j);
                                    aug = true;
                                    vrex--;
                                } else {
                                    if (Q[mateR[j]] == false) {
                                        Q[mateR[j]] = true;
                                        numQ++;
                                    }
                                }
                            }
                        }
                    }
                    j++;
                }

                if (aug == false && numQ == 0) {
                    LL tao = INF;
                    for (int k = 0; k < n; k++)
                        if (slack[k] > 0)
                            tao = min(tao, slack[k]);
                    for (int k = 0; k < n; k++)
                        if (m[k])
                            lx[k] -= tao;

                    int x = -1;
                    bool X[MAXV];
                    for (int k = 0; k < n; k++)
                        if (slack[k] == 0)
                            ly[k] += tao;
                        else {
                            slack[k] -= tao;
                            if (slack[k] == 0 && mateR[k] == -1) x = k;
                            if (slack[k] == 0) X[k] = true;
                            else X[k] = false;
                        }

                    if (x == -1) {
                        for (int k = 0; k < n; k++)
                            if (X[k]) {
                                Q[mateR[k]] = true;
                                numQ++;
                            }
                    } else {
                        augment(x);
                        aug = true;
                        vrex--;
                    }
                }
            } while (aug == false);
        }

        LL ans = 0;
        for (int i = 0; i < n; i++)
            ans += (lx[i] + ly[i]);
        return ans;
    }

    void addEdge(int u, int v, int w) {
        W[u][v] = w;
    }

    void solve()
    {
        for (int i=0;i<n;i++)
            for (int j=0;j<n;j++) addEdge(i,j,0);

        solve_hungarian();
    }

}

int main(){
    memset(W, 0, sizeof(W));
    V=getNum<int>();
    for(int i=0;i<V;i++)
        for(int j=0;j<V;j++)
            W[i][j]=getNum<LL>();

    memset(mateL, -1, sizeof(mateL));
    memset(mateR, -1, sizeof(mateR));
    for(int i=0;i<V;i++)
    {
        for(int j=0;j<V;j++) lx[i] = max(lx[i], W[i][j]);
        ly[i] = 0;
    }
    LL ans = hungarian();

    char type[2];
    int Q, u, v;
    LL w;
    Q=getNum<int>();
    while(Q--)
    {
        GetStr(type);
        if(type[0]=='C')
        {
            u=getNum<int>();
            v=getNum<int>();
            w=getNum<LL>();
            W[u][v] = w;
            mateR[ mateL[u] ] = -1; mateL[u] = -1;
            lx[u] = 0; for(int i=0;i<V;i++) lx[u] = max(lx[u], W[u][i]-ly[i]);
        }
        else if(type[0]=='X')
        {
            u=getNum<int>();
            for(int i=0;i<V;i++) W[u][i]=getNum<LL>();
            mateR[ mateL[u] ] = -1; mateL[u] = -1;
            lx[u] = 0; for(int i=0;i<V;i++) lx[u] = max(lx[u], W[u][i]-ly[i]);
        }
        else if(type[0]=='Y')
        {
            u=getNum<int>();
            for(int i=0;i<V;i++) W[i][u]=getNum<LL>();
            if(mateR[u]!=-1)
            {
                mateL[ mateR[u] ] = -1;
                mateR[u] = -1;
            }
            ly[u] = 0; for(int i=0;i<V;i++) ly[u] = max(ly[u], W[i][u]-lx[i]);
        }
        else if(type[0]=='A')
        {
            int u = V++;
            ly[u] = 0; for(int i=0;i<V;i++) ly[u] = max(ly[u], W[i][u]-lx[i]);
        }
        else if(type[0]=='Q') printf("%lld\n", hungarian());
    }
    return 0;
}
