#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <list>
#include <vector>
#include <queue>
#include <time.h>
#include<string.h>
using namespace std;
const int MAX_N = 100;            //地图长
const int MAX_M = 100;            //地图宽
const int MAX_K = 50;
const int Queue_len = MAX_N * MAX_M + MAX_K;
//四个方向
const int dx[4] = { 0 ,0, -1, 1 };
const int dy[4] = { 1, -1, 0, 0 };
int un,um,uk;
int n, m, k;
bool if_create_map = true;
int center_len, center_x, center_y;    //表示终点周围center_len * center_len的地方是空地, 以及终点的位置
int rand_barrier = 10;          // 表示有1/rand_barrier的概率生成有障碍的点。
char mapBuffer[MAX_N][MAX_M];   //地图数据
int vis[MAX_N][MAX_M];


struct point{
	int x,y;
	point() {
		x = -1;
		y = -1;
	}
} path[MAX_K][Queue_len];
int path_len[MAX_K];

struct Node {
    int x, y;                 // 坐标值
    int cost;                 // 起点到这长度
    int pred;                 // 预测总长度
    Node* father;             // 父节点
	Node()=default; 
    Node(int X, int Y, int c, Node* _father) : x(X), y(Y), cost(c), father(_father) {
        pred = abs(X - center_x) + abs(Y - center_y) + c;
    }
};

Node start_Node[MAX_K + 1];

//比较器，用以优先队列的指针类型比较
struct NodePtrCompare {
    bool operator()(Node* x, Node* y) {
        return x->pred > y->pred;
    }
};

//使用最大优先队列
std::priority_queue<Node*, std::vector<Node*>, NodePtrCompare> openlist;

//如果超出地图边界了,就返回true，否则返回false。
bool check_out_of_bound(int tx, int ty) {
    if (tx < 0 || ty < 0 || tx >= n || ty >= m)
        return true;
    return false;
}

void explore(Node& cur_node) {
    openlist.pop();
    for (int i = 0; i < 4; ++i)
    {
        int tx = cur_node.x + dx[i];
        int ty = cur_node.y + dy[i];
        if (!check_out_of_bound(tx, ty) && vis[tx][ty] == 0 && mapBuffer[tx][ty] != 'X') {
        	Node *tmp = new Node(tx, ty, cur_node.cost + 1, &cur_node);
            openlist.push(tmp);
        }
    }
    vis[cur_node.x][cur_node.y] = 1;
}

//开始搜索路径
std::list<Node*> findpath(Node snode) {
	while (!openlist.empty()) openlist.pop();
    std::list<Node*> road;
    memset(vis, 0, sizeof(vis));
    openlist.push(&snode);
    Node* current_node = nullptr;
    //重复寻找预测和花费之和最小节点开启检查
    while (!openlist.empty())
    {
        current_node = openlist.top();
        // 找到终点后，则停止搜索
        if (current_node->x == center_x && current_node->y == center_y) break; 
        explore(*current_node);
    }
    for (auto rs = current_node; rs != nullptr; rs = rs->father)  
        road.push_back(rs); 
    return road;
}

void dfs(int x, int y) {
    vis[x][y] = 1;
    for (int i = 0; i < 4; i++) {
        int tx = x + dx[i];
        int ty = y + dy[i];
        if (!check_out_of_bound(tx, ty) && vis[tx][ty] == 0 && mapBuffer[tx][ty] != 'X') {
            dfs(tx, ty);
        }
    }
}

void readMap(){
	int cnt = 0;
    int qx[2 * (MAX_N + MAX_M)]; int qy[2 * (MAX_N + MAX_M)];
    int idx[MAX_K];
    for (int i = 0; i <= n; i++)
    {
        if (mapBuffer[i][0] == 'E') {
            qx[cnt] = i;
            qy[cnt] = 0;
            cnt = cnt + 1;
        }
        if (mapBuffer[i][m-1] == 'E') {
            qx[cnt] = i;
            qy[cnt] = m - 1;
            cnt = cnt + 1;
        }
    }

    for (int j = 1; j <= m - 2; j++)
    {
        if (mapBuffer[0][j] == 'E') {
            qx[cnt] = 0;
            qy[cnt] = j;
            cnt = cnt + 1;
        }
        if (mapBuffer[m - 1][j] == 'E') {
            qx[cnt] = m - 1;
            qy[cnt] = j;
            cnt = cnt + 1;
        }
    }

    for (int i = 0; i < k; i++)
        idx[i] = i;

    for (int i = 0; i < k; i++) {
        int tx = qx[idx[i]]; int ty = qy[idx[i]];
        mapBuffer[tx][ty] = 'E';
        start_Node[i] = Node(tx, ty, 0, nullptr);
    }
}

//创建地图 如果出口数不足k个，就return false，否则return true
bool createMap() {
    //随机生成地图
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < m; ++j) {
            //以一定的概率概率生成障碍物，不可走
            if (rand() % rand_barrier == 0) 
                mapBuffer[i][j] = 'X';
            else 
                mapBuffer[i][j] = '.';

        }
    center_x = n / 2;  center_y = m / 2;  //表示终点F的位置。
    //F的周围是center_len长度的空地
    mapBuffer[center_x][center_y] = 'F';
    for (int i = - center_len / 2; i <= center_len / 2; i++)
        for (int j = -center_len / 2; j <= center_len / 2; j++)
        {
            int tx = i + center_x; int ty = j + center_y;
            if (tx == center_x && ty == center_y) continue;
            if (check_out_of_bound(tx, ty)) continue;
            mapBuffer[tx][ty] = '.';
        }

    //找到合法的入口点E
    memset(vis, 0, sizeof(vis));
    dfs(center_x, center_y);
    int cnt = 0;
    int qx[2 * (MAX_N + MAX_M)]; int qy[2 * (MAX_N + MAX_M)];
    int idx[MAX_K];
    for (int i = 0; i <= n; i++)
    {
        if (vis[i][0] == 1) {
            qx[cnt] = i;
            qy[cnt] = 0;
            cnt = cnt + 1;
        }
        if (vis[i][m-1] == 1) {
            qx[cnt] = i;
            qy[cnt] = m - 1;
            cnt = cnt + 1;
        }
    }

    for (int j = 1; j <= m - 2; j++)
    {
        if (vis[0][j] == 1) {
            qx[cnt] = 0;
            qy[cnt] = j;
            cnt = cnt + 1;
        }
        if (vis[m - 1][j] == 1) {
            qx[cnt] = m - 1;
            qy[cnt] = j;
            cnt = cnt + 1;
        }
    }
    if (cnt < k) return false;
    bool flag = true;
    while (flag) {
        for (int i = 0; i < k; i++)
            idx[i] = rand() % cnt;
        flag = false;
        for (int i = 0; i < k; i++)
            for (int j = 0; j < i; j++)
                if (idx[i] == idx[j]) flag = true;
    }
    for (int i = 0; i < k; i++) {
        int tx = qx[idx[i]]; int ty = qy[idx[i]];
        mapBuffer[tx][ty] = 'E';
        start_Node[i] = Node(tx, ty, 0, nullptr);
    }
    return true;
}

//打印地图
void printMap() {
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j)
            cout << mapBuffer[i][j];
        cout << endl;
    }
    cout << endl;
}

void perform_single(int l){
	cout << endl << endl;
	cout << "The total length is " << path_len[l] << endl;
	char tmp[MAX_N][MAX_M];
	for (int i = 0; i < n; i++)
	  for (int j = 0; j < m; j++){
	  	tmp[i][j] = mapBuffer[i][j];
	  }
	for (int i = 1; i <= path_len[l]; i++)
	{
		cout << "Epoch = " << i << endl; 
		mapBuffer[path[l][i - 1].x][path[l][i - 1].y] = 'P';
		printMap();
		mapBuffer[path[l][i - 1].x][path[l][i - 1].y] = char(int('0') + l);
	}
	for (int i = 0; i < n; i++)
	  for (int j = 0; j < m; j++){
	  	mapBuffer[i][j] = tmp[i][j];
	  }		
}

point current[MAX_K][Queue_len];
point pre[MAX_K];
int q[MAX_K];
int last_time[MAX_K];
bool block[MAX_K];


void perform_all(int time_len){
	cout << "The total length is " << time_len << endl;
	char tmp[MAX_N][MAX_M];
	for (int i = 0; i < n; i++)
	  for (int j = 0; j < m; j++){
	  	tmp[i][j] = mapBuffer[i][j];
	  }
	for (int i = 0; i <= time_len; i++)
	{
		cout << "Epoch = " << i + 1 << endl; 
		for (int j = 0; j < k; j++){
			int x = current[j][i].x;
			int y = current[j][i].y;
			if (x == -1) continue;		
			mapBuffer[x][y] = 'P';	
		}
		
		printMap();
		
		for (int j = 0; j < k; j++){
			int x = current[j][i].x;
			int y = current[j][i].y;
			if (x == -1) continue;		
			mapBuffer[x][y] = char(int('0') + j);	
		}
	}
	
	
	for (int i = 0; i < n; i++)
	  for (int j = 0; j < m; j++){
	  	mapBuffer[i][j] = tmp[i][j];
	  }		
}


void solving_k_persons(int k){
	cout << endl << endl;
	memset(q, 0, sizeof(q));
	memset(last_time, 0, sizeof(last_time));
	for (int i = 0; i < k; i++) block[i] = false;
	int finished = 0;
	int blocked = 0;
	
	int time_idx = 0;
	for (int i = 0; i < k; i++)
		current[i][0] = path[i][0];
	  
	while (finished + blocked < k){
		time_idx = time_idx + 1;
		for (int i = 0; i < k; i++){
			if (q[i] == path_len[i] - 1){
				continue;
			} 
			if (block[i]) continue;
			int idx = q[i] + 1;
			int x = path[i][idx].x;
			int y = path[i][idx].y;
			int flag = 0;
			for (int j = 0; j < k; j++)
			{
				if (j == i) continue;
				if (x == current[j][time_idx - 1].x && y == current[j][time_idx - 1].y){
					flag = 1;
					break;
				}
			}
			
			if (flag == 1){
				last_time[i] += 1;
				if (last_time[i] > k) {
					block[i] = true;
					blocked++;
				}
				current[i][time_idx] = path[i][q[i]];
			} else {
				last_time[i] += 0;
				q[i] = q[i] + 1;
				current[i][time_idx] = path[i][q[i]];
				if (q[i] == path_len[i] - 1) {
					finished += 1; 
				}
			}
		}
	}
	cout << "The result of all the player finding the path:" << endl;
	if (finished == k) cout << "A maze is fully solvable as all players can reach the finishing point" << endl;
	else if (blocked == k) cout << "A maze is not solvable due to all players blocking each other" << endl;
	else cout << "A maze is partially solvable as some players can reach the finishing point" << endl;
	
	cout << endl << endl;
	cout << "Perform the path of all the persons" << endl;
	perform_all(time_idx);
}

void generate_and_save(int number){
	int cnt = 1;
	freopen("maze.txt","w", stdout);
	cout << number << endl;
	while (cnt <= number){
		cout << endl;
		n = rand() % (un - 9) + 10;
		m = rand() % (um - 9) + 10;
		k = rand() % (uk - 1) + 2;
		cout << cnt << " " <<  n << " " << " " << m << " " << k << endl; 
		bool b = createMap();
		while (!b) {
			b = createMap();
		}
		printMap();
		cnt = cnt + 1;
		
	}
//	fclose(stdout);	
	return;
}

int type_int(int down, int up){
	int opt;
	while(1){ 
		cin >> opt;
		try{
			if(cin.fail())
				throw 1;
		}
		catch(int){
			cin.clear();
			cin.sync();
			cout << "Notice: Please type the integer in [" << down << ":" << up << "]: ";
			continue;
		}
		if (opt >= down && opt <= up) break;
	    else {
	    	cin.clear();
			cin.sync();
			cout << "Notice: Please type the integer in [" << down << ":" << up << "]: ";
			continue;
		}
    }
    return opt;
}

void process(){
	srand(time(NULL));
    //创建地图
    if (if_create_map == true)
		createMap();
	else
		readMap();
	cout << "-----------1. The generated map is as follows:-----------" << endl;
	printMap();
	for (int i = 0; i < k; i++)
	{
		//A*搜索得到一条路径
	    std::list<Node*> road = findpath(start_Node[i]);
	    //将A*搜索的路径经过的点标记为'O'
	    
	    path_len[i] = road.size();
	    int j = 0;
	    for (list<Node*>::const_iterator p = road.begin(); p !=  road.end(); p++)
	        //mapBuffer[(*p)->x][(*p)->y] = 'P'; 
            {
            	j = j + 1;
            	path[i][path_len[i] - j].x = (*p)->x;
            	path[i][path_len[i] - j].y = (*p)->y;
			}
	}
		    
    //print single person
    cout << "-----------2. Single one person, for example the path of the 1st person---------" << endl;
    int want_k = 1;
    perform_single(want_k);
    
    //solving k persons
    cout << "-----------3. Begin solving all the persons ------------------------" << endl;
    solving_k_persons(k);
}


int main() {
	int opt;
	cout << "Please type one integer 1 to enter the default schema or 2 to interaction schema: ";
	while(1){ 
		cin >> opt;
		try{
			if(cin.fail())
				throw 1;
		}
		catch(int){
			cin.clear();
			cin.sync();
			cout << "Notice: Please type the integer 1 or 2" << endl;
			cout << "Please type one integer 1 to enter the default schema or 2 to interaction schema: ";
			continue;
		}
		if (opt == 1 || opt == 2) break;
	    else {
	    	cin.clear();
			cin.sync();
			cout << "Notice: Please type the integer 1 or 2" << endl;
			cout << "Please type one integer 1 to enter the default schema or 2 to interaction schema: ";
		}
    }
    if (opt == 1){
    	n = 10; m = 20; k = 3; center_len = 3;
	    process();
	} if (opt == 2) {
		n = 10; m = 20; k = 3; center_len = 3;
		int opt2;
		cout << "1.Type 1 to generate the maps and save them:" << endl;
		cout << "2.Type 2 to see the process of the program:" << endl;
		cout << "3.Type 3 to load data from the maze.txt" << endl; 
		cout << "Type your choice: ";
		int ch;
		ch = type_int(1, 3);
		
		if (ch == 1)
		{
			cout << "Type a integer between [10,100] to decide the uppder bound of the height of the maze:";
			un = type_int(10, 100);
			cout << "Type a integer between [10,100] to decide the uppder bound of the length of the maze:";
			um = type_int(10, 100);
			
			cout << "Type a integer between [2,50] to decide the uppder bound of number of persons:";
			uk = type_int(2, 50);
			
			cout << "Generating the 100 random mazes and save them in the file maze.txt, Type to exit" << endl;
			generate_and_save(100);
//			cout << "Finish generating" << endl;
		} else if (ch == 2){
			cout << "1.Type 1 to save the process in the file output.txt:" << endl;
			cout << "2.Type 2 to see the process in the screen:" << endl;
			cout << "Type your choice: ";
			opt2 = type_int(1, 2);
			cout << "Type a integer between [10,30] to decide the height of the maze:";
			n = type_int(10, 30);
			cout << "Type a integer between [10,50] to decide the length of the maze:";
			m = type_int(10, 50);
			cout << "Type a integer between [2,9] to decide number of persons:";
			k = type_int(2, 9);
			if (opt2 == 1){
				freopen("output.txt","w", stdout);	
			}
			process();
		} else if (ch == 3){
			cout << "There are 100 different maze for your to select. Please type one interger: ";
			int opt3= type_int(1, 100);
			freopen("maze.txt", "r", stdin);
			int tot, id;
			cin >> tot;
			for (int i = 1; i <= opt3 - 1; i++){
				cin >> id >> n >> m >> k;
				string st;
				for (int j = 1; j <= n; j++)
				  cin >> st;
			}
			
			cin >> id >> n >> m >> k;
			for (int i = 0; i < n; i++){
				string st;
				cin >> st;
				for (int j = 0; j < m; j++)
					mapBuffer[i][j] = st[j];
			}
			
			
			if_create_map = false;
			process();
		}
	} 
	
    //打印走过路后的地图
    system("pause");
    return 0;
}