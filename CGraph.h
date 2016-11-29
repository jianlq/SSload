#ifndef CGRAPH_H
#define CGRAPH_H
#include"Common.h"

class CVertex{
public:
	double d;
	int p;
	int ID;
	CVertex(){d = INF; p = NULL;}
	CVertex(int i){ID=i;d = INF; p = NULL;}
	~CVertex(){;}
};
bool pVertexComp ( CVertex* x, CVertex* y )
{    if ( x->d < y->d ) return 1;
return 0;
};
class Status{
public:
	int ID;
	double d;
	list<int> passby;
	Status* parent;

	Status(){ID = NULL; d = INF ; };
	Status(int id, double w){ID = id; d = w ; passby.clear();};
	Status(int id, double w, Status* p){ID = id; d = w ; parent = p; passby.clear();};
	Status(int id, double w, Status* p, list<int> listVID){ID = id; d = w ; parent = p; passby = listVID;};

	~Status(){;}
};
bool pStatusComp ( Status* x, Status* y )
{    if ( x->d < y->d ) return 1;
return 0;
};

class CEdge{
public:
	int  id, tail, head;
	double use, latency, load, capacity;
	double dist;
	CEdge(){ ;}
	CEdge(int i, int a, int b, double dist,double cap){
		id = i;
		tail = a;
		head = b;
		dist = dist;
		capacity = cap;
		load = 0; 
		latency = 0.001;
		use  = 0;
	}
	double getWeight(){
		return dist;
	}
	int getHead(){
		return head;
	}
	int getTail(){
		return tail;
	}
	~CEdge(){;}

};

class SSdemand{
public:
	int user;
	double flow;
	SSdemand(){;}
	SSdemand(int u,double f){
		user = u;
		flow = f;
	}
	~SSdemand(){;}
};

class demand{
public:
	int org;
	int des;
	double flow;
	demand(){;}
	demand(int a,int b ,double c)
	{
		org=a;
		des=b;
		flow=c;
	}
	~demand(){;}
};

class CPath{
public:
	double length;
	vector<CEdge*> listEdge;
	list<CVertex*> listVertex;

	CPath(){length = 0; listEdge.clear(); listVertex.clear();};
	CPath(Status *beginning, Status *ending, map<int, CVertex*> mapVID_Vertex, map<int, vector<CEdge*> > mapVID_listEdge);
	~CPath(){;};
};
bool pPathComp ( CPath* x, CPath* y )
{    if ( x->length < y->length ) return 1;
return 0;
};
CPath::CPath(Status *beginning, Status *ending, map<int, CVertex*> mapVID_Vertex,map<int, vector<CEdge*> > mapVID_listEdge)
{
	length = ending->d;
	Status *status = ending;
	while(1)
	{
		int id = status->ID;
		//cout<<id<<"  ";
		listVertex.push_front(mapVID_Vertex[id]);
		if(status == beginning)
			break;
		status = status->parent;
		vector<CEdge*>::iterator it;
		for(it = mapVID_listEdge[status->ID].begin(); it != mapVID_listEdge[status->ID].end(); it++)
		{
			if((**it).getHead() == id)
				listEdge.push_back(*it);
		}
	}
}

class CGraph{
private:
	void dfs(int cur){
			visit[cur] = 1;
			for(unsigned int i = 0; i < adjL[cur].size(); i++)
				if(!visit[adjL[cur][i]->head])
					dfs(adjL[cur][i]->head);
		}
public:
	int n, m;
	vector<CEdge*> Link;
	vector<int> ver;	// 所有的顶点
	vector<vector<CEdge*> > adjL, adjRL; //出、入度边

	//KSP
	map<int, CVertex*> mapVID_Vertex;	
	map<int, vector<CEdge*> > mapVID_listEdge; // 记录与顶点关联的出度边	
	list<Status*> listTemp;///记录临时状态指针	
	list<Status*> listSolu;///记录被标记为解的状态	
	vector<CPath*> listPath; ///记录path集合

	//遗传
	vector<int> reqPathNo; //所有req走的路径编号
	//vector<vector<CPath*> > reqlistPath;//所有req的路径集合
	bool GAinit(vector<demand>& req);//初始化每个req的路径
	void myDFS(int s,int t);
	int DFSflag;
	vector<vector<vector<CEdge*>>> reqlistPath;
	vector<int> pathver;

	//纳什
	double TEcplex(vector<demand> &eq,CGraph *GOR,bool needcal);
	double TESS(vector<demand> &eq,CGraph *GOR,bool needcal);
	double SScplex(vector<SSdemand> &eq); 
	double rst(vector<demand>& bg);
	vector<vector<float> > xdl;
	double ORcplex(vector<demand> *eq); 
	vector<demand> eqfo;

	// dijkstra
	vector<vector<int> > reqPathID;
	double dijkstra(int id,int s, int t, double dm ,bool needpath,bool hop);
	double dijkstraOR(int s, int t,double dm);
	double LoadScaleFactor(vector<demand> &req);

	//dfs
	vector<bool> visit;

	//独裁
	double mlu;
	double delay;

public:
	CGraph(char* inputFile);
	void KSP(int s, int t, unsigned int k);

	void clearOcc(){
		for(int i = 0; i < m; i++){
			Link[i]->load = 0; 
			Link[i]->use = 0;
			Link[i]->latency = 0.001;
		}
	}
	void SetUNVISITED(){
		   for(int i = 0;i < n;++i){
			   visit[i] = false;
		   }
		   DFSflag = 0;
		   pathver.clear();
	   }

	int canNotReach(int s, int t){
		visit.resize(n, 0);
		dfs(s);
		return !visit[t];
	}

	~CGraph(){
		for(unsigned int i = 0; i < Link.size(); i++)
			if(!Link[i])
				delete Link[i];
	}
};


CGraph::CGraph(char* inputFile)
{
	ifstream file(inputFile);
	file >> n >> m;
	adjL.resize(n); 
	adjRL.resize(n); 
	reqPathID.resize(200);
	set<int> vert;
	int a, b; double cap;double dist;
	for (int i = 0; i < m; i++)
	{
		file >> a >> b >> dist >> cap;
		vert.insert(a);
		vert.insert(b);
		CEdge *e=new CEdge(i,a,b,dist,cap+MINCAPACITY);
		Link.push_back(e);
		adjL[a].push_back(e); //////////////出度边
		adjRL[b].push_back(e); //////////// 入度边
	}
	file.close();	
	vector<CEdge*> emptylist;
	vector<CEdge*>::iterator it;
	for(it=Link.begin(); it!=Link.end(); it++)
	{
		mapVID_Vertex.insert(pair<int,CVertex*>((**it).getTail(),new CVertex((**it).getTail())));
		mapVID_Vertex.insert(pair<int,CVertex*>((**it).getHead(),new CVertex((**it).getHead())));
		mapVID_listEdge.insert(pair<int,vector<CEdge*>>((**it).getTail(),emptylist));
		mapVID_listEdge.insert(pair<int,vector<CEdge*>>((**it).getHead(),emptylist));
		mapVID_listEdge[(**it).getTail()].push_back(*it);
	}
	set<int>::iterator i;
	for(i=vert.begin();i!=vert.end();i++){ 
		ver.push_back(*i);
	}

}

double CGraph::dijkstra(int id,int s, int t, double dm ,bool needpath,bool hop){
	vector<int> p, flag;
	vector<double> d;//带宽利用率
	for(int i = 0; i < n; i++){
		p.push_back(-1);
		flag.push_back(0);
		d.push_back(INF);
	}
	d[s] = 0;
	int cur = s;
	do{
		flag[cur] = 1;
		for(unsigned int i = 0; i < adjL[cur].size(); i++){
			CEdge *e = adjL[cur][i];
			if(hop){  //以跳数来计算最短路
				if(e->capacity - e->use >= dm && d[e->head] > d[e->tail] + 1){
					d[e->head] = d[e->tail] + 1;
					p[e->head] = e->id;
				}
			}
			else{
				double util = ( dm + e->use )/e->capacity; // 带宽利用率最小
				double tail_util = max(util,d[e->tail]);
				if(e->capacity - e->use >= dm && d[e->head] > tail_util ){
					d[e->head] = tail_util;
					p[e->head] = e->id;
				}
			}

		}
		cur = -1;
		for(int i = 0; i < n; i++)
			if(!flag[i] && (cur == -1 || d[cur] > d[i] ))
				cur = i;
	}while(cur != -1);

	cur = t;
	do{
		if(p[cur] == -1)
			break;
		Link[p[cur]]->use += dm;
		cur = Link[p[cur]]->tail;
	}while(cur != s);

	reqPathID[id].clear();
	if(needpath){
		cur = t;
		do{
			if(p[cur] == -1)
				break;
			this->reqPathID[id].push_back(p[cur]);
			cur = Link[p[cur]]->tail;
		}while(cur != s);
		reverse(reqPathID[id].begin(),reqPathID[id].end());
	}

	return d[t];
}

double CGraph::dijkstraOR(int s, int t,double dm){
	vector<int> p, flag;
	vector<double> d; //delay
	for(int i = 0; i < n; i++){ 
		p.push_back(-1);
		flag.push_back(0);
		d.push_back(INF);
	}
	d[s] = 0;
	int cur = s;
	do{
		flag[cur] = 1;
		for(unsigned int i = 0; i < adjL[cur].size(); i++){
			CEdge *e = adjL[cur][i];				
			if( d[e->head] > d[e->tail] + e->latency ){
				d[e->head] = d[e->tail] + e->latency;
				p[e->head] = e->id;
			}		
		}
		cur = -1;
		for(unsigned int i = 0; i < this->ver.size(); i++)
			if(!flag[ver[i]] && (cur == -1 || d[cur] > d[ver[i]] ))
				cur = ver[i];
	}while(cur != -1);

	cur = t;
	do{
		if(p[cur] == -1)
			break;
		Link[p[cur]]->use += dm;
		cur = Link[p[cur]]->tail;
	}while(cur != s);

	return d[t];
}


void genGraph(int n, int m, char route[]){ //也会产生重复的边
	FILE *out = fopen(route, "w");
	fprintf(out,"%d %d\n",n,m*2);
	for(int i = 1; i < min(n, m+1); i++){
		int t = rand()%i, w = rand()%MAXWEIGHT+1;
		fprintf(out, "%d %d %d %d\n", i, t, w,rand()%MAXCAPACITY);
		fprintf(out, "%d %d %d %d\n", t, i, w,rand()%MAXCAPACITY);
	}
	for(int i = 0; i < m-n+1; i++){
		int s = rand()%n, t = rand()%n, w = rand()%MAXWEIGHT+1;
		while(t == s)
			t = rand()%n;
		fprintf(out, "%d %d %d %d\n", s, t, w,rand()%MAXCAPACITY);
		fprintf(out, "%d %d %d %d\n", t, s, w,rand()%MAXCAPACITY);
	}
	fclose(out);
}

void genSSGraph(int total,int ns, int nu,char route[]){ //会产生重复的边
	FILE *out=fopen(route,"w");
	fprintf(out,"%d %d\n",total,ns*nu);
	int* a = new int[total];//标记数组
	for(int i = 0; i < total;i++)
		a[i] = 0;
	//产生server集合
	set<int> Sset; 
	int i = 0;
	while(i<ns){
		int s = rand()%total;
		while(a[s]){
			s = rand()%total;
		}		
		Sset.insert(s);
		a[s] = 1;
		i++;
	}
	//cout<<ns << "   "<<Sset.size()<<endl;
	Sver.clear();
	for(set<int>::iterator i1 = Sset.begin(); i1 != Sset.end(); i1++ ){
		Sver.push_back((*i1));
	}
	//产生user集合
	set<int> Uset; 
	i = 0;
	while(i < nu )
	{
		int u = rand()%total;
		while(a[u]){
			u = rand()%total;
		}		
		Uset.insert(u);
		a[u] = 1;
		i++;
	}
	//cout<< nu << "   "<< Uset.size()<<endl;
	Uver.clear();
	for(set<int>::iterator i1 = Uset.begin(); i1 != Uset.end(); i1++ ){
		Uver.push_back((*i1));
	}

	set<int>::iterator i1 ,end1 = Sset.end();
	set<int>::iterator i2 ,end2 = Uset.end();
	for(i1 = Sset.begin(); i1 != end1; i1++ ){
		for(i2 = Uset.begin(); i2 != end2; i2++ ){
			int c = rand()%MAXCAPACITY+2, w = rand()%MAXWEIGHT+1;
			fprintf(out, "%d %d %d %d\n", (*i1),(*i2),w,c);
			// fprintf(out, "%d %d %d %d\n", (*i2),(*i1),w,c);
		}
	}
	fclose(out);
}

double CGraph::LoadScaleFactor(vector<demand> &req){
	int num = req.size();
	this->clearOcc(); 
	for(int d = 0 ;d < num; ++d){
		this->dijkstra(d,req[d].org,req[d].des,req[d].flow,0,1);
	}
	double util = 0;
	for(int i = 0; i < this->m; i++){
		double xc = this->Link[i]->use/this->Link[i]->capacity;
		if(util < xc )
			util = xc;
	}
	return util;
}

#endif