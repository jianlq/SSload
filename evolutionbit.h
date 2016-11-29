#ifndef EVOLUTIONBIT_H
#define EVOLUTIONBIT_H
#include"KSP.h"

// 个体 individual
class evoluDivbit{
private:
	static const int MUT = 6; //变异位数
	static const int HER = 15; //学习位数
	double consider;
	CGraph *G;
	CGraph *GSS;
	vector<demand> *dem; 
	vector<demand> *demor; 
public:
	vector<vector<int>> x;//解  各个req经过的路径编号 二进制编码 req*4
	double ability; //个体能力
	double delay, mlu;//个体能力值的体现
	double delaybase;
	// 构造函数
	evoluDivbit() {;}
	double GAability();
	double GAabilityNoCplex();
	void Caldelay();
	void Init(){
		ability = MIN;
		delay = INF;
		mlu = INF;
	}
	//m为req的数量
	evoluDivbit(int m, CGraph *g, CGraph *gor,vector<demand> *d,vector<demand> *dor,double delaybest, double con){
		Init();
		x.resize(m);
		G = g;
		GSS=gor;
		dem = d;
		demor = dor;
		delaybase=delaybest;
		consider = con;
		randNature(); 
	}

	evoluDivbit(vector<vector<int> > &tx, CGraph *g,CGraph *gor, vector<demand> *d, vector<demand> *dor,double delaybest,double con){
		Init();
		x.clear();
		G = g;
		GSS=gor;
		dem = d;
		demor=dor;
		delaybase=delaybest;
		consider = con;
		for(unsigned int i = 0; i < tx.size(); i++)
			x.push_back(tx[i]);
		// 也可以用  x = tx ; 代替上面的操作
	}

	/////////////交配杂交  crossover
	evoluDivbit crossover(evoluDivbit other){
		vector<vector<int> > nx;	
		/*
		//分成两截互换 在num处截断  即是单点杂交
		int n=dem->size();
		int num=rand()%(n/2);
		for(int i=0;i<num;i++) 
		nx.push_back(x[i]);
		for(unsigned int ij=num;ij<x.size();ij++)
		nx.push_back(other.x[ij]);	
		return evoluDivbit(nx, G,GOR, dem,demor);	
		*/			
		vector<vector<int> > onezero;   //多点交叉  随机产生一串0-1序列  为0不交叉  为1交叉
		for(unsigned int i=0;i<x.size();i++){
			vector<int> bit;
			for(int j=0;j<4;j++)  
				bit.push_back(rand()%2);
			onezero.push_back(bit); 
		}
		for(unsigned int n=0;n<x.size();n++)
			nx.push_back(x[n]);
		for(unsigned int i=0;i<onezero.size();i++)
			for(int j=0;j<4;j++){  
				if(onezero[i][j]==1)
					nx[i][j]=other.x[i][j];
			}
			return evoluDivbit(nx, G,GSS, dem,demor,other.delaybase,other.consider);	
	}

	//将二进制转为十进制  路径编号的具体值
	int Decoding(vector<int> &bit){
		int z=0;
		z = bit[0]*1+bit[1]*2+bit[2]*4+bit[2]*8;
		return z;
	}

	//计算适应值
	void calAbility(){
		G->reqPathNo.clear(); //清空上一个个体解的路径编号
		for(unsigned int i=0;i<x.size();i++) 
			G->reqPathNo.push_back(Decoding(x[i])); 
		ability = GAabilityNoCplex();	
	}

	void randNature(){
		for(unsigned int i = 0; i < x.size(); i++){
			vector<int> bit;
			for(int b=0;b<4;b++)        
				bit.push_back(rand()%2);// 4位编码 随机产生0或1  表示路径编号
			x[i] = bit; 
		}
	}

	void mutation(){	//变异
		int i=0;
		while(i<MUT){
			int row= rand()% dem->size();//产生需要变异的req路径编号
			int col=rand()%4;//产生需要变异的位置  
			if(x[row][col]==0) x[row][col]=1;
			else if(x[row][col]==1) x[row][col]=0; 
			i++;
		}
	}

	/////////学习历史最优解
	void culture(evoluDivbit hero){
		int n=0;
		while(n<HER){
			int j=rand()%x.size();
			for(unsigned int i = 0; i < x.size(); i++){
				if(i==j) x[i] =  hero.x[i];
			}
			n++;
		}	
	}

};

bool Cmp2(evoluDivbit a, evoluDivbit b){
	return a.ability > b.ability; //ability和适应度成反比，ability越小（最小化问题），适应度越大
}

//种群 种群由个体组成
class evoluPopubit{
private:
	static const int YEAR = 70;
	static const int YEARCUL = 50;
	static const int NOHERO = 30;
	double pm;
	vector<evoluDivbit> popu;
	FILE *herofile;
public:
	evoluPopubit(){;}
	evoluDivbit hero;
	// n 种群大小  m:req数目 初始化种群
	evoluPopubit(int n, int m, CGraph *g, CGraph *gor,vector<demand> *d,vector<demand> *dor,double delaybest,double con){
		popu.clear();
		pm = 0.20;
		for(int i = 0; i < n; i++){
			evoluDivbit divi(m, g, gor,d,dor,delaybest,con);
			popu.push_back(divi);
		}
		hero = evoluDivbit(m, g,gor, d,dor,delaybest,con);
		herofile = fopen("outputFile//hero.txt","a");
	}
	///////////  轮盘赌和
	int wheelchose(double sum){
		double m=0;
		double r=rand()%1000*0.001;
		int res = rand()%popu.size();
		for(unsigned int i=0;i<popu.size();i++){
			m += popu[i].ability/sum;
			if(r<=m) {
				return i;
			}
		}
		return res; //如果for循环中没有返回   // 解决 ：“evoluPopubit::wheelchose”: 不是所有的控件路径都返回
	}

	/////////// 种群进化 //////////////////
	evoluDivbit evolution(){
		int nohero=0;//记录进化情况
		fprintf(herofile,"Start:\n ");
		vector<int> h0;
		for(int b=0;b<4;b++)  
			h0.push_back(0);
		for(unsigned int i = 0; i < hero.x.size(); i++)
			hero.x[i] = h0;
		hero.calAbility();
		if(hero.ability>MIN) 
			fprintf(herofile, "%f\t%f\t%f\n", hero.mlu,hero.delay,hero.ability);
		for(unsigned int i = 0; i < popu.size(); i++)
			popu[i].calAbility();
		sort(popu.begin(), popu.end(), Cmp2);
		//繁殖的代数
		for(int curYear = 1; curYear <= YEAR; curYear++){
			//轮盘赌和选择n个个体
			int n = popu.size(), getMore = 0;
			double sum=0;
			for( int i=0;i<n;i++)
				sum += popu[i].ability;
			vector<evoluDivbit> chosepopu;
			chosepopu.clear();
			for(int i=0;i<n;i++) {
				int ch=wheelchose(sum);
				chosepopu.push_back(popu[ch]);
			}
			popu.clear();
			for(unsigned int i=0;i<chosepopu.size();i++)
				popu.push_back(chosepopu[i]);
			vector<evoluDivbit> sons;	
			for(unsigned int i = 0; i+1 < popu.size(); i+=2){ //杂交
				sons.push_back(popu[i].crossover(popu[i+1]));
				sons.push_back(popu[i+1].crossover(popu[i]));
				sons.push_back(popu[i].crossover(popu[i+1]));
			}
			// sons.push_back(hero);  //精英保留策略
			int m = sons.size();
			for(int i = 0; i < m; i++){
				double p=rand()%100*0.01;
				if(p<pm) 
					sons[i].mutation();//个体变异以概率pm
				if(curYear > YEARCUL)
					sons[i].culture(hero); //向hero靠近
				sons[i].calAbility();
			}
			sort(sons.begin(), sons.end(), Cmp2);
			popu.clear(); //清空上一代种群
			for(int i = 0; i < n; i++){
				popu.push_back(sons[i]); 
				if(abs(sons[i].ability - hero.ability) < 1e-4 ){	//判断相等
					continue;
				}
				else if(sons[i].ability > hero.ability){
					hero = sons[i]; //选出能力最好的作为hero
					getMore = 1;
				}
			}
			if(getMore){
				//fprintf(herofile, "Year %d: find hero \n", curYear);
				fprintf(herofile,"%f\t%f\t%f\n", hero.mlu,hero.delay,hero.ability);
			}
			else 
				nohero++;
			if(nohero> NOHERO){
				break;
			}
		}
		fprintf(herofile,"end\n\n\n\n");
		fclose(herofile);
		return hero;
	}

};

double evoluDivbit::GAabilityNoCplex(){ 
	this->G->clearOcc(); //清空每条边上负载 
	for(unsigned int d=0;d<G->reqPathNo.size();d++){
		int num=G->reqPathNo[d]; 
		//vector<CEdge*> lsEg=G->reqlistPath[d][num]->listEdge;
		vector<CEdge*> lsEg=G->reqlistPath[d][num];
		vector<CEdge*>::iterator it,end=lsEg.end();
		for(it=lsEg.begin();it!=end;it++){
			(*it)->use += (*dem)[d].flow;
		}
	}
	this->GSS->clearOcc();
	Caldelay();

	/*for(int i = 0;i<GSS->m;i++)
		cout<<GSS->Link[i]->latency<<"  ";
	cout <<endl;*/

	double del = 0;
	for (unsigned int d = 0; d < demor->size(); d++){
		//cout << (*demor)[d].flow<<"  "<<GSS->dijkstraOR((*demor)[d].org, (*demor)[d].des, (*demor)[d].flow)<<"  "<<del<<endl;
		del += GSS->dijkstraOR((*demor)[d].org, (*demor)[d].des, (*demor)[d].flow)*(*demor)[d].flow;
	}
	//exit(1);

	// 将GSS->Link->use 加到reqTE中得到新的流量矩阵计算TE的目标
	vector<demand> req;
	for (int i = 0; i < GSS->m; i++){
		if(GSS->Link[i]->use > 0 )
			req.push_back(demand(GSS->Link[i]->tail, GSS->Link[i]->head, GSS->Link[i]->use));
	}
	for (unsigned int i = demor->size(); i < dem->size(); i++){
		req.push_back((*dem)[i]);
	}

	// 计算TE的目标值
	G->clearOcc(); //use
	double result = 0;
	int block = 0, success = 0;
	int n = req.size();
	for (int i = 0; i < n; i++){
		double ret = 0;
		if(req[i].flow>0) 
			ret = G->dijkstra(i,req[i].org, req[i].des, req[i].flow,1,0);		
		if ((INF - ret) < 1e-4)  // ret == INF
			block++;
		else{
			success++;
			if (result < ret)
				result = ret;
		}
	}
	double ab = MIN;
	if (success >= n){
		this->mlu = result;
		this->delay = del;
		ab = 1.0 / result + this->consider*this->delaybase /del;
	}
	return ab;
}


void evoluDivbit::Caldelay(){
	double mlu = 0;
	for(int ij=0;ij<G->m;ij++){
		if(G->Link[ij]->use/G->Link[ij]->capacity > mlu)
			mlu = G->Link[ij]->use/G->Link[ij]->capacity;
		if(G->Link[ij]->use>=G->Link[ij]->capacity) 
			G->Link[ij]->latency = 1.0;
		else 
			G->Link[ij]->latency=1.0/(G->Link[ij]->capacity-G->Link[ij]->use);  //传参为单位延迟  
	}
	//cout << " mlu "<<mlu<<endl;
	int totalnum = (*dem).size();
	for(int m=0;m<GSS->m;m++){	
		bool flag=false;
		double dmin=0;
		for(int d=0;d<totalnum;d++){
			if (GSS->Link[m]->tail == (*dem)[d].org && GSS->Link[m]->head == (*dem)[d].des ){
				if(flag==false){
					flag=true;
					//vector<CEdge*> lg=G->reqlistPath[d][G->reqPathNo[d]]->listEdge; 
					vector<CEdge*> lg=G->reqlistPath[d][G->reqPathNo[d]];
					for(vector<CEdge*>::iterator lgi=lg.begin();lgi!=lg.end();lgi++){ 		
						dmin += (*lgi)->latency;       
					}
				}
			}
			if(flag==true){
				GSS->Link[m]->latency = dmin;	
				break;
			}
		} 		
	} 
}


//////////////////************** 评价函数 类（evoluDiv)个体的成员函数*************************////////////////////////
////知道每个req走哪条路之后（reqPathNo)  计算te的反应（反应完之后的mlu），反应完之后OR的目标值delay
/*
double evoluDivbit::GAability(){ 
	//计算每条边上的负载
	this->G->clearOcc(); //清空每条边上负载 
	for(unsigned int d=0;d<G->reqPathNo.size();d++)
	{
		int num=G->reqPathNo[d]; 
		vector<CEdge*> lsEg=G->reqlistPath[d][num]->listEdge;
		vector<CEdge*>::iterator it,end=lsEg.end();
		for(it=lsEg.begin();it!=end;it++){
			(*it)->load += (*dem)[d].flow;
		}
	}

	GOR->clearOcc();
	if(!CONSTANT){  //非常数  //计算Overlay的边上延迟
		for(int m=0;m<GOR->m;m++){	
			bool flag=false;
			double dmin=0;//延迟
			for(unsigned int d=0;d<(*dem).size();d++){
				if(GOR->Link[m]->tail==(*dem)[d].org && GOR->Link[m]->head==(*dem)[d].des){ //OR边在需求中
					if(flag==false){
						flag=true;
						//则看这个req经过哪条路径   把路径上的延迟加起来
						vector<CEdge*> lg=G->reqlistPath[d][G->reqPathNo[d]]->listEdge; 
						for(vector<CEdge*>::iterator lgi=lg.begin();lgi!=lg.end();lgi++){ 		
							if((*lgi)->load>=(*lgi)->capacity) dmin += 1.0;				
							else dmin += (double)1.0/(double)((*lgi)->capacity-(*lgi)->load);//!!!!!!!!!!double	          
						}
					}
				}
				if(flag==true){    
					GOR->Link[m]->latency=dmin;
					break;
				}
			}
		}	
	}
	else{  // CONSTANT
		for(int m=0;m<GOR->m;m++){	
			bool flag=false;
			double dmin=0;//延迟
			for(unsigned int d=0;d<(*dem).size();d++){
				if(GOR->Link[m]->tail==(*dem)[d].org && GOR->Link[m]->head==(*dem)[d].des){ //OR边在需求中
					if(flag==false){
						flag=true;
						//则看这个req经过哪条路径   把路径上的延迟加起来
						vector<CEdge*> lg=G->reqlistPath[d][G->reqPathNo[d]]->listEdge; 
						for(vector<CEdge*>::iterator lgi=lg.begin();lgi!=lg.end();lgi++){ 
							if((*lgi)->load>=(*lgi)->capacity) dmin += 5.0;
							else dmin += (*lgi)->dist;		          
						}
					}
				}
				if(flag==true){    
					GOR->Link[m]->latency=dmin;
					break;
				}
			}
		}
	}

	double orobj=GOR->ORcplex(demor); 

	//OR决策之后TE的决策  eqfo  
	vector<demand> req2;
	int num1=GOR->eqfo.size();
	int num2=(*dem).size();
	for(int d=0;d<num1;++d) 
		req2.push_back(GOR->eqfo[d]); 
	int k = (*demor).size();
	for(int d = k; d < num2; ++d)
		req2.push_back((*dem)[d]);
	//this->G->clearOcc(); //清空每条边上负载 
	//for(unsigned int d=0;d<req2.size();++d){   // overlay需求装在前面
	//	int num=G->reqPathNo[d]; //cout<<num<<" ";
	//	vector<CEdge*> lsEg=G->reqlistPath[d][num]->listEdge;
	//	vector<CEdge*>::iterator it,end=lsEg.end();
	//	for(it=lsEg.begin();it!=end;++it){
	//		(*it)->load += req2[d].flow;
	//	}
	//}
	//double teobj=0; //如果下层路由不变 则TE的目标值
	//for(int i=0;i<G->m;i++){
	//	if(teobj<G->Link[i]->load/G->Link[i]->capacity)
	//		teobj=G->Link[i]->load/G->Link[i]->capacity;
	//}

	double teobj2 = G->TESS(req2,GOR,0);
	if(teobj2>=INF){ 
		cout<<" return te2  "<<endl;return MIN;
	}

	double obj = MIN;
	this->mlu = result;
	this->delay = del;
	obj = 1.0 / result + this->consider*this->delaybase /del;

	return obj;
}
*/

//在网络中以带宽利用率最小来进行选路  得到的带宽利用率和延迟  OR在Overlay层未进行选路


pair<double,double> Calability(CGraph *G,CGraph *GSS,vector<demand> &req){
	pair<double,double> res(INF,INF);
	G->clearOcc();
	double mlu = 0;
	int block = 0,success = 0;
	for(unsigned int i = 0; i < req.size(); i++){
		double ret = G->dijkstra(i,req[i].org, req[i].des, req[i].flow,1,0); //以带宽利用率最小来进行选路
		if(ret >= INF)
			block++;
		else{
			success++;
			if(mlu<ret)
				mlu = ret;
		}
	}
	double teobj = INF,orobj = INF,ilp = 0;
	double ability = 0.1;
	if(success == req.size()){
		teobj = mlu;
		///// calculate delay
		for(int ij=0;ij<G->m;ij++){
			if(G->Link[ij]->use>=G->Link[ij]->capacity) 
				G->Link[ij]->latency = 1.0;
			else 
				G->Link[ij]->latency=1.0/(G->Link[ij]->capacity-G->Link[ij]->use);  //传参数为单位延迟  
		}
		int totalnum = req.size();
		for(int m = 0; m < GSS->m; m++){	
			bool flag=false;
			double dmin=0;
			for(int d = 0; d < totalnum; d++){
				if(GSS->Link[m]->tail==req[d].org && GSS->Link[m]->head==req[d].des ){ 
					if(flag==false){
						flag=true;
						for(unsigned int ij=0;ij<G->reqPathID[d].size();ij++){	
							dmin += G->Link[G->reqPathID[d][ij]]->latency;
						}
					}
				}
				if(flag==true){
					GSS->Link[m]->latency=dmin;
					break;
				}
			} 		
		} 

		vector<demand> reqor;
		for(int i = 0;i < GSS->m ; ++i)
			reqor.push_back(req[i]);
		//orobj = GOR->ORcplex(&reqor); 
		double del = 0;
		for(unsigned int d = 0; d < reqor.size(); d++){
			del += GSS->dijkstraOR(reqor[d].org,reqor[d].des,reqor[d].flow)*reqor[d].flow; //以延迟为权重的dijkstra
		}
		orobj = del ;
	}  // end of if
	res.first = teobj;
	res.second = orobj;
	return res;
}


#endif
