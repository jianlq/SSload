#ifndef KSP_H
#define KSP_H
#include"CGraph.h"

void CGraph::KSP(int s, int t, unsigned int k)
{
	listSolu.clear();
	listTemp.clear();
	Status *beginning = new Status(s,0);
	beginning->passby.push_back(s);
	listTemp.push_back(beginning);
	cout<<" ksp while 111        ";
	while(listSolu.size() < k && listTemp.size() != 0)
	{
		listTemp.sort(pStatusComp);///在更新序列里进行排序
		int id = (**listTemp.begin()).ID;///拿出节点序列的第一个点的ID
		if(id == t)
			listSolu.push_back(*listTemp.begin());///如果这个点就是终点，那么把这个状态丢到解集合里头去。
		double d = (**listTemp.begin()).d;///拿出存储的d值
		list<int> passby = (**listTemp.begin()).passby;
		vector<CEdge*>::iterator it;
		//cout << " ksp for "<<endl; //!!!!! 死循环  11-3
		for(it = mapVID_listEdge[id].begin(); it != mapVID_listEdge[id].end(); it++)///遍历该点的出度边
		{
		
			int head = (**it).getHead();///拿出头结点ID
			if(find(passby.begin(),passby.end(),head) == passby.end())
			{
				double weight = (**it).getWeight() + d;///拿出该边的重量和d相加得到路径长度
				list<int> templist = passby;
				templist.push_back(head);
				Status *newstatus = new Status(head,weight,*listTemp.begin(),templist);///形成某个状态
				listTemp.push_back(newstatus);///把状态丢进更新序列里
			}
		}
		listTemp.pop_front();
	}
	cout<<" ksp while 222 "<<endl;
	listPath.clear(); ////////每个OD对之间算的路径集合  必须清空
	list<Status*>::iterator Sit;
	for(Sit = listSolu.begin(); Sit != listSolu.end(); Sit++)
	{
		listPath.push_back(new CPath(beginning, *Sit, mapVID_Vertex, mapVID_listEdge));
		////输出
		/*Status* it = *Sit;
		cout<<"d="<<it->d<<endl<<"回溯经过点：";		
		while(1){
			cout<<it->ID<<"  ";
			it = it->parent;
			if(it == beginning){
				cout<<it->ID<<endl;
				break;
			}
		}*/
	}
}

////  计算每个 OD对之间的路径   初始化路径数目等参数
bool CGraph::GAinit(vector<demand> &req){
	////KSP
	/*
	reqlistPath.clear();
	for(unsigned int i=0;i<req.size();i++)
	{  	cout<< "req "<<req[i].org<<"   "<<req[i].des <<"   ";
		KSP(req[i].org,req[i].des,K); //计算得到一个OD对的Path list
		reqlistPath.push_back(listPath); //将listPath存到reqlistPath对应位置   vector<vector<CPath*> > 
		if(listPath.size()<K)
			return false;
	}
	return true;
	*/

	////DFS
	reqlistPath.clear();
	for(unsigned int i=0;i<req.size();i++){
		int j = 0;
		vector<vector<CEdge*>> reqpath;
		while(j < K){
			SetUNVISITED();
			myDFS(req[i].org,req[i].des);
			if(DFSflag ==1 ){
				j++;
				vector<CEdge*> tmp;
				for(int p = 0;p <pathver.size();p += 2){
					int s = pathver[p],t = pathver[p+1];
					for(unsigned int  a = 0; a <Link.size(); a++){
						if(Link[a]->tail == s && Link[a]->head == t ){
							tmp.push_back(Link[a]);
						    break;
						}
					}
				}
				reqpath.push_back(tmp);
			}
		}
		reqlistPath.push_back(reqpath);
	}

	return true;

	//// 输出测试
	// vector<vector<CPath*>>::iterator it;
	// for(it = reqlistPath.begin();it != reqlistPath.end();it++){
	//  for(int k = 0; k < 16; k++){
	//  vector<CEdge*>::iterator i;
	//  cout << k << " : ";
	//  reverse((*it)[k]->listEdge.begin(),(*it)[k]->listEdge.end());
	//  for( i = (*it)[k]->listEdge.begin(); i!= (*it)[k]->listEdge.end();i++){
	//	     cout << (*i)->tail << "  " << (*i)->head <<"  ";
	//  }
	//  cout<<endl;
	// }
	// }
	// exit(0);

}


void CGraph::myDFS(int s,int t){
		if (!visit[s]){
		   pathver.push_back(s);
		   visit[s] = true;
		}
		if( s == t ){   // 找到了终点
			DFSflag = 1;
			return ; 
		}
		vector<int> unvit;
		int ss ,ret=0; // ret表示回溯点
		for(unsigned  int i =0;i<adjL[s].size();i++){
			if(visit[adjL[s][i]->head])
				continue;	
			else
				unvit.push_back(adjL[s][i]->head); //没有访问的顶点编号		
		}		
		if(unvit.size()== 0) {  // 没有未被访问的邻节点
			if(pathver.size()>0)
				pathver.pop_back();
			if(pathver.size()>0) { 
				ss = pathver[pathver.size()-1];
				ret = 1;  //表示有回溯点
			}
		}
		else{
			int j = rand() % unvit.size();
			ss = unvit[j]; 
			ret = 1;  //表示有继续深入点
		}
		if(ret)
			myDFS(ss,t);
		else
			return;
 }

#endif