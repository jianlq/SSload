#ifndef SERVER_H
#define SERVER_H
#include "CGraph.h"
#include <ilcplex/ilocplex.h>

/////////////////////////////Nash SS
double CGraph::SScplex(vector<SSdemand> &eq){
    IloEnv env;
    IloModel model(env);
    IloCplex SSsolver(model);

	int num=eq.size();
    IloArray<IloNumVarArray> x(env, num); //IloNumVarArray 可以，进行分流，决策后为小数
    for(int d = 0; d < num; d++)
		x[d] = IloNumVarArray(env, this->m, 0, 1); 

    //优化目标  最小化延迟之和  
    IloExpr delay(env);
	for(int ij = 0; ij  < this->m; ij++){		
		IloNumExpr ep1(env);
		for(int d = 0; d < num; d++){
			if(CONSTANT)
				ep1 += x[d][ij] * Link[ij]->dist;  //d之和	
			else
				ep1 += x[d][ij] * Link[ij]->latency*eq[d].flow;  //d*f之和		
		}	
			delay += ep1;		 		
	}
	model.add(IloMinimize(env, delay)); 

    //约束2  流量约束
	// 1)对宿点约束
    for(int d = 0; d < num; d++)
		for(unsigned int i = 0; i < Uver.size(); i++){   
			IloExpr constraint(env);
			for(unsigned int k = 0; k < adjL[Uver[i]].size(); k++) // 出度边
				constraint += x[d][adjL[Uver[i]][k]->id];
			for(unsigned int k = 0; k < adjRL[Uver[i]].size(); k++) // 入度边
				constraint -= x[d][adjRL[Uver[i]][k]->id];		

			if(Uver[i] == eq[d].user)  //用户节点为宿点
				model.add(constraint == -1);
		}

	SSsolver.setOut(env.getNullStream());
	double obj = INF;
	SSsolver.solve();
	if(SSsolver.getStatus() == IloAlgorithm::Infeasible)
		 env.out() << "SS No Solution" << endl;
	else{
		obj = SSsolver.getObjValue();
		/*for(int i=0;i<this->m;i++)
			cout<< this->Link[i]->tail<<"  "<<this->Link[i]->head<<"  "<<this->Link[i]->dist<<endl;
		cout<<endl;
		for(int d = 0; d < num; d++)
			cout<<eq[d].user<<"   ";
			cout<<endl;
			*/
		// 输出决策变量
		/*for(int i=0;i<this->m;i++){
			for(int d = 0; d < num; d++)	
				cout<<SSsolver.getValue(x[d][i])<<"  ";
		cout<<endl;
		}*/
		
		eqfo.clear();	 //*********传参数给TE*********//  
		for(int i=0;i<this->m;i++){
			double de = 0;
			for(int d=0;d < num;d++){	
					de += SSsolver.getValue(x[d][i])*eq[d].flow;		
			}					
			if(de>0)
			eqfo.push_back(demand(Link[i]->tail,Link[i]->head,de));	
		}	
   }
   env.end();
   return obj;
}

double CGraph::TESS(vector<demand> &eq,CGraph *GSS,bool needcal){
	IloEnv env; 
	IloModel model(env);
	IloCplex TESS(model);

	int totalnum=eq.size();
	IloArray<IloIntVarArray> x(env, totalnum);	
	for(int d=0; d < totalnum; d++)
		x[d] = IloIntVarArray(env, this->m, 0, 1); 

	//约束1  流量守恒约束  对每个点进行流量守恒约束
	for(int d = 0; d < totalnum; d++)
		for( int i = 0; i < this->n; i++){    
			IloExpr constraint(env);
			for(unsigned int k = 0; k <adjL[i].size(); k++) // 出度边
				constraint += x[d][adjL[i][k]->id];
			for(unsigned int k = 0; k <adjRL[i].size(); k++) // 入度边
				constraint -= x[d][adjRL[i][k]->id];	

			if(i == (eq)[d].org) 
				model.add(constraint == 1);
			else if(i == (eq)[d].des)
				model.add(constraint == -1);
			else
				model.add(constraint == 0);
		}
		//约束2 带宽约束
		for(int i = 0; i < this->m; i++){
			IloExpr constraint(env);
			for(int d = 0; d <  totalnum; d++)
				constraint += eq[d].flow*x[d][i];
			model.add(constraint<=Link[i]->capacity);  
		}
		//优化目标 最小化带宽利用率  
		IloNumVar z(env,0,1);
		for(int i = 0; i < this->m; i++){
			IloExpr load(env);
			for(int d = 0; d < totalnum; d++)
				load += eq[d].flow*x[d][i];
			model.add(load<=z*Link[i]->capacity);	
		}
		model.add(IloMinimize(env,z));

		TESS.setOut(env.getNullStream());
		double obj = INF;
		TESS.solve();
		if(TESS.getStatus() == IloAlgorithm::Infeasible)
			env.out() << "TE No Solution" << endl;
		else{
			
			/*for(int d = 0; d < totalnum; d++){
			for(int i=0;i<this->m;i++){
				cout<<TESS.getValue(x[d][i])<<"  ";
			}
			cout<<endl;
		   }*/

			obj=TESS.getObjValue();			
			if(needcal){  
				if(!CONSTANT){     
					for(int ij=0;ij<this->m;ij++){
						double la=0;
						for(int d=0;d<totalnum;d++)
							la += TESS.getValue(x[d][ij])*eq[d].flow; //link上的负载
						if(la>=Link[ij]->capacity) Link[ij]->latency=1.0;
						else Link[ij]->latency=1.0/(Link[ij]->capacity-la);  
					}
					//计算OR边延迟
					for(int m=0;m<GSS->m;m++){	
						bool flag=false;
						double dmin=0;//延迟
						for(int d=0;d<totalnum;d++){
							if(GSS->Link[m]->tail==eq[d].org && GSS->Link[m]->head==eq[d].des && (eq[d].flow>0) ){ //OR边在需求中
								if(flag==false){
									//cout<<m<<" time "<<"flag"<<endl;
									flag=true;
									for(int ij=0;ij<this->m;ij++)
										dmin += TESS.getValue(x[d][ij])*Link[ij]->latency; 
								}
							}
							if(flag==true){
								GSS->Link[m]->latency=dmin;
								break;
							}
						} 
						if(flag==false){
							GSS->Link[m]->latency = this->dijkstraOR(GSS->Link[m]->tail,GSS->Link[m]->head,0);
						  //  cout<< " GSS->m is not in the req "<<endl;
						}
					}
				}
			}
		}
		env.end();
		return obj;
}

double CGraph::rst(vector<demand>& bg){
	IloEnv env; 
	IloModel model(env);
	IloCplex rst(model);

	int totalnum=bg.size();
	IloArray<IloIntVarArray> x(env, totalnum);	
	for(int d=0; d < totalnum; d++)
		x[d] = IloIntVarArray(env, this->m, 0, 1); 

	//约束1  流量守恒约束  对每个点进行流量守恒约束
	for(int d = 0; d < totalnum; d++)
		for( int i = 0; i < this->n; i++){    
			IloExpr constraint(env);
			for(unsigned int k = 0; k <adjL[i].size(); k++) // 出度边
				constraint += x[d][adjL[i][k]->id];
			for(unsigned int k = 0; k <adjRL[i].size(); k++) // 入度边
				constraint -= x[d][adjRL[i][k]->id];	

			if(i == (bg)[d].org)  //源点
				model.add(constraint == 1);
			else if(i == (bg)[d].des) //宿点
				model.add(constraint == -1);
			else
				model.add(constraint == 0);
		}
		//约束2 带宽约束
		for(int i = 0; i < this->m; i++){
			IloExpr constraint(env);
			for(int d = 0; d <  totalnum; d++)
				constraint += bg[d].flow*x[d][i];
			model.add(constraint<=Link[i]->capacity);  
		}
		//优化目标 最小化带宽利用率  
		IloNumVar z(env,0,1);
		for(int i = 0; i < this->m; i++){
			IloExpr load(env);
			for(int d = 0; d < totalnum; d++)
				load += bg[d].flow*x[d][i];
			model.add(load<=z*Link[i]->capacity);	
		}
		model.add(IloMinimize(env,z));

		rst.setOut(env.getNullStream());
		double obj = INF;
		rst.solve();
		if(rst.getStatus() == IloAlgorithm::Infeasible)
			env.out() << "rst No Solution" << endl;
		else{
			obj = rst.getObjValue();	
			//*********记录rst*********//
			this->xdl.clear();
			for(int i = 0; i < this->m; i++){
				for(int d = 0; d < totalnum; d++){
					cout << rst.getValue(x[d][i])<<"  ";
					this->xdl[i].push_back(rst.getValue(x[d][i]));					
				}
			}
			//*********记录fbg*********//
			for(int i = 0; i < this->m; i++){
				double u = 0;
				for(int d = 0; d < totalnum; d++){
					u += rst.getValue(x[d][i])*bg[d].flow;
				}
				this->Link[i]->use = u;
			}
		}
		env.end();
		return obj;
}


double OptSS(CGraph *G,CGraph *GSS,vector<SSdemand> &ss){    /////  eq的前ornum(GOR->m)个先装Overlay的需求  ////////
    IloEnv env;
    IloModel model(env);
    IloCplex OptSS(model);

	int num=ss.size();
    IloArray<IloIntVarArray> x(env, num); 
    for(int d = 0; d < num; d++)
		x[d] = IloIntVarArray(env, G->m, 0, 1); // eq->num * G->m 的二维矩阵
	
	IloNumVarArray cost(env,G->m,0.0 ,IloInfinity); 
	
	IloExpr res(env);
	for(int i=0;i<G->m;i++){
		IloNumExpr load(env);
	     for(int d = 0; d < num; d++) 
			 load += ss[d].flow*x[d][i];
		 model.add(cost[i]>=load/(G->Link[i]->capacity-G->Link[i]->use-1.0));   // [1,1/3]
	model.add(cost[i]>=3.0*load/(G->Link[i]->capacity-G->Link[i]->use-1.0)-(2.0/3.0));  //[1/3,2/3]
	model.add(cost[i]>=10.0*load/(G->Link[i]->capacity-G->Link[i]->use-1.0)-(14.0/3.0)); // [2/3,9/10]
	model.add(cost[i]>=70.0*load/(G->Link[i]->capacity-G->Link[i]->use-1.0)-(163.0/3.0));  //[9/10,1]
	res += cost[i];
	}
	model.add(IloMinimize(env,res));

	 //  流量约束// 1)对宿点约束
    for(int d = 0; d < num; d++)
		for(unsigned int i = 0; i < Uver.size(); i++){    // ver.size()为顶点数,ver[i]为顶点编号
			IloExpr constraint(env);
			for(unsigned int k = 0; k < GSS->adjL[Uver[i]].size(); k++) // 出度边
				constraint += x[d][GSS->adjL[Uver[i]][k]->id];
			for(unsigned int k = 0; k < GSS->adjRL[Uver[i]].size(); k++) // 入度边
				constraint -= x[d][GSS->adjRL[Uver[i]][k]->id];		
			if(Uver[i] == ss[d].user)  //用户节点为宿点
				model.add(constraint == -1);
		}

     // 计算fcp
     for(int i = 0; i<G->m;i++){
	 }

	//带宽约束
    for(int i = 0; i < G->m; i++){
		IloExpr constraint(env);
	for(int d = 0; d <  num; d++)
			constraint += ss[d].flow*x[d][i];
	model.add(constraint<=G->Link[i]->capacity-G->Link[i]->use);  
     }

	OptSS.setOut(env.getNullStream());
	double obj = INF;
	OptSS.solve();
	if(OptSS.getStatus() == IloAlgorithm::Infeasible)
		 env.out() << "OR Dictator No Solution" << endl;
	else{
		obj = OptSS.getObjValue();
		//****************************// 计算TE的链路利用率
		double util=0;
		for(int i=0;i<G->m;i++){
			double de=0;
			for(int d=0;d<num;d++)
				de += OptSS.getValue(x[d][i])*ss[d].flow;
			if(util<de/G->Link[i]->capacity)
				util=de/G->Link[i]->capacity;
		}
		G->mlu=util;
    }
	env.end();
    return obj;
}



#endif