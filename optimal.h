#ifndef OPTIMAL_H
#define OPTIMAL_H
#include"CGraph.h"
#include <ilcplex/ilocplex.h>

double SSdictor(CGraph *G,vector<demand> &req,int ss_req_num){    
	IloEnv env;
	IloModel model(env);
	IloCplex SSsolver(model);

	int num = req.size();
	IloArray<IloIntVarArray> x(env, num); 
	for(int d = 0; d < num; d++)
		x[d] = IloIntVarArray(env, G->m, 0, 1); // num * G->m 的二维矩阵

	IloNumVarArray cost(env,G->m,0.0 ,IloInfinity); 
	IloExpr res(env);
	for(int i = 0; i < G->m; i++){
		IloNumExpr load(env);
		for(int d = 0; d < ss_req_num; d++) 
			load += req[d].flow*x[d][i];
		model.add(cost[i]>=load/( G->Link[i]->capacity-1.0) );   // [1,1/3]
		model.add(cost[i]>=3.0*load/( G->Link[i]->capacity ) - 2.0/3.0  );  //[1/3,2/3]
		model.add(cost[i]>=10.0*load/( G->Link[i]->capacity ) -14.0/3.0  ); // [2/3,9/10]
		model.add(cost[i]>=70.0*load/( G->Link[i]->capacity )- 163.0/3.0  );  //[9/10,1]
		model.add(cost[i]>=500*load/(G->Link[i]->capacity ) - 1368.0/3.0  ); // [1,11/10]
		model.add(cost[i]>=5000*load/(G->Link[i]->capacity )- 14548.0/3.0  ); //[11/10,...]
		res += cost[i];
	}
	model.add(IloMinimize(env,res));

	//对每个点进行流量守恒约束  
	for(int d = 0; d < num; d++){
		for(int i = 0; i < G->n; i++){    // n为顶点数
			IloExpr constraint(env);
			for(unsigned int k = 0; k < G->adjL[i].size(); k++) // 出度边
				constraint += x[d][G->adjL[i][k]->id];
			for(unsigned int k = 0; k < G->adjRL[i].size(); k++) // 入度边
				constraint -= x[d][G->adjRL[i][k]->id];
			// 出 - 入
			if(i == req[d].org)
				model.add(constraint == 1);
			else if(i == req[d].des)
				model.add(constraint == -1);
			else
				model.add(constraint == 0);
		}
	}
	//带宽约束
	for(int i = 0; i < G->m; i++){
		IloExpr constraint(env);
		for(int d = 0; d <  num; d++)
			constraint += req[d].flow*x[d][i];
		model.add( constraint <= (G->Link[i]->capacity-G->Link[i]->use) );  
	}

	SSsolver.setOut(env.getNullStream());
	double obj = INF;
	SSsolver.solve();
	if(SSsolver.getStatus() == IloAlgorithm::Infeasible)
		env.out() << "SS Dictator No Solution" << endl;
	else{
		obj = SSsolver.getObjValue();
		/*
		for(int i=0;i<G->m;i++){  //计算每条边上的负载
			double de = 0;
			for(int d=0;d<num;d++)
				de += SSsolver.getValue(x[d][i])*req[d].flow;
			G->Link[i]->load = de + G->Link[i]->use;
		}
		double lat=0;
		for(int d = 0; d < ss_req_num; d++){   
			for(int ij = 0; ij < G->m; ij++){
				double del;
				if(G->Link[ij]->load >= G->Link[ij]->capacity) 
					del=1.0;
				else 
					del=1.0/(G->Link[ij]->capacity-G->Link[ij]->load);
				lat += SSsolver.getValue(x[d][ij])*del*req[d].flow; // 1/(C-x)  * x1  // 其中x1表示SS的流需求
			}
		}
		G->delay=lat;
		cout<<endl<<" cal "<<lat<<endl;
		*/

		// 计算TE
		double util=0;
		for(int i = 0; i < G->m; i++){  
			double load = 0;
			for(int d = 0; d < num; d++)
				load += SSsolver.getValue(x[d][i])*req[d].flow;
			load += G->Link[i]->use;
			double cur = load / G->Link[i]->capacity; 
			if( util < cur )
				util = cur;
		}
		G->mlu = util;
	}
	env.end();
	return obj;
}


double TEdictor(CGraph *G,vector<demand> & req,int ss_req_num){
	IloEnv env;
	IloModel model(env);
	IloCplex TEsolver(model);

	int num=req.size();	
	IloArray<IloIntVarArray> x(env, num); 
	for(int d = 0; d < num; d++)
		x[d] = IloIntVarArray(env, G->m, 0, 1); 	

	// 对每个点进行流量守恒约束  
	for(int d = 0; d < num; d++){
		for(int i = 0; i < G->n; i++){    // n为顶点数
			IloExpr constraint(env);
			for(unsigned int k = 0; k < G->adjL[i].size(); k++) // 出度边
				constraint += x[d][G->adjL[i][k]->id];
			for(unsigned int k = 0; k < G->adjRL[i].size(); k++) // 入度边
				constraint -= x[d][G->adjRL[i][k]->id];

			if(i == req[d].org)
				model.add(constraint == 1);
			else if(i == req[d].des)
				model.add(constraint == -1);
			else
				model.add(constraint == 0);
		}
	}

	for(int i = 0; i < G->m; i++){
		IloExpr constraint(env);
		for(int d = 0; d <  num; d++)
			constraint += req[d].flow*x[d][i];
		model.add( constraint <= (G->Link[i]->capacity-G->Link[i]->use) );  
	}

	//优化目标 最小化带宽利用率  
	IloNumVar z(env,0,1);
	for(int i = 0; i < G->m; i++){
		IloExpr load(env);
		for(int d = 0; d < num; d++)
			load += req[d].flow*x[d][i];
		model.add( load <= z*(G->Link[i]->capacity-G->Link[i]->use) );	
	}
	model.add(IloMinimize(env,z));

	TEsolver.setOut(env.getNullStream());
	double obj = INF;
	TEsolver.solve();
	if(TEsolver.getStatus() == IloAlgorithm::Infeasible)
		env.out() << "TE Dictator No Solution" << endl;
	else{
		obj = TEsolver.getObjValue();

		for(int i=0;i<G->m;i++){  //计算每条边上的负载
			double de = 0;
			for(int d=0;d<num;d++)
				de += TEsolver.getValue(x[d][i])*req[d].flow;
			G->Link[i]->load = de + G->Link[i]->use;
		}
		double lat=0;
		for(int d = 0; d < ss_req_num; d++){   
			double del;
			for(int ij = 0; ij < G->m; ij++){				
				if(G->Link[ij]->load >= G->Link[ij]->capacity) 
					del=1.0;
				else 
					del=1.0/(G->Link[ij]->capacity-G->Link[ij]->load);
				lat += TEsolver.getValue(x[d][ij])*del*req[d].flow; // 1/(C-x) * x1  // 其中x1表示SS的流需求
			}
		}
		G->delay=lat;
	}
	env.end();
	return obj;
}



#endif