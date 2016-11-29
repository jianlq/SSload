#include"evolutionbit.h"
#include"optimal.h"
#include<vld.h> ////Visual Leak Detector 内存泄露检测工具

vector<SSdemand> reqSS;
vector<demand> reqTE;
vector<demand> reqbase;
vector<demand> reqSSdem;
vector<double> consider;

int TESS_Stackelebrg(CGraph *G,CGraph *GSS,int ss_req_num,double consider,double &cmpmlu,double &cmpdelay){
	FILE *out = fopen("outputFile//tess_s.csv", "a");
	//TE optimal
	G->clearOcc();
	fprintf(out, "%f\n",consider); 
	double tedic=INF;
	tedic = TEdictor(G,reqTE,ss_req_num);
	fprintf(out, "%f,%f\n\n",tedic,G->delay); 
	printf("%f\t%f\n",tedic,G->delay);

	// Server Selection optimal
	G->clearOcc();
	double ssdic = INF;
	ssdic = SSdictor(G,reqTE,ss_req_num);
	fprintf(out, "%f,%f\n\n", G->mlu,ssdic); 
	printf("%f\t%f\n", G->mlu,ssdic); 

	if( tedic >= INF || ssdic >= INF)
		return 0;

	//Stackelberg
	if(!G->GAinit(reqTE))
		return 0;
	int n = 250;//种群个体数目
	int m = reqTE.size();	
	evoluPopubit popubit(n,m,G,GSS,&reqTE,&reqSSdem,ssdic,consider);			
	popubit.evolution();
	fprintf(out, "%f,%f\n\n",popubit.hero.mlu,popubit.hero.delay);
	printf("stackelberg\n%f\t%f\n\n",popubit.hero.mlu,popubit.hero.delay); 
	fclose(out);
	cmpmlu = popubit.hero.mlu/tedic;
	cmpdelay = popubit.hero.delay/ssdic;

	return 1;	

}
int main(){	

	//CGraph *G = new CGraph("inputFile//graphATT.txt");
	//G->KSP(6,19,16);
	//exit(1);

	srand((unsigned)time(NULL));
	int Time = 5;
	int CASEnum = 10;
	int Consider_Value = 50;
	double con = 0;
	consider.push_back(con);

	for(int i = 1 ; i <= Consider_Value; i++){
		con += 0.1;
		consider.push_back(con);		
	}

	for(int t =0;t < Time;t++){
		for(int ij = 0; ij < consider.size(); ij ++){
			double cmpmlu = 0, cmpdelay = 0;
			int sucCase = 0;
			FILE *out = fopen("outputFile//result.csv", "a");

			//int n = 16;
			//genGraph(n,45,"inputFile//genGraph.txt");
			//CGraph *G = new CGraph("inputFile//genGraph.txt");
			CGraph *G = new CGraph("inputFile//graphATT.txt");

			genSSGraph(G->n,G->n/6,G->n/5,"inputFile//ss.txt");
			CGraph *GSS = new CGraph("inputFile//ss.txt");

			for(int casenum = 0; casenum < CASEnum; casenum++){	
				reqbase.clear();//background traffic
				for(int i = 0; i < NUMDEMAND; i++){
					int s = rand()%G->n, t;
					do{
						t = rand()%G->n;
					}while(s == t || G->canNotReach(s, t));
					reqbase.push_back(demand(s, t, rand()%(MAXDEMAND)+2));
				}

				reqSS.clear(); 
				for(vector<int>::iterator it = Uver.begin();it != Uver.end();it++)
					reqSS.push_back(SSdemand((*it),rand()%(MAXDEMAND)+10));

				// 进行 1/server分解 初始化业务量矩阵
				reqTE.clear();
				reqSSdem.clear();
				int j = 0;
				int sn = Sver.size();
				while(j < reqSS.size()){
					for( int d = 0; d < sn; d++){
						if(G->canNotReach(Sver[d],reqSS[j].user))
							continue;
						else{
							demand cur(Sver[d],reqSS[j].user,reqSS[j].flow/sn);
							reqTE.push_back(cur);
							reqSSdem.push_back(cur);
						}
					}
					j++;
				}
				int ss_req_num = reqTE.size();
				for(unsigned int i = 0; i < reqbase.size(); i++)
					reqTE.push_back(reqbase[i]);

				/*	G->GAinit(reqTE);
				cout << ij << "  &&&&&&&&&&&&&&&&&&&&&&&&&&&&&& " <<casenum<<endl;*/

				double m ,d;
				int sucess = TESS_Stackelebrg(G,GSS,ss_req_num,consider[ij],m,d);
				sucCase += sucess;
				if(sucess){
					cmpmlu += m;
					cmpdelay += d;
				}			
			}		
			fprintf(out, "%f,%f,%f\n", consider[ij],cmpmlu/sucCase, cmpdelay/sucCase);		
			fclose(out);
			delete G;
			delete GSS;
		}
	}
	system("pause");
	return 0;	

}