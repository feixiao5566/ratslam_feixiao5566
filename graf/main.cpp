#include <iostream>
#include <vector>
#include <list>
#include <algorithm>
#include <utility>
#include <limits.h>
//#include "dependant_api/UhfRfid.h"
#include "uhf_rfid_api/UhfRfid.h"
#include "ratslam_ros/ViewTemplate.h"

#include "ros/ros.h"

using namespace std;

typedef size_t utint;

template <class T,class V>




//定义一些全局数据,初始功能实现后,定义为类的全局变量 public
int cur_id_ = 0;
int pre_id_ = 0;

class pair_first_comp  //初对比较

{
public:
	T operand;
	bool operator()(pair<T,V> value)
	{
		return operand==value.first;
	}
};


template<class T>
class graf
{
private:
	vector<T> data;    //数组 data      
	vector<list<pair <utint,int> > > links;     // 数组<链表<对组> >
	utint edgeCount;     //边
public:
	graf( const vector<T>& _data)      //   构造函数
	{
		data=_data;
		utint n=_data.size();
		for(utint i=0;i<n;i++)
		{
			list<pair <utint,int> > temp;   // 第一个uint是data.size,第二个是vector的值
			links.push_back(temp);  //存入数组
		}
	}
	//增加边连接
	int addLink(utint from, utint to, int weight)   //    前个节点,后个节点 ,权
	{
		if(links.size()>from && links.size()>to)    //  ?
		{
			links[from].push_back(pair<utint,int>(to,weight));     //  找到from,在from后链表加入to和权
			return 0;
		}else{
			return -1;
		}
		edgeCount++;    //边
	}
	//增加点连接
	void addNode(const T& element)
	{
		data.push_back(element);   //点在data数组中
		list<pair <utint,int> > temp;    //这对pair值哪里来的
		links.push_back(temp);
	}
	//打印图
	void printGraf()
	{
		for(utint i=0;i<links.size();i++)
		{
			cout<<data[i]<<endl;
			list<pair<utint,int> >::iterator it=links[i].begin();
			while(it!=links[i].end())
			{
				cout<<data[i]<<" ->"<<data[(*it).first]<<" [ label = \""<<(*it).second<<"\"]"<<endl;
				it++;
			}
		}
	}
	//直连路   直接节点路径
	pair<bool,int> isDirectWay(utint from,utint to)
	{
		pair_first_comp<utint,int> cmp;
		cmp.operand=to;
		list<pair<utint,int> >::iterator it=find_if(links[from].begin(),links[from].end(),cmp);//it 结构体链表
		if(links[from].end()==it)  //如果it到了链表的末尾
		{
			return pair<bool,int>(false,0);  //返回假和0
		}else{
			return pair<bool,int>(true,it->second); //返回真和他的下一个
		}
	}
	//dijkstar算法找最短路径
	pair<bool,int> Dijkstra(utint from,utint to)// O(N^2)
	{//using array
		vector< pair<bool,int> > dist(data.size(), pair<bool,int>(0,-1));// array of lenght (bool=0 => unexplored)        这个数组叫dist  放着pair  为啥还有个size? 
		vector<int> prev (data.size(),-1);//array of previous vertexes in the way  顶点集 数组 前一个
		dist[from]=pair<bool,int>(1,0);
		utint curr=from;
		bool complete=false;
		while(!complete)
		{
			list<pair<utint,int> >::iterator it=links[curr].begin();
			while(it!=links[curr].end())
			{
				if(!dist[it->first].first)
				{
					if(dist[it->first].second!=-1)
					{
						dist[it->first].second=min(dist[it->first].second, dist[curr].second + it->second);
					}else{
						dist[it->first].second= dist[curr].second + it->second;
					}
				}
				it++;
			}
			int min=INT_MAX; //magic
			int imin=-1;
			for(utint i=0;i<dist.size();i++)
			{
				if(dist[i].first==0 && dist[i].second!=-1)
				{
					if(dist[i].second<min)
					{
						min=dist[i].second;
						imin=i;
					}
				}
			}
			if(-1==imin)
			{
				complete=true;
			}else{
				curr=imin;
				dist[imin].first=true;
			}
			
		}
		return dist[to];
	}
	
  //最短路径	
	pair<bool,int> shortestWay(utint from,utint to)
	{//Bellman-Ford alhorithm
		 vector< pair<bool,int> > dist(data.size(), pair<bool,int>(0,0));// array of lenght (bool=0 => infinite)
		 vector<int> prev (data.size(),-1);//array of previous vertexes in the way
		 dist[from]=pair<bool,int>(1,0);
		 for(utint i=0;i<data.size()-1;i++)
		 {
			 for(utint j=0;j<data.size();j++)
			 {
				 if(dist[j].first)
				 {
					 list<pair<utint,int> >::iterator it=links[j].begin();
					 while(it!=links[j].end())
					 {
						 if(dist[it->first].first)
						 {
							 //not inf
							 if(dist[it->first].second>(dist[j].second+it->second))
							 {
								 dist[it->first].second=dist[j].second+it->second;
								 prev[it->first]=j;
							 }
						 }else{
							 //inf
							 dist[it->first]=pair<bool,int>(1,dist[j].second+it->second);
							 prev[it->first]=j;
						 }
						 it++;
					 }
				 }
			 }
		 }
		 
		
		 cout<<data[to]<<"<-";
		 int p=prev[to];
		 while(p!=-1)
		 {
			 cout<<data[p]<<"<-";
			 p=prev[p];
		 }
		 cout<<endl;
		 
		 return dist[to];
	}
};
void node_callback(ratslam_ros::ViewTemplate nodes)
{
  pre_id_ = cur_id_;
  cur_id_ = nodes.current_id;
}

int main()
{
  vector<int>  arr;
  int chs_mode = 0;
  int head,tail;
  ROS_INFO_STREAM(<< " - openRatSLAM Copyright (C) 2012 David Ball and Scott Heath");
  ROS_INFO_STREAM("algorithm use dijkstar");
  ROS_INFO_STREAM("Distributed under the GNU GPL v3, see the included license file.");
  ROS_INFO_STREAM(" - feixiao ceshi.");
  
  ros::init(argc, argv, "graph");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/irat_red/LocalView/Template", 0, node_callback);
  
  cout << "choose mode: 1--->way   2--->draw   3--->maps"
  cin >> chs_mode;
  switch(chs_mode)
  {
    case 1:arr.push_back(cur_id_);
           //arr.push_back(2);
	         //arr.push_back(3);
	         //arr.push_back(4);
	         //arr.push_back(5);
	         graf<int> g(arr);  //graf类的构造函数  把数组和数组大小 做成链表 存入数组
	         g.addLink(pre_id_, cur_id_, 3);
	         //g.addLink(0,3,2);
	         //g.addLink(1,4,2);
	         //g.addLink(2,4,1);
	         //g.addLink(3,2,1);
	         //g.addLink(3,4,4);
	
	         //g.addLink(0,1,5);
	         //g.addLink(0,3,10);
	         //g.addLink(1,2,5);
	         //g.addLink(1,4,1);
	         //g.addLink(2,4,1);
	         //g.addLink(3,2,-7);
	         //g.addLink(3,4,-4);
	
	         break;
	
	  case 2:cout<<"want to find head and tail is: "<<endl;
	         cin>>head>>tail;
	         cout<<"shortest way: "<<g.shortestWay(head,tail).second<<endl;
	
	         cout<<"dijkstra: "<<g.Dijkstra(head,tail).second<<endl;
	         cout<<"way: "<<g.isDirectWay(0,4).second<<endl;
	         break;
	         
	  case 3:g.printGraf();
	         break;
	  default: break;
	
	}
	ros::spin();
	return 0;
}
