#include <iostream>
#include <vector>
#include <list>
#include <algorithm>
#include <utility>
#include <limits.h>
#include "uhf_rfid_api/UhfRfid.h"
#include "ratslam_ros/ViewTemplate.h"

#include "ros/ros.h"

using namespace std;

typedef size_t utint;
int cur_id_ = 0;
int pre_id_ = 0;

template <class T,class V>
class pair_first_comp

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
	vector<T> data;    
	vector<list<pair <utint,int> > > links;
	utint edgeCount;
public:
	graf( const vector<T>& _data)
	{
		data=_data;
		utint n=_data.size();
		for(utint i=0;i<n;i++)
		{
			list<pair <utint,int> > temp;
			links.push_back(temp);
		}
	}
	int addLink(utint from, utint to, int weight)
	{
		if(links.size()>from && links.size()>to)
		{
			links[from].push_back(pair<utint,int>(to,weight));
			return 0;
		}else{
			return -1;
		}
		edgeCount++;
	}
	void addNode(const T& element)
	{
		data.push_back(element);
		list<pair <utint,int> > temp;
		links.push_back(temp);
	}
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
	pair<bool,int> isDirectWay(utint from,utint to)
	{
		pair_first_comp<utint,int> cmp;
		cmp.operand=to;
		list<pair<utint,int> >::iterator it=find_if(links[from].begin(),links[from].end(),cmp);
		if(links[from].end()==it)
		{
			return pair<bool,int>(false,0);
		}else{
			return pair<bool,int>(true,it->second);
		}
	}
	pair<bool,int> Dijkstra(utint from,utint to)// O(N^2)
	{
		vector< pair<bool,int> > dist(data.size(), pair<bool,int>(0,-1));
		vector<int> prev (data.size(),-1);
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
  if((0 != nodes.current_id)&&(cur_id_ != nodes.current_id))
  {
    pre_id_ = cur_id_;
    cur_id_ = nodes.current_id;
   cout <<"in callback"<<cur_id_<<pre_id_<<endl;
  }
}

int main(int argc, char **argv)
{
  vector<int>  arr;
  int chs_mode = 0;
  int head,tail;
  
	int i = 0;
  ROS_INFO_STREAM(" - openRatSLAM Copyright (C) 2012 David Ball and Scott Heath");
  ROS_INFO_STREAM("algorithm use dijkstar");
  ROS_INFO_STREAM("Distributed under the GNU GPL v3, see the included license file.");
  ROS_INFO_STREAM(" - feixiao ceshi.");
  
  ros::init(argc, argv, "graph");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/irat_red/LocalView/Template", 0, node_callback);
	ros::Rate loop_rate(1);
	pre_id_ = 0;
	arr.push_back(pre_id_);
	arr.push_back(1);
	arr.push_back(2);
	arr.push_back(3);
	arr.push_back(4);
	arr.push_back(5);
	arr.push_back(6);
	arr.push_back(7);
	arr.push_back(8);
	arr.push_back(9);
	arr.push_back(10);
	arr.push_back(11);
	arr.push_back(12);
	arr.push_back(13);
	arr.push_back(14);
	arr.push_back(15);
	arr.push_back(16);
	arr.push_back(17);
	arr.push_back(18);
	arr.push_back(19);
	arr.push_back(20);
	graf<int> g(arr);
	cout<< "初始化成功\n";
	while(ros::ok())
  {	
	  ROS_INFO_STREAM("i don`t repeat\n");
	  cout<<"cur_id:"<< cur_id_<<endl;
	  if(pre_id_ != cur_id_)
	  {
	    cout<<"in while\n";
	    cout<<"cur_id_: "<<cur_id_ << endl;
	    int tmp;
	    if((tmp != cur_id_)&&(0!=cur_id_))
	    {
	      cout<<"dudududdudududududududdududu"<<endl;
	      g.addLink(pre_id_,cur_id_,1);
	      i++;
	      tmp = cur_id_;
	    }
	    if(5 == i)
	    {
	      i = 0;
	    }
	}
	cout<<"shortest way: "<<g.shortestWay(1,17).second<<endl;

	cout<<"dijkstra: "<<g.Dijkstra(1, 17).second<<endl;
	
	g.printGraf();
	cout<<"way: "<<g.isDirectWay(1, 17).second<<endl;
	  ros::spinOnce();
	 loop_rate.sleep();
  }
	return 0;
}
