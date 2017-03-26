#include<iostream>
#include <math.h>
#include <vector>
#include <openrave/plugin.h>
#include <stdlib.h>
#include <ctime>
#include <time.h>
#define  PI 3.14;
using namespace std;
using namespace OpenRAVE;
vector<GraphHandlePtr> handles;
// calculating euclidean distance
double eucl(std::vector<double> a,std::vector<double> b)
{
  double eu=0;
  //adding weights
  std::vector<double> weight;
  weight.push_back(1);
  weight.push_back(1);
  weight.push_back(1);
  weight.push_back(1);
  weight.push_back(0);
  weight.push_back(0);
  weight.push_back(0);
  for (unsigned int i=0;i<a.size();i++)
    eu+=weight[i]*pow((a[i]-b[i]),2);
  return pow(eu,0.5);
}

//class definitions
class RRTNode
{
  std::vector<double> config;
  RRTNode *parent;

  public:
    // constructor with only conf
    RRTNode(std::vector<double> v)
    {
      config=v;
      parent=NULL;
    }
    // constructor with conf and parent

    RRTNode(std::vector<double> v,RRTNode* par)
    {
      config=v;
      parent=par;
    }

    // getting parent
    RRTNode* get_par()
    {
      return parent;
    }

    // setting parent
    void set_par(RRTNode* par)
    {
      parent=par;
    }

    // getting conf
    std::vector<double> get_conf()
    {
      return config;
    }

    // operatort overloading to compate 2 objects
    bool operator==(RRTNode obj)
    {
      for(unsigned int i=0;i<config.size();i++)
      {
        if(config[i]!=obj.config[i])
          return false;
      }
      return true;
    }

    // displaying a node
    void disp()
    {
      for(unsigned int i=0;i<config.size();i++)
        cout<<config[i]<<" ";
    }

};

//class of nodes
class NodeTree
{
  std::vector<RRTNode*> list;

  public:

    // constuctor to make a branch
    NodeTree(std::vector<RRTNode*> nodes)
    {
      list=nodes;
    }
    //constuctor to clear
    NodeTree()
    {
      list.clear();
    }
    // adding nodes to list
    void addnode(RRTNode* n)
    {
      list.push_back(n);
    }
    void number_nodes()
    {
      cout<<"the number of nodes are "<<list.size()<<"\n";
    }
     // deleting nodes from list
    void deletenode(RRTNode* n)
    {
      for (unsigned int i=0;i<list.size();i++)
      {
        if(list[i]==n)
        {
          list.erase(list.begin()+i);
          break;
        }
      }
    }

    // getting nodes from list
    std::vector<RRTNode*> getnode()
    {
      return list;
    }

    // nearest neighbor function
    RRTNode* near_neigh(std::vector<double> rc)
    {
      RRTNode *a;
      //defining first node as nearest to set bar to compare
      double dist,min=eucl(list[0]->get_conf(),rc);
      a=list[0];
      for (unsigned int i=1;i<list.size();i++)
      {
        dist=eucl(list[i]->get_conf(),rc);
        if(dist<min)
        {
        //setting nearer node as nearest
         min=dist;
         a=list[i];
       }
      }
      return a;
    }

    // displaying list from goal to start
    std::vector<std::vector<double> > display()
    {
      RRTNode *a;
      std::vector<std::vector<double> > path;
      a=list[list.size()-1];
      while(a->get_par()!=NULL)
      {
        path.push_back(a->get_conf());
        a=a->get_par();
      }
      path.push_back(a->get_conf());
      return path;
    }

};

//finidng the direction of the vector pointing to new node and finding node withing a step
std::vector<double> direction(std::vector<double> nn, std::vector<double> rc,float s)
{
  std::vector<double> v;
  double x;
  x=eucl(rc,nn);
  for(unsigned int i=0;i<nn.size();i++)
    v.push_back(nn[i]+((rc[i]-nn[i])*s/x));
  return v;
}

// random sampling
std::vector<double> random(std::vector<double> low,std::vector<double> high)
{
  std::vector<double> pt;
  for(unsigned int i=0;i<low.size();i++)
    pt.push_back((((double)rand()/RAND_MAX)*(high[i]-low[i]))+low[i]);
  return pt;
}
//drawing in OpenRAVE
void draw_path(std::vector<std::vector<double> > path,int color,RobotBasePtr pr2,OpenRAVE::EnvironmentBasePtr env)
{
  std::vector<double> links;
  std::vector<float> points;
  float red[4]={1,0,0,1},blue[4]={0,0,1,1};
  for(unsigned int i=0;i<path.size();i++)
  {
    points.clear();
    links.clear();
    for(unsigned int j=0;j<path[i].size();j++)
      links.push_back(path[i][j]);
    pr2->SetActiveDOFValues(links);
    //getting end effector position
    Transform T=pr2->GetLinks()[49]->GetTransform();
    points.push_back((float)T.trans.x);
    points.push_back((float)T.trans.y);
    points.push_back((float)T.trans.z);
    points.push_back(1);
    if(color==1)
      handles.push_back(env->plot3(&points[0],1,1,5,red,0));
    else
      handles.push_back(env->plot3(&points[0],1,1,5,blue,0));
  }
}
//smoothing algorithm
std::vector<std::vector<double> > smooth(std::vector<std::vector<double> > path,float step,OpenRAVE::EnvironmentBasePtr env,RobotBasePtr pr2)
{
  std::vector<double> node1,node2,dir;
  std::vector<std::vector<double> > inter;
  unsigned int index1,index2;
  bool flag;
  // double smooth_node_len=0,smooth_path_len=0;
  for(int i=0;i<200;i++)
  {
    //printing for each itration
    // smooth_path_len=0;
    // for(unsigned int j=0;j<(path.size()-1);j++)
    // {
    //   smooth_node_len=0;
    //   for(unsigned int k=0;k<=path[j].size();k++)
    //   {
    //     smooth_node_len+=pow((path[j][k]-path[j+1][k]),2);
    //   }
    //   smooth_path_len+=pow(smooth_node_len,0.5);
    // }
    // cout<<"path length of smoothened path after "<<i<<"iterations is "<<smooth_path_len<<"\n";
    //clearing before next itration
    node1.clear();
    node2.clear();
    dir.clear();
    inter.clear();
    flag=false;
    do
    {
      // generating 2 random nodes
      index1=rand()%(path.size());
      index2=rand()%(path.size());
    }
    while(index1==index2);
    // making sure lowest node is first
    if(index2<index1)
      swap(index1,index2);
    node1=path[index1];
    inter.push_back(node1);
    node2=path[index2];
    dir=node1;
    // path between two nodes shortest without collision
    do
    {
      if(eucl(dir,node2)<=step)
      {
        pr2->SetActiveDOFValues(node2);
        if (env->CheckCollision(pr2)||pr2->CheckSelfCollision())
        {
          flag=false;
          break;
        }
        else
        {
          inter.push_back(node2);
          flag=true;
          break;
        }
      }
      else
      {
        dir=direction(dir,node2,step);
        pr2->SetActiveDOFValues(dir);
        if (env->CheckCollision(pr2)||pr2->CheckSelfCollision())
        {
          flag=false;
          break;
        }
        else
          inter.push_back(dir);
          flag=true;
      }
    }
    while(flag!=false);
    // if no collision delete intermediate nodes inset new tree
    if(flag!=false)
    {
      path.erase(path.begin()+index1,path.begin()+index2);
      path.insert(path.begin()+index1,inter.begin(),inter.end());
    }
  }
  return path;
}

// RRT planner
void RRTpath(OpenRAVE::EnvironmentBasePtr env,std::vector<double>goal,float goalb,float step,std::vector<double> start)
{
  // initializations
  time_t time_s,time_e,time_en,time_st;
  time_s=time(NULL);
  std::vector<double> low,high;
  std::vector<RobotBasePtr> robots;
  RobotBasePtr pr2;
  double rrt_node_len=0,smooth_node_len=0,rrt_path_len=0,smooth_path_len=0;
  env->GetRobots(robots);
  pr2=robots[0];
  //getting limits
  pr2->GetActiveDOFLimits(low,high);
  //changing limits for 5th and 7th link
  low[4]=-PI;
  low[6]=-PI;
  high[4]=PI;
  high[6]=PI;
  RRTNode *node,*inter,*st;
  st=new RRTNode(start);
  NodeTree tree;
  tree.addnode(st);
  std::vector<std::vector<double> > path;
  std::vector<double> samp,dir;
  bool goalr= false;

  do
  {
    if((double)rand()/RAND_MAX<goalb) // to check if sample is goal
    {
      samp=goal; // setting sample as goal
      goalr= true;
    }
    else
      samp=random(low,high); // chosing random sample
  //  extending
    while(1)
    {
      node=tree.near_neigh(samp); // getting nearest neigbor to sample
      //if reached goal node
      if (eucl(samp,node->get_conf())<=step)
      {
        pr2->SetActiveDOFValues(node->get_conf());
        if (env->CheckCollision(pr2)||pr2->CheckSelfCollision())
        {
          goalr = false;
          break;
        }
        else
        {
          if (goalr)
            cout<<"goal !!!!!!!!!\n";
          inter=new RRTNode(samp,node);
          tree.addnode(inter);
          break;
        }
      }
      else
      {
        dir=direction(node->get_conf(),samp,step); // getting direction vector to sample
        pr2->SetActiveDOFValues(dir);
        if(env->CheckCollision(pr2)||pr2->CheckSelfCollision())
        {
          goalr=false;
          break;
        }
        else
        {
          inter=new RRTNode(dir,node);
          tree.addnode(inter);
        }
      }
      time_e=time(NULL);
    }
    //if time is more than 400 seconds , terminates
    if ((time_e-time_s)>400)
      break;
  }
  while(!goalr);
  time_e=time(NULL);
  cout<<"goal bias is \t"<<goalb<<"\n";
  if ((time_e-time_s)>400)
    cout<<"could not compute in time\n";
  else
  cout<<"seconds = "<<(time_e-time_s)<<"\n";
  //printing number of nodes sampled
  tree.number_nodes();
  //getting path
  path=tree.display();
  //reversing path
  std::reverse(path.begin(),path.end());
  //getting length of path
  for(unsigned int i=0;i<(path.size()-1);i++)
  {
    rrt_node_len=0;
    for(unsigned int j=0;j<=path[i].size();j++)
    {
      rrt_node_len+=pow((path[i][j]-path[i+1][j]),2);
    }
    rrt_path_len+=pow(rrt_node_len,0.5);
  }
  cout<<"path length of rrt path is "<<rrt_path_len<<"\n";
  //drawing path in red
  draw_path(path,1,pr2,env);
  //time for smoothing
  time_st=time(NULL);
  path=smooth(path,step,env,pr2);
  time_en=time(NULL);
  cout<<"time to smoothen the path is "<<time_en-time_st<<"\n";
  //getting new path length
  for(unsigned int i=0;i<(path.size()-1);i++)
  {
    smooth_node_len=0;
    for(unsigned int j=0;j<=path[i].size();j++)
    {
      smooth_node_len+=pow((path[i][j]-path[i+1][j]),2);
    }
    smooth_path_len+=pow(smooth_node_len,0.5);
  }
  cout<<"path length of smoothened path is "<<smooth_path_len<<"\n";\
  //drawing path in blue
  draw_path(path,0,pr2,env);
  //creating trajectory in OpenRAVE
   TrajectoryBasePtr ptraj = RaveCreateTrajectory(env,"");
   ConfigurationSpecification conspec=pr2->GetActiveConfigurationSpecification("linear");
   conspec.AddDeltaTimeGroup();
   ptraj->Init(conspec);
   std::vector<double> path_pt;
  for(unsigned int i=0;i<path.size();i++)
  {
    path_pt=path[i];
    path_pt.push_back(i*0.01);
    ptraj->Insert(i,path_pt,conspec,true);
  }
  pr2->GetController()->SetPath(ptraj);
}
