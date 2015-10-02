#include <set>
#include <limits.h>
#include <array>
#include <thread>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <unordered_map>
#include <string>
#include <vector>
#include <sstream>
#include "Timer.h"
#include <boost/circular_buffer.hpp>
#include "Channel.hpp"

using namespace std;

struct Unique_id_generator{
  unordered_map<string ,int> foward;
  unordered_map<int ,string> reverse;

  int get_id( const string &s){
    auto where = foward.find(s);
    if( where != foward.end() ){
      return where->second;
    } else {
      int ret = foward.size();
      foward.emplace( s, ret);
      reverse.emplace( ret, s);
      return ret;
    }
  }

  string operator()(const int i ){
    return reverse_id( i );
  }

  int operator()(const string &s){
    return get_id(s);
  }

  string reverse_id( int id ){
    auto where = reverse.find( id );
    if( where == reverse.end()){
      throw std::invalid_argument("id not found");
    }
    return where->second;
  }
};

Unique_id_generator gen;

pair<int,int> graph_line(const string &line){
  pair<int,int> ret;
  string word1, word2;

  {
    string trash;
    stringstream ss(line);
    ss>>word1>>trash>>word2;

    word1 = word1.substr( 5, 9);
    word2 = word2.substr( 5, 9);
  }
  ret = make_pair( gen(word1), gen(word2) );
  return ret;
}

vector<string> get_lines ( istream &is ){
  vector<string> ret;
  string line;
  while(true){
    getline( is, line);
    if(is.good()){
      ret.push_back( move(line) );
    } else {
      break;
    }
  }
  return ret;
}

typedef unordered_multimap<int,int> Graph;



//find deepest breath first search
int bredth_first( const Graph &graph, int source){
  const int vertex_count = gen.foward.size()+1;

  vector<bool> visited(vertex_count, false);
  int visited_count=0;
  boost::circular_buffer<int> buff(vertex_count);

  buff.push_back( source );
  visited_count++;
  visited.at(source)=true;

  int depth=0;
  while( buff.size() > 0 ){
    depth++;
    int next = buff.front();
    buff.pop_front();

    auto range = graph.equal_range( next );
    for( ; range.first != range.second; range.first++){
      int canidate = range.first->second;
      if( visited[canidate] ) continue;
      visited[canidate]=true;
      buff.push_back(canidate);
    }
  }

  return {depth};
}

typedef int vertex_t;
typedef int weight_t;

int DijkstraComputePaths( Graph graph, int src) {
  const int n = gen.foward.size();
  const int max_weight = INT_MAX;
  vector<int> min_distance;
  min_distance.resize(n, max_weight);
  min_distance[src] = 0;
  vector<int> previous;
  previous.resize(n, -1);
  std::set<std::pair<weight_t, vertex_t> > vertex_queue;
  vertex_queue.insert(std::make_pair(min_distance[src], src));

  while (!vertex_queue.empty()) {
    weight_t dist = vertex_queue.begin()->first;
    vertex_t u = vertex_queue.begin()->second;
    vertex_queue.erase(vertex_queue.begin());

    // Visit each edge exiting u
    std::vector<int> neighbors ;
    {
      auto pair = graph.equal_range(src);
      for( ; pair.first != pair.second; pair.first++){
        neighbors.push_back( pair.first->second );
      }
    }
    for (std::vector<int>::const_iterator neighbor_iter = neighbors.begin();
         neighbor_iter != neighbors.end();
         neighbor_iter++) {
      vertex_t v = *neighbor_iter;
      weight_t weight = 1;
      weight_t distance_through_u = dist + weight;
      if (distance_through_u < min_distance[v]) {
        vertex_queue.erase(std::make_pair(min_distance[v], v));

        min_distance[v] = distance_through_u;
        previous[v] = u;
        vertex_queue.insert(std::make_pair(min_distance[v], v));
      }
    }
  }
  transform( min_distance.begin(), min_distance.end(), min_distance.begin(),
      [](auto a){
        if( a == INT_MAX ){
          a=0;
        }
        return a;
      });
  return *max_element( min_distance.begin(), min_distance.end() );
}

int floyd_warshall( Graph graph ){
  const int nodesCount = gen.foward.size();
  vector<vector<int>> distance;
  {
    vector<int>temp(nodesCount, INT_MAX);
    distance.resize(nodesCount, temp);
  }

  for( auto &p: graph){
    distance[p.first][p.second]=1;
  }

  //Floyd-Warshall
  for (int k=0;k<nodesCount;++k){
    for (int i=0;i<nodesCount;++i){
      if (distance[i][k]!=INT_MAX){
        for (int j=0;j<nodesCount;++j){
          if (distance[k][j]!=INT_MAX && (distance[i][j]==INT_MAX || distance[i][k]+distance[k][j]<distance[i][j])){
              distance[i][j]=distance[i][k]+distance[k][j];
          }
        }
      }
    }
  }

  int diameter=-1;

  //look for the most distant pair
  for (int i=0;i<nodesCount;++i){
    for (int j=0;j<nodesCount;++j){
      if (diameter<distance[i][j]){
        diameter=distance[i][j];
        printf("%d %d\n", i, j);
      }
    }
  }
  return {0};
}

const auto di = bredth_first;

Graph build_graph( istream &is ){
  Graph ret;

  auto lines = get_lines( is );
  for( auto & line : lines ){
    ret.insert( graph_line(line));
  }
  return ret;
}

void do_dijk( const Graph &graph, Channel<int> *in, Channel<int> *out){
  int val;
  while( in->get(val)){
    //cout<<val<<endl;
    out->put( di( graph, val) );
  }
}


int main(int argc, char **argv){
  Timer t1;
  string fname="story.nq";
  if( argc == 2 ){
    fname=argv[1];
  }
  fstream file( fname );
  if( ! file ){
    cerr<<"Aborting: file no good"<<endl;
  }
  t1.start();
  Graph graph = build_graph( file );
  t1.stop();

  cout<<"Reading graph took: "<<t1.getTime()<<endl;

  floyd_warshall(graph);
  cout<<"floyd_warshall done"<<endl;



  int last=-1;
  vector<int> costs;
  Channel<int> out;
  Channel<int> in;

  t1.start();
  for( auto &p : graph ){
    if( p.first == last ) continue;
    out.put(p.first);
  }
  out.close();

  int val;

  const int thread_count = 8;
  array<thread,thread_count> threads;
  for( int i=0; i<thread_count; i++){
    threads[i] = thread( do_dijk, graph, &out, &in);
  }

  for( int i=0; i<thread_count; i++){
    threads[i].join();
  }

  in.close();


  while( in.get( val )){
    costs.push_back(val);
  }

  int answer = *std::max_element(costs.begin(), costs.end());
  t1.stop();


  cout << "maximum minimum lenght path " << answer <<endl;
  cout << "answering took: "<<t1.getTime()<<endl;


  return 0;
}
