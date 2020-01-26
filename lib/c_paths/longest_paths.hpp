#ifndef LONGEST_PATHS_HPP
#define LONGEST_PATHS_HPP

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/copy.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/labeled_graph.hpp>

#include <boost/graph/connected_components.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

#include <boost/functional/hash.hpp>
#include <algorithm>
#include <queue>
#include <unordered_set>
#include <vector>
#include <iostream>

using namespace boost;

namespace longest_paths {
  struct Point {
    double x, y;
    bool operator==(const Point q) const { return x == q.x && y == q.y; }
    friend std::size_t hash_value(Point const &p) {
      std::size_t seed = 0;
      boost::hash_combine(seed, p.x);
      boost::hash_combine(seed, p.y);
      return seed;
    }
  };

  struct Segment {
    Point src, trg;
    double weight, cap;
  };
}

using Node = longest_paths::Point;
using Graph = adjacency_list<
    vecS, vecS, undirectedS, Node,
    property<edge_weight_t, double, property<edge_capacity_t, double>>>;
using Vertex = Graph::vertex_descriptor;
using Edge = Graph::edge_descriptor;

using LGraph = labeled_graph<Graph, longest_paths::Point, hash_mapS>;

template <class Type> using Predicate = std::function<bool(Type)>;
using FGraph = filtered_graph<Graph, Predicate<Vertex>, Predicate<Edge>>;

template <class Edges> Graph from_edges(const Edges &edges) {
  LGraph lg;
  for (auto e : edges) {
    lg.add_vertex(e.src, e.src);
    lg.add_vertex(e.trg, e.trg);
    add_edge_by_label(e.src, e.trg, {e.weight, e.cap}, lg);
  }
  return lg.graph();
}

// returns a copy of the graph, only keeping edges with cap >= min_cap
Graph filter_capacity(Graph &g, double min_cap) {
  auto capacity_map = get(edge_capacity, g);
  Predicate<Edge> edges_with_cap = [min_cap, &capacity_map](const Edge e) {
    return get(capacity_map, e) >= min_cap;
  };
  auto g_cap_filtered = make_filtered_graph(g, edges_with_cap, keep_all());
  Graph g_cap;
  copy_graph(g_cap_filtered, g_cap);
  return g_cap;
}

// returns a copy of the graph, only keeping nodes with degree > 0
Graph filter_singleton_nodes(Graph &g) {
  Predicate<Vertex> nodes_with_edges = [&g](Vertex v) {
    return degree(v, g) > 0;
  };
  auto g_cap_filtered = make_filtered_graph(g, keep_all(), nodes_with_edges);
  Graph g_cap;
  copy_graph(g_cap_filtered, g_cap);
  return g_cap;
}

// returns a graph for every connected component of g
std::vector<Graph> into_components(Graph &g) {
  std::vector<size_t> components(num_vertices(g));
  size_t num_components =
      connected_components(g, boost::make_iterator_property_map(
                                  components.begin(), get(vertex_index, g)));

  std::vector<Graph> component_graphs;
  for (size_t i = 0; i < num_components; ++i) {
    Predicate<Vertex> nodes_of_comp = [i, &components](Vertex v) {
      return components[v] == i;
    };
    auto g_ci_filtered = make_filtered_graph(g, keep_all(), nodes_of_comp);
    Graph g_ci;
    copy_graph(g_ci_filtered, g_ci);
    component_graphs.push_back(g_ci);
  }
  return component_graphs;
}

std::vector<Vertex> component_vertices(Graph &g) {
  std::vector<size_t> components(num_vertices(g));
  size_t num_components =
      connected_components(g, boost::make_iterator_property_map(
                                  components.begin(), get(vertex_index, g)));
  std::vector<Vertex> representatives(num_components);
  for (auto vertex : make_iterator_range(vertices(g))) {
    representatives[components[vertex]] = vertex;
  }
  return representatives;
}

struct LastExaminedVisitor : public default_dijkstra_visitor {
  LastExaminedVisitor(Vertex *vd) : vd(vd) {}
  void examine_vertex(Vertex v, const Graph &) { *vd = v; }
  Vertex *vd;
};

template <class VertexIt>
Vertex furthest_node(const Graph &g, VertexIt vertices_begin,
                     VertexIt vertices_end) {
  std::vector<Vertex> pred(num_vertices(g));
  std::vector<double> dist(num_vertices(g));
  Vertex last_visited;
  dijkstra_shortest_paths(
      g, vertices_begin, vertices_end,
      make_iterator_property_map(pred.begin(), get(vertex_index, g)), // pred
      make_iterator_property_map(dist.begin(), get(vertex_index, g)), // dist
      get(edge_weight, g),                                            // weight
      get(vertex_index, g),                                           //
      std::less<double>(), closed_plus<double>(), // operations
      std::numeric_limits<double>::max(), 0.,     // operations
      LastExaminedVisitor(&last_visited));        // visitor
  return last_visited;
}

struct LastRootedExaminedVisitor : public default_dijkstra_visitor {
  LastRootedExaminedVisitor(std::vector<Vertex> &cf, std::vector<size_t> &root,
                            std::vector<Vertex> &pred)
      : component_furthest(cf), root(root), pred(pred) {}
  void examine_vertex(Vertex v, const Graph &) {
    auto v_pred = pred[v];
    auto v_root = root[v_pred];
    root[v] = v_root;
    component_furthest[v_root] = v;
  }
  std::vector<Vertex> &component_furthest;
  std::vector<size_t> &root;
  std::vector<Vertex> &pred;
};

template <class VertexIt>
std::vector<Vertex> furthest_rooted_nodes(const Graph &g,
                                          VertexIt vertices_begin,
                                          VertexIt vertices_end) {
  std::vector<Vertex> pred(num_vertices(g));
  std::vector<double> dist(num_vertices(g));
  std::vector<Vertex> component_furthest(vertices_begin, vertices_end);
  std::vector<size_t> root(num_vertices(g));

  size_t i = 0;
  for (auto vit = vertices_begin; vit != vertices_end; ++vit, ++i) {
    root[*vit] = i;
  }

  dijkstra_shortest_paths(
      g, vertices_begin, vertices_end,
      make_iterator_property_map(pred.begin(), get(vertex_index, g)), // pred
      make_iterator_property_map(dist.begin(), get(vertex_index, g)), // dist
      get(edge_weight, g),                                            // weight
      get(vertex_index, g),                                           //
      std::less<double>(), closed_plus<double>(),                 // operations
      std::numeric_limits<double>::max(), 0.,                     // operations
      LastRootedExaminedVisitor(component_furthest, root, pred)); // visitor
  return component_furthest;
}

// Vertex furthest_node(const Graph &g, Vertex v) {
//  if (degree(v, g) == 0)
//    return v;
//  return furthest_node(g, &v, &v + 1);
//}

std::unordered_set<Vertex> component_furthest_vertices(Graph &g) {
  auto vertices = component_vertices(g);
  //  std::unordered_set<Vertex> furthest_vertices;
  //  std::transform(vertices.begin(), vertices.end(),
  //                 std::inserter(furthest_vertices, furthest_vertices.end()),
  //                 [&g](Vertex v) { return furthest_node(g, v); });
  auto furthest_vertices =
      furthest_rooted_nodes(g, vertices.begin(), vertices.end());
  return std::unordered_set<Vertex>(furthest_vertices.begin(),
                                    furthest_vertices.end());
}

std::vector<Vertex> unpack_path(const std::vector<Vertex> &pred, Vertex v) {
  std::vector<Vertex> path;
  path.push_back(v);
  while (pred[v] != v) {
    v = pred[v];
    path.push_back(v);
  }
  return path;
}

template <class G, class VertexIt>
std::pair<double, std::vector<Vertex>>
longest_path_from(const G &g, VertexIt vertices_begin, VertexIt vertices_end) {
  std::vector<typename G::vertex_descriptor> pred(num_vertices(g));
  std::vector<double> dist(num_vertices(g));
  typename G::vertex_descriptor last_visited;
  dijkstra_shortest_paths(
      g, vertices_begin, vertices_end,
      make_iterator_property_map(pred.begin(), get(vertex_index, g)), // pred
      make_iterator_property_map(dist.begin(), get(vertex_index, g)), // dist
      get(edge_weight, g),                                            // weight
      get(vertex_index, g),                                           //
      std::less<double>(), closed_plus<double>(), // operations
      std::numeric_limits<double>::max(), 0.,     // operations
      LastExaminedVisitor(&last_visited));        // visitor
  auto path = unpack_path(pred, last_visited);
  auto path_dist = dist[last_visited];
  return {path_dist, path};
}

std::vector<std::vector<Vertex>>
find_distinct_paths(Graph &graph, double aspect, double STEP, size_t k = 10) {
  std::vector<std::vector<Vertex>> paths;
  if (num_edges(graph) == 0)
    return paths;

  double mincap = get(edge_capacity, graph, *edges(graph).first);
  double maxcap = get(edge_capacity, graph, *edges(graph).first);
  for (auto e : make_iterator_range(edges(graph))) {
    double cap = get(edge_capacity, graph, e);
    mincap = std::min(mincap, cap);
    maxcap = std::max(maxcap, cap);
  }

  for (double CAP = maxcap;
       (CAP >= (mincap / STEP) || paths.size()==0) && paths.size() < k; CAP /= STEP) {

    auto cap_graph = filter_capacity(graph, CAP);
    auto node_set = component_furthest_vertices(cap_graph);
    while (paths.size() < k) {
      auto [dist1, path1] =
          longest_path_from(cap_graph, node_set.begin(), node_set.end());
      auto [dist, path] = 
          longest_path_from(cap_graph, path1.begin(), path1.begin() + 1);

      if (dist <= CAP / aspect)
        break;

      node_set.insert(path.begin(), path.end());
      paths.push_back(path);
    };
  }

  return paths;
}

struct LongestPath {
    Vertex u, v;
    double dist;
    std::vector<Vertex> path;

    LongestPath() : u(0), v(0), dist(0), path() {}

    LongestPath(Vertex u, Vertex v, double dist, std::vector<Vertex> path)
    : u(u), v(v), dist(dist), path(path) {
        // if v is the minimum vertex id, swap direction
        if (v < u) {
            this->u = v;
            this->v = u;
            std::reverse(this->path.begin(), this->path.end());
        }
        assert(this->path.at(0) == this->u);
        assert(this->path.at(path.size()-1) == this->v);
    }

    std::string to_string() {
        std::string res = "";
        res += "Path from " + std::to_string(u) + " to " + std::to_string(v) + " has distance " + std::to_string(dist) + ":\n";
        for (auto& v : path) {
            res += std::to_string(v) + ",";
        }
        res.pop_back();
        return res;
    }

    friend std::size_t hash(LongestPath const &lp) {
      std::size_t seed = 0;
      boost::hash_combine(seed, lp.u);
      boost::hash_combine(seed, lp.v);
      boost::hash_combine(seed, lp.dist);
      return seed;
    }
};

inline bool operator<(const LongestPath& lhs, const LongestPath& rhs) {
    return lhs.dist < rhs.dist;
}
inline bool operator>(const LongestPath& lhs, const LongestPath& rhs){
    return rhs < lhs;
}
inline bool operator<=(const LongestPath& lhs, const LongestPath& rhs) {
    return !(lhs > rhs);
}
inline bool operator>=(const LongestPath& lhs, const LongestPath& rhs) {
    return !(lhs < rhs);
}

inline bool operator==(const LongestPath& lhs, const LongestPath& rhs) {
    return (lhs.u == rhs.u && lhs.v == rhs.v) || (lhs.u == rhs.v && lhs.v == rhs.u);
}

inline bool operator!=(const LongestPath& lhs, const LongestPath& rhs) {
    return !(lhs == rhs);
}

struct LPHash {
    std::size_t operator() (LongestPath const& lp) const noexcept {
        std::size_t seed = 0;
        boost::hash_combine(seed, lp.u);
        boost::hash_combine(seed, lp.v);
        boost::hash_combine(seed, lp.dist);
        return seed;
    }
};

struct ExaminedLeafVisitor : public default_dijkstra_visitor {
    ExaminedLeafVisitor(std::vector<Vertex>& lV) : leafVertices(lV) {}
    void examine_vertex(Vertex v, const Graph &g) {
        if (degree(v, g) == 1) {
            leafVertices.push_back(v);
        }
    }
    std::vector<Vertex>& leafVertices;
};

std::vector<LongestPath>
longest_paths_from(const Graph &g, Vertex &from, std::vector<Vertex>& visited) {
    std::vector<Vertex> leafVertices;
    std::vector<Vertex> pred(num_vertices(g));
    std::vector<double> dist(num_vertices(g));
    dijkstra_shortest_paths(
        g, from,
        make_iterator_property_map(pred.begin(), get(vertex_index, g)), // pred
        make_iterator_property_map(dist.begin(), get(vertex_index, g)), // dist
        get(edge_weight, g),                                            // weight
        get(vertex_index, g),                                           //
        std::less<double>(), closed_plus<double>(), // operations
        std::numeric_limits<double>::max(), 0.,     // operations
        ExaminedLeafVisitor(leafVertices)
    );

    std::vector<LongestPath> longest_paths;
    for (auto& to : leafVertices) {
        if (std::find(visited.begin(), visited.end(), to) != visited.end()) {
            auto path = unpack_path(pred, to);
            longest_paths.push_back({to, from, dist[to], path});
        }
    }

    return longest_paths;
}

bool sane(std::vector<Vertex>& p, Graph& g) {
    for (auto it1 = p.begin(), it2 = p.begin()+1, end = p.end(); it2 != end; it1 = it2++) {
        assert(edge(*it1, *it2, g).second);
    }

    return true;
}

std::vector<Vertex> leaf_vertices(Graph &g) {
  std::vector<Vertex> res;
  Graph::vertex_iterator vi, vend;
  for (tie(vi, vend) = vertices(g); vi != vend; ++vi) {
      if (boost::degree(*vi, g) == 1) {
          res.push_back(*vi);
      }
  }
  return res;
}

// returns a copy of the graph, only keeping nodes with degree > 1
Graph simplifyGraph(const Graph &g, size_t iterations) {
    Graph g_res;
    copy_graph(g, g_res);
    for (size_t i = 0; i < iterations; ++i) {
        Graph g_tmp;
        copy_graph(g_res, g_tmp);
        Predicate<Vertex> nodes_with_deg2 = [&g_tmp](Vertex v) {
            return degree(v, g_tmp) > 1;
        };
        auto g_filtered = make_filtered_graph(g_tmp, keep_all(), nodes_with_deg2);
        g_res.clear();
        copy_graph(g_filtered, g_res);
    }
    return g_res;
}

std::vector<LongestPath> find_initial_leaf_paths(Graph &g, size_t k)  {
    auto leafs = leaf_vertices(g);
    // candidates is a min queue w.r.t. path length
    std::priority_queue<LongestPath, std::vector<LongestPath>, std::greater<LongestPath>> candidates;
    std::vector<Vertex> visited;
    double currentMinDist = 0;
    for (auto& from: leafs) {
        visited.push_back(from);
        auto paths = longest_paths_from(g, from, visited);
        for (auto p : paths) {
            if (candidates.size() < k || p.dist > currentMinDist) {
                candidates.push(p);
                if (candidates.size() > k) {
                    candidates.pop();   // pop the shortest of the 50 candidate paths
                    currentMinDist = candidates.top().dist;
                }
            }
            assert(candidates.size() <= k);
        }
    }
    std::vector<LongestPath> paths;
    while (candidates.size() > 0) {
        paths.push_back(candidates.top());
        candidates.pop();
    }

    return paths;
}

std::vector<std::vector<Vertex>>
find_longest_paths(Graph &graph, size_t k = 50) {
    std::vector<std::vector<Vertex>> res;
    if (num_edges(graph) == 0) {
        std::cout << "Skeleton did not contain any edges!" << std::endl;
        return res;
    }

    size_t iterCount = 5;
    graph = simplifyGraph(graph, iterCount);

    auto initial_paths = find_initial_leaf_paths(graph, k);

    std::priority_queue<LongestPath> paths;
    for (auto& path : initial_paths) {
        //std::cout << path.to_string() << std::endl;
        sane(path.path, graph);
        paths.push(path);
    }

    auto edge_dist = get(edge_weight, graph);

    for (size_t i = 0; i < k; ++i) {
        if (paths.size() <= 0) {
            break;
        }
        auto longest = paths.top();
        paths.pop();
        res.push_back(longest.path);
        if (longest.path.size() > 2) {
            sane(longest.path, graph);
            size_t subpath_length = longest.path.size() - 1;
            // subart starting at the second node
            auto t = boost::edge(longest.u, longest.path.at(1), graph);
            assert (t.second);
            auto edgeDist = get(edge_dist, t.first);
            std::vector<Vertex> subpath(longest.path.begin()+1, longest.path.end());
            sane(subpath, graph);
            paths.push(LongestPath(
                longest.path.at(1), longest.v, longest.dist - edgeDist, subpath
            ));
            // subart ending at the second last node
            t = boost::edge(longest.path.at(subpath_length - 1), longest.v, graph);
            assert (t.second);
            edgeDist = get(edge_dist, t.first);
            subpath = std::vector<Vertex>(longest.path.begin(), longest.path.end()-1);
            sane(subpath, graph);
            paths.push(LongestPath(
                longest.u, longest.path.at(subpath_length - 1),
                longest.dist - edgeDist, subpath
            ));
        }
    }

    return res;
}

#endif /* LONGEST_PATHS_HPP */
