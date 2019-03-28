// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2015 Olga Diamanti <olga.diam@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.

#ifndef IGL_DIJKSTRA
#define IGL_DIJKSTRA

#ifdef IGL_INLINE
#undef IGL_INLINE
#endif

#define IGL_STATIC_LIBRARY abc

#ifdef IGL_STATIC_LIBRARY
#  define IGL_INLINE inline
#else
#  define IGL_INLINE
#endif

#include <Eigen/Dense>
#include <vector>
#include <set>

namespace igl {

  // Dijstra's algorithm for shortest paths, with multiple targets.
  // Adapted from http://rosettacode.org/wiki/Dijkstra%27s_algorithm .
  //
  // Inputs:
  //   source           index of source vertex
  //   targets          target vector set
  //   VV               #V list of lists of incident vertices (adjacency list), e.g.
  //                    as returned by igl::adjacency_list
  //
  // Output:
  //   min_distance     #V by 1 list of the minimum distances from source to all vertices
  //   previous         #V by 1 list of the previous visited vertices (for each vertex) - used for backtracking
  //
  template <typename IndexType, typename DerivedD, typename DerivedP>
  IGL_INLINE int dijkstra_compute_paths(const IndexType &source,
                                        const std::set<IndexType> &targets,
                                        const std::vector<std::vector<IndexType> >& VV,
                                        Eigen::PlainObjectBase<DerivedD> &min_distance,
                                        Eigen::PlainObjectBase<DerivedP> &previous);

  // Backtracking after Dijstra's algorithm, to find shortest path.
  //
  // Inputs:
  //   vertex           vertex to which we want the shortest path (from same source as above)
  //   previous         #V by 1 list of the previous visited vertices (for each vertex) - result of Dijkstra's algorithm
  //
  // Output:
  //   path             #P by 1 list of vertex indices in the shortest path from source to vertex
  //
  template <typename IndexType, typename DerivedP>
  IGL_INLINE void dijkstra_get_shortest_path_to(const IndexType &vertex,
                                                const Eigen::PlainObjectBase<DerivedP> &previous,
                                                std::vector<IndexType> &path);
};



// Explicit template specialization
// template int igl::dijkstra_compute_paths<int, Eigen::Matrix<double, -1, 1, 0, -1, 1>, Eigen::Matrix<int, -1, 1, 0, -1, 1> >(int const&, std::set<int, std::less<int>, std::allocator<int> > const&, std::vector<std::vector<int, std::allocator<int> >, std::allocator<std::vector<int, std::allocator<int> > > > const&, Eigen::PlainObjectBase<Eigen::Matrix<double, -1, 1, 0, -1, 1> >&, Eigen::PlainObjectBase<Eigen::Matrix<int, -1, 1, 0, -1, 1> >&);
// template void igl::dijkstra_get_shortest_path_to<int, Eigen::Matrix<int, -1, 1, 0, -1, 1> >(int const&, Eigen::PlainObjectBase<Eigen::Matrix<int, -1, 1, 0, -1, 1> > const&, std::vector<int, std::allocator<int> >&);

#endif /* defined(IGL_DIJKSTRA) */