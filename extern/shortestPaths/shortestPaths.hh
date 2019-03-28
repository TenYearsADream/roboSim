#pragma once

#include <Eigen/Dense>
#include <vector>

int minKey(const Eigen::ArrayXi& key, const Eigen::ArrayXi& mstSet);
std::vector<size_t> primMST(const Eigen::MatrixXi& graph, size_t startNode = 0);