#include "com_mapbox_g2o_SparseOptimizer.h"

#include "core/block_solver.h"
#include "core/factory.h"
#include "core/optimization_algorithm_gauss_newton.h"
#include "core/optimization_algorithm_levenberg.h"
#include "core/sparse_optimizer.h"
#include "solvers/eigen/linear_solver_eigen.h"
#include "types/slam2d/vertex_se2.h"
#include "types/slam2d/vertex_point_xy.h"
#include "types/slam2d/edge_se2.h"
#include "types/slam2d/edge_se2_pointxy.h"
#include "types/slam2d/edge_pointxy.h"

#include <atomic>
#include <memory>
#include <unordered_map>
#include <iostream>

JNIEXPORT jlong JNICALL Java_com_mapbox_g2o_SparseOptimizer_nativeCreateOptimizationAlgo
  (JNIEnv *, jclass, jlong algoType, jlong solverType)
{
  std::cout << "nativeCreateOptimizationAlgo 1" << std::endl;
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1> > SlamBlockSolver;
  typedef g2o::LinearSolverEigen<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

  std::unique_ptr<SlamLinearSolver> solver;
  std::cout << "nativeCreateOptimizationAlgo 2" << std::endl;
  switch (solverType) {
    case 1:
      std::cout << "nativeCreateOptimizationAlgo 3" << std::endl;
      solver = std::make_unique<SlamLinearSolver>();
      std::cout << "nativeCreateOptimizationAlgo 4" << std::endl;
      break;
    default:
      std::cout << "nativeCreateOptimizationAlgo 5" << std::endl;
      std::cerr << "Bad solver type: " << solverType << std::endl;
      std::cout << "nativeCreateOptimizationAlgo 6" << std::endl;
      exit(1);
  }
  std::cout << "nativeCreateOptimizationAlgo 7" << std::endl;

  std::unique_ptr<SlamBlockSolver> blockSolver = std::make_unique<SlamBlockSolver>(std::move(solver));
  std::cout << "nativeCreateOptimizationAlgo 8" << std::endl;
  g2o::OptimizationAlgorithm* algo = nullptr;
  std::cout << "nativeCreateOptimizationAlgo 9" << std::endl;
  switch (algoType) {
    case 1:
      std::cout << "nativeCreateOptimizationAlgo 10" << std::endl;
      algo = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));
      std::cout << "nativeCreateOptimizationAlgo 11" << std::endl;
      break;
    case 2:
      std::cout << "nativeCreateOptimizationAlgo 12" << std::endl;
      algo = new g2o::OptimizationAlgorithmGaussNewton(std::move(blockSolver));
      std::cout << "nativeCreateOptimizationAlgo 13" << std::endl;
      break;
    default:
      std::cout << "nativeCreateOptimizationAlgo 14" << std::endl;
      std::cerr << "Bad algo type: " << algoType << std::endl;
      std::cout << "nativeCreateOptimizationAlgo 15" << std::endl;
      exit(1);
  }

  std::cout << "nativeCreateOptimizationAlgo 16" << std::endl;
  return reinterpret_cast<jlong>(algo);
}

JNIEXPORT jlong JNICALL Java_com_mapbox_g2o_SparseOptimizer_nativeCreateSparseOptimizer
  (JNIEnv *, jclass, jlong optimizationAlgo)
{
  std::cout << "nativeCreateSparseOptimizer 1" << std::endl;
  auto optimizer = std::make_unique<g2o::SparseOptimizer>();
  std::cout << "nativeCreateSparseOptimizer 2" << std::endl;
  auto optimizationAlgoPtr = reinterpret_cast<g2o::OptimizationAlgorithm*>(optimizationAlgo);
  std::cout << "nativeCreateSparseOptimizer 3" << std::endl;
  optimizer->setAlgorithm(optimizationAlgoPtr);
  std::cout << "nativeCreateSparseOptimizer 4" << std::endl;
  return reinterpret_cast<jlong>(optimizer.release());
}

JNIEXPORT void JNICALL Java_com_mapbox_g2o_SparseOptimizer_nativeDestroySparseOptimizer
  (JNIEnv *, jclass, jlong optimizer, jlong optimizationAlgo)
{
  std::cout << "nativeDestroySparseOptimizer 1" << std::endl;
  auto optimizationAlgoPtr = reinterpret_cast<g2o::OptimizationAlgorithm*>(optimizationAlgo);
  std::cout << "nativeDestroySparseOptimizer 2" << std::endl;
  delete optimizationAlgoPtr;

  std::cout << "nativeDestroySparseOptimizer 3" << std::endl;
  auto optimizerPtr = reinterpret_cast<g2o::SparseOptimizer*>(optimizer);
  std::cout << "nativeDestroySparseOptimizer 4" << std::endl;
  optimizerPtr->clear();
  std::cout << "nativeDestroySparseOptimizer 5" << std::endl;
  delete optimizerPtr;
  std::cout << "nativeDestroySparseOptimizer 6" << std::endl;
}

JNIEXPORT void JNICALL Java_com_mapbox_g2o_SparseOptimizer_nativeOptimize
  (JNIEnv *, jclass, jlong optimizer, jint iterations)
{
  std::cout << "nativeOptimize 1" << std::endl;
  auto optimizerPtr = reinterpret_cast<g2o::SparseOptimizer*>(optimizer);
  std::cout << "nativeOptimize 2" << std::endl;
  optimizerPtr->initializeOptimization();
  std::cout << "nativeOptimize 3" << std::endl;
  optimizerPtr->optimize(iterations);
  std::cout << "nativeOptimize 4" << std::endl;
}

JNIEXPORT void JNICALL Java_com_mapbox_g2o_SparseOptimizer_nativeAddVertexSE2
  (JNIEnv *, jclass, jlong optimizer, jlong vertexId, jdouble x, jdouble y, jdouble theta, jboolean isFixed)
{
  std::cout << "nativeAddVertexSE2 1" << std::endl;
  auto* vertex = new g2o::VertexSE2;
  vertex->setId(vertexId);
  vertex->setEstimate(g2o::SE2(x, y, theta));
  if (isFixed) {
    vertex->setFixed(true);
  }

  auto optimizerPtr = reinterpret_cast<g2o::SparseOptimizer*>(optimizer);
  optimizerPtr->addVertex(vertex);
  std::cout << "nativeAddVertexSE2 2" << std::endl;
}

JNIEXPORT void JNICALL Java_com_mapbox_g2o_SparseOptimizer_nativeAddVertexXY
  (JNIEnv *, jclass, jlong optimizer, jlong vertexId, jdouble x, jdouble y, jboolean isFixed)
{
  std::cout << "nativeAddVertexXY 1" << std::endl;
  auto* vertex = new g2o::VertexPointXY;
  vertex->setId(vertexId);
  vertex->setEstimate(Eigen::Vector2d(x, y));
  if (isFixed) {
    vertex->setFixed(true);
  }

  auto optimizerPtr = reinterpret_cast<g2o::SparseOptimizer*>(optimizer);
  optimizerPtr->addVertex(vertex);
  std::cout << "nativeAddVertexXY 2" << std::endl;
}

JNIEXPORT void JNICALL Java_com_mapbox_g2o_SparseOptimizer_nativeAddEdgeSE2SE2
  (JNIEnv* env, jclass, jlong optimizer, jlong fromVertex, jlong toVertex, jdouble relativeX, jdouble relativeY, jdouble relativeTheta, jdoubleArray infMatrix)
{
  std::cout << "nativeAddEdgeSE2SE2 1" << std::endl;
  auto optimizerPtr = reinterpret_cast<g2o::SparseOptimizer*>(optimizer);
  auto* edge = new g2o::EdgeSE2;
  edge->vertices()[0] = optimizerPtr->vertex(fromVertex);
  edge->vertices()[1] = optimizerPtr->vertex(toVertex);
  edge->setMeasurement(g2o::SE2(relativeX, relativeY, relativeTheta));

  jdouble infoElements[9];
  env->GetDoubleArrayRegion(infMatrix, 0, 9, infoElements);
  Eigen::Matrix3d information;
  information(0, 0) = infoElements[0];
  information(0, 1) = infoElements[1];
  information(0, 2) = infoElements[2];
  information(1, 0) = infoElements[3];
  information(1, 1) = infoElements[4];
  information(1, 2) = infoElements[5];
  information(2, 0) = infoElements[6];
  information(2, 1) = infoElements[7];
  information(2, 2) = infoElements[8];
  edge->setInformation(information);

  optimizerPtr->addEdge(edge);
  std::cout << "nativeAddEdgeSE2SE2 2" << std::endl;
}

JNIEXPORT void JNICALL Java_com_mapbox_g2o_SparseOptimizer_nativeAddEdgeSE2XY
  (JNIEnv* env, jclass, jlong optimizer, jlong fromVertex, jlong toVertex, jdouble relativeX, jdouble relativeY, jdoubleArray infMatrix)
{
  std::cout << "nativeAddEdgeSE2XY 1" << std::endl;
  auto optimizerPtr = reinterpret_cast<g2o::SparseOptimizer*>(optimizer);
  auto* edge = new g2o::EdgeSE2PointXY;
  edge->vertices()[0] = optimizerPtr->vertex(fromVertex);
  edge->vertices()[1] = optimizerPtr->vertex(toVertex);
  Eigen::Vector2d relativePosition(relativeX, relativeY);
  edge->setMeasurement(relativePosition);
  Eigen::Matrix2d information;
  jdouble infoElements[4];
  env->GetDoubleArrayRegion(infMatrix, 0, 4, infoElements);
  information(0, 0) = infoElements[0];
  information(0, 1) = infoElements[1];
  information(1, 0) = infoElements[2];
  information(1, 1) = infoElements[3];
  edge->setInformation(information);

  optimizerPtr->addEdge(edge);
  std::cout << "nativeAddEdgeSE2XY 2" << std::endl;
}

JNIEXPORT void JNICALL Java_com_mapbox_g2o_SparseOptimizer_nativeAddEdgeXYXY
  (JNIEnv* env, jclass, jlong optimizer, jlong fromVertex, jlong toVertex, jdouble relativeX, jdouble relativeY, jdoubleArray infMatrix)
{
  std::cout << "nativeAddEdgeXYXY 1" << std::endl;
  auto optimizerPtr = reinterpret_cast<g2o::SparseOptimizer*>(optimizer);
  auto* edge = new g2o::EdgePointXY;
  edge->vertices()[0] = optimizerPtr->vertex(fromVertex);
  edge->vertices()[1] = optimizerPtr->vertex(toVertex);
  Eigen::Vector2d relativePosition(relativeX, relativeY);
  edge->setMeasurement(relativePosition);
  Eigen::Matrix2d information;
  jdouble infoElements[4];
  env->GetDoubleArrayRegion(infMatrix, 0, 4, infoElements);
  information(0, 0) = infoElements[0];
  information(0, 1) = infoElements[1];
  information(1, 0) = infoElements[2];
  information(1, 1) = infoElements[3];
  edge->setInformation(information);

  optimizerPtr->addEdge(edge);
  std::cout << "nativeAddEdgeXYXY 2" << std::endl;
}

JNIEXPORT jdoubleArray JNICALL Java_com_mapbox_g2o_SparseOptimizer_nativeGetPose
  (JNIEnv *env, jclass, jlong optimizer, jlong vertexId)
{
  std::cout << "nativeGetPose 1" << std::endl;
  auto optimizerPtr = reinterpret_cast<g2o::SparseOptimizer*>(optimizer);
  auto vertex = optimizerPtr->vertex(vertexId);
  double estimate[3];
  vertex->getEstimateData(estimate);

  const size_t resultSize = 2;
  jdoubleArray result = env->NewDoubleArray(resultSize);
  env->SetDoubleArrayRegion(result, 0, resultSize, estimate );

  std::cout << "nativeGetPose 2" << std::endl;
  return result;
}
