#include "com_mapbox_g2o_SparseOptimizer.h"

#include "core/block_solver.h"
#include "core/factory.h"
#include "core/optimization_algorithm_gauss_newton.h"
#include "core/optimization_algorithm_levenberg.h"
#include "core/sparse_optimizer.h"
#include "solvers/eigen/linear_solver_eigen.h"
#include "types/slam2d/vertex_se2.h"
#include "types/slam2d/edge_se2.h"
#include "types/slam2d/edge_se2_pointxy.h"

#include <atomic>
#include <memory>
#include <unordered_map>
#include <iostream>

JNIEXPORT jlong JNICALL Java_com_mapbox_g2o_SparseOptimizer_nativeCreateOptimizationAlgo
  (JNIEnv *, jclass, jlong algoType, jlong solverType)
{
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1> > SlamBlockSolver;
  typedef g2o::LinearSolverEigen<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

  std::unique_ptr<SlamLinearSolver> solver;
  switch (solverType) {
    case 1:
      solver = std::make_unique<SlamLinearSolver>();
      break;
    default:
      std::cerr << "Bad solver type: " << solverType << std::endl;
      exit(1);
  }

  std::unique_ptr<SlamBlockSolver> blockSolver = std::make_unique<SlamBlockSolver>(std::move(solver));
  g2o::OptimizationAlgorithm* algo = nullptr;
  switch (algoType) {
    case 1:
      algo = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));
      break;
    case 2:
      algo = new g2o::OptimizationAlgorithmGaussNewton(std::move(blockSolver));
      break;
    default:
      std::cerr << "Bad algo type: " << algoType << std::endl;
      exit(1);
  }

  return reinterpret_cast<jlong>(algo);
}

JNIEXPORT jlong JNICALL Java_com_mapbox_g2o_SparseOptimizer_nativeCreateSparseOptimizer
  (JNIEnv *, jclass, jlong optimizationAlgo)
{
  auto optimizer = std::make_unique<g2o::SparseOptimizer>();
  auto optimizationAlgoPtr = reinterpret_cast<g2o::OptimizationAlgorithm*>(optimizationAlgo);
  optimizer->setAlgorithm(optimizationAlgoPtr);
  return reinterpret_cast<jlong>(optimizer.release());
}

JNIEXPORT void JNICALL Java_com_mapbox_g2o_SparseOptimizer_nativeDestroySparseOptimizer
  (JNIEnv *, jclass, jlong optimizer, jlong optimizationAlgo)
{
  auto optimizationAlgoPtr = reinterpret_cast<g2o::OptimizationAlgorithm*>(optimizationAlgo);
  delete optimizationAlgoPtr;

  auto optimizerPtr = reinterpret_cast<g2o::SparseOptimizer*>(optimizer);
  optimizerPtr->clear();
  delete optimizerPtr;
}

JNIEXPORT void JNICALL Java_com_mapbox_g2o_SparseOptimizer_nativeOptimize
  (JNIEnv *, jclass, jlong optimizer, jint iterations)
{
  auto optimizerPtr = reinterpret_cast<g2o::SparseOptimizer*>(optimizer);
  optimizerPtr->initializeOptimization();
  optimizerPtr->optimize(iterations);
}

JNIEXPORT void JNICALL Java_com_mapbox_g2o_SparseOptimizer_nativeAddVertex
  (JNIEnv *, jclass, jlong optimizer, jlong vertexId, jdouble x, jdouble y, jdouble theta, jboolean isFixed)
{
  auto* vertex = new g2o::VertexSE2;
  vertex->setId(vertexId);
  vertex->setEstimate(g2o::SE2(x, y, theta));
  if (isFixed) {
    vertex->setFixed(true);
  }

  auto optimizerPtr = reinterpret_cast<g2o::SparseOptimizer*>(optimizer);
  optimizerPtr->addVertex(vertex);
}

JNIEXPORT void JNICALL Java_com_mapbox_g2o_SparseOptimizer_nativeAddEdgeSE2
  (JNIEnv* env, jclass, jlong optimizer, jlong fromVertex, jlong toVertex, jdouble relativeX, jdouble relativeY, jdouble relativeTheta, jdoubleArray infMatrix)
{
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
}

JNIEXPORT void JNICALL Java_com_mapbox_g2o_SparseOptimizer_nativeAddEdgeXY
  (JNIEnv* env, jclass, jlong optimizer, jlong fromVertex, jlong toVertex, jdouble relativeX, jdouble relativeY, jdoubleArray infMatrix)
{
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
}

JNIEXPORT jdoubleArray JNICALL Java_com_mapbox_g2o_SparseOptimizer_nativeGetPose
  (JNIEnv *env, jclass, jlong optimizer, jlong vertexId)
{
  auto optimizerPtr = reinterpret_cast<g2o::SparseOptimizer*>(optimizer);
  auto vertex = optimizerPtr->vertex(vertexId);
  double estimate[3];
  vertex->getEstimateData(estimate);

  const size_t resultSize = 2;
  jdoubleArray result = env->NewDoubleArray(resultSize);
  env->SetDoubleArrayRegion(result, 0, resultSize, estimate );

  return result;
}
