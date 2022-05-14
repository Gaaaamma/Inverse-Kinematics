#include "kinematics.h"

#include <iostream>
#include <algorithm>
#include <stack>
#include <Eigen/SVD>  
#include <Eigen/Dense>

#include "utils.h"

//using Eigen::MatrixXf;    
using namespace Eigen;    
using namespace Eigen::internal;    
using namespace Eigen::Architecture;  

void forwardKinematics(const Posture& posture, Bone* bone) {
  // TODO (FK)
  // Same as HW2, but have some minor change
  // Hint:
  //   1. If you don't use `axis` in this function, you can copy-paste your code
  // Note:
  //   1. bone.axis becomes quaternion instead of vector3f

  std::stack<Bone*> traverseStack;
  traverseStack.push(bone);
  int alreadyTraverse[31] = {};
  while (!traverseStack.empty()) {
    if (traverseStack.top()->child != nullptr && alreadyTraverse[traverseStack.top()->child->idx] == 0) {
      if(alreadyTraverse[traverseStack.top()->idx] ==0){
        // Since we need to modify parent first
        alreadyTraverse[traverseStack.top()->idx] = 1;
        //std::cout << "Traverse: " << traverseStack.top()->idx << "\n";
        // 2. Read local coordinate data from posture first
        Eigen::Quaternionf localRotation = posture.rotations[traverseStack.top()->idx];
        Eigen::Vector3f localTranslation = posture.translations[traverseStack.top()->idx];

        // 3. Set 3 variable value
        // bone->rotation ;
        // bone->startPosition ;
        // bone->endPosition ;
        if(traverseStack.top()->parent == nullptr){
          // root
          traverseStack.top()->startPosition =localTranslation;
          traverseStack.top()->endPosition = traverseStack.top()->startPosition;
          traverseStack.top()->rotation = localRotation;
        }else{
          // not root
          traverseStack.top()->rotation = traverseStack.top()->parent->rotation * traverseStack.top()->rotationParentCurrent * localRotation;
          traverseStack.top()->startPosition =traverseStack.top()->parent->endPosition ;
          traverseStack.top()->endPosition =
            traverseStack.top()->startPosition +(traverseStack.top()->rotation * traverseStack.top()->direction * traverseStack.top()->length);
        }
      }
      traverseStack.push(traverseStack.top()->child);
    } else {
      // No child or Already check its child -> Check itself.
      if (alreadyTraverse[traverseStack.top()->idx] == 0) {
        alreadyTraverse[traverseStack.top()->idx] = 1;
        //std::cout << "Traverse: " << traverseStack.top()->idx << "\n";
        // 2. Read local coordinate data from posture first
        Eigen::Quaternionf localRotation = posture.rotations[traverseStack.top()->idx];
        Eigen::Vector3f localTranslation = posture.translations[traverseStack.top()->idx];

        // 3. Modify 3 variable value
        // bone->rotation ;
        // bone->startPosition ;
        // bone->endPosition ;
        traverseStack.top()->rotation = traverseStack.top()->parent->rotation * traverseStack.top()->rotationParentCurrent * localRotation;
        traverseStack.top()->startPosition =traverseStack.top()->parent->endPosition ;
        traverseStack.top()->endPosition = 
          traverseStack.top()->startPosition +(traverseStack.top()->rotation* traverseStack.top()->direction * traverseStack.top()->length );
      }

      // Check sibling
      if (traverseStack.top()->sibling != nullptr && alreadyTraverse[traverseStack.top()->sibling->idx] == 0) {
        traverseStack.push(traverseStack.top()->sibling);
      } else {
        traverseStack.pop();
      }

    }
  }
}

Eigen::VectorXf leastSquareSolver(const Eigen::Matrix3Xf& jacobian, const Eigen::Vector3f& target) {
  // TODO (find x which min(| jacobian * x - target |))
  // Hint:
  //   1. Linear algebra - least squares solution
  //   2. https://en.wikipedia.org/wiki/Moore%E2%80%93Penrose_inverse#Construction
  // Note:
  //   1. SVD or other pseudo-inverse method is useful
  //   2. Some of them have some limitation, if you use that method you should check it.
  std::cout << "leastSquareSolver is called\n";
  JacobiSVD<MatrixXf> svd(jacobian, ComputeThinU | ComputeThinV);
  std::cout << "leastSquareSolver p1\n";

  Matrix3f U = svd.matrixU();
  std::cout << "leastSquareSolver p2\n";

  MatrixXf V = svd.matrixV();
  std::cout << "leastSquareSolver p3\n";

  Matrix3Xf S = U.inverse() * jacobian * V.transpose().inverse();
  std::cout << "leastSquareSolver p4\n";

  // psudo inverse J+ = V * Summation+ * U.transpose()
  MatrixX3f JPlus = V * S.transpose() * U.transpose();
  std::cout << "leastSquareSolver p5\n";

  Eigen::VectorXf solution(jacobian.cols());
  solution.setZero();
  std::cout << "leastSquareSolver p6\n";

  solution = JPlus * target;
  std::cout << "leastSquareSolver p7\n";

  return solution;
}

void inverseKinematics(const Eigen::Vector3f& target, Bone* start, Bone* end, Posture& posture) {
  constexpr int maxIterations = 10000;
  constexpr float epsilon = 1E-3f;
  constexpr float step = 0.1f;
  // Since bone stores in bones[i] that i == bone->idx, we can use bone - bone->idx to find bones[0] which is root.
  Bone* root = start - start->idx;
  std::vector<Bone*> boneList;
  // TODO
  // Hint:
  //   1. Traverse from end to start is easier than start to end (since there is only 1 parent)
  //   2. If start bone is not reachable from end. Go to root first.
  // Note:
  //   1. Both start and end should be in the list

  // Write your code here.
  Bone* traverser = end;
  while (traverser->idx != start->idx && traverser->idx != root->idx) {
    // while will stop when traverser is on startBone or rootBone
    boneList.push_back(traverser);
    // traverse its parent
    traverser = traverser->parent;
  }
  // startBone or rootBone also need to push to boneList
  boneList.push_back(traverser);

  size_t boneNum = boneList.size();
  Eigen::Matrix3Xf jacobian(3, 3 * boneNum);
  jacobian.setZero();
  Eigen::Vector3f xUnit = Eigen::Vector3f(1, 0, 0);
  Eigen::Vector3f yUnit = Eigen::Vector3f(0, 1, 0);
  Eigen::Vector3f zUnit = Eigen::Vector3f(0, 0, 1);

  for (int i = 0; i < maxIterations; ++i) {
    forwardKinematics(posture, root);
    // TODO (compute jacobian)
    //   1. Compute jacobian columns
    //   2. Compute dTheta
    // Hint:
    //   1. You should not put rotation in jacobian if it doesn't have that DoF.
    //   2. jacobian.col(/* some column index */) = /* jacobian column */
    //   3. Call leastSquareSolver to compute dTheta

    // Write your code here.
    
    for (int i = 0; i < boneNum; i++) {
      jacobian.col(i * 3) = (boneList[i]->rotation * xUnit).cross(end->endPosition - boneList[i]->startPosition);
      jacobian.col(i * 3 + 1) = (boneList[i]->rotation * yUnit).cross(end->endPosition - boneList[i]->startPosition);
      jacobian.col(i * 3 + 2) = (boneList[i]->rotation * zUnit).cross(end->endPosition - boneList[i]->startPosition);
    }
    
    Eigen::VectorXf dTheta = leastSquareSolver(jacobian,target - end->endPosition);
    std::cout << "dTheta is end\n";
    for (size_t j = 0; j < boneNum; j++) {
      const auto& bone = *boneList[j];
      // TODO (update rotation)
      //   1. Update posture's eulerAngle using deltaTheta
      // Hint:
      //   1. Use posture.eulerAngle to get posture's eulerAngle
      //   2. All angles are in radians.
      //   3. You can ignore rotation limit of the bone.
      // Bonus:
      //   1. You cannot ignore rotation limit of the bone.

      // Write your code here.

      posture.rotations[bone.idx] = Eigen::AngleAxisf(posture.eulerAngle[bone.idx][2], Eigen::Vector3f::UnitZ()) *
                                    Eigen::AngleAxisf(posture.eulerAngle[bone.idx][1], Eigen::Vector3f::UnitY()) *
                                    Eigen::AngleAxisf(posture.eulerAngle[bone.idx][0], Eigen::Vector3f::UnitX());
    }
  }
}
