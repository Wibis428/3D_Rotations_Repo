// 3d_rotations.cpp
#include <iostream>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>

using Eigen::Matrix3f;
using Eigen::Vector3f;
using std::cout;
using std::endl;

Vector3f xHat(1, 0, 0);
Vector3f yHat(0, 1, 0);
Vector3f zHat(0, 0, 1);

Vector3f rotatePointAboutAxis(Vector3f point, Vector3f axis, float angle) {
  // check if we've gotten a valid rotation axis
  float axisLength = axis.norm();
  if (axisLength == 0) {
    return point;
  }

  // step one: rotate space about x axis so that the rotation vector lies in the xz plane
  // this is done by projecting the rotation vector into the yz plane,
  // finding the angle between that projection and the z axis,
  // then rotating space about the x axis using that angle.

  Matrix3f rotationX;
  Vector3f yzProjection(0, axis.y(), axis.z());
  float yzProjectionLength = yzProjection.norm();

  if (yzProjectionLength != 0) {
    float sinTheta = yzProjection.y() / yzProjectionLength;
    float cosTheta = yzProjection.z() / yzProjectionLength;
    /*
          Z    /\
          /\   / <--- projection
          |   /
          |  /
          |---------> Y
       We want theta to be positive on one side of the Z axis, and negative on the other side
       of the Z axis. That's exactly the property that the X axis has when looking at 2d rotations.
       If you take this pic of the projection, flip the whole pic 180 degrees over Z
       then rotate the pic to the right by 90 degrees, you can see why the given expressions
       for sinTheta and cosTheta correspond to the theta with the correct sign.
    */

    rotationX << 1,     0,        0,
                 0, cosTheta, -sinTheta,
                 0, sinTheta, cosTheta;
  }
  else {
    // rotation axis already lies on the x axis
    rotationX.setIdentity();
  }

  // update coordinates of axis so we can use it in the next step
  axis = rotationX * axis;

  // step two: rotate about y axis so that the rotation vector lies on the z axis
  // you can use the same procedure as outlined above to see why these equations
  // correspond to the correct theta.
  // Note that this time, we'll also have to invert the x axis, in order
  // to comply with the direction in the xz plane that corresponds
  // to positive rotations about the y axis.

  Matrix3f rotationY;
  float sinTheta = (-1 * axis.x()) / axisLength;
  float cosTheta = axis.z() / axisLength;

  rotationY << cosTheta, 0, sinTheta,
                  0,     1,     0,
              -sinTheta, 0, cosTheta;

  // step three: now the rotation axis should be on the z-axis, meaning we can just do a regular rotation
  Matrix3f rotationZ;
  rotationZ << cos(angle), -sin(angle), 0,
               sin(angle),  cos(angle), 0,
                   0,            0,     1;

  // step four: compose all rotations and inverses together to get the final result!
  point = rotationX.inverse() * rotationY.inverse() * rotationZ * rotationY * rotationX * point;
  return point;
}

Vector3f rotatePointAboutAxis(Vector3f point, Vector3f p1, Vector3f p2, float angle) {
  // translate everything so that p1 is the origin
  Vector3f rotationAxis = p2 - p1;
  point = point - p1;

  // calculate the rotation
  point = rotatePointAboutAxis(point, rotationAxis, angle);

  // translate back to the original position
  point = point + p1;
  return point;
}



int main() {
  float k_pi = 3.141592653589793;
  float k_tau = 2.0 * k_pi;
  Vector3f p(1, 0, 0);
  Vector3f r(1, 1, 1);
  float rAngle = -k_tau / 3.0;

  Vector3f resultPoint = rotatePointAboutAxis(p, r, rAngle);
  cout << "result:\n" << resultPoint << endl;

  return 0;
}
