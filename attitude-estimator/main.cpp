#include <cmath>
#include <iostream>
#include "attitude_estimator.h"

using namespace stateestimation;
using namespace std;

int main() {
  std::cout << "att-est: starting..." << std::endl;

  AttitudeEstimator est;
  while (1) {
    double dt = 0.02;
    double a[3] = {};
    double g[3] = {};
    double m[3] = {};

    est.setQLTime(5);

    cin >> a[0] >> a[1] >> a[2] >> g[0] >> g[1] >> g[2];

    for (int i = 0; i < 3; i++) {
      g[i] /= 131.0;         // rescale MPU-6050 raw values
      g[i] *= 0.0174532925;  // deg to rad
    }
    // we don't have to rescale accel data because AttitudeEstimator doesn't
    // care read the documentation

    est.update(dt, g[0], g[1], g[2], a[0], a[1], a[2], m[0], m[1], m[2]);

    double q[4];
    est.getAttitude(q);
    // cout << acos(q[0]) * 2 * 57.2957795 << "\t" << q[1] << "\t" << q[2] <<
    // "\t" << q[3] << "\t" << endl;
    cout << q[0] << "\t" << q[1] << "\t" << q[2] << "\t" << q[3] << "\t"
         << endl;
  }

  std::cout << "att-est: exiting..." << std::endl;
  return 0;
}