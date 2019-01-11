/* Point representation
 */

#pragma once

struct Point {
  Point(double x, double y) {
    x_ = x;
    y_ = y;
  }
  double x_;
  double y_;
};
