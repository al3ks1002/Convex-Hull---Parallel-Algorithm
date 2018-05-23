#include <cmath>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <cassert>
#include <cstdlib>
#include <iomanip>
#include <thread>
#include <chrono>

const double kEps = 1e-6;

// why fabs (which is in C) and not std::abs(). true, abs from C only works on
// integers, however you're using C++ so there's no need to use fabs
bool Equals(double a, double b) {
  return fabs(a - b) < kEps;
}

struct Point {
  Point() : x(0.0), y(0.0) {}

  // unused constructor
  Point(double x, double y) : x(x), y(y) {}

  // when you're tired of using "a" and "b" so you start using "b" and "c"
  double CrossProduct(const Point &b, const Point &c) const {
    return (b.x - x) * (c.y - y) - (c.x - x) * (b.y - y);
  }

  // s/fabs/std::abs
  // when you're tired of using "a" and "b" so you start using "b" and "c"
  // distance to what? consider making a struct/class named "Line"
  // a function named DistanceTo(Line) is so much more intuitive
  double Distance(const Point &b, const Point &c) const {
    // maybe create separate function for euclidean distance;
    // create intermediate variables to clarify computation
    return fabs((c.y - b.y) * x - (c.x - b.x) * y + c.x * b.y - c.y * b.x) /
        sqrt((c.y - b.y) * (c.y - b.y) + (c.x - b.x) * (c.x - b.x));
  }

  // consider making a class/struct for Triangle // TODO
  bool InTriangle(const Point &a, const Point &b, const Point &c) const {
    bool c1 = CrossProduct(a, b) < 0.0;
    bool c2 = CrossProduct(b, c) < 0.0;
    bool c3 = CrossProduct(c, a) < 0.0;

    return (c1 == c2) && (c2 == c3);
  }

  bool operator==(const Point &b) const {
    return Equals(x, b.x) && Equals(y, b.y);
  }

  bool operator!=(const Point &b) const {
    return !(*this == b);
  }

  // not sure of this as I wasn't patient enough to search the google code style
  // thoroughly; weren't data members supposed to be declared on separate lines?
  double x, y;
};
// final comment on "Point"; is there any motivation behind declaring this as a
// struct instead of a class?
// I personally see structs as carriers of data which do not have much
// functionality. Point doesn't have many function but I feel like it should
// have been a class.

class ConvexHullSolver {
 public:
  explicit ConvexHullSolver(const std::string &input_file) {
    ReadPoints(input_file);
  }

  // would have been so much prettier for std::vector<Point> to have been a
  // class/struct called ConvexHull
  // I admit it would have been a pretty useless wrapper, but I think it would
  // have increased the readability
  std::vector<Point> SolveSequential() {
    return FindQuickHull(1);
  }

  std::vector<Point> SolveParallel(const int num_threads) {
    return FindQuickHull(num_threads);
  }

 private:
  void ReadPoints(const std::string &input_file) {
    std::ifstream fin(input_file);

    int num_points;
    fin >> num_points;

    for (int i = 0; i < num_points; i++) {
      double x, y;
      fin >> x >> y;
      points_.emplace_back(x, y);
    }
    //is the file automatically closed when "fin" is removed from the stack?
  }

  // consider splitting this function in several ones
  // std::pair<Point, Point> GetHighAndLow(points)
  // std::pair<std::vector<Point>, std::vector<Point>> SplitInSides(
  //                                             points, high_point, low_point)
  // recursive part
  // std::vector<Point> ConstructConvexHull(...)
  std::vector<Point> FindQuickHull(const int num_threads) {
    //produces segmentation fault for files with 0 points. edgy-case but still
    Point low_point = *points_.begin();
    Point high_point = *points_.begin();
    for (const Point &point : points_) {
      if (point.y < low_point.y) {
        low_point = point;
      }
      if (point.y > high_point.y) {
        high_point = point;
      }
    }

    std::vector<Point> left_side;
    std::vector<Point> right_side;
    for (const Point &point : points_) {
      // unnecessary indent. replace with
      // if (point == low_point || point == high_point) { continue; } ...
      if (point != low_point && point != high_point) {
        double cross_product = point.CrossProduct(high_point, low_point);
        if (Equals(cross_product, 0.0)) {
          continue;
        }
        if (cross_product < 0) {
          left_side.push_back(point);
        } else {
          right_side.push_back(point);
        }
      }
    }

    std::vector<Point> left_convex_hull;
    std::vector<Point> right_convex_hull;

    if (num_threads == 1) {
      // made modifications to solve bug
      SolveRecursively(low_point, high_point,
                       left_side, left_convex_hull, 1);
      SolveRecursively(low_point, high_point,
                       right_side, right_convex_hull, 1);
    } else {
      std::thread left_thread(&ConvexHullSolver::SolveRecursively,
                              this,
                              std::ref(low_point),
                              std::ref(high_point),
                              std::ref(left_side),
                              std::ref(left_convex_hull),
                              num_threads / 2);
      std::thread right_thread(&ConvexHullSolver::SolveRecursively,
                               this,
                               std::ref(low_point),
                               std::ref(high_point),
                               std::ref(right_side),
                               std::ref(right_convex_hull),
                               num_threads - num_threads / 2);

      left_thread.join();
      right_thread.join();
    }

    std::vector<Point> convex_hull;
    convex_hull.insert(convex_hull.end(), left_convex_hull.begin(),
                       left_convex_hull.end());
    convex_hull.push_back(high_point);
    convex_hull.insert(convex_hull.end(), right_convex_hull.begin(),
                       right_convex_hull.end());
    convex_hull.push_back(low_point);
    return convex_hull;
  }

  void SolveRecursively(const Point &low_point,
                        const Point &high_point,
                        const std::vector<Point> &candidates,
                        std::vector<Point> &convex_hull,
                        const int num_threads) {
    if (candidates.empty()) {
      return;
    }

    double best_distance = 0.0;
    Point best_point;
    for (const Point &point : candidates) {
      // s/point_distance/distance_to_line
      double point_distance = point.Distance(low_point, high_point);
      if (point_distance > best_distance) {
        best_distance = point_distance;
        best_point = point;
      }
    }

    std::vector<Point> left_side;
    std::vector<Point> right_side;
    for (const Point &point : candidates) {
      // unnecessary indent. replace with
      // if (point == best_point) { continue; } ...
      if (point != best_point) {
        if (point.InTriangle(low_point, high_point, best_point)) {
          continue;
        }

        if (point.CrossProduct(low_point, best_point) > 0) {
          left_side.push_back(point);
        } else {
          right_side.push_back(point);
        }
      }
    }

    // these recursive calls are 90% similar with the ones in
    // ConvexHullSolver#FindQuickHull; you could easily get rid of the
    // code duplication without losing readability
    std::vector<Point> left_convex_hull;
    std::vector<Point> right_convex_hull;

    if (num_threads == 1) {
      SolveRecursively(high_point, best_point,
                       left_side, left_convex_hull, 1);
      SolveRecursively(low_point, best_point,
                       right_side, right_convex_hull, 1);
    } else {
      std::thread left_thread(&ConvexHullSolver::SolveRecursively,
                              this,
                              std::ref(high_point),
                              std::ref(best_point), std::ref(left_side),
                              std::ref(left_convex_hull),
                              num_threads / 2);
      std::thread right_thread(&ConvexHullSolver::SolveRecursively,
                               this,
                               std::ref(low_point),
                               std::ref(best_point),
                               std::ref(right_side),
                               std::ref(right_convex_hull),
                               num_threads - num_threads / 2);

      left_thread.join();
      right_thread.join();
    }

    convex_hull.insert(convex_hull.end(), left_convex_hull.begin(),
                       left_convex_hull.end());
    convex_hull.push_back(best_point);
    convex_hull.insert(convex_hull.end(), right_convex_hull.begin(),
                       right_convex_hull.end());
  }

  std::vector<Point> points_;
};

int main(int argc, char **argv) {
  if (argc != 3) {
    std::cout << "Usage: ./seq <input_file> <num_threads>" << '\n';
    exit(1);
  }

  // CLion complained of atoi; apparently it bypasses conversion errors;
  // for example, if i give as input "fwefw" for the number of threads
  // the result from atoi is 0. not cool. it doesn't affect the program
  // at all in this case but it's pretty ugly
  // consider s/atoi/std::stoi in order to output a relevant error message

  int num_threads = atoi(argv[2]);
  if (num_threads < 2) {
    std::cout << "The number of threads must be > 1" << '\n';
    exit(1);
  }

  ConvexHullSolver solver{std::string(argv[1])};

  auto initial_time = std::chrono::high_resolution_clock::now();

  auto sequential_convex_hull = solver.SolveSequential();
  auto sequential_time = std::chrono::high_resolution_clock::now();

  auto parallel_convex_hull = solver.SolveParallel(num_threads);
  auto parallel_time = std::chrono::high_resolution_clock::now();

  assert(sequential_convex_hull == parallel_convex_hull);

  // looks ugly af but I've got no alternative
  std::cout << std::fixed << std::setprecision(7) << "Sequential time: " <<
            1.0 * std::chrono::duration<double>(sequential_time -
                initial_time).count() << '\n';

  std::cout << std::fixed << std::setprecision(7) << "Parallel time with " <<
            num_threads << " threads: " <<
            1.0 * std::chrono::duration<double>(parallel_time -
                sequential_time).count() << '\n';

  return 0;
}
