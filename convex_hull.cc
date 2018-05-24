#include <bits/stdc++.h>

const double kEps = 1e-6;

bool Equals(double a, double b) {
  return fabs(a - b) < kEps;
}

struct Point {
  Point() : x(0.0), y(0.0) {}

  Point(double x, double y) : x(x), y(y) {}

  double CrossProduct(const Point& b, const Point& c) const {
    return (b.x - x) * (c.y - y) - (c.x - x) * (b.y - y);
  }

  double Distance(const Point& b, const Point& c) const {
    return fabs((c.y - b.y) * x - (c.x - b.x) * y + c.x * b.y - c.y * b.x) /
           sqrt((c.y - b.y) * (c.y - b.y) + (c.x - b.x) * (c.x - b.x));
  }

  bool InTriangle(const Point& a, const Point& b, const Point& c) const {
    bool c1 = CrossProduct(a, b) < 0.0;
    bool c2 = CrossProduct(b, c) < 0.0;
    bool c3 = CrossProduct(c, a) < 0.0;

    return (c1 == c2) && (c2 == c3);
  }

  bool operator == (const Point& b) const {
    return Equals(x, b.x) && Equals(y, b.y);
  }

  bool operator != (const Point& b) const {
    return !(*this == b);
  }

  double x, y;
};

class ConvexHullSolver {
 public:
  explicit ConvexHullSolver(const std::string& input_file) {
    ReadPoints(input_file);
  }

  std::vector<Point> SolveSequential() {
    return FindQuickHull(1);
  }

  std::vector<Point> SolveParallel(const int num_threads) {
    return FindQuickHull(num_threads);
  }

 private:
  void ReadPoints(const std::string& input_file) {
    std::ifstream fin(input_file);

    int num_points;
    fin >> num_points;

    for (int i = 0; i < num_points; i++) {
      double x, y;
      fin >> x >> y;
      points_.emplace_back(x, y);
    }
  }

  std::vector<Point> FindQuickHull(const int num_threads) {
    Point low_point = *points_.begin();
    Point high_point = *points_.begin();
    for (const Point& point : points_) {
      if (point.y < low_point.y) {
        low_point = point;
      }
      if (point.y > high_point.y) {
        high_point = point;
      }
    }

    std::vector<Point> left_side;
    std::vector<Point> right_side;
    for (const Point& point : points_) {
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

  void SolveRecursively(const Point& low_point,
                        const Point& high_point,
                        const std::vector<Point>& candidates,
                        std::vector<Point>& convex_hull,
                        const int num_threads) {
    if (candidates.empty()) {
      return;
    }

    double best_distance = 0.0;
    Point best_point;
    for (const Point& point : candidates) {
      double point_distance = point.Distance(low_point, high_point);
      if (point_distance > best_distance) {
        best_distance = point_distance;
        best_point = point;
      }
    }

    std::vector<Point> left_side;
    std::vector<Point> right_side;
    for (const Point& point : candidates) {
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

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cout << "Usage: ./seq <input_file> <num_threads>" << '\n';
    exit(1);
  }

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

  std::cout << std::fixed << std::setprecision(7) << "Sequential time: " <<
            1.0 * std::chrono::duration<double>(sequential_time -
                initial_time).count() << '\n';

  std::cout << std::fixed << std::setprecision(7) << "Parallel time with " <<
            num_threads << " threads: " <<
            1.0 * std::chrono::duration<double>(parallel_time -
                sequential_time).count() << '\n';

  return 0;
}
