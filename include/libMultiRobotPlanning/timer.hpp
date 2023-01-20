#pragma once

#include <chrono>
#include <iostream>

struct State {
    State(int time, int x, int y) : time(time), x(x), y(y) {}

    bool operator==(const State &s) const {
        return time == s.time && x == s.x && y == s.y;
    }

    bool equalExceptTime(const State &s) const { return x == s.x && y == s.y; }

    friend std::ostream &operator<<(std::ostream &os, const State &s) {
        return os << s.time << ": (" << s.x << "," << s.y << ")";
        // return os << "(" << s.x << "," << s.y << ")";
    }

    int time;
    int x;
    int y;
};

class Timer {
 public:
  Timer()
      : start_(std::chrono::high_resolution_clock::now()),
        end_(std::chrono::high_resolution_clock::now()) {}

  void reset() { start_ = std::chrono::high_resolution_clock::now(); }

  void stop() { end_ = std::chrono::high_resolution_clock::now(); }

  double elapsedSeconds() const {
    auto timeSpan = std::chrono::duration_cast<std::chrono::duration<double>>(
        end_ - start_);
    return timeSpan.count();
  }

 private:
  std::chrono::high_resolution_clock::time_point start_;
  std::chrono::high_resolution_clock::time_point end_;
};

class ScopedTimer : public Timer {
 public:
  ScopedTimer() {}

  ~ScopedTimer() {
    stop();
    std::cout << "Elapsed: " << elapsedSeconds() << " s" << std::endl;
  }
};
