// Portable harness: exercise the production controller on a generated route.
#include "kart_control/pure_pursuit.hpp"
#include <iostream>
int main() {
  size_t n; std::cin >> n;
  std::vector<kart_control::Point> line(n);
  for (auto &p: line) std::cin >> p.x >> p.y >> p.speed;
  size_t begin,end; double x,y,yaw,lookahead;
  while (std::cin >> begin >> end >> x >> y >> yaw >> lookahead) {
    std::vector<kart_control::Point> window(line.begin()+begin,line.begin()+end);
    auto r=kart_control::pursue(window,false,{x,y,yaw},
      {.257,.3839724354,lookahead,10.,.15,.04,.4});
    std::cout << r.valid << ' ' << r.steering << ' ' << r.speed << std::endl;
  }
}
