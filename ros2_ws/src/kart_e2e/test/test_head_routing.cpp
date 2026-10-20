#include "kart_e2e/head_routing.hpp"
#include <cassert>
int main() {
  kart_e2e::HeadRouting router(2,"route",.3);
  std::vector<float> values{.1f,.2f,-.5f,.4f};
  assert(!router.fresh(1.));
  assert(router.update(100,0,true,"route",1.));
  assert(!router.select(101,values,2,1.1));
  assert(router.update(101,1,true,"route",1.1));
  // Old inference arriving after the boundary must use the old head.
  assert(router.select(100,values,2,1.2)->at(0)==.1f);
  assert(router.select(101,values,2,1.2)->at(0)==-.5f);
  assert(router.select(101,values,2,1.5)->empty());
  assert(!router.update(102,0,true,"different map",1.6));
  assert(router.select(100,values,2,1.6)->empty());
  assert(!router.update(103,5,true,"route",1.7));
  router.reset();assert(!router.fresh(1.7));
}
