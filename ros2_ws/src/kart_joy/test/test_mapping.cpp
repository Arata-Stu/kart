#include "kart_joy/mapping.hpp"

#include <cstdlib>
#include <iostream>
#include <limits>

void check(bool condition, const char * message)
{
  if (!condition) {std::cerr << message << '\n'; std::exit(1);}
}

int main()
{
  using namespace kart_joy;
  AxisBinding stick;
  stick.code = 0;
  stick.validate();
  check(stick.map(0.03) == 0.0F, "deadzone must remove noise");
  check(stick.map(1.0) == 1.0F && stick.map(-1.0) == -1.0F, "stick endpoints");
  stick.negative = 1.0;
  stick.positive = -1.0;
  stick.validate();
  check(stick.map(-1.0) == 1.0F, "inverted axis");
  AxisBinding trigger;
  trigger.code = 2;
  trigger.unipolar = true;
  trigger.center = -1.0;
  trigger.validate();
  check(trigger.map(-1.0) == 0.0F && trigger.map(1.0) == 1.0F, "trigger release/full");
  auto axes_map = std::vector<AxisBinding>(axis_names.size());
  axes_map[4] = trigger;
  auto buttons_map = std::vector<ButtonBinding>(button_names.size());
  buttons_map[0].code = 304;
  buttons_map[13] = ButtonBinding{17, true, -1, 0.5};
  std::vector<float> axes(kAxisCount, 0.0F);
  std::vector<int32_t> keys(kKeyCount, 0);
  axes[2] = 1.0F;
  axes[17] = -1.0F;
  keys[304] = 1;
  auto mapped = map_state(true, axes, keys, axes_map, buttons_map);
  check(mapped.axes[4] == 1.0F && mapped.buttons[0] == 1 && mapped.buttons[13] == 1, "active controls");
  mapped = map_state(false, axes, keys, axes_map, buttons_map);
  check(mapped.axes[4] == 0.0F && mapped.buttons[0] == 0 && mapped.buttons[13] == 0, "disconnect must neutralize held controls");
  axes[2] = -1.0F;
  keys[304] = 0;
  axes[17] = 0.0F;
  mapped = map_state(true, axes, keys, axes_map, buttons_map);
  check(mapped.axes[4] == 0.0F && mapped.buttons[0] == 0, "reconnect released state");
  mapped = map_state(true, {}, {}, axes_map, buttons_map);
  check(mapped.axes[4] == 0.0F, "missing input must not create throttle");
  for (const double bad : {1.0, -0.1, std::numeric_limits<double>::quiet_NaN()}) {
    stick.deadzone = bad;
    bool threw = false;
    try {stick.validate();} catch (const std::invalid_argument &) {threw = true;}
    check(threw, "invalid deadzone must be rejected");
  }
  std::cout << "Mapping checks passed\n";
}
