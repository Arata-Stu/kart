// ROS-free in-process ABI around the SAME GP implementation as the shadow node.
#include "kart_racing/opponent_model_c.h"
#include "kart_racing/predictive_spliner.hpp"

namespace {
kart_racing::OpponentModel* model(void* handle) {
  return static_cast<kart_racing::OpponentModel*>(handle);
}
}

extern "C" {
uint32_t kart_opponent_model_abi_version(void) { return 1; }

void* kart_opponent_model_create(double course_length) {
  try { return new kart_racing::OpponentModel(course_length); }
  catch (...) { return nullptr; }
}

void kart_opponent_model_destroy(void* handle) {
  delete model(handle);
}

int32_t kart_opponent_model_clear(void* handle) {
  if (!handle) return 0;
  try { model(handle)->clear(); return 1; }
  catch (...) { return 0; }
}

int32_t kart_opponent_model_observe(void* handle, uint64_t id, double time, double s,
                                  double d, double v, double d_variance, double v_variance) {
  if (!handle) return 0;
  try { return model(handle)->observe_retained(id, time, s, d, v, d_variance, v_variance) ? 1 : 0; }
  catch (...) { return 0; }
}

int32_t kart_opponent_model_ready(void* handle) {
  if (!handle) return 0;
  try { return model(handle)->ready() ? 1 : 0; }
  catch (...) { return 0; }
}

double kart_opponent_model_coverage(void* handle) {
  if (!handle) return 0.;
  try { return model(handle)->coverage(); }
  catch (...) { return 0.; }
}

int32_t kart_opponent_model_predict(void* handle, double s, KartOpponentPrediction* prediction) {
  if (!prediction) return -1;
  *prediction = {0., 0., 1., 1., 0};
  if (!handle || !kart_racing::finite(s)) return -1;
  try {
    const auto value = model(handle)->predict(s);
    *prediction = {value.d, value.v, value.d_sigma, value.v_sigma, value.valid ? 1 : 0};
    return 0;
  } catch (...) { return -1; }
}
}
