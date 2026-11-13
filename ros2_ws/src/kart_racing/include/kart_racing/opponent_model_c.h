#pragma once

#include <stdint.h>

#if defined(_WIN32)
#define KART_GP_EXPORT __declspec(dllexport)
#else
#define KART_GP_EXPORT __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct KartOpponentPrediction {
  double d;
  double v;
  double d_sigma;
  double v_sigma;
  int32_t valid;
} KartOpponentPrediction;

/* ABI 1, opaque handles. No C++ exception crosses this boundary. */
KART_GP_EXPORT uint32_t kart_opponent_model_abi_version(void);
KART_GP_EXPORT void* kart_opponent_model_create(double course_length);
KART_GP_EXPORT void kart_opponent_model_destroy(void* handle);
KART_GP_EXPORT int32_t kart_opponent_model_clear(void* handle);
KART_GP_EXPORT int32_t kart_opponent_model_observe(void* handle, uint64_t id, double time,
  double s, double d, double v, double d_variance, double v_variance);
KART_GP_EXPORT int32_t kart_opponent_model_ready(void* handle);
KART_GP_EXPORT double kart_opponent_model_coverage(void* handle);
/* Returns 0 on success, -1 on failure. prediction.valid is independent. */
KART_GP_EXPORT int32_t kart_opponent_model_predict(void* handle, double s, KartOpponentPrediction* prediction);

#ifdef __cplusplus
}
#endif
