/* Freestanding ARM library: no libc dependency, loadable by Buildroot/uClibc Python. */
#include <stdint.h>
#include <stddef.h>

int hcr_pack(const float *src, void *dst, uint32_t n, int type, float scale, int zp) {
  if (!(scale > 0) || !__builtin_isfinite(scale))
    return -1;
  float lo, hi;
  if (type == 2) {
    lo = -128;
    hi = 127;
  } else if (type == 3) {
    lo = 0;
    hi = 255;
  } else if (type == 4) {
    lo = -32768;
    hi = 32767;
  } else
    return -2;
  for (uint32_t i = 0; i < n; ++i) {
    if (!__builtin_isfinite(src[i]))
      return -3;
    float v = src[i] / scale + zp;
    if (v < lo)
      v = lo;
    if (v > hi)
      v = hi;
    int q = (int)v;
    float rem = v - q;
    if (rem > .5f || (rem == .5f && (q & 1)))
      ++q;
    if (rem < -.5f || (rem == -.5f && (q & 1)))
      --q;
    if (type == 2)
      ((int8_t *)dst)[i] = (int8_t)q;
    else if (type == 3)
      ((uint8_t *)dst)[i] = (uint8_t)q;
    else
      ((int16_t *)dst)[i] = (int16_t)q;
  }
  return 0;
}

int hcr_unpack(const void *src, float *dst, uint32_t n, int type, float scale, int zp) {
  for (uint32_t i = 0; i < n; ++i) {
    float v;
    if (type == 0)
      v = ((const float *)src)[i];
    else if (type == 2)
      v = (((const int8_t *)src)[i] - zp) * scale;
    else if (type == 3)
      v = (((const uint8_t *)src)[i] - zp) * scale;
    else if (type == 4)
      v = (((const int16_t *)src)[i] - zp) * scale;
    else
      return -2;
    if (!__builtin_isfinite(v))
      return -3;
    dst[i] = v;
  }
  return 0;
}

typedef struct {
  void *memory, *data;
  float *values;
  uint32_t n;
  int type;
  float scale;
  int zp;
} TensorView;
typedef int (*Run)(uint32_t, void *);
typedef int (*Sync)(uint32_t, void *, int);
int hcr_execute(uint32_t context, Run run, Sync sync, TensorView *inputs, TensorView *outputs) {
  for (int i = 0; i < 2; i++) {
    TensorView *t = inputs + i;
    int code = hcr_pack(t->values, t->data, t->n, t->type, t->scale, t->zp);
    if (code)
      return -100 + i;
    code = sync(context, t->memory, 1);
    if (code)
      return -200 + i;
  }
  if (run(context, 0))
    return -300;
  for (int i = 0; i < 2; i++) {
    TensorView *t = outputs + i;
    if (sync(context, t->memory, 2))
      return -400 + i;
    if (hcr_unpack(t->data, t->values, t->n, t->type, t->scale, t->zp))
      return -500 + i;
  }
  return 0;
}
