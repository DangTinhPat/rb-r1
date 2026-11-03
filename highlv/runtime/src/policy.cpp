#include "rb/runtime/policy.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <vector>

#include <dlfcn.h>

#include "rb/runtime/json.hpp"
#include "rb/runtime/sha256.hpp"

namespace rb::runtime {

namespace {
constexpr std::array<double, 4> kDefault = {-0.7, -0.7, 0.9, 0.9};
double monotonicMs() {
  using namespace std::chrono;
  return duration<double, std::milli>(steady_clock::now().time_since_epoch()).count();
}
}  // namespace

Action encodeTargets(const std::array<double, 6> &t) {
  return {(t[0] + 0.7) / 0.25, (t[1] + 0.7) / 0.25, (t[2] - 0.2) / 0.25,
          (t[3] - 0.2) / 0.25, t[4] * 0.035 / 0.8,  t[5] * 0.035 / 0.8};
}

std::array<double, 6> decodeActions(const Action &a) {
  return {-0.7 + 0.25 * a[0], -0.7 + 0.25 * a[1], 0.2 + 0.25 * a[2],
          0.2 + 0.25 * a[3],  0.8 / 0.035 * a[4], 0.8 / 0.035 * a[5]};
}

Observation observation(const hcr::Telemetry &t, const Action &previous,
                        const std::array<double, 2> &velocity) {
  Observation o{};
  std::size_t n = 0;
  auto put = [&](double v) { o[n++] = v; };
  for (double v : t.gyro) put(v);
  for (double v : t.gravity) put(v);
  put(velocity[0]);
  put(0.0);
  put(velocity[1]);
  for (unsigned i = 0; i < 4; ++i) put(t.q[i] - kDefault[i]);
  for (double v : t.qd) put(v);
  for (double v : previous) put(v);
  put(0.0);
  put(0.0);
  put(0.16);
  for (unsigned i = 0; i < 4; ++i) put(0.0);
  return o;
}

// ---------------------------------------------------------------- RKNN backend
// Struct layouts mirror rknn_api.h as used by the Python ctypes backend.
extern "C" {
struct HcrTensorView {  // native/tensor_math.c
  void *memory, *data;
  float *values;
  std::uint32_t n;
  int type;
  float scale;
  int zp;
};
int hcr_execute(std::uint32_t context, int (*run)(std::uint32_t, void *),
                int (*sync)(std::uint32_t, void *, int), HcrTensorView *inputs,
                HcrTensorView *outputs);
}

namespace {
struct RknnIoNum { std::uint32_t n_input, n_output; };
struct RknnTensorAttr {
  std::uint32_t index, n_dims, dims[16];
  char name[256];
  std::uint32_t n_elems, size;
  int fmt, type, qnt_type;
  std::int8_t fl;
  std::int32_t zp;
  float scale;
  std::uint32_t w_stride, size_with_stride;
  std::uint8_t pass_through;
  std::uint32_t h_stride;
};
struct RknnTensorMem {
  void *virt_addr;
  std::uint64_t phys_addr;
  std::int32_t fd, offset;
  std::uint32_t size, flags;
  void *priv_data;
};

std::string readFile(const std::string &path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) throw std::runtime_error("cannot read " + path);
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

class RknnPolicy final : public Policy {
 public:
  explicit RknnPolicy(const std::string &root) {
    verifyArtifacts(root);
    const Json normalizer = Json::parse(readFile(root + "/models/beni512_19500_normalizer.json"));
    for (unsigned i = 0; i < 32; ++i) {
      mean_[i] = normalizer.at("mean").items().at(i).number();
      divisor_[i] = normalizer.at("divisor").items().at(i).number();
    }
    load(root + "/models/beni512_dfp16.rknn");
    reset();
    // Allocate/touch tensor and runtime paths before serving any motor command.
    Observation warm{};
    warm[5] = -1.0;
    warm[27] = 0.16;
    for (int i = 0; i < 8; ++i) step(warm);
    reset();
  }
  ~RknnPolicy() override { close(); }

  void reset() override {
    hidden_.fill(0.F);
    steps_ = 0;
    last_ms_ = 0;
  }

  Action step(const Observation &obs) override {
    for (double v : obs)
      if (!std::isfinite(v)) throw std::invalid_argument("invalid observation");
    const double start = monotonicMs();
    for (unsigned i = 0; i < 32; ++i)
      obs_[i] = static_cast<float>((obs[i] - mean_[i]) / divisor_[i]);
    in_[1].values = hidden_.data();
    const double native_start = monotonicMs();
    int code = hcr_execute(context_, run_, sync_, in_, out_);
    last_native_ms_ = monotonicMs() - native_start;
    if (code) throw std::runtime_error("native inference failed: " + std::to_string(code));
    last_ms_ = monotonicMs() - start;
    Action action;
    for (unsigned i = 0; i < 6; ++i) action[i] = action_[i];  // unpack already checked finite
    hidden_ = hiddenOut_;
    ++steps_;
    return action;
  }

  int steps() const override { return steps_; }
  double lastMs() const override { return last_ms_; }
  double lastNativeMs() const override { return last_native_ms_; }

 private:
  template <class T> T symbol(const char *name) {
    void *p = dlsym(lib_, name);
    if (!p) throw std::runtime_error(std::string("librknnmrt: missing ") + name);
    return reinterpret_cast<T>(p);
  }
  static void check(int code, const char *label) {
    if (code != 0) throw std::runtime_error(std::string(label) + " failed: " + std::to_string(code));
  }

  static void verifyArtifacts(const std::string &root) {
    const Json manifest = Json::parse(readFile(root + "/models/manifest.json"));
    for (const auto &entry : manifest.at("sha256").members()) {
      const std::string data = readFile(root + "/" + entry.first);
      if (sha256Hex(reinterpret_cast<const std::uint8_t *>(data.data()), data.size()) !=
          entry.second.string())
        throw std::runtime_error("artifact hash mismatch: " + entry.first);
    }
  }

  void load(const std::string &path) {
    lib_ = dlopen("/oem/usr/lib/librknnmrt.so", RTLD_NOW);
    if (!lib_) throw std::runtime_error(std::string("cannot load librknnmrt: ") + dlerror());
    auto init = symbol<int (*)(std::uint32_t *, void *, std::uint32_t, std::uint32_t, void *)>("rknn_init");
    auto query = symbol<int (*)(std::uint32_t, int, void *, std::uint32_t)>("rknn_query");
    create_mem_ = symbol<RknnTensorMem *(*)(std::uint32_t, std::uint32_t)>("rknn_create_mem");
    auto set_io_mem = symbol<int (*)(std::uint32_t, RknnTensorMem *, RknnTensorAttr *)>("rknn_set_io_mem");
    destroy_mem_ = symbol<int (*)(std::uint32_t, RknnTensorMem *)>("rknn_destroy_mem");
    destroy_ = symbol<int (*)(std::uint32_t)>("rknn_destroy");
    run_ = symbol<int (*)(std::uint32_t, void *)>("rknn_run");
    sync_ = symbol<int (*)(std::uint32_t, void *, int)>("rknn_mem_sync");

    model_data_ = readFile(path);
    check(init(&context_, model_data_.data(), static_cast<std::uint32_t>(model_data_.size()), 0, nullptr),
          "rknn_init");
    open_ = true;
    RknnIoNum io{};
    check(query(context_, 0, &io, sizeof io), "query_io");
    if (io.n_input != 2 || io.n_output != 2) throw std::runtime_error("expected 2 inputs and 2 outputs");
    for (int command : {1, 2}) {
      for (std::uint32_t index = 0; index < 2; ++index) {
        RknnTensorAttr &attr = (command == 1 ? in_attr_ : out_attr_)[index];
        std::memset(&attr, 0, sizeof attr);
        attr.index = index;
        check(query(context_, command, &attr, sizeof attr), "query_attr");
        RknnTensorMem *mem = create_mem_(context_, attr.size_with_stride ? attr.size_with_stride : attr.size);
        if (!mem) throw std::runtime_error("rknn_create_mem failed");
        (command == 1 ? in_mem_ : out_mem_)[index] = mem;
        if (command == 1) attr.pass_through = 1;
        check(set_io_mem(context_, mem, &attr), "set_io_mem");
      }
    }
    if (in_attr_[0].n_elems != 32 || in_attr_[1].n_elems != 512)
      throw std::runtime_error("expected obs32 followed by hidden512");
    const bool a6 = out_attr_[0].n_elems == 6 && out_attr_[1].n_elems == 512;
    const bool b6 = out_attr_[0].n_elems == 512 && out_attr_[1].n_elems == 6;
    if (!a6 && !b6) throw std::runtime_error("expected action6 and hidden512 outputs");
    const unsigned action_index = a6 ? 0 : 1;
    for (int i = 0; i < 2; ++i) {
      fill(in_[i], in_attr_[i], in_mem_[i]);
      fill(out_[i], out_attr_[i], out_mem_[i]);
    }
    in_[0].values = obs_.data();
    out_[action_index].values = action_.data();
    out_[1 - action_index].values = hiddenOut_.data();
  }

  static void fill(HcrTensorView &view, const RknnTensorAttr &attr, RknnTensorMem *mem) {
    float scale = 1.F;
    int zp = 0;
    if (attr.type != 0) {
      if (attr.qnt_type == 2) { scale = attr.scale; zp = attr.zp; }
      else if (attr.qnt_type == 1) scale = std::ldexp(1.F, -attr.fl);
      else throw std::runtime_error("unsupported quantization");
    }
    view = {mem, mem->virt_addr, nullptr, attr.n_elems, attr.type, scale, zp};
  }

  void close() {
    if (!open_) return;
    for (auto *m : in_mem_) if (m) destroy_mem_(context_, m);
    for (auto *m : out_mem_) if (m) destroy_mem_(context_, m);
    destroy_(context_);
    open_ = false;
    if (lib_) dlclose(lib_);
  }

  void *lib_ = nullptr;
  bool open_ = false;
  std::uint32_t context_ = 0;
  std::string model_data_;
  RknnTensorMem *(*create_mem_)(std::uint32_t, std::uint32_t) = nullptr;
  int (*destroy_mem_)(std::uint32_t, RknnTensorMem *) = nullptr;
  int (*destroy_)(std::uint32_t) = nullptr;
  int (*run_)(std::uint32_t, void *) = nullptr;
  int (*sync_)(std::uint32_t, void *, int) = nullptr;
  RknnTensorAttr in_attr_[2], out_attr_[2];
  RknnTensorMem *in_mem_[2] = {}, *out_mem_[2] = {};
  HcrTensorView in_[2], out_[2];
  std::array<float, 32> obs_{};
  std::array<float, 6> action_{};
  std::array<float, 512> hidden_{}, hiddenOut_{};
  std::array<double, 32> mean_{}, divisor_{};
  int steps_ = 0;
  double last_ms_ = 0, last_native_ms_ = 0;
};
}  // namespace

std::unique_ptr<Policy> makeRknnPolicy(const std::string &root) { return std::make_unique<RknnPolicy>(root); }

}  // namespace rb::runtime
