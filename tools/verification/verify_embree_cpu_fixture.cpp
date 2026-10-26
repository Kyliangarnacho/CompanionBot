// Diagnostic only: Embree dependency/ownership proof, never a sensor or controller.
// API usage adapted from Intel Embree 4.3.0 triangle_geometry/dynamic_scene tutorials.
// Upstream tutorial copyright: Intel Corporation, SPDX-License-Identifier: Apache-2.0.
// Provenance/license lock: visual/active_stereo_audit/audit.json, phase1/package_lock.json.
#include <embree4/rtcore.h>
#include <oneapi/tbb/blocked_range.h>
#include <oneapi/tbb/global_control.h>
#include <oneapi/tbb/parallel_for.h>
#include <oneapi/tbb/task_arena.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {
struct Vertex { float x, y, z, padding; };
struct Triangle { std::uint32_t a, b, c; };
struct Result {
  unsigned int geometry = RTC_INVALID_GEOMETRY_ID;
  unsigned int primitive = RTC_INVALID_GEOMETRY_ID;
  float distance = std::numeric_limits<float>::infinity();
  bool short_blocked = false;
  bool long_blocked = false;
};
struct Participation {
  std::atomic<unsigned int> active{0};
  std::atomic<unsigned int> peak{0};
  std::set<std::thread::id> threads;
  std::mutex mutex;
};
struct RangeParticipation {
  Participation& shared;
  explicit RangeParticipation(Participation& state) : shared(state) {
    const auto count = shared.active.fetch_add(1) + 1;
    auto peak = shared.peak.load();
    while (count > peak && !shared.peak.compare_exchange_weak(peak, count)) {}
    std::lock_guard<std::mutex> lock(shared.mutex);
    shared.threads.insert(std::this_thread::get_id());
  }
  ~RangeParticipation() { shared.active.fetch_sub(1); }
};

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

struct SceneOwner {
  RTCDevice device = nullptr;
  RTCScene scene = nullptr;
  RTCGeometry geometry = nullptr;
  unsigned int geometry_id = RTC_INVALID_GEOMETRY_ID;
  std::atomic<int> callback_error{RTC_ERROR_NONE};

  ~SceneOwner() {
    if (geometry) rtcReleaseGeometry(geometry);
    if (scene) rtcReleaseScene(scene);
    if (device) rtcReleaseDevice(device);
  }
  static void error(void* user, RTCError code, const char*) noexcept {
    static_cast<SceneOwner*>(user)->callback_error.store(code);
  }
  void check() {
    require(rtcGetDeviceError(device) == RTC_ERROR_NONE, "embree_device_error");
    require(callback_error.load() == RTC_ERROR_NONE, "embree_error_callback");
  }
  void initialize(unsigned int threads) {
    const std::string configuration = "threads=" + std::to_string(threads);
    device = rtcNewDevice(configuration.c_str());
    require(device != nullptr, "embree_device_unavailable");
    rtcSetDeviceErrorFunction(device, &error, this);
    require(rtcGetDeviceProperty(device, RTC_DEVICE_PROPERTY_VERSION) == RTC_VERSION,
            "compiled_loaded_embree_version_mismatch");
    require(rtcGetDeviceProperty(device, RTC_DEVICE_PROPERTY_TRIANGLE_GEOMETRY_SUPPORTED),
            "embree_triangle_geometry_unavailable");
    scene = rtcNewScene(device);
    require(scene != nullptr, "embree_scene_unavailable");
    rtcSetSceneFlags(scene, RTC_SCENE_FLAG_DYNAMIC | RTC_SCENE_FLAG_ROBUST);
    rtcSetSceneBuildQuality(scene, RTC_BUILD_QUALITY_LOW);
    geometry = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);
    require(geometry != nullptr, "embree_geometry_unavailable");
    rtcSetGeometryBuildQuality(geometry, RTC_BUILD_QUALITY_REFIT);
    auto* vertices = static_cast<Vertex*>(rtcSetNewGeometryBuffer(
        geometry, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, sizeof(Vertex), 3));
    auto* triangles = static_cast<Triangle*>(rtcSetNewGeometryBuffer(
        geometry, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, sizeof(Triangle), 1));
    require(vertices && triangles, "embree_buffers_unavailable");
    vertices[0] = {-1.f, -1.f, 2.f, 0.f};
    vertices[1] = {1.f, -1.f, 2.f, 0.f};
    vertices[2] = {0.f, 1.f, 2.f, 0.f};
    triangles[0] = {0, 1, 2};
    rtcCommitGeometry(geometry);
    geometry_id = rtcAttachGeometry(scene, geometry);
    require(geometry_id != RTC_INVALID_GEOMETRY_ID, "embree_attach_failed");
    rtcCommitScene(scene);
    check();
  }
  void update(float shift_x, float z) {
    auto* vertices = static_cast<Vertex*>(
        rtcGetGeometryBufferData(geometry, RTC_BUFFER_TYPE_VERTEX, 0));
    require(vertices != nullptr, "embree_update_buffer_unavailable");
    vertices[0] = {shift_x - 1.f, -1.f, z, 0.f};
    vertices[1] = {shift_x + 1.f, -1.f, z, 0.f};
    vertices[2] = {shift_x, 1.f, z, 0.f};
    // No ray query runs concurrently with this update/commit transaction.
    rtcUpdateGeometryBuffer(geometry, RTC_BUFFER_TYPE_VERTEX, 0);
    rtcCommitGeometry(geometry);
    rtcCommitScene(scene);
    check();
  }
};

RTCRay ray(float x, float y, float far) {
  RTCRay value{};
  value.org_x = x;
  value.org_y = y;
  value.org_z = 0.f;
  value.dir_x = 0.f;
  value.dir_y = 0.f;
  value.dir_z = 1.f;  // Unit direction: ray parameter has metres as its unit.
  value.tnear = 0.f;
  value.tfar = far;
  value.time = 0.f;
  value.mask = 0xFFFFFFFFu;
  value.id = 0u;
  value.flags = 0u;
  return value;
}

RTCRayHit primary(RTCScene scene, float x, float y) {
  RTCRayHit value{};
  value.ray = ray(x, y, 10.f);
  value.hit.geomID = RTC_INVALID_GEOMETRY_ID;
  value.hit.primID = RTC_INVALID_GEOMETRY_ID;
  for (auto& instance : value.hit.instID) instance = RTC_INVALID_GEOMETRY_ID;
  RTCIntersectArguments arguments;
  rtcInitIntersectArguments(&arguments);
  arguments.feature_mask = RTC_FEATURE_FLAG_TRIANGLE;
  rtcIntersect1(scene, &value, &arguments);
  return value;
}

bool blocked(RTCScene scene, float x, float y, float far) {
  RTCRay value = ray(x, y, far);
  RTCOccludedArguments arguments;
  rtcInitOccludedArguments(&arguments);
  arguments.feature_mask = RTC_FEATURE_FLAG_TRIANGLE;
  rtcOccluded1(scene, &value, &arguments);
  return value.tfar < 0.f;
}

void check_state(SceneOwner& owner, float shift, float z, bool moved) {
  const auto hit = primary(owner.scene, shift, 0.f);
  require(hit.hit.geomID == owner.geometry_id && hit.hit.primID == 0u,
          "primary_triangle_identity_mismatch");
  require(std::abs(hit.ray.tfar - z) <= 1e-6f, "primary_distance_mismatch");
  require(primary(owner.scene, 7.f, 0.f).hit.geomID == RTC_INVALID_GEOMETRY_ID,
          "primary_miss_not_preserved");
  require(!blocked(owner.scene, shift, 0.f, z - .5f), "finite_short_shadow_blocked");
  require(blocked(owner.scene, shift, 0.f, z + .5f), "finite_long_shadow_not_blocked");
  if (moved) {
    require(primary(owner.scene, 0.f, 0.f).hit.geomID == RTC_INVALID_GEOMETRY_ID,
            "refit_old_location_still_visible");
    require(!blocked(owner.scene, 0.f, 0.f, 10.f), "refit_old_shadow_still_visible");
  }
  owner.check();
}

void batch(SceneOwner& owner, oneapi::tbb::task_arena& arena, float shift, float z,
           unsigned int state, std::ofstream& output, Participation& participation) {
  std::vector<Result> values(4096);
  arena.execute([&] {
    oneapi::tbb::parallel_for(oneapi::tbb::blocked_range<std::size_t>(0, values.size(), 64),
                            [&](const oneapi::tbb::blocked_range<std::size_t>& range) {
      RangeParticipation active(participation);
      for (std::size_t i = range.begin(); i != range.end(); ++i) {
        const float x = shift + (static_cast<float>(i % 64) + .25f) / 20.f - 1.6f;
        const float y = (static_cast<float>(i / 64) + .25f) / 20.f - 1.6f;
        const auto hit = primary(owner.scene, x, y);
        values[i].geometry = hit.hit.geomID;
        values[i].primitive = hit.hit.primID;
        if (hit.hit.geomID != RTC_INVALID_GEOMETRY_ID) values[i].distance = hit.ray.tfar;
        values[i].short_blocked = blocked(owner.scene, x, y, z - .5f);
        values[i].long_blocked = blocked(owner.scene, x, y, z + .5f);
      }
    });
  });
  owner.check();
  for (std::size_t i = 0; i < values.size(); ++i) {
    const auto& value = values[i];
    output << state << ' ' << i << ' ' << value.geometry << ' ' << value.primitive
           << ' ' << std::hexfloat << value.distance << ' ' << value.short_blocked
           << ' ' << value.long_blocked << '\n';
  }
  require(output.good(), "fixture_output_write_failed");
}
}  // namespace

int main(int argc, char** argv) {
  try {
    require(argc == 5 && std::strcmp(argv[1], "--threads") == 0 &&
            std::strcmp(argv[3], "--output") == 0 &&
            (std::strcmp(argv[2], "1") == 0 || std::strcmp(argv[2], "2") == 0),
            "usage_fixture_--threads_1_or_2_--output_path");
    const unsigned int threads = argv[2][0] == '1' ? 1u : 2u;
    oneapi::tbb::global_control limit(oneapi::tbb::global_control::max_allowed_parallelism, threads);
    oneapi::tbb::task_arena arena(static_cast<int>(threads));
    SceneOwner owner;
    owner.initialize(threads);
    std::ofstream output(argv[4]);
    require(output.is_open(), "fixture_output_open_failed");
    Participation participation;
    const auto begin = std::chrono::steady_clock::now();
    check_state(owner, 0.f, 2.f, false);
    batch(owner, arena, 0.f, 2.f, 0u, output, participation);
    owner.update(0.f, 3.f);
    check_state(owner, 0.f, 3.f, false);
    batch(owner, arena, 0.f, 3.f, 1u, output, participation);
    owner.update(3.f, 3.f);
    check_state(owner, 3.f, 3.f, true);
    batch(owner, arena, 3.f, 3.f, 2u, output, participation);
    require(participation.active.load() == 0u && participation.peak.load() <= threads,
            "tbb_concurrent_tasks_exceed_requested_cap");
    output.close();
    require(!output.fail(), "fixture_output_close_failed");
    const auto wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
    std::cout << "{\"passed\":true,\"diagnostic_only\":true,\"sensor_or_IR_result\":false,"
              << "\"compiled_embree_version\":" << RTC_VERSION
              << ",\"runtime_embree_version\":" << rtcGetDeviceProperty(owner.device, RTC_DEVICE_PROPERTY_VERSION)
              << ",\"requested_thread_cap\":" << threads
              << ",\"actual_distinct_batch_threads\":" << participation.threads.size()
              << ",\"max_concurrent_batch_tasks\":" << participation.peak.load()
              << ",\"geometry_states\":3,\"batched_samples_per_state\":4096,\"ray_queries_per_state\":12288,"
              << "\"primary_shadow_refit_checks\":true,\"wall_s_including_text_output\":" << wall
              << ",\"throughput_claim\":false}\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "embree_cpu_diagnostic_failure: " << error.what() << '\n';
    return 1;
  }
}
