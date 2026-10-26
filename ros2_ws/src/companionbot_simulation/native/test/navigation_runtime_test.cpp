#include "companionbot_simulation/navigation_runtime.hpp"
#include <stdexcept>
#include <cmath>
#include <iostream>

using namespace companionbot;
using json = nlohmann::json;

static void require(bool condition) {
  if (!condition) throw std::runtime_error("native_navigation_invariant_failed");
}

int main() {
  const follow::ProducerConfig producer{};
  const cb_mcu_config mcu{{CB_DT,.5,.8,1.},.15,.012,0.,.2,.8};
  auto authority = std::make_shared<follow::MotionAuthority>("fixture_map", "fixture_v1");
  simulation::NavigationRuntime runtime(authority, producer, mcu);
  auto control = [&](std::uint64_t sequence, std::string command, std::string task, double now) {
    return authority->control({std::move(task),"native_test",sequence,std::move(command),.8,
        "fixture_map","fixture_v1",now,now});
  };
  bool acquired = false, restarted = false, stop_during_resume = false;
  std::uint32_t first_epoch = 0;
  double moving_peak = 0., canceled_peak = 0.;
  std::uint64_t sequence = 0;
  std::string task = "first";
  for (std::uint32_t tick = 0; tick < 7000; ++tick) {
    const double now = tick * CB_DT;
    authority->update_health(true, "healthy", now);
    if (tick == 500) { acquired = control(++sequence,"ACQUIRE",task,now).accepted; require(acquired); }
    if (acquired && tick % 100 == 0 && tick < 3000)
      require(control(++sequence,"RENEW",task,now).accepted);
    if (acquired && tick % 50 == 0 && tick < 3000)
      require(authority->receive_cmd(.2,.1,now,now,now,"native_fixture"));
    if (tick == 3000) { require(control(++sequence,"STOP",task,now).accepted); acquired = false; }
    if (tick == 5000) {
      task = "explicit_second"; sequence = 0;
      restarted = control(++sequence,"ACQUIRE",task,now).accepted;
      require(restarted);
      require(authority->receive_cmd(.2,0.,now,now,now,"native_fixture"));
    }
    // Cancel after the restart's future resume has armed, before activation.
    if (tick == 5010) {
      first_epoch = runtime.epoch();
      require(control(++sequence,"STOP",task,now).accepted);
      stop_during_resume = true;
    }
    const auto output = runtime.step(tick, now, 0., 0., 0.);
    require(std::isfinite(output.command.reference.p));
    require(output.command.reference.u_ff == 0. && output.command.reference.theta == 0.);
    require(std::isfinite(output.next_reference.p));
    if (tick > 1800 && tick < 3000) moving_peak = std::max(moving_peak,output.command.reference.v);
    if (tick > 5010) {
      canceled_peak = std::max(canceled_peak,std::abs(output.command.reference.v));
      require(output.command.safety_state == CB_SAFE_HOLD);
      require(output.command.epoch == first_epoch + 1);
    }
  }
  require(acquired == false && restarted && stop_during_resume && moving_peak > .19 && canceled_peak == 0.);
  const auto report = json::parse(runtime.report_json());
  require(report.at("metrics").at("step_count") == 7000);
  require(report.at("metrics").at("reference_underruns") == 0);
  require(report.at("metrics").at("queue_capacity_rejections") == 0);
  require(report.at("metrics").at("logs").at("producer").at("overwritten") == 0);
  unsigned resumes = 0;
  for (const auto &event : report.at("safety_events"))
    if (event.at("event") == "normal_resumed") ++resumes;
  require(resumes == 0); // First task ran without a safety rearm; second was revoked.
  bool invalid_rejected = false;
  try { runtime.step(7002,14.004,0.,0.,0.); }
  catch (const std::invalid_argument &) { invalid_rejected = true; }
  require(invalid_rejected);
  std::cout << "Native scheduler: normal stream, task STOP, explicit rearm cancellation, bounded queue and tick guard passed\n";
}
