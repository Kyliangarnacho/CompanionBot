#include "companionbot_follow/follow_algorithms.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <thread>

using namespace companionbot::follow;
namespace {
void require(bool ok, const char* why) { if (!ok) throw std::runtime_error(why); }
bool close(double a, double b) { return std::abs(a - b) <= 2e-12; }
}

int main() {
  try {
    // Declared software contract fixture. Production reads canonical JSON in
    // its composition layer and explicitly passes these immutable values.
    auto authority = std::make_shared<MotionAuthority>("bookstore", "v1", .25, .5, .3, .35,
        FollowAuthorityConfig{.6, .5, .3, .2, .35, .15});
    authority->update_health(true, "", 10.);
    authority->update_mcu_status("NORMAL", "", 10.);
    require(authority->update_follow_obstacle(10., 5.1, 5.1, 10.), "scan fixture rejected");
    require(authority->update_follow_braking(.5, 10.), "braking fixture rejected");
    FollowController controller(authority, {.05, .6, .03, .8},
        {1.1, 1.4, 1.1, 3., .6, 0., .25, .05, 3, .05}, 2., 1., .5, .01);
    auto absent = controller.accept_measurement("camera", 0, 5.0, 5.1, 10., 1.8, 0., true, "");
    require(!absent.accepted && absent.reason == "follow_state_history_unavailable", "missing sensor history accepted");
    for (std::uint64_t i = 0; i <= 10; ++i)
      require(controller.accept_state({i, 4.8 + i * .05, .1}), "state fixture rejected");
    auto readiness = controller.accept_measurement("camera", 1, 5.02, 5.1, 10., 1.8, 0., true, "");
    require(readiness.accepted && !readiness.command_accepted && !authority->snapshot(false).authorized,
            "readiness granted task or emitted motion");
    require(authority->control({"task", "agent", 1, "ACQUIRE_FOLLOW", 1.2, "bookstore", "v1", 5.1, 10.}).accepted,
            "explicit follow grant rejected");
    auto pregrant = controller.accept_measurement("camera", 1, 5.02, 5.1, 10., 8., 0., true, "");
    require(!pregrant.accepted && pregrant.reason == "follow_pregrant_observation", "pregrant image seeded task algorithm");
    auto first = controller.accept_measurement("camera", 2, 5.12, 5.18, 10.08, 1.8, 0., true, "");
    require(first.command_accepted && first.radial && first.radial->estimate && first.governor,
            "full native follow composition rejected valid sensor-aligned measurement");
    require(close(first.radial->estimate->master_radial_velocity_hat_m_s, 0.) &&
                close(first.governor->intent.linear_velocity_target_m_s, (1.8 - 1.1) / 3.),
            "new task did not use frozen KF/governor initialization");
    require(close(authority->sample(5.18, 10.08).linear_velocity_target_m_s, (1.8 - 1.1) / 3.),
            "algorithm output not consumed by unique authority");
    auto obsolete = controller.accept_measurement("camera", 2, 5.12, 5.18, 10.08, 0., 0., false, "late_master_lost");
    require(!obsolete.command_accepted && authority->snapshot(false).authorized &&
                close(authority->sample(5.18, 10.08).linear_velocity_target_m_s, (1.8 - 1.1) / 3.),
            "obsolete invalid frame revoked a newer valid command");
    auto invalid = controller.accept_measurement("camera", 3, 5.17, 5.23, 10.13, 0., 0., false, "master_lost");
    require(!invalid.command_accepted && !authority->snapshot(false).authorized &&
                authority->take_stop()->second == "master_lost", "LOST retained latched intent");
    // Both public APIs release the GIL in Python and can run on separate native
    // threads. Neither calls back into the simulator/transport runtime.
    std::thread sensor([&] {
      for (std::uint64_t i = 11; i < 1000; ++i) controller.accept_state({i, 4.8 + i * .05, .1});
    });
    std::thread vision([&] {
      for (std::uint64_t i = 4; i < 500; ++i)
        controller.accept_measurement("camera", i, 5.2 + i * .05, 5.3 + i * .05, 10.2 + i * .05,
                                      0., 0., false, "depth_unavailable");
    });
    sensor.join(); vision.join();
    require(!authority->snapshot(false).authorized && controller.snapshot().state_history.accepted_count == 1000,
            "concurrent invalid measurements restored motion or lost consumed states");
    std::cout << "native follow composition sensor alignment/task reset/lost/concurrency passed\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n'; return 1;
  }
}
