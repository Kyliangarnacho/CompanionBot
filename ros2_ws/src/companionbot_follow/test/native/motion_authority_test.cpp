#include "companionbot_follow/motion_authority.hpp"

#include <iostream>
#include <limits>
#include <stdexcept>

using companionbot::follow::AuthorityControl;
using companionbot::follow::MotionAuthority;

namespace {
void require(bool ok, const char* message) {
  if (!ok) throw std::runtime_error(message);
}
AuthorityControl request(std::uint64_t sequence, std::string command = "ACQUIRE") {
  return {"task", "epoch", sequence, std::move(command), 1.2, "bookstore", "v1", 5., 10.};
}
void ready(MotionAuthority& authority) {
  authority.update_health(true, "", 10.);
  authority.update_mcu_status("NORMAL", "", 10.);
}
}

int main() {
  try {
    MotionAuthority authority("bookstore", "v1");
    ready(authority);
    require(authority.control(request(1)).accepted, "grant rejected");
    require(authority.receive_cmd(.4, .8, 5.01, 5.02, 10.02), "fresh command rejected");
    auto intent = authority.sample(5.02, 10.02);
    require(intent.linear_velocity_target_m_s == .25 && intent.yaw_rate_target_rad_s == .5,
            "visible envelope limiting changed");
    require(!authority.control(request(1)).accepted, "old sequence accepted");
    auto stop = request(2, "STOP");
    require(authority.control(stop).accepted, "STOP rejected");
    require(authority.sample(5.02, 10.02).linear_velocity_target_m_s == 0., "STOP retained intent");
    require(authority.take_stop()->second == "navigation_stop", "STOP not observable");
    require(!authority.take_stop(), "STOP delivered twice");
    authority.update_mcu_status("SAFE_HOLD", "heartbeat_timeout", 10.03);
    require(!authority.control(request(3)).accepted, "independent fault rearmed");
    authority.update_mcu_status("SAFE_HOLD", "navigation_stop", 10.04);
    require(authority.control(request(4)).accepted, "explicit task rearm rejected");
    require(!authority.receive_cmd(.2, 0., 4.99, 5.01, 10.05), "pregrant command accepted");
    require(authority.receive_cmd(.2, 0., 5.01, 5.02, 10.05), "new fresh command rejected");
    authority.update_health(true, "", 10.35);
    require(authority.sample(5.02, 10.36).linear_velocity_target_m_s == 0.,
            "paused source clock bypassed receipt watchdog");

    MotionAuthority lease("bookstore", "v1");
    ready(lease);
    require(lease.control(request(1)).accepted, "lease setup rejected");
    lease.update_health(true, "", 11.3);
    auto renew = request(2, "RENEW"); renew.source_now_s = 6.3; renew.receipt_now_s = 11.3;
    require(!lease.control(renew).accepted && !lease.snapshot().authorized,
            "expired RENEW silently reacquired authority");

    MotionAuthority capacity("bookstore", "v1");
    ready(capacity);
    for (std::size_t i = 0; i < MotionAuthority::kWatermarkCapacity; ++i) {
      auto check = request(1, "CHECK"); check.task_id = "check_" + std::to_string(i);
      require(capacity.control(check).accepted, "capacity filled too early");
    }
    auto extra = request(1); extra.task_id = "new_identity";
    require(!capacity.control(extra).accepted, "new key evicted old replay watermark");
    require(capacity.snapshot().retained_watermarks == MotionAuthority::kWatermarkCapacity,
            "watermark capacity exceeded");
    extra.command = "STOP";
    require(capacity.control(extra).accepted && capacity.take_stop().has_value(),
            "capacity blocked safety revocation");
    require(capacity.snapshot().recovery_locked, "capacity did not lock rearm");

    MotionAuthority events("bookstore", "v1");
    ready(events); require(events.control(request(1)).accepted, "events setup rejected");
    for (std::size_t i = 0; i < MotionAuthority::kEventCapacity + 11; ++i) {
      const double stamp = 5. + (i + 1) * 1e-6;
      require(events.receive_cmd(.2, 0., stamp, stamp, 10.), "events command rejected");
    }
    const auto snapshot = events.snapshot();
    require(snapshot.retained_events == MotionAuthority::kEventCapacity && snapshot.events_overwritten == 12,
            "event retention unbounded or lost overwrite evidence");

    MotionAuthority follow("bookstore", "v1", .25, .5, .3, .35,
        companionbot::follow::FollowAuthorityConfig{.6, .5, .3, .2, .35, .15});
    ready(follow);
    require(follow.update_follow_obstacle(10., 5., 5., 10.), "obstacle fixture rejected");
    require(follow.update_follow_braking(.5, 10.), "braking fixture rejected");
    auto check_follow = request(1, "CHECK_FOLLOW");
    require(!follow.control(check_follow).accepted, "FOLLOW ready without perception health");
    require(follow.update_follow_health(true, "", "camera", 1, 5., 5., 10.), "follow health setup rejected");
    auto acquire_follow = request(2, "ACQUIRE_FOLLOW");
    require(follow.control(acquire_follow).accepted, "explicit FOLLOW grant rejected");
    const auto follow_epoch = follow.snapshot(false).authority_epoch;
    require(!follow.receive_follow_cmd("task", "epoch", follow_epoch, "camera", 1, 5., 5., 10., .5, .1),
            "pregrant visual frame moved new task");
    require(follow.update_follow_health(true, "", "camera", 2, 5.05, 5.05, 10.05), "new frame rejected");
    require(follow.receive_follow_cmd("task", "epoch", follow_epoch, "camera", 2, 5.05, 5.05, 10.05, .5, .1),
            "follow own command rejected");
    require(follow.sample(5.05, 10.05).linear_velocity_target_m_s == .5, "FOLLOW clipped to navigation envelope");
    require(!follow.receive_cmd(std::numeric_limits<double>::quiet_NaN(), 0., 0., 5.05, 10.05),
            "foreign Nav2 callback accepted in FOLLOW");
    require(follow.snapshot(false).authorized, "foreign Nav2 callback revoked FOLLOW");
    require(!follow.invalidate_navigation("navigation_bad_frame", 10.05) && follow.snapshot(false).authorized,
            "Nav-only invalidation crossed into FOLLOW");
    auto cross_mode = request(3, "ACQUIRE"); cross_mode.source_now_s = 5.05; cross_mode.receipt_now_s = 10.05;
    require(!follow.control(cross_mode).accepted, "mode switched without revoke");
    require(!follow.update_follow_health(false, "master_lost", "camera", 3, 5.1, 5.1, 10.1),
            "LOST health reported available");
    require(!follow.snapshot(false).authorized && follow.snapshot(false).mode == "NONE" &&
                follow.take_stop()->second == "master_lost", "LOST retained authority");
    require(follow.update_follow_health(true, "", "camera", 4, 5.15, 5.15, 10.15), "new healthy frame rejected");
    require(!follow.receive_follow_cmd("task", "epoch", follow_epoch, "camera", 4, 5.15, 5.15, 10.15, .5, .1),
            "healthy image implicitly restored old task");

    MotionAuthority provenance("bookstore", "v1");
    ready(provenance);
    require(provenance.control(request(1)).accepted, "reason provenance setup rejected");
    provenance.stop("HEARTBEAT_TIMEOUT", 10.);
    provenance.update_mcu_status_with_provenance("SAFE_HOLD", "HEARTBEAT_TIMEOUT", true, 10.);
    require(provenance.control(request(2)).accepted, "trusted external reason incorrectly treated as local fault");
    provenance.update_mcu_status_with_provenance("SAFE_HOLD", "HEARTBEAT_TIMEOUT", false, 10.);
    require(!provenance.snapshot(false).authorized && !provenance.snapshot(false).mcu_recovery_owned &&
                !provenance.control(request(3, "CHECK")).accepted && !provenance.control(request(4)).accepted,
            "same-named local fault gained old external stop recovery qualification");
    std::cout << "motion_authority native safety/time/capacity tests passed\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
