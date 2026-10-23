#include "companionbot_control/serial_port.hpp"
#include "companionbot/wire.h"
#include <companionbot_interfaces/msg/mcu_frame.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <algorithm>
#include <array>
#include <deque>
#include <fstream>
#include <memory>
#include <mutex>
#include <thread>

namespace companionbot {
using McuFrame=companionbot_interfaces::msg::McuFrame;
using namespace std::chrono_literals;
class McuBridge final : public rclcpp::Node {
 public:
  McuBridge():Node("mcu_bridge") {
    for(auto &counter:reasons_) counter.store(0);
    std::ifstream boot_file("/proc/sys/kernel/random/boot_id"); std::string boot;
    if(!(boot_file>>boot) || boot.size()!=36) throw std::runtime_error("cannot prove this Linux monotonic clock identity");
    clock_id_="linux-boot:"+boot;
    if(get_parameter("use_sim_time").as_bool()) throw std::runtime_error("MCU transport requires host monotonic time");
    rcl_interfaces::msg::ParameterDescriptor immutable; immutable.read_only=true;
    const auto device=declare_parameter<std::string>("device","",immutable);
    const int baud=declare_parameter<int>("baud",921600,immutable);
    allow_tx_=declare_parameter<bool>("allow_command_tx",false,immutable);
    parameter_guard_=add_on_set_parameters_callback([](const std::vector<rclcpp::Parameter> &values){
      rcl_interfaces::msg::SetParametersResult result; result.successful=true;
      for(const auto &value:values) if(value.get_name()=="use_sim_time" && value.as_bool()) {
        result.successful=false; result.reason="MCU deadlines require host monotonic time";
      }
      return result;
    });
    if(device.empty()) throw std::runtime_error("set an explicit serial device; no implicit hardware selection");
    port_=std::make_unique<SerialPort>(device,static_cast<unsigned>(baud));
    rx_pub_=create_publisher<McuFrame>("mcu/rx",rclcpp::QoS(4).best_effort());
    diagnostic_pub_=create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics",10);
    tx_sub_=create_subscription<McuFrame>("mcu/tx",rclcpp::QoS(8),[this](const McuFrame &msg){enqueue(msg);});
    publish_timer_=create_wall_timer(20ms,[this](){publish();});
    diagnostic_timer_=create_wall_timer(1s,[this](){diagnose();});
    worker_=std::thread([this](){run();});
    RCLCPP_INFO(get_logger(),"serial transport ready, command_tx=%s; MCU controls at 500 Hz locally",allow_tx_?"enabled":"disabled");
  }
  ~McuBridge() override { running_=false; if(worker_.joinable()) worker_.join(); }
 private:
  struct Pending { cb_frame frame; uint64_t deadline; };
  enum Rejection { ACCEPTED=0, TX_DISABLED, LINK_FAILED, DEVICE_STALE, HOST_DEADLINE,
    SESSION, MCU_DEADLINE, SCHEMA, QUEUE_FULL, SUPERSEDED, RX_SEQUENCE, RX_SCHEMA, HOST_CLOCK, REJECTION_COUNT };
  void reject(Rejection reason,uint64_t count=1) { rejected_+=count; reasons_[reason]+=count; }
  Rejection gate_locked(const Pending &p,uint64_t now) const {
    if(!allow_tx_) return TX_DISABLED;
    if(failed_) return LINK_FAILED;
    if(!rx_received_ || now-rx_received_>250000000ULL) return DEVICE_STALE;
    const std::size_t raw_size=20u+p.frame.length;
    const auto wire_ns=static_cast<uint64_t>(port_->transmission_time(raw_size+raw_size/254u+2u).count());
    if(p.deadline<=now || p.deadline-now>100000000ULL || p.deadline-now<=wire_ns) return HOST_DEADLINE;
    if(p.frame.session!=session_) return SESSION;
    // Conservative interval: device can have advanced since the last observation.
    const uint64_t elapsed=(now-rx_received_+wire_ns)/2000000ULL;
    return uint64_t(p.frame.tick)>=uint64_t(rx_tick_)+elapsed+1 &&
           uint64_t(p.frame.tick)<=uint64_t(rx_tick_)+125 ? ACCEPTED : MCU_DEADLINE;
  }
  void enqueue(const McuFrame &m) {
    Pending p{}; p.frame.type=m.kind; p.frame.length=static_cast<uint16_t>(m.payload.size());
    p.frame.session=m.session_id; p.frame.sequence=m.sequence_id; p.frame.tick=m.control_tick;
    p.deadline=m.host_deadline_monotonic_ns;
    if(m.host_clock_id!=clock_id_) { reject(HOST_CLOCK); return; }
    if(m.payload.size()>CB_WIRE_PAYLOAD) { reject(SCHEMA); return; }
    std::copy(m.payload.begin(),m.payload.end(),p.frame.payload);
    if(cb_wire_validate_command(&p.frame)!=CB_OK) { reject(SCHEMA); return; }
    std::lock_guard<std::mutex> lock(mutex_);
    const auto reason=gate_locked(p,monotonic_ns());
    if(reason!=ACCEPTED) { reject(reason); return; }
    if(pending_.size()>=8 && p.frame.type!=CB_MSG_SAFETY) { reject(QUEUE_FULL); return; }
    if(p.frame.type==CB_MSG_SAFETY) {
      // Discard queued intentions before emergency arbitration; never mint heartbeats.
      reject(SUPERSEDED,pending_.size()); pending_.clear(); pending_.push_front(p);
    } else pending_.push_back(p);
  }
  void run() noexcept {
    try {
      cb_parser parser{}; cb_frame frame{}; uint8_t bytes[1024],encoded[CB_WIRE_ENCODED];
      while(running_) {
        bool pending;
        { std::lock_guard<std::mutex> lock(mutex_); pending=!pending_.empty(); }
        const auto n=port_->read_some(bytes,sizeof(bytes),pending?0:5);
        for(std::size_t i=0;i<n;i++) {
          const int result=cb_parser_feed(&parser,bytes[i],&frame);
          if(result<0) { crc_rejected_++; continue; }
          if(result!=1) continue;
          if(frame.type!=CB_MSG_STATUS || frame.length!=CB_STATUS_BYTES || !frame.session) { reject(RX_SCHEMA); continue; }
          std::lock_guard<std::mutex> lock(mutex_);
          if(session_!=frame.session) { reject(SUPERSEDED,pending_.size()); pending_.clear(); session_=frame.session; have_rx_sequence_=false; }
          if(have_rx_sequence_ && (frame.sequence<=rx_sequence_ || frame.tick<rx_tick_)) { reject(RX_SEQUENCE); continue; }
          rx_tick_=frame.tick; rx_sequence_=frame.sequence; have_rx_sequence_=true;
          rx_received_=monotonic_ns(); latest_=frame; has_latest_=true; received_++;
        }
        Pending p{}; bool send=false;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          if(!pending_.empty()) {
            p=pending_.front(); pending_.pop_front();
            const auto reason=gate_locked(p,monotonic_ns());send=reason==ACCEPTED;if(!send) reject(reason);
          }
        }
        if(send) {
          const auto length=cb_frame_encode(&p.frame,encoded,sizeof(encoded));
          if(!length) throw std::runtime_error("frame encoding failed");
          const auto now=monotonic_ns();
          if(p.deadline<=now) { reject(HOST_DEADLINE); continue; }
          const auto remaining=p.deadline-now;
          port_->write_all(encoded,length,std::chrono::steady_clock::now()+std::chrono::nanoseconds(remaining));
          sent_++;
        }
      }
    } catch(const std::exception &e) {
      std::lock_guard<std::mutex> lock(mutex_); error_=e.what(); failed_=true;
      reject(LINK_FAILED,pending_.size()); pending_.clear();
    }
  }
  void publish() {
    cb_frame f{}; uint64_t arrival;
    { std::lock_guard<std::mutex> lock(mutex_);
      if(failed_) throw std::runtime_error(error_);
      if(!has_latest_) return;
      f=latest_; arrival=rx_received_; has_latest_=false;
    }
    McuFrame out; out.kind=f.type; out.session_id=f.session; out.sequence_id=f.sequence;
    out.control_tick=f.tick; out.host_deadline_monotonic_ns=arrival;
    out.host_clock_id=clock_id_;
    out.payload.assign(f.payload,f.payload+f.length); rx_pub_->publish(out);
  }
  void diagnose() {
    diagnostic_msgs::msg::DiagnosticArray array; array.header.stamp=now();
    diagnostic_msgs::msg::DiagnosticStatus s; s.name=get_fully_qualified_name(); s.hardware_id="STM32 transport (unverified physical board)";
    std::lock_guard<std::mutex> lock(mutex_);
    const bool stale=!rx_received_ || monotonic_ns()-rx_received_>250000000ULL;
    s.level=failed_?2:stale?1:0; s.message=failed_?error_:stale?"awaiting fresh MCU status":"receiving MCU status";
    const auto add=[&s](const std::string &key,const std::string &value){diagnostic_msgs::msg::KeyValue kv;kv.key=key;kv.value=value;s.values.push_back(kv);};
    add("session",std::to_string(session_)); add("rx",std::to_string(received_.load())); add("tx",std::to_string(sent_.load()));
    add("rejected",std::to_string(rejected_.load())); add("framing_crc_rejected",std::to_string(crc_rejected_.load()));
    add("queued",std::to_string(pending_.size())); add("command_tx",allow_tx_?"true":"false");
    static const std::array<const char *,REJECTION_COUNT> names={"accepted","tx_disabled","link_failed","device_stale",
      "host_deadline","session","mcu_deadline","schema","queue_full","superseded","rx_sequence","rx_schema","host_clock"};
    for(std::size_t i=1;i<names.size();i++) add(std::string("rejected_")+names[i],std::to_string(reasons_[i].load()));
    add("hardware_execution_ready","false"); array.status.push_back(s); diagnostic_pub_->publish(array);
  }
  std::unique_ptr<SerialPort> port_;
  rclcpp::Publisher<McuFrame>::SharedPtr rx_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_pub_;
  rclcpp::Subscription<McuFrame>::SharedPtr tx_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_,diagnostic_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_guard_;
  std::thread worker_; std::atomic<bool> running_{true},failed_{false};
  std::atomic<uint64_t> received_{0},sent_{0},rejected_{0},crc_rejected_{0};
  std::array<std::atomic<uint64_t>,REJECTION_COUNT> reasons_{};
  std::mutex mutex_; std::deque<Pending> pending_; cb_frame latest_{};
  uint32_t session_=0,rx_tick_=0,rx_sequence_=0; uint64_t rx_received_=0;
  bool allow_tx_=false,has_latest_=false,have_rx_sequence_=false; std::string error_,clock_id_;
};
}  // namespace companionbot
int main(int argc,char **argv) {
  rclcpp::init(argc,argv); int result=0;
  try { rclcpp::spin(std::make_shared<companionbot::McuBridge>()); }
  catch(const std::exception &e) { RCLCPP_ERROR(rclcpp::get_logger("mcu_bridge"),"%s",e.what()); result=1; }
  rclcpp::shutdown(); return result;
}
