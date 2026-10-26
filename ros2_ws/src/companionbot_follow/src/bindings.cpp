#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "companionbot_follow/lightweight_producer.hpp"
namespace py = pybind11;
void bind_motion_authority(py::module_ &);
void bind_navigation_ingress(py::module_ &);
void bind_follow_algorithms(py::module_ &);
void bind_full_reference(py::module_ &);
PYBIND11_MODULE(_native_runtime, module) {
  module.doc() = "CompanionBot ROS-free native motion owners; no implicit fallback";
  module.attr("ABI_VERSION") = 1;
  module.attr("RUCKIG_VERSION") = "0.19.4";
  using namespace companionbot::follow;
  py::class_<ProducerConfig>(module, "ProducerConfig")
      .def(py::init<>())
      .def_readwrite("dt", &ProducerConfig::dt)
      .def_readwrite("block_samples", &ProducerConfig::block_samples)
      .def_readwrite("max_velocity", &ProducerConfig::max_velocity)
      .def_readwrite("max_acceleration", &ProducerConfig::max_acceleration)
      .def_readwrite("max_jerk", &ProducerConfig::max_jerk)
      .def_readwrite("full_delta_velocity", &ProducerConfig::full_delta_velocity);
  py::class_<ProducedBlock>(module, "ProducedBlock")
      .def_property_readonly("bytes", [](const ProducedBlock &b) {
        return py::bytes(reinterpret_cast<const char *>(b.bytes.data()), b.bytes.size());
      })
      .def_readonly("samples", &ProducedBlock::samples)
      .def_readonly("epoch", &ProducedBlock::epoch)
      .def_readonly("sequence", &ProducedBlock::sequence)
      .def_readonly("start_tick", &ProducedBlock::start_tick)
      .def_readonly("generated_time", &ProducedBlock::generated_time)
      .def_readonly("source_time", &ProducedBlock::source_time)
      .def_readonly("compute_wall", &ProducedBlock::compute_wall)
      .def_readonly("compute_cpu", &ProducedBlock::compute_cpu)
      .def_readonly("target_velocity", &ProducedBlock::target_velocity)
      .def_readonly("yaw", &ProducedBlock::yaw)
      .def_readonly("duration", &ProducedBlock::duration)
      .def_readonly("replanned", &ProducedBlock::replanned);
  py::class_<LightweightProducer>(module, "LightweightProducer")
      .def(py::init<ProducerConfig, std::uint32_t>(), py::arg("config"), py::arg("epoch") = 0)
      .def("reset", &LightweightProducer::reset)
      .def("produce", &LightweightProducer::produce, py::arg("start_tick"),
           py::arg("generated_time"), py::arg("source_time"), py::arg("velocity"),
           py::arg("yaw"), py::arg("observe_command") = true,
           py::call_guard<py::gil_scoped_release>())
      .def_property_readonly("next_reference", &LightweightProducer::next_reference)
      .def_property_readonly("reference_epoch", &LightweightProducer::epoch)
      .def_property_readonly("next_sequence", &LightweightProducer::next_sequence);
  bind_motion_authority(module);
  bind_navigation_ingress(module);
  bind_follow_algorithms(module);
  bind_full_reference(module);
}
