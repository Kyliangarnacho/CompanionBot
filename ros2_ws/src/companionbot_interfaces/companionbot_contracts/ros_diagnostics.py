"""Small ROS-only helpers; no import from controller or estimator cores."""
import math
import os

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult


def positive_period(node, name, default):
    value = node.declare_parameter(name, default, ParameterDescriptor(
        read_only=True, description="Host steady timer period in seconds; restart to change")).value
    if not isinstance(value, (float, int)) or isinstance(value, bool) or not math.isfinite(value) or value <= 0:
        raise ValueError(name + " must be finite and positive")
    return float(value)


def require_host_time(node):
    if node.get_parameter("use_sim_time").value:
        raise ValueError(node.get_name() + " requires host clocks, not use_sim_time")
    def validate(parameters):
        if any(p.name == "use_sim_time" and p.value for p in parameters):
            return SetParametersResult(successful=False, reason="host_clock_required")
        return SetParametersResult(successful=True)
    node.add_on_set_parameters_callback(validate)


def publish_status(node, publisher, state, values, *, level=DiagnosticStatus.OK, message=None):
    status = DiagnosticStatus(level=level, name=node.get_fully_qualified_name(),
                              hardware_id="host-process:" + str(os.getpid()), message=message or state)
    status.values = [KeyValue(key=str(k), value=str(v)) for k, v in
                     ({"runtime_state": state, "hardware_execution_ready": False} | values).items()]
    result = DiagnosticArray(status=[status])
    result.header.stamp = node.get_clock().now().to_msg()
    publisher.publish(result)
