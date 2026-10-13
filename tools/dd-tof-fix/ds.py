# Applies the `/**/<plugin>` parameters from Duckietown's px4_config.yaml to the MAVROS plugin
# nodes (/mavros/<plugin>), since mavros_node does not pass them on (dt-ros2-interface bug 2026-09-23).
import os, sys, yaml, rclpy
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

CFG = os.environ.get("MAVROS_CONFIG", "/code/src/dt-ros2-interface/assets/mavros/px4_config.yaml")

def value(v):
    if isinstance(v, bool):  return ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=v)
    if isinstance(v, int):   return ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=v)
    if isinstance(v, float): return ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=v)
    if isinstance(v, str):   return ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=v)
    raise TypeError(f"unsupported type: {v!r}")

conf = yaml.safe_load(open(CFG))
rclpy.init()
node = rclpy.create_node("dd_mavros_param_fix")
# use_quaternion cannot be changed at runtime with rmw_zenoh (mavros recreates the subscription and
# fails with "Invalid history policy"), so it is skipped: use /mavros/setpoint_raw/attitude instead.
SKIP = {("/mavros/setpoint_attitude", "use_quaternion")}
ok = True  # only distance_sensor is critical (without it PX4 has no altitude)
for key, body in conf.items():
    if not key.startswith("/**/"):
        continue  # /mavros (allowlist) is applied normally
    target = "/mavros/" + key[len("/**/"):]
    params = [Parameter(name=k, value=value(v)) for k, v in body["ros__parameters"].items()
              if (target, k) not in SKIP]
    if not params:
        continue
    cli = node.create_client(SetParameters, target + "/set_parameters")
    if not cli.wait_for_service(timeout_sec=20):
        print(f"{target}: service not available"); ok &= target != "/mavros/distance_sensor"; continue
    fut = cli.call_async(SetParameters.Request(parameters=params))
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10)
    res = fut.result()
    good = res is not None and all(r.successful for r in res.results)
    print(f"{target}: {[p.name for p in params]} -> {'OK' if good else 'FAILED'}")
    if target == "/mavros/distance_sensor":
        ok &= good
sys.exit(0 if ok else 1)
