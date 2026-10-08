# Aplica a los nodos plugin de MAVROS (/mavros/<plugin>) los parámetros `/**/<plugin>` del
# px4_config.yaml de Duckietown, que mavros_node no les pasa (bug de dt-ros2-interface 2026-09-23).
import os, sys, yaml, rclpy
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

CFG = os.environ.get("MAVROS_CONFIG", "/code/src/dt-ros2-interface/assets/mavros/px4_config.yaml")

def value(v):
    if isinstance(v, bool):  return ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=v)
    if isinstance(v, int):   return ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=v)
    if isinstance(v, float): return ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=v)
    if isinstance(v, str):   return ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=v)
    raise TypeError(f"tipo no soportado: {v!r}")

conf = yaml.safe_load(open(CFG))
rclpy.init()
node = rclpy.create_node("dd_mavros_param_fix")
# use_quaternion no se puede cambiar en caliente con rmw_zenoh (mavros recrea la suscripción y
# falla con "Invalid history policy"); se omite: usar /mavros/setpoint_raw/attitude en su lugar.
SKIP = {("/mavros/setpoint_attitude", "use_quaternion")}
ok = True  # solo el distance_sensor es crítico (sin él PX4 no tiene altitud)
for key, body in conf.items():
    if not key.startswith("/**/"):
        continue  # /mavros (allowlist) sí se aplica normalmente
    target = "/mavros/" + key[len("/**/"):]
    params = [Parameter(name=k, value=value(v)) for k, v in body["ros__parameters"].items()
              if (target, k) not in SKIP]
    if not params:
        continue
    cli = node.create_client(SetParameters, target + "/set_parameters")
    if not cli.wait_for_service(timeout_sec=20):
        print(f"{target}: servicio no disponible"); ok &= target != "/mavros/distance_sensor"; continue
    fut = cli.call_async(SetParameters.Request(parameters=params))
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10)
    res = fut.result()
    good = res is not None and all(r.successful for r in res.results)
    print(f"{target}: {[p.name for p in params]} -> {'OK' if good else 'FALLO'}")
    if target == "/mavros/distance_sensor":
        ok &= good
sys.exit(0 if ok else 1)
