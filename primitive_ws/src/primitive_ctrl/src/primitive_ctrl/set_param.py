'''
-> RUN INSTRUCTIONS <-: 
ros2 run primitive_ctrl set_param --ros-args --params-file src/primitive_ctrl/config/solver_safe_params.yaml

-> FUNCTION OF THE NODE <- Modifying following Parameters:
NODE NAME : /cartesian_motion_controller
PARAM NAMES : 
    Parameter name: solver.error_scale
        Type: double
        Constraints:
    Parameter name: solver.iterations
        Type: integer
        Constraints:

-> EQUIVALENT TERMINAL CMDS <-
ros2 param set /cartesian_motion_controller solver.error_scale 0.05
ros2 param get /cartesian_motion_controller solver.error_scale
'''

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType



# This function edits the "Text Color" or "Background Color" of terminal text using ANSI codes
def colorize(color_code, message):
    # TEXT::: color_code = 31<Red>, 32<Green>, 93<Yellow>, 34<Blue>, 36<Cyan>, 95<BrtMagenta>
    # BACKGRD:: color_code = 41<Red>, 42<Green>, 103<Yellow>, 44<Blue>, 46<Cyan>, 105<BrtMagenta>
    # \033[  <Initiate ANSI escape code>, and  \033[0m  <Reset formatting to default>
    return f"\033[{color_code}m{message}\033[0m"


class ParameterSetter(Node):
    def __init__(self):
        super().__init__('set_param_node')
        self.get_logger().info(colorize(42, "Parameter Setter Node has started."))

        # Define the target node and parameter
        self.target_node = '/cartesian_motion_controller'
        
        # YAML Parameters that are being read:
        self.declare_parameter('solver-error_scale', 0.1)
        self.declare_parameter('solver-iterations', 25)
        # For 'dtype_code', see: https://docs.ros2.org/foxy/api/rcl_interfaces/msg/ParameterType.html
        # For 'dtype', see: https://docs.ros2.org/foxy/api/rcl_interfaces/msg/ParameterValue.html
        PARAMETERS_TO_MODIFY = [
                                {'name': 'solver.error_scale',
                                'value': self.get_parameter('solver-error_scale').value,  #0.027,
                                'dtype_code': ParameterType.PARAMETER_DOUBLE,
                                'dtype': 'double_value'
                                },

                                {'name': 'solver.iterations',
                                'value': self.get_parameter('solver-iterations').value,  #27,
                                'dtype_code': ParameterType.PARAMETER_INTEGER,
                                'dtype': 'integer_value'
                                }
                            ]

        # Get the Parameters that need to be modified, with the values and dtype details
        self.mod_param_details = PARAMETERS_TO_MODIFY

        # Create a service client for the target node's set_parameters service
        self.parameter_client = self.create_client(SetParameters, 
                                                   f'{self.target_node}/set_parameters')

        # Wait for the service to be available
        self.get_logger().info(colorize(103, f"Waiting for parameter service on {self.target_node}..."))
        if not self.parameter_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(colorize(41, f"Parameter service not available on {self.target_node}"))
            return

        # Set the parameter
        self.set_parameters()


    def set_parameters(self):
        list_of_params = []
        for param_dets in self.mod_param_details:
            # 
            param_name = param_dets['name']
            param_val = param_dets['value']
            param_dtype_code = param_dets['dtype_code']
            param_dtype = param_dets['dtype']
            # Create a Parameter Value ;  Set its "double_value" or "integer_value" or similar
            # SEE: https://docs.ros2.org/foxy/api/rcl_interfaces/msg/ParameterValue.html
            param_value = ParameterValue(type = param_dtype_code)
            setattr(param_value, f"{param_dtype}", param_val)
            # Create a parameter object
            parameter = Parameter(
                            name = param_name,
                            value = param_value    # Type 3 corresponds to double)  
                        )
            list_of_params.append(parameter)
        
        # Create a Request
        request = SetParameters.Request(parameters = list_of_params)
        # Call the service
        future = self.parameter_client.call_async(request)
        future.add_done_callback(self.parameter_callback)
        
        '''rclpy.spin_until_future_complete(self, future)'''


    def parameter_callback(self, future):
        try:
            response = future.result()
            print(response.results)
            if all(result.successful for result in response.results):
                self.get_logger().info(colorize(42, f"All parameters {[a['name'] for a in self.mod_param_details]}"  +
                                         " set successfully."))
            else:
                self.get_logger().error(colorize(41, f"Some parameters failed to set."))
        except Exception as e:
            self.get_logger().error(colorize(41, f"Service call failed: {e}"))
        finally:
            # Shut down the node after setting the parameters
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ParameterSetter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Parameter Setter Node.")
    finally:
        node.destroy_node()
        #rclpy.shutdown()


if __name__ == '__main__':
    main()
