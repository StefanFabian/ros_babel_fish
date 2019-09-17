The data for the benchmark can be downloaded using the Python 3 download script in the project folder.
The bagfile contains 
Subscribes to a sensor_msgs/Image topic and sums the seq numbers in the header field.  
The bag file used for this benchmark contains 249 image messages, 2135 float msgs and 15926 joint state messages.

The results were obtained on Ubuntu 18.04 with ROS Melodic using an i7-6700K.  
Performance is compared to a baseline which is the known message and ros_type_introspection.

### Float Message Benchmark
Sums the value of the data field in the float messages.
 
##### Results
| Name | First | Avg without first
|---|---:|---:|
| Base | 0.05us | 0.04us |
| RBF | 2189.65us | 2.89us |
| RBF Extractor | 2429.07 us | 0.29 us |
| RTI | 142.43us | 1.81us |
 
### Image Simple Benchmark
Sums the seq number in the header field of large image messages.
The ros_type_introspection version knows about the Header message to use an optimized version that is much quicker
 in accessing the desired value.

##### Results
| Name | First | Avg without first
|---|---:|---:|
| Base | 0.07 us | 0.08 us |
| RBF | 3964.40 us | 10.23 us |
| RBF Extractor | 3535.41 us | 0.85 us |
| RTI | 3871.21 us | 811.36 us |

### Joint State Message Benchmark
Sums the joint state for the 'arm_roll_joint'. Not all messages contain the joint state for this joint.

##### Results
| Name | First | Avg without first
|---|---:|---:|
| Base | 0.51 us | 0.30 us |
| RBF | 3902.08 us | 3.35 us |
| RTI | 459.63 us | 4.65 us |
 