[![Build Status](https://travis-ci.org/StefanFabian/ros_babel_fish.svg?branch=master)](https://travis-ci.org/StefanFabian/ros_babel_fish)
[![codecov](https://codecov.io/gh/StefanFabian/ros_babel_fish/branch/master/graph/badge.svg)](https://codecov.io/gh/StefanFabian/ros_babel_fish)

### ALPHA: This library is currently in alpha stadium. Tests for >90% line coverage are written but it requires more testing.

This library enables ROS nodes written in C++ to communicate using message types that are unknown at compile time.

You can subscribe and even publish any available message type.  
It also supports both providing and calling a service.

Please strongly consider whether you actually need this library because there may be a better solution.   
Possible use cases where you do need it are:
* UIs displaying the content of various at compile time unknown messages
* Plugins for (script) languages that can not access the C++ message definitions without modification
  * Spot for shameless self-advertising: Check out my ROS QML plugin which uses this library to allow subscribing, publishing and more directly in QML 

The main focus of this library was usability but it is also very performant since it uses a lazy copy mechanism for bigger fields such as big arrays.  
Instead of copying the message it will retain a pointer at the start of the field. A copy is only made if explicitly requested or a value in the
field is changed.  

## Examples
### Subscribing
```C++
NodeHandle nh;
BabelFish fish;
// Subscribe topic /pose
ros::Subscriber sub = nh.subscribe<ros_babel_fish::BabelFishMessage>( topic, 1, &callback );

/* ... */

void callback( const ros_babel_fish::BabelFishMessage::ConstPtr &msg )
{
  std::cout << "Message received!" << std::endl;
  std::cout << "Data Type:" << msg->dataType() << std::endl; // geometry_msgs/Pose
  std::cout << "MD5:" << msg->md5Sum() << std::endl; // e45d45a5a1ce597b249e23fb30fc871f
  std::cout << "Message Definition:" << std::endl;
  std::cout << msg->definition() << std::endl;
  std::cout << "Message Content:" << std::endl;
  TranslatedMessage::Ptr translated = fish->translateMessage( msg );
  auto &compound = translated->translated_message->as<CompoundMessage>();
  std::cout << "Position: " << compound["position"]["x"].value<double>() << ", " << compound["position"]["y"].value<double>() << ", "
            << compound["position"]["z"].value<double>() << std::endl;
  std::cout << "Orientation: " << compound["orientation"]["w"].value<double>() << ", " << compound["orientation"]["x"].value<double>() << ", "
            << compound["orientation"]["y"].value<double>() << ", " << compound["orientation"]["z"].value<double>() << std::endl;
};
```

### Publishing
```C++
NodeHandle nh;
BabelFish fish;
// Advertise a publisher on topic /pose
ros::Publisher pub_pose = fish.advertise( nh, "geometry_msgs/Pose", "/pose", 1, true );

Message::Ptr message = fish.createMessage( "geometry_msgs/Pose" );
auto &compound = message->as<CompoundMessage>();
compound["position"].as<CompoundMessage>()["x"].as<ValueMessage<double>>().setValue(1.1);
// or using convenience methods
compound["position"]["y"].as<ValueMessage<double>>().setValue(2.4);
// or using even more convenience methods
compound["position"]["z"] = 3.6;
// Be careful with your types here. Casting to a wrong type will throw an exception!
// The as<ValueMessage<T>> method is also a bit faster because the convenience method
// will automatically convert to the right type and perform bound and compatibility checks.
// This makes it more robust but comes with a little overhead.
// Note that assigning a double to a float field will always throw an exception because
// the float may not have the required resolution.
// Assigning, e.g., an int to a uint8 field will only throw if the int is out of bounds (0-255)
// otherwise a warning will be printed because uint8 is not compatible with all possible values
// of int. This warning can be disabled as a compile option. 

compound["orientation"]["w"] = 0.384;
compound["orientation"]["x"] = -0.003;
compound["orientation"]["y"] = -0.876;
compound["orientation"]["z"] = 0.292;

BabelFishMessage::Ptr translated_message = fish.translateMessage( message );
pub_pose.publish( translated_message );
```

### Message Extraction
If you are only interested in a specific part of the message, you may not want to deserialize the entire message.
```C++
BabelFish fish;
MessageExtractor extractor(fish);
// We want the position of a pose stamped
SubMessageLocation location = extractor.retrieveLocationForPath( "geometry_msgs/PoseStamped", "pose.position" );
BabelFishMessage::ConstPtr msg = ros::topic::waitForMessage<BabelFishMessage>( "/topic" );
TranslatedMessage::Ptr translated = extractor.extractMessage( msg, location );
auto &compound = translated->translated_message->as<CompoundMessage>();
std::cout << "Position: " << compound["x"].value<double>() << ", " << compound["y"].value<double>() << ", "
          << compound["z"].value<double>() << std::endl;

// You can also extract primitives (but you need to make sure you extract the right type or the extractor will throw!)
SubMessageLocation location_of_x = extractor.retrieveLocationForPath( "geometry_msgs/PoseStamped", "pose.position.x" );
std::cout << "Position X: " << extractor.extractValue<double>( msg, location_of_x ) << std::endl;
```

For more in-depth examples check the example folder.


## Benchmarks
Check the separate [ros_babel_fish_benchmarks repo](https://github.com/StefanFabian/ros_babel_fish_benchmarks) for benchmarks.

## Current TODOs

- [x] Calling a service with at compile time unknown service definition
- [x] Advertising a service with at compile time unknown service definition
- [ ] Creating a service client with at compile time unknown service definition
- [ ] Search more paths when looking up message definitions
- [ ] Documentation
- [x] Benchmark
- [ ] Calling actions
- [ ] Providing an action server(?)
