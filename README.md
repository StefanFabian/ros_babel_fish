# ros_babel_fish

This library enables ROS nodes written in C++ to communicate using message types that are unknown at compile time.

You can subscribe and even publish any available message type.  
It also supports both providing and calling a service.

You most likely don't need this library.  
Possible use cases where you do need it are:
* UIs displaying the content of various at compile time unknown messages
* Plugins for script languages that can not access the C++ message definitions without modification
  * Spot for shameless self-advertising: Check out my ROS QML plugin which uses this library to allow subscribing, publishing and more directly in QML 

The main focus of this library was usability but it is also very performant.  
Check out the examples in the example folder to find out how to use this library.

Here's an example on how to publish a message:

```
NodeHandle nh;
BabelFish fish;
// Advertise a publisher on topic /pose
ros::Publisher pub_pose = fish.advertise( nh, "geometry_msgs/Pose", "/pose", 1, true );

Message::Ptr message = fish.createMessage( "geometry_msgs/Pose" );
auto &compound = message->as<CompoundMessage>();
compound["position"]["x"].as<ValueMessage<double>>().setValue(1.1);
compound["position"]["y"].as<ValueMessage<double>>().setValue(2.4);
compound["position"]["z"].as<ValueMessage<double>>().setValue(3.6);

compound["orientation"]["w"].as<ValueMessage<double>>().setValue( 0.384 );
compound["orientation"]["x"].as<ValueMessage<double>>().setValue( -0.003 );
compound["orientation"]["y"].as<ValueMessage<double>>().setValue( -0.876 );
compound["orientation"]["z"].as<ValueMessage<double>>().setValue( 0.292 );

BabelFishMessage::Ptr translated_message = fish.translateMessage( message );
pub_pose.publish( translated_message );
```

## Current TODOs

- [x] Subscribing topics with at compile time unknown message type
- [x] Publishing messages with at compile time unknown message type
- [x] Calling a service with at compile time unknown service definition
- [x] Advertising a service with at compile time unknown service definition
- [ ] Creating a service client with at compile time unknown service definition
- [ ] Add more error checks to fail gracefully in case of errors
- [ ] Documentation (I'll probably get around to document this at the earliest in september)
- [ ] Calling actions
- [ ] Providing an action server(?)
