//
// Created by Stefan Fabian on 07.09.19.
//

#include "message_comparison.h"

#include <ros_babel_fish/generation/message_creation.h>
#include <ros_babel_fish/generation/message_template.h>
#include <ros_babel_fish/messages/internal/value_compatibility.h>
#include <ros_babel_fish/babel_fish.h>

#include <rosgraph_msgs/Log.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

using namespace ros_babel_fish;

template<typename T>
::testing::AssertionResult compareArrays( const T *a, const T *b, size_t count )
{
  for ( size_t i = 0; i < count; ++i )
  {
    if ( a[i] == b[i] ) continue;
    return ::testing::AssertionFailure() << "Difference at index " << i << ":" << std::endl
                                         << "a[[" << i << "]: " << a[i] << std::endl
                                         << "b[" << i << "]: " << b[i];
  }
  return ::testing::AssertionSuccess();
}

template<typename T1, typename T2>
::testing::AssertionResult compareVectors( const T1 &a, const T2 &b, size_t count )
{
  for ( size_t i = 0; i < count; ++i )
  {
    if ( a[i] == b[i] ) continue;
    return ::testing::AssertionFailure() << "Difference at index " << i << ":" << std::endl
                                         << "a[[" << i << "]: " << a[i] << std::endl
                                         << "b[" << i << "]: " << b[i];
  }
  return ::testing::AssertionSuccess();
}

TEST( MessageTest, message )
{
  // UINT8
  {
    ValueMessage<uint8_t> vm_in( 42 );
    Message &vm = vm_in;
    EXPECT_THROW( vm["invalid"], BabelFishException );
    const Message &vmc = vm;
    EXPECT_THROW( vmc["answer"], BabelFishException );
    EXPECT_EQ( vm.value<uint8_t>(), 42U );
    EXPECT_NO_THROW( vm = uint8_t( 55 ));
    EXPECT_EQ( vmc.value<int32_t>(), 55 );
    EXPECT_THROW( vm = true, BabelFishException );

    // This is within the limits of the type and should trigger a warning at most but not throw
    EXPECT_NO_THROW( vm = 255 );
    EXPECT_EQ( vm.value<uint8_t>(), 255 );
    // This is not within the limits of the type and should throw
    EXPECT_THROW( vm = 256, BabelFishException );

    EXPECT_THROW( vm = true, BabelFishException );
    EXPECT_THROW( vm = ros::Time( 42 ), BabelFishException );
    EXPECT_THROW( vm = ros::Duration( 42 ), BabelFishException );
    EXPECT_THROW( vm.value<std::string>(), BabelFishException );
    EXPECT_THROW( vm.value<ros::Duration>(), BabelFishException );
    EXPECT_THROW( vm.value<bool>(), BabelFishException );

    ASSERT_EQ( vm._sizeInBytes(), 1U );
    uint8_t stream[1];
    EXPECT_EQ( vm.writeToStream( stream ), 1U );
    size_t bytes_read = 0;
    Message::Ptr msg = createValueMessageFromData( MessageTypes::UInt8, stream, bytes_read );
    EXPECT_EQ( bytes_read, 1U );
    EXPECT_EQ( msg->value<uint8_t>(), 255U );
  }

  // UINT16
  {
    ValueMessage<uint16_t> vm_in( 42 );
    Message &vm = vm_in;
    EXPECT_EQ( vm.value<uint16_t>(), 42U );
    EXPECT_NO_THROW( vm = uint16_t( 355 ));
    EXPECT_EQ( vm.value<int32_t>(), 355 );

    ASSERT_EQ( vm._sizeInBytes(), 2U );
    uint8_t stream[2];
    EXPECT_EQ( vm.writeToStream( stream ), 2U );
    size_t bytes_read = 0;
    Message::Ptr msg = createValueMessageFromData( MessageTypes::UInt16, stream, bytes_read );
    EXPECT_EQ( bytes_read, 2U );
    EXPECT_EQ( msg->value<uint16_t>(), 355 );
  }

  // UINT32
  {
    ValueMessage<uint32_t> vm_in( 42 );
    Message &vm = vm_in;
    EXPECT_EQ( vm.value<uint32_t>(), 42U );
    EXPECT_NO_THROW( vm = uint32_t( 133755 ));
    EXPECT_EQ( vm.value<int64_t>(), 133755 );

    ASSERT_EQ( vm._sizeInBytes(), 4U );
    uint8_t stream[4];
    EXPECT_EQ( vm.writeToStream( stream ), 4U );
    size_t bytes_read = 0;
    Message::Ptr msg = createValueMessageFromData( MessageTypes::UInt32, stream, bytes_read );
    EXPECT_EQ( bytes_read, 4U );
    EXPECT_EQ( msg->value<uint32_t>(), 133755U );
  }

  // UINT64
  {
    ValueMessage<uint64_t> vm_in( 42 );
    Message &vm = vm_in;
    EXPECT_EQ( vm.value<uint64_t>(), 42U );
    EXPECT_NO_THROW( vm = uint64_t( 2 ) << uint64_t( 34U ));
    EXPECT_EQ( vm.value<int64_t>(), int64_t( 2 ) << int64_t( 34 ));

    ASSERT_EQ( vm._sizeInBytes(), 8U );
    uint8_t stream[8];
    EXPECT_EQ( vm.writeToStream( stream ), 8U );
    size_t bytes_read = 0;
    Message::Ptr msg = createValueMessageFromData( MessageTypes::UInt64, stream, bytes_read );
    EXPECT_EQ( bytes_read, 8U );
    EXPECT_EQ( msg->value<uint64_t>(), uint64_t( 2 ) << uint64_t( 34 ));
  }

  // INT8
  {
    ValueMessage<int8_t> vm_in( 42 );
    Message &vm = vm_in;
    EXPECT_THROW( vm["invalid"], BabelFishException );
    const Message &vmc = vm;
    EXPECT_THROW( vmc["answer"], BabelFishException );
    EXPECT_EQ( vm.value<int8_t>(), 42 );
    EXPECT_NO_THROW( vm = int8_t( 55 ));
    EXPECT_EQ( vmc.value<int32_t>(), 55 );
    EXPECT_THROW( vm = true, BabelFishException );

    // This is within the limits of the type and should trigger a warning at most but not throw
    EXPECT_NO_THROW( vm = 127 );
    EXPECT_EQ( vm.value<int8_t>(), 127 );
    // This is not within the limits of the type and should throw
    EXPECT_THROW( vm = 128, BabelFishException );
    EXPECT_THROW( vm = -129, BabelFishException );

    EXPECT_THROW( vm = true, BabelFishException );
    EXPECT_THROW( vm = ros::Time( 42 ), BabelFishException );
    EXPECT_THROW( vm = ros::Duration( 42 ), BabelFishException );

    ASSERT_EQ( vm._sizeInBytes(), 1U );
    uint8_t stream[1];
    EXPECT_EQ( vm.writeToStream( stream ), 1U );
    size_t bytes_read = 0;
    Message::Ptr msg = createValueMessageFromData( MessageTypes::Int8, stream, bytes_read );
    EXPECT_EQ( bytes_read, 1U );
    EXPECT_EQ( msg->value<int8_t>(), 127 );
  }

  // INT16
  {
    ValueMessage<int16_t> vm_in( 42 );
    Message &vm = vm_in;
    EXPECT_EQ( vm.value<int16_t>(), 42 );
    EXPECT_NO_THROW( vm = int16_t( 129 ));
    EXPECT_EQ( vm.value<int32_t>(), 129 );

    ASSERT_EQ( vm._sizeInBytes(), 2U );
    uint8_t stream[2];
    EXPECT_EQ( vm.writeToStream( stream ), 2U );
    size_t bytes_read = 0;
    Message::Ptr msg = createValueMessageFromData( MessageTypes::Int16, stream, bytes_read );
    EXPECT_EQ( bytes_read, 2U );
    EXPECT_EQ( msg->value<int16_t>(), 129 );
  }

  // INT32
  {
    ValueMessage<int32_t> vm_in( 42 );
    Message &vm = vm_in;
    EXPECT_EQ( vm.value<int32_t>(), 42 );
    EXPECT_NO_THROW( vm = int32_t( 70501 ));
    EXPECT_EQ( vm.value<int32_t>(), 70501 );

    ASSERT_EQ( vm._sizeInBytes(), 4U );
    uint8_t stream[4];
    EXPECT_EQ( vm.writeToStream( stream ), 4U );
    size_t bytes_read = 0;
    Message::Ptr msg = createValueMessageFromData( MessageTypes::Int32, stream, bytes_read );
    EXPECT_EQ( bytes_read, 4U );
    EXPECT_EQ( msg->value<int32_t>(), 70501 );
  }

  // INT64
  {
    ValueMessage<int64_t> vm_in( 42 );
    Message &vm = vm_in;
    EXPECT_EQ( vm.value<int64_t>(), 42 );
    EXPECT_NO_THROW( vm = static_cast<int64_t >(int64_t( 2 ) << int64_t( 34 )));
    EXPECT_EQ( vm.value<int64_t>(), int64_t( 2 ) << int64_t( 34 ));
    EXPECT_THROW( vm.value<int32_t>(), BabelFishException );

    ASSERT_EQ( vm._sizeInBytes(), 8U );
    uint8_t stream[8];
    EXPECT_EQ( vm.writeToStream( stream ), 8U );
    size_t bytes_read = 0;
    Message::Ptr msg = createValueMessageFromData( MessageTypes::Int64, stream, bytes_read );
    EXPECT_EQ( bytes_read, 8U );
    EXPECT_EQ( msg->value<int64_t>(), int64_t( 2 ) << int64_t( 34 ));
  }

  // BOOL
  {
    ValueMessage<bool> vm_in( false );
    Message &vm = vm_in;
    EXPECT_FALSE( vm.value<bool>());
    EXPECT_THROW( vm.value<int32_t>(), BabelFishException );
    EXPECT_NO_THROW( vm = true );
    EXPECT_TRUE( vm.value<bool>());
    EXPECT_THROW( vm = 42, BabelFishException );

    ASSERT_EQ( vm._sizeInBytes(), 1U );
    uint8_t stream[1];
    EXPECT_EQ( vm.writeToStream( stream ), 1U );
    size_t bytes_read = 0;
    Message::Ptr msg = createValueMessageFromData( MessageTypes::Bool, stream, bytes_read );
    EXPECT_EQ( bytes_read, 1U );
    EXPECT_EQ( msg->value<bool>(), true );
  }

  // FLOAT
  {
    ValueMessage<float> vm_in( 42.0 );
    Message &vm = vm_in;
    EXPECT_FLOAT_EQ( vm.value<float>(), 42.0f );
    ASSERT_NO_THROW( vm.value<double>());
    EXPECT_DOUBLE_EQ( vm.value<double>(), 42.0 );
    EXPECT_NO_THROW( vm = 50.0f );
    EXPECT_FLOAT_EQ( vm.value<float>(), 50.0f );
    EXPECT_NO_THROW( vm = 42.0 );
    EXPECT_FLOAT_EQ( vm.value<float>(), 42.0f );
    EXPECT_NO_THROW( vm = 42 );
    EXPECT_FLOAT_EQ( vm.value<float>(), 42.0f );
    EXPECT_EQ( vm.value<int32_t>(), 42 );

    ASSERT_EQ( vm._sizeInBytes(), 4U );
    uint8_t stream[4];
    EXPECT_EQ( vm.writeToStream( stream ), 4U );
    size_t bytes_read = 0;
    Message::Ptr msg = createValueMessageFromData( MessageTypes::Float32, stream, bytes_read );
    EXPECT_EQ( bytes_read, 4U );
    EXPECT_FLOAT_EQ( msg->value<float>(), 42.0f );
  }

  // DOUBLE
  {
    ValueMessage<double> vm_in( 42.0 );
    Message &vm = vm_in;
    EXPECT_DOUBLE_EQ( vm.value<double>(), 42.0 );
    EXPECT_FLOAT_EQ( vm.value<float>(), 42.0f );
    EXPECT_NO_THROW( vm = 50.0f );
    EXPECT_DOUBLE_EQ( vm.value<double>(), 50.0 );
    EXPECT_NO_THROW( vm = 42.0 );
    EXPECT_DOUBLE_EQ( vm.value<double>(), 42.0 );
    EXPECT_NO_THROW( vm = 50 );
    EXPECT_DOUBLE_EQ( vm.value<double>(), 50.0 );
    EXPECT_EQ( vm.value<int32_t>(), 50 );

    ASSERT_EQ( vm._sizeInBytes(), 8U );
    uint8_t stream[8];
    EXPECT_EQ( vm.writeToStream( stream ), 8U );
    size_t bytes_read = 0;
    Message::Ptr msg = createValueMessageFromData( MessageTypes::Float64, stream, bytes_read );
    EXPECT_EQ( bytes_read, 8U );
    EXPECT_DOUBLE_EQ( msg->value<double>(), 50.0 );
  }

  // TIME
  {
    ValueMessage<ros::Time> vm_in( ros::Time( 1.0 ));
    Message &vm = vm_in;
    EXPECT_EQ( vm.value<ros::Time>(), ros::Time( 1.0 ));
    EXPECT_NO_THROW( vm = ros::Time( 42.0 ));
    EXPECT_EQ( vm.value<ros::Time>(), ros::Time( 42.0 ));
    EXPECT_THROW( vm = "test", BabelFishException );
    EXPECT_THROW( vm.value<int32_t>(), BabelFishException );
    EXPECT_THROW( vm.value<double>(), BabelFishException );
    EXPECT_THROW( vm = 42.0, BabelFishException );

    ASSERT_EQ( vm._sizeInBytes(), 8U );
    uint8_t stream[8];
    EXPECT_EQ( vm.writeToStream( stream ), 8U );
    size_t bytes_read = 0;
    Message::Ptr msg = createValueMessageFromData( MessageTypes::Time, stream, bytes_read );
    EXPECT_EQ( bytes_read, 8U );
    EXPECT_EQ( msg->value<ros::Time>(), ros::Time( 42.0 ));
  }

  // DURATION
  {
    ValueMessage<ros::Duration> vm_in( ros::Duration( -1.0 ));
    Message &vm = vm_in;
    EXPECT_EQ( vm.value<ros::Duration>(), ros::Duration( -1.0 ));
    EXPECT_NO_THROW( vm = ros::Duration( 42.0 ));
    EXPECT_EQ( vm.value<ros::Duration>(), ros::Duration( 42.0 ));
    EXPECT_THROW( vm.value<int32_t>(), BabelFishException );
    EXPECT_THROW( vm.value<double>(), BabelFishException );
    EXPECT_THROW( vm = 12.0, BabelFishException );

    ASSERT_EQ( vm._sizeInBytes(), 8U );
    uint8_t stream[8];
    EXPECT_EQ( vm.writeToStream( stream ), 8U );
    size_t bytes_read = 0;
    Message::Ptr msg = createValueMessageFromData( MessageTypes::Duration, stream, bytes_read );
    EXPECT_EQ( bytes_read, 8U );
    EXPECT_EQ( msg->value<ros::Duration>(), ros::Duration( 42.0 ));
  }

  // STRING
  {
    ValueMessage<std::string> vm_in( "test" );
    Message &vm = vm_in;
    EXPECT_EQ( vm.value<std::string>(), "test" );
    EXPECT_NO_THROW( vm = "the answer" );
    EXPECT_EQ( vm.value<std::string>(), "the answer" );
    EXPECT_NO_THROW( vm = std::string( "42" ));
    EXPECT_EQ( vm.value<std::string>(), "42" );
    EXPECT_THROW( vm.value<int32_t>(), BabelFishException );
    EXPECT_THROW( vm.value<double>(), BabelFishException );
    EXPECT_THROW( vm.value<ros::Time>(), BabelFishException );
    EXPECT_THROW( vm = 12.0, BabelFishException );

    ASSERT_EQ( vm._sizeInBytes(), 6U );
    uint8_t stream[6];
    EXPECT_EQ( vm.writeToStream( stream ), 6U );
    size_t bytes_read = 0;
    Message::Ptr msg = createValueMessageFromData( MessageTypes::String, stream, bytes_read );
    EXPECT_EQ( bytes_read, 6U );
    EXPECT_EQ( msg->value<std::string>(), "42" );
  }

  // COMPOUND
  {
    ros_babel_fish::BabelFish fish;
    CompoundMessage m( fish.descriptionProvider()->getMessageDescription( "std_msgs/Header" )->message_template );
    Message &vm = m;
    EXPECT_THROW( vm = 42, BabelFishException );
  }

  size_t bytes_read = 0;
  uint8_t stream[2048];
  EXPECT_THROW( createValueMessageFromData( MessageTypes::Compound, stream, bytes_read ), BabelFishException );
  EXPECT_THROW( createValueMessageFromData( MessageTypes::Array, stream, bytes_read ), BabelFishException );
  EXPECT_THROW( createValueMessageFromData( MessageTypes::None, stream, bytes_read ), BabelFishException );
}

TEST( MessageTest, valueMessage )
{
  // BOOL
  {
    uint8_t stream[1] = { 1 };
    uint8_t copy_stream[1] = { 0 };
    size_t bytes_read = 0;
    ValueMessage<bool> *vm = ValueMessage<bool>::fromStream( reinterpret_cast<const uint8_t *>(stream), 1, bytes_read );
    EXPECT_EQ( vm->getValue(), true );
    EXPECT_EQ( bytes_read, 1U );
    EXPECT_EQ( vm->_sizeInBytes(), 1U );

    vm->setValue( false );
    EXPECT_EQ( vm->getValue(), false );
    EXPECT_EQ( stream[0], true );
    EXPECT_EQ( vm->writeToStream( reinterpret_cast<uint8_t *>(stream)), 1U );
    EXPECT_EQ( stream[0], false );
    delete vm;

    stream[0] = true;
    vm = new ValueMessage<bool>( reinterpret_cast<const uint8_t *>(stream));
    EXPECT_EQ( vm->getValue(), true );
    EXPECT_EQ( vm->writeToStream( reinterpret_cast<uint8_t *>(copy_stream)), 1U );
    EXPECT_EQ( copy_stream[0], true );
    delete vm;
  }

  // TIME
  {
    uint32_t stream[2];
    stream[0] = 42; // secs
    stream[1] = 1337; // nsecs
    uint32_t copy_stream[2] = { 0, 0 };
    size_t bytes_read = 0;
    ValueMessage<ros::Time> *vm = ValueMessage<ros::Time>::fromStream( reinterpret_cast<const uint8_t *>(stream), 8,
                                                                       bytes_read );
    EXPECT_EQ( vm->getValue(), ros::Time( 42, 1337 ));
    EXPECT_EQ( bytes_read, 8U );
    EXPECT_EQ( vm->_sizeInBytes(), 8U );

    vm->setValue( ros::Time( 55, 0 ));
    EXPECT_EQ( vm->getValue(), ros::Time( 55, 0 ));
    EXPECT_EQ( stream[0], 42U );
    EXPECT_EQ( stream[1], 1337U );
    EXPECT_EQ( vm->writeToStream( reinterpret_cast<uint8_t *>(stream)), 8U );
    EXPECT_EQ( stream[0], 55U );
    EXPECT_EQ( stream[1], 0U );
    delete vm;

    stream[1] = 1337;
    vm = new ValueMessage<ros::Time>( reinterpret_cast<const uint8_t *>(stream));
    EXPECT_EQ( vm->getValue(), ros::Time( 55, 1337 ));
    EXPECT_EQ( vm->writeToStream( reinterpret_cast<uint8_t *>(copy_stream)), 8U );
    EXPECT_EQ( copy_stream[0], 55U );
    EXPECT_EQ( copy_stream[1], 1337U );
    delete vm;
  }

  // DURATION
  {
    int32_t stream[2];
    stream[0] = -42; // secs
    stream[1] = 1337; // nsecs
    int32_t copy_stream[2] = { 0, 0 };
    size_t bytes_read = 0;
    ValueMessage<ros::Duration> *vm = ValueMessage<ros::Duration>::fromStream(
      reinterpret_cast<const uint8_t *>(stream), 8,
      bytes_read );
    EXPECT_EQ( vm->getValue(), ros::Duration( -42, 1337 ));
    EXPECT_EQ( bytes_read, 8U );
    EXPECT_EQ( vm->_sizeInBytes(), 8U );

    vm->setValue( ros::Duration( -55, 0 ));
    EXPECT_EQ( vm->getValue(), ros::Duration( -55, 0 ));
    EXPECT_EQ( stream[0], -42 );
    EXPECT_EQ( stream[1], 1337 );
    EXPECT_EQ( vm->writeToStream( reinterpret_cast<uint8_t *>(stream)), 8U );
    EXPECT_EQ( stream[0], -55 );
    EXPECT_EQ( stream[1], 0 );
    delete vm;

    stream[1] = 1337;
    vm = new ValueMessage<ros::Duration>( reinterpret_cast<const uint8_t *>(stream));
    EXPECT_EQ( vm->getValue(), ros::Duration( -55, 1337 ));
    EXPECT_EQ( vm->writeToStream( reinterpret_cast<uint8_t *>(copy_stream)), 8U );
    EXPECT_EQ( copy_stream[0], -55 );
    EXPECT_EQ( copy_stream[1], 1337 );

    Message *clone = vm->clone();
    ASSERT_NE( clone, nullptr );
    EXPECT_EQ( clone->as<ValueMessage<ros::Duration>>().getValue(), vm->getValue());
    delete clone;

    vm->detachFromStream();
    stream[0] = -43;
    EXPECT_EQ( vm->getValue(), ros::Duration( -55, 1337 ));

    Message &m = *vm;
    ValueMessage<ros::Duration> vm_copy;
    Message &m_copy = vm_copy;
    m_copy = m;
    EXPECT_EQ( vm_copy.getValue(), vm->getValue());

    delete vm;
  }

  // STRING
  {
    const char stream[25] = { 21, 0, 0, 0, 't', 'h', 'i', 's', ' ', 'i', 's', ' ', 'a', ' ', 't', 'e', 's', 't', ' ',
                              's', 't', 'r',
                              'i', 'n', 'g' };
    char copy_stream[50];
    std::fill_n( copy_stream, 50, '\0' );
    auto *vm = new ValueMessage<std::string>( reinterpret_cast<const uint8_t *>(stream));
    EXPECT_EQ( vm->getValue(), "this is a test string" );
    EXPECT_EQ( vm->_sizeInBytes(), 25U );
    EXPECT_EQ( vm->writeToStream( reinterpret_cast<uint8_t *>(copy_stream)), 25U );
    EXPECT_TRUE( compareArrays( stream, copy_stream, 25 ));
    delete vm;
  }
}

TEST( MessageTest, compoundMessage )
{
  MessageTemplate::Ptr tmpl = std::make_shared<MessageTemplate>();
  tmpl->type = MessageTypes::Compound;
  tmpl->compound.datatype = "random_type/Msg";
  tmpl->compound.names.emplace_back( "Test" );
  tmpl->compound.names.emplace_back( "OtherKey" );
  MessageTemplate::Ptr test_tmpl = std::make_shared<MessageTemplate>();
  test_tmpl->type = MessageTypes::Bool;
  MessageTemplate::Ptr other_key_tmpl = std::make_shared<MessageTemplate>();
  other_key_tmpl->type = MessageTypes::Int32;
  tmpl->compound.types.push_back( test_tmpl );
  tmpl->compound.types.push_back( other_key_tmpl );
  // TODO New tests for CompoundMessage
  CompoundMessage cm( tmpl );
  const CompoundMessage &ccm = cm;
  cm["Test"] = false;
  cm["OtherKey"].as<ValueMessage<int32_t>>().setValue( 42 );
  ASSERT_TRUE( cm.containsKey( "Test" ));
  ASSERT_EQ( cm["Test"].type(), MessageTypes::Bool );
  EXPECT_EQ( cm["Test"].as<ValueMessage<bool>>().getValue(), false );
  ASSERT_EQ( cm["OtherKey"].type(), MessageTypes::Int32 );
  EXPECT_EQ( cm["OtherKey"].as<ValueMessage<int32_t>>().getValue(), 42 );
  ASSERT_EQ( cm.values().size(), 2U );
  EXPECT_EQ( cm.values()[0], &cm[cm.keys()[0]] );
  EXPECT_EQ( cm.values()[1], &cm[cm.keys()[1]] );

  ASSERT_FALSE( cm.containsKey( "Invalid" ));
  EXPECT_THROW( cm["Invalid"], std::runtime_error );
  EXPECT_THROW( ccm["Invalid"], std::runtime_error );

  auto *clone = dynamic_cast<CompoundMessage *>(cm.clone());
  EXPECT_NE( clone, nullptr );
  EXPECT_EQ( clone->keys().size(), 2U );
  EXPECT_TRUE( clone->containsKey( "Test" ));
  EXPECT_TRUE( clone->containsKey( "OtherKey" ));
  EXPECT_THROW((*clone)["Invalid"], std::runtime_error );
  delete clone;
}

TEST( MessageTest, arrayMessage )
{
  // Compound
  {
    BabelFish fish;
    CompoundArrayMessage am( fish.descriptionProvider()->getMessageDescription( "std_msgs/Header" )->message_template );
    for ( int i = 0; i < 20; ++i )
    {
      auto &cm = am.appendEmpty();
      cm["seq"] = i; // Not normally set but we won't send this message anyway
      cm["stamp"] = ros::Time( 20 * i );
      cm["frame_id"] = std::string( "frame " ) + std::to_string( i );
    }
    EXPECT_EQ( am.length(), 20U );
    EXPECT_THROW( am.assign( 20, nullptr ), BabelFishException );

    CompoundArrayMessage am_copy( am.elementTemplate());
    for ( int i = 0; i < 10; ++i )
    {
      auto *cm = fish.createMessage( "std_msgs/Header" )->clone();
      (*cm)["seq"] = 100 + i; // Not normally set but we won't send this message anyway
      (*cm)["stamp"] = ros::Time( 200 * i );
      (*cm)["frame_id"] = std::string( "copy frame " ) + std::to_string( i );
      am_copy.push_back( cm );
    }
    EXPECT_EQ( am_copy.length(), 10U );
    am_copy = am;
    EXPECT_EQ( am_copy.elementDataType(), "std_msgs/Header" );
    EXPECT_EQ( am_copy.length(), 20U );

    auto *am_clone = dynamic_cast<CompoundArrayMessage *>(am.clone());
    EXPECT_EQ( am_clone->length(), 20U );
    am_clone->at( 0 )["frame_id"] = "different_frame";
    EXPECT_EQ( am_clone->at( 0 )["frame_id"].value<std::string>(), "different_frame" );
    EXPECT_EQ( am[0]["frame_id"].value<std::string>(), "frame 0" );
    delete am_clone;

    CompoundArrayMessage different_am(
      fish.descriptionProvider()->getMessageDescription( "geometry_msgs/Pose" )->message_template );
    EXPECT_THROW( am_copy = different_am, BabelFishException );


    ArrayMessage<Message> aa( MessageTypes::Array );
    aa.push_back( am.clone());
    aa.push_back( am_copy.clone());
    EXPECT_EQ( aa.length(), 2U );
    EXPECT_EQ( aa[0].as<CompoundArrayMessage>()[0]["frame_id"].value<std::string>(), "frame 0" );
    auto *aa_clone = dynamic_cast<ArrayMessage<Message> *>(aa.clone());
    ASSERT_NE( aa_clone, nullptr );
    EXPECT_EQ( aa_clone->length(), 2U );
    EXPECT_EQ( aa[0].as<CompoundArrayMessage>()[0]["frame_id"].value<std::string>(), "frame 0" );
    delete aa_clone;
  }

  // BOOL
  {
    ArrayMessage<bool> am;
    for ( int i = 0; i < 20; ++i )
    {
      am.push_back((i & 1) == 1 );
    }
    EXPECT_EQ( am.length(), 20U );
    EXPECT_EQ( am[0], false );
    EXPECT_EQ( am[1], true );
    ASSERT_EQ( am._sizeInBytes(), 24U );
    uint8_t stream[24];
    EXPECT_EQ( am.writeToStream( stream ), 24U );
    EXPECT_EQ( stream[4], 0 );
    EXPECT_EQ( stream[5], 1 );

    ArrayMessage<bool> am_from_stream( 20U, false, stream + 4 );
    for ( int i = 0; i < 20; ++i )
    {
      EXPECT_EQ( am[i], am_from_stream[i] ) << "at iteration " << i;
    }

    uint8_t second_stream[24] = { 0, 0, 0, 0 };
    EXPECT_EQ( am_from_stream.writeToStream( second_stream ), 24U );
    EXPECT_TRUE( compareArrays( stream, second_stream, 24 ));

    const ArrayMessage<bool> &const_am_from_stream = am_from_stream;
    EXPECT_EQ( const_am_from_stream[0], false );
    stream[4] = 1;
    EXPECT_EQ( const_am_from_stream[0], true );
    am_from_stream.detachFromStream();
    stream[4] = 0;
    EXPECT_EQ( const_am_from_stream[0], true );

    auto *clone = dynamic_cast<ArrayMessage<bool> *>(const_am_from_stream.clone());
    ASSERT_NE( clone, nullptr );
    EXPECT_EQ( clone->at( 0 ), true );

    Message &m = am_from_stream;
    m = *static_cast<Message *>(&am);
    EXPECT_EQ( const_am_from_stream[0], false );

    EXPECT_EQ( clone->at( 0 ), true );
    delete clone;
  }

  // STRING
  {
    ArrayMessage<std::string> am;
    for ( int i = 0; i < 5; ++i )
    {
      am.push_back( std::string( "String " ) + std::to_string( i ));
    }
    EXPECT_EQ( am.length(), 5U );
    ASSERT_EQ( am._sizeInBytes(), 64U );
    EXPECT_EQ( am[3], "String 3" );
    EXPECT_THROW( am[5], std::runtime_error );
    uint8_t stream[64];
    ASSERT_EQ( am.writeToStream( stream ), 64U );

    ArrayMessage<std::string> am_from_stream( 5, false, stream + 4 );
    EXPECT_EQ( am_from_stream[4], "String 4" );

    uint8_t second_stream[64];
    ASSERT_EQ( am_from_stream.writeToStream( second_stream ), 64U );
    EXPECT_TRUE( compareArrays( stream, second_stream, 64 ));
  }

  // TIME
  {
    ArrayMessage<ros::Time> am;
    for ( int i = 0; i < 5; ++i )
    {
      am.push_back( ros::Time((i + 1) * 42, 0 ));
    }
    EXPECT_EQ( am.length(), 5U );
    EXPECT_EQ( am._sizeInBytes(), 44U );
    EXPECT_EQ( am[0], ros::Time( 42, 0 ));
    EXPECT_THROW( am[5], std::runtime_error );
    uint8_t stream[44];
    ASSERT_EQ( am.writeToStream( stream ), 44U );

    ArrayMessage<ros::Time> am_from_stream( 5, false, stream + 4 );
    EXPECT_EQ( am_from_stream[4], ros::Time( 210, 0 ));

    uint8_t second_stream[44];
    ASSERT_EQ( am_from_stream.writeToStream( second_stream ), 44U );
    EXPECT_TRUE( compareArrays( stream, second_stream, 44 ));
  }

  // DURATION
  {
    ArrayMessage<ros::Duration> am;
    for ( int i = 0; i < 5; ++i )
    {
      am.push_back( ros::Duration(((i & 1) == 0 ? -1 : 1) * (i + 1) * 42, 0 ));
    }
    EXPECT_EQ( am.length(), 5U );
    ASSERT_EQ( am._sizeInBytes(), 44U );
    EXPECT_EQ( am[0], ros::Duration( -42, 0 ));
    EXPECT_THROW( am[5], std::runtime_error );
    uint8_t stream[44];
    ASSERT_EQ( am.writeToStream( stream ), 44U );

    ArrayMessage<ros::Duration> am_from_stream( 5, false, stream + 4 );
    EXPECT_EQ( am_from_stream[3], ros::Duration( 168, 0 ));

    uint8_t second_stream[44];
    ASSERT_EQ( am_from_stream.writeToStream( second_stream ), 44U );
    EXPECT_TRUE( compareArrays( stream, second_stream, 44 ));
  }

  // FIXED SIZE
  {
    ArrayMessage<bool> am( 20, true );
    EXPECT_THROW( am.push_back( true ), BabelFishException );
  }
}

TEST( MessageTest, isCompatible )
{
  using namespace ros_babel_fish::internal;
  ASSERT_EQ((isCompatible<bool, bool>()), true );
  ASSERT_EQ((isCompatible<bool, uint8_t>()), true );
  ASSERT_EQ((isCompatible<bool, float>()), true );
  ASSERT_EQ((isCompatible<uint8_t, bool>()), false );
  ASSERT_EQ((isCompatible<int8_t, bool>()), false );
  ASSERT_EQ((isCompatible<float, bool>()), false );
  ASSERT_EQ((isCompatible<uint8_t, uint8_t>()), true );
  ASSERT_EQ((isCompatible<uint8_t, uint16_t>()), true );
  ASSERT_EQ((isCompatible<uint8_t, uint32_t>()), true );
  ASSERT_EQ((isCompatible<uint8_t, uint64_t>()), true );
  ASSERT_EQ((isCompatible<uint8_t, int8_t>()), false );
  ASSERT_EQ((isCompatible<uint8_t, int16_t>()), true );
  ASSERT_EQ((isCompatible<uint8_t, int32_t>()), true );
  ASSERT_EQ((isCompatible<uint8_t, int64_t>()), true );
  ASSERT_EQ((isCompatible<uint8_t, float>()), true );
  ASSERT_EQ((isCompatible<uint8_t, double>()), true );
  ASSERT_EQ((isCompatible<uint16_t, uint8_t>()), false );
  ASSERT_EQ((isCompatible<uint16_t, uint16_t>()), true );
  ASSERT_EQ((isCompatible<uint16_t, uint32_t>()), true );
  ASSERT_EQ((isCompatible<uint16_t, uint64_t>()), true );
  ASSERT_EQ((isCompatible<uint16_t, int8_t>()), false );
  ASSERT_EQ((isCompatible<uint16_t, int16_t>()), false );
  ASSERT_EQ((isCompatible<uint16_t, int32_t>()), true );
  ASSERT_EQ((isCompatible<uint16_t, int64_t>()), true );
  ASSERT_EQ((isCompatible<uint16_t, float>()), true );
  ASSERT_EQ((isCompatible<uint16_t, double>()), true );
  ASSERT_EQ((isCompatible<uint32_t, uint8_t>()), false );
  ASSERT_EQ((isCompatible<uint32_t, uint16_t>()), false );
  ASSERT_EQ((isCompatible<uint32_t, uint32_t>()), true );
  ASSERT_EQ((isCompatible<uint32_t, uint64_t>()), true );
  ASSERT_EQ((isCompatible<uint32_t, int8_t>()), false );
  ASSERT_EQ((isCompatible<uint32_t, int16_t>()), false );
  ASSERT_EQ((isCompatible<uint32_t, int32_t>()), false );
  ASSERT_EQ((isCompatible<uint32_t, int64_t>()), true );
  ASSERT_EQ((isCompatible<uint32_t, float>()), true );
  ASSERT_EQ((isCompatible<uint32_t, double>()), true );
  ASSERT_EQ((isCompatible<uint64_t, uint8_t>()), false );
  ASSERT_EQ((isCompatible<uint64_t, uint16_t>()), false );
  ASSERT_EQ((isCompatible<uint64_t, uint32_t>()), false );
  ASSERT_EQ((isCompatible<uint64_t, uint64_t>()), true );
  ASSERT_EQ((isCompatible<uint64_t, int8_t>()), false );
  ASSERT_EQ((isCompatible<uint64_t, int16_t>()), false );
  ASSERT_EQ((isCompatible<uint64_t, int32_t>()), false );
  ASSERT_EQ((isCompatible<uint64_t, int64_t>()), false );
  ASSERT_EQ((isCompatible<uint64_t, float>()), true );
  ASSERT_EQ((isCompatible<uint64_t, double>()), true );
  ASSERT_EQ((isCompatible<int8_t, uint8_t>()), false );
  ASSERT_EQ((isCompatible<int8_t, uint16_t>()), false );
  ASSERT_EQ((isCompatible<int8_t, uint32_t>()), false );
  ASSERT_EQ((isCompatible<int8_t, uint64_t>()), false );
  ASSERT_EQ((isCompatible<int8_t, int8_t>()), true );
  ASSERT_EQ((isCompatible<int8_t, int16_t>()), true );
  ASSERT_EQ((isCompatible<int8_t, int32_t>()), true );
  ASSERT_EQ((isCompatible<int8_t, int64_t>()), true );
  ASSERT_EQ((isCompatible<int8_t, float>()), true );
  ASSERT_EQ((isCompatible<int8_t, double>()), true );
  ASSERT_EQ((isCompatible<int16_t, uint8_t>()), false );
  ASSERT_EQ((isCompatible<int16_t, uint16_t>()), false );
  ASSERT_EQ((isCompatible<int16_t, uint32_t>()), false );
  ASSERT_EQ((isCompatible<int16_t, uint64_t>()), false );
  ASSERT_EQ((isCompatible<int16_t, int8_t>()), false );
  ASSERT_EQ((isCompatible<int16_t, int16_t>()), true );
  ASSERT_EQ((isCompatible<int16_t, int32_t>()), true );
  ASSERT_EQ((isCompatible<int16_t, int64_t>()), true );
  ASSERT_EQ((isCompatible<int16_t, float>()), true );
  ASSERT_EQ((isCompatible<int16_t, double>()), true );
  ASSERT_EQ((isCompatible<int32_t, uint8_t>()), false );
  ASSERT_EQ((isCompatible<int32_t, uint16_t>()), false );
  ASSERT_EQ((isCompatible<int32_t, uint32_t>()), false );
  ASSERT_EQ((isCompatible<int32_t, uint64_t>()), false );
  ASSERT_EQ((isCompatible<int32_t, int8_t>()), false );
  ASSERT_EQ((isCompatible<int32_t, int16_t>()), false );
  ASSERT_EQ((isCompatible<int32_t, int32_t>()), true );
  ASSERT_EQ((isCompatible<int32_t, int64_t>()), true );
  ASSERT_EQ((isCompatible<int32_t, float>()), true );
  ASSERT_EQ((isCompatible<int32_t, double>()), true );
  ASSERT_EQ((isCompatible<int64_t, uint8_t>()), false );
  ASSERT_EQ((isCompatible<int64_t, uint16_t>()), false );
  ASSERT_EQ((isCompatible<int64_t, uint32_t>()), false );
  ASSERT_EQ((isCompatible<int64_t, uint64_t>()), false );
  ASSERT_EQ((isCompatible<int64_t, int8_t>()), false );
  ASSERT_EQ((isCompatible<int64_t, int16_t>()), false );
  ASSERT_EQ((isCompatible<int64_t, int32_t>()), false );
  ASSERT_EQ((isCompatible<int64_t, int64_t>()), true );
  ASSERT_EQ((isCompatible<int64_t, float>()), true );
  ASSERT_EQ((isCompatible<int64_t, double>()), true );
  ASSERT_EQ((isCompatible<float, uint8_t>()), false );
  ASSERT_EQ((isCompatible<float, uint16_t>()), false );
  ASSERT_EQ((isCompatible<float, uint32_t>()), false );
  ASSERT_EQ((isCompatible<float, uint64_t>()), false );
  ASSERT_EQ((isCompatible<float, int8_t>()), false );
  ASSERT_EQ((isCompatible<float, int16_t>()), false );
  ASSERT_EQ((isCompatible<float, int32_t>()), false );
  ASSERT_EQ((isCompatible<float, int64_t>()), false );
  ASSERT_EQ((isCompatible<float, float>()), true );
  ASSERT_EQ((isCompatible<float, double>()), true );
  ASSERT_EQ((isCompatible<double, uint8_t>()), false );
  ASSERT_EQ((isCompatible<double, uint16_t>()), false );
  ASSERT_EQ((isCompatible<double, uint32_t>()), false );
  ASSERT_EQ((isCompatible<double, uint64_t>()), false );
  ASSERT_EQ((isCompatible<double, int8_t>()), false );
  ASSERT_EQ((isCompatible<double, int16_t>()), false );
  ASSERT_EQ((isCompatible<double, int32_t>()), false );
  ASSERT_EQ((isCompatible<double, int64_t>()), false );
  ASSERT_EQ((isCompatible<double, float>()), true );
  ASSERT_EQ((isCompatible<double, double>()), true );
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  ros::init( argc, argv, "test_message" );
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
