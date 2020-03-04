//
// Created by Stefan Fabian on 15.09.19.
//

#include <ros_babel_fish/generation/providers/integrated_description_provider.h>
#include <ros_babel_fish/generation/providers/message_only_description_provider.h>

#include <roscpp_tutorials/TwoInts.h>

#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

#include <rosapi/DeleteParam.h>
#include <rosapi/GetActionServers.h>

#define default mdefault

#include <rosapi/GetParam.h>

#undef default

#include <rosapi/GetParamNames.h>
#include <rosapi/GetTime.h>
#include <rosapi/HasParam.h>
#include <rosapi/MessageDetails.h>
#include <rosapi/NodeDetails.h>
#include <rosapi/Nodes.h>
#include <rosapi/Publishers.h>
#include <rosapi/SearchParam.h>
#include <rosapi/ServiceHost.h>
#include <rosapi/ServiceNode.h>
#include <rosapi/ServiceProviders.h>
#include <rosapi/ServiceRequestDetails.h>
#include <rosapi/ServiceResponseDetails.h>
#include <rosapi/ServiceType.h>
#include <rosapi/Services.h>
#include <rosapi/ServicesForType.h>
#include <rosapi/SetParam.h>
#include <rosapi/Subscribers.h>
#include <rosapi/TopicType.h>
#include <rosapi/Topics.h>
#include <rosapi/TopicsForType.h>

#include <gtest/gtest.h>
#include <ros/ros.h>

using namespace ros_babel_fish;

template<typename MsgType, typename ProviderType>
::testing::AssertionResult compareDescription()
{
  namespace st = ros::service_traits;
  namespace mt = ros::message_traits;
  {
    SCOPED_TRACE( "Service description lookup" );
    ProviderType provider;
    ServiceDescription::ConstPtr desc = provider.getServiceDescription( st::DataType<MsgType>::value());
    if ( desc == nullptr )
      return ::testing::AssertionFailure() << "Failed to get message description for: "
                                           << st::DataType<MsgType>::value();
    if ( st::MD5Sum<MsgType>::value() != desc->md5 )
      return ::testing::AssertionFailure() << "MD5 Sum differed!" << std::endl
                                           << "BF  MD5: " << desc->md5 << std::endl
                                           << "MSG MD5: " << st::MD5Sum<MsgType>::value();
    if ( st::DataType<MsgType>::value() != desc->datatype )
      return ::testing::AssertionFailure() << "Datatype differed!" << std::endl
                                           << "BF  Datatype: " << desc->datatype << std::endl
                                           << "MSG Datatype: " << st::DataType<MsgType>::value();

    {
      if ( mt::datatype<typename MsgType::Request>() != desc->request->datatype )
        return ::testing::AssertionFailure() << "Request Datatype differed!" << std::endl
                                             << "BF  Datatype: " << desc->request->datatype << std::endl
                                             << "MSG Datatype: " << mt::datatype<typename MsgType::Request>();
      if ( mt::md5sum<typename MsgType::Request>() != desc->request->md5 )
        return ::testing::AssertionFailure() << "Request MD5 Sum differed!" << std::endl
                                             << "BF  MD5: " << desc->request->md5 << std::endl
                                             << "MSG MD5: " << mt::md5sum<typename MsgType::Request>();
      if ( mt::definition<typename MsgType::Request>() != desc->request->message_definition )
        return ::testing::AssertionFailure() << "Request Definition differed!" << std::endl
                                             << "--------------------------- BF  Definition: ---------------------------"
                                             << std::endl << desc->request->message_definition << std::endl
                                             << "--------------------------- MSG Definition: ---------------------------"
                                             << std::endl << mt::Definition<typename MsgType::Request>::value();
    }

    {
      SCOPED_TRACE( "Response" );
      if ( mt::datatype<typename MsgType::Response>() != desc->response->datatype )
        return ::testing::AssertionFailure() << "Response Datatype differed!" << std::endl
                                             << "BF  Datatype: " << desc->response->datatype << std::endl
                                             << "MSG Datatype: " << mt::datatype<typename MsgType::Response>();
      if ( mt::md5sum<typename MsgType::Response>() != desc->response->md5 )
        return ::testing::AssertionFailure() << "Response MD5 Sum differed!" << std::endl
                                             << "BF  MD5: " << desc->response->md5 << std::endl
                                             << "MSG MD5: " << mt::md5sum<typename MsgType::Response>();
      if ( mt::definition<typename MsgType::Response>() != desc->response->message_definition )
        return ::testing::AssertionFailure() << "Response Definition differed!" << std::endl
                                             << "--------------------------- BF  Definition: ---------------------------"
                                             << std::endl << desc->response->message_definition << std::endl
                                             << "--------------------------- MSG Definition: ---------------------------"
                                             << std::endl << mt::Definition<typename MsgType::Response>::value();
    }
  }
  return ::testing::AssertionSuccess();
}

TEST( ServiceLookupTest, integratedDescriptionProvider )
{
  EXPECT_TRUE((compareDescription<roscpp_tutorials::TwoInts, IntegratedDescriptionProvider>()));

  EXPECT_TRUE((compareDescription<std_srvs::Empty, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<std_srvs::SetBool, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<std_srvs::Trigger, IntegratedDescriptionProvider>()));

  EXPECT_TRUE((compareDescription<rosapi::DeleteParam, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<rosapi::GetActionServers, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<rosapi::GetParam, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<rosapi::GetParamNames, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<rosapi::GetTime, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<rosapi::HasParam, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<rosapi::MessageDetails, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<rosapi::NodeDetails, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<rosapi::Nodes, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<rosapi::Publishers, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<rosapi::SearchParam, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<rosapi::ServiceHost, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<rosapi::ServiceNode, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<rosapi::ServiceProviders, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<rosapi::ServiceRequestDetails, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<rosapi::ServiceResponseDetails, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<rosapi::ServiceType, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<rosapi::Services, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<rosapi::ServicesForType, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<rosapi::SetParam, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<rosapi::Subscribers, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<rosapi::TopicType, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<rosapi::Topics, IntegratedDescriptionProvider>()));
  EXPECT_TRUE((compareDescription<rosapi::TopicsForType, IntegratedDescriptionProvider>()));
}

TEST( ServiceLookupTest, messageOnlyDescriptionProvider )
{
  namespace st = ros::service_traits;
  MessageOnlyDescriptionProvider provider;
  EXPECT_THROW( provider.getServiceDescription( st::datatype<rosapi::DeleteParam>()), BabelFishException );
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  ros::init( argc, argv, "test_message_lookup" );
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
