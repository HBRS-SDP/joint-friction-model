#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <stdlib.h> /* abs */
#include <thread>
#include <chrono>
#include <time.h>
#include <unistd.h>
#include <friction_controller.hpp>
#include <SessionClientRpc.h>
#include <SessionManager.h>
#include <RouterClient.h>
#include <TransportClientUdp.h>
#include <TransportClientTcp.h>
#include <google/protobuf/text_format.h>
#include <google/protobuf/util/json_util.h>
#include<yaml/Yaml.hpp>

using namespace std;
namespace k_api = Kinova::Api;
using namespace std::this_thread;     // sleep_for, sleep_until
using namespace std::chrono_literals; // ns, us, ms, s, h, etc.
using std::chrono::system_clock;

#define IP_ADDRESS "192.168.1.10"
#define PORT 10000
#define PORT_REAL_TIME 10001
// Maximum allowed waiting time during actions
constexpr auto TIMEOUT_PROMISE_DURATION = std::chrono::seconds{20};
float TIME_DURATION = 30.0f; // Duration of the example (seconds)

//Create variable for controller
friction_controller controller;
// Create an event listener that will set the promise action event to the exit value
// Will set promise to either END or ABORT
// Use finish_promise.get_future.get() to wait and get the value
std::function<void(k_api::Base::ActionNotification)> 
    create_event_listener_by_promise(std::promise<k_api::Base::ActionEvent>& finish_promise)
{
    return [&finish_promise] (k_api::Base::ActionNotification notification)
    {
        const auto action_event = notification.action_event();
        switch(action_event)
        {
        case k_api::Base::ActionEvent::ACTION_END:
        case k_api::Base::ActionEvent::ACTION_ABORT:
            finish_promise.set_value(action_event);
            break;
        default:
            break;
        }
    };
}
/**************************
 * Example core functions *
 **************************/
void example_move_to_home_position(k_api::Base::BaseClient* base)
{
    // Make sure the arm is in Single Level Servoing before executing an Action
    Yaml::Node root;
    Yaml::Parse(root, "../configs/constants.yml");
    string arm_position_configuration = root["arm_position_configuration"].As<string>();

    auto servoingMode = k_api::Base::ServoingModeInformation();
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Move arm to ready position
    cout << "Moving the arm to a safe position" << endl;
    auto action_type = k_api::Base::RequestedActionType();
    action_type.set_action_type(k_api::Base::REACH_JOINT_ANGLES);
    auto action_list = base->ReadAllActions(action_type);
    auto action_handle = k_api::Base::ActionHandle();
    action_handle.set_identifier(0);

    for (auto action : action_list.action_list()) 
    {
        if (action.name() == arm_position_configuration)
        {
            action_handle = action.handle();
        }
    }

    if (action_handle.identifier() == 0)
    {
        cout << "Can't reach safe position, exiting" << endl;
        return;
    }
    else
    {
        // Connect to notification action topic
        std::promise<k_api::Base::ActionEvent> finish_promise;
        auto finish_future = finish_promise.get_future();
        auto promise_notification_handle = base->OnNotificationActionTopic(
            create_event_listener_by_promise(finish_promise),
            k_api::Common::NotificationOptions()
        );

        // Execute action
        base->ExecuteActionFromReference(action_handle);

        // Wait for future value from promise
        const auto status = finish_future.wait_for(TIMEOUT_PROMISE_DURATION);
        base->Unsubscribe(promise_notification_handle);

        if(status != std::future_status::ready)
        {
            cout << "Timeout on action notification wait" << endl;
        }
        const auto promise_event = finish_future.get();
    }
    base->Stop();
    // std::this_thread::sleep_for(std::chrono::milliseconds(1500));
}
int main(int argc, char **argv)
{
    // Create API objects
    auto error_callback = [](k_api::KError err){ cout << "_________ callback error _________" << err.toString(); };
    Yaml::Node root;
    Yaml::Parse(root, "../configs/constants.yml");
    bool offline_mode = root["offline_mode"].As<bool>();
    cout << "Creating transport objects" << endl;
    auto transport = new k_api::TransportClientTcp();
    auto router = new k_api::RouterClient(transport, error_callback);
    if(!offline_mode){
        transport->connect(IP_ADDRESS, PORT);
    }
    cout << "Creating transport real time objects" << endl;
    auto transport_real_time = new k_api::TransportClientUdp();
    auto router_real_time = new k_api::RouterClient(transport_real_time, error_callback);
    if(!offline_mode){
        transport_real_time->connect(IP_ADDRESS, PORT_REAL_TIME);
    }
    // Set session data connection information
    auto create_session_info = k_api::Session::CreateSessionInfo();
    create_session_info.set_username("admin");
    create_session_info.set_password("kinova1_area4251");
    create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
    create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)
    // Session manager service wrapper
    cout << "Creating sessions for communication" << endl;
    auto session_manager = new k_api::SessionManager(router);
    if(!offline_mode){
        session_manager->CreateSession(create_session_info);
    }
    auto session_manager_real_time = new k_api::SessionManager(router_real_time);
    if(!offline_mode){
        session_manager_real_time->CreateSession(create_session_info);
    }
    cout << "Sessions created" << endl;
    // Create services
    auto base = new k_api::Base::BaseClient(router);
    auto base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(router_real_time);
    auto actuator_config = new k_api::ActuatorConfig::ActuatorConfigClient(router);
    // Example core

    if(!offline_mode){
        example_move_to_home_position(base);
    }
    
    auto isOk = controller.cyclic_torque_control(base, base_cyclic, actuator_config);    
    if (!isOk)
    {
        cout << "There has been an unexpected error in cyclic_torque_control() function." << endl;;
    }
    
    // Close API session
    session_manager->CloseSession();
    session_manager_real_time->CloseSession();
    // Deactivate the router and cleanly disconnect from the transport object
    router->SetActivationStatus(false);
    transport->disconnect();
    router_real_time->SetActivationStatus(false);
    transport_real_time->disconnect();
    // Destroy the API
    delete base;
    delete base_cyclic;
    delete actuator_config;
    delete session_manager;
    delete session_manager_real_time;
    delete router;
    delete router_real_time;
    delete transport;
    delete transport_real_time;
}