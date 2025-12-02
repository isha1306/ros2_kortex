#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <SessionManager.h>
#include <TransportClientTcp.h>
#include <TransportClientUdp.h>
#include <iostream>
#include <thread>
#include <chrono>

namespace k_api = Kinova::Api;

int main(int argc, char** argv)
{
    // -------------------------
    // 1. CONNECT
    // -------------------------
    std::string robot_ip = "192.168.1.10";  
    int tcp_port = 10000;
    int udp_port = 10001;

    k_api::TransportClientTcp tcp_transport;
    k_api::RouterClient router(&tcp_transport, [](k_api::KError err){});
    tcp_transport.connect(robot_ip, tcp_port);

    k_api::TransportClientUdp udp_transport;
    k_api::RouterClient rt_router(&udp_transport, [](k_api::KError err){});
    udp_transport.connect(robot_ip, udp_port);

    k_api::Session::CreateSessionInfo session_info;
    session_info.set_username("admin");
    session_info.set_password("admin");
    session_info.set_session_inactivity_timeout(60000);
    session_info.set_connection_inactivity_timeout(2000);

    k_api::SessionManager session(&router);
    session.CreateSession(session_info);

    k_api::SessionManager session_rt(&rt_router);
    session_rt.CreateSession(session_info);

    k_api::Base::BaseClient base(&router);
    k_api::BaseCyclic::BaseCyclicClient base_cyclic(&rt_router);

    // -------------------------
    // 2. SWITCH TO LOW-LEVEL SERVOING
    // -------------------------
    k_api::Base::ServoingModeInformation servo_mode;
    servo_mode.set_servoing_mode(k_api::Base::LOW_LEVEL_SERVOING);
    base.SetServoingMode(servo_mode);

    std::cout << "LOW LEVEL SERVOING ENABLED" << std::endl;

    // -------------------------
    // 3. GET ACTUATOR COUNT
    // -------------------------
    int actuator_count = base.GetActuatorCount().count();
    int last_joint = actuator_count - 1;

    std::cout << "Actuators: " << actuator_count 
              << " (Last Joint Index = " << last_joint << ")" << std::endl;

    // -------------------------
    // 4. PREPARE TORQUE COMMAND STRUCTURE
    // -------------------------
    k_api::BaseCyclic::Feedback feedback;
    k_api::BaseCyclic::Command command;

    feedback = base_cyclic.RefreshFeedback();
    command = base_cyclic.RefreshFeedback();

    // -------------------------
    // 5. SET TORQUE VALUE HERE
    // -------------------------
    double torque_cmd = 0.8;   // Nm – change as needed

    // -------------------------
    // 6. TORQUE CONTROL LOOP
    // -------------------------
    std::cout << "Sending torque to last joint..." << std::endl;

    while (true)
    {
        feedback = base_cyclic.RefreshFeedback();

        // Copy previous command frame
        command = k_api::BaseCyclic::Command();
        command.set_frame_id(feedback.frame_id() + 1);

        for (int i = 0; i < actuator_count; i++)
        {
            auto* joint_cmd = command.add_actuators();
            joint_cmd->set_command_id(i);

            if (i == last_joint)
            {
                // Send torque only to last joint
                joint_cmd->set_torque_motor(torque_cmd);
            }
            else
            {
                joint_cmd->set_torque_motor(0.0); // zero torque for others
            }
        }

        base_cyclic.SendCommand(command);

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // -------------------------
    // 7. CLOSE SESSION (never reached in loop)
    // -------------------------
    session_rt.CloseSession();
    session.CloseSession();
    tcp_transport.disconnect();
    udp_transport.disconnect();

    return 0;
}
