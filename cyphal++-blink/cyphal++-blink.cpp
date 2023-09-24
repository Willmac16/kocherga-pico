/*
 * This example creates a OpenCyphal node. The builtin LED can be switched on and off using OpenCyphal.
 * It also shows periodic transmission of a OpenCyphal heartbeat message via CAN.
 *
 * switch built in LED on with
 *   yakut pub 1620:uavcan.primitive.scalar.Bit.1.0 'value: true'
 *
 * switch built in LED off with
 *   yakut pub 1620:uavcan.primitive.scalar.Bit.1.0 'value: false'
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <107-Arduino-Cyphal.h>

#include "appDesc.hpp"
#include "argsFromApp.hpp"

extern "C" {
    #include "crossCoreCan2040.h"
}

#include "RP2040.h"

#include <stdlib.h>
#include <stdint.h>

#include "hardware/watchdog.h"
#include "hardware/sync.h"

#include "pico/stdlib.h"


/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

using namespace uavcan::node;
using namespace uavcan::primitive::scalar;

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static int const CAN2040_RX_PIN = 4;
static int const CAN2040_TX_PIN = 5;
static CanardPortID const BIT_PORT_ID = 1620U;

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

ExecuteCommand::Response_1_1 onExecuteCommand_1_1_Request_Received(ExecuteCommand::Request_1_1 const &, cyphal::TransferMetadata const &);
GetInfo::Response_1_0 onGetInfo_1_0_Request_Received(GetInfo::Request_1_0 const &, cyphal::TransferMetadata const &);
void onBit_1_0_Received (Bit_1_0 const & msg);

void picoRestart()
{
    // Reset the watchdog timer
    watchdog_enable(1, 0);
    while(1) {
        tight_loop_contents(); // Literally a no-op, but the one the SDK uses
    }
}


bool repeating_timer_callback(struct repeating_timer *t) {
    watchdog_update();
    return true;
}

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

cyphal::Node::Heap<cyphal::Node::DEFAULT_O1HEAP_SIZE> node_heap;
cyphal::Node node_hdl(node_heap.data(), node_heap.size(), time_us_64, [] (CanardFrame const & frame) { crossCoreCanSend(&frame); return true; }, 111);

cyphal::Publisher<Heartbeat_1_0> heartbeat_pub = node_hdl.create_publisher<Heartbeat_1_0>
  (1*1000*1000UL /* = 1 sec in usecs. */);
cyphal::Subscription bit_subscription = node_hdl.create_subscription<Bit_1_0>
  (BIT_PORT_ID, onBit_1_0_Received);
cyphal::ServiceServer execute_command_srv = node_hdl.create_service_server<ExecuteCommand::Request_1_1, ExecuteCommand::Response_1_1>(
  2*1000*1000UL,
  onExecuteCommand_1_1_Request_Received);
cyphal::ServiceServer get_info_srv = node_hdl.create_service_server<GetInfo::Request_1_0, GetInfo::Response_1_0>(
  10*1000*1000UL,
  onGetInfo_1_0_Request_Received);

static const volatile AppDescriptor g_app_descriptor __attribute__((used, section(".app_descriptor")));

/******************************************5********************************************
 * MAIN
 **************************************************************************************/

int main() {
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    crossCoreCanInit(1'000'000, CAN2040_RX_PIN, CAN2040_TX_PIN);

    // struct repeating_timer timer;
    // add_repeating_timer_ms(-250, repeating_timer_callback, NULL, &timer);

    // watchdog_enable(500, 0);


    while (true) {
        /* Process all pending OpenCyphal actions.
        */
        CanardFrame frame;

        uint8_t payload[8];

        frame.payload = payload;

        // Empty Queue of received frames
        while(crossCoreCanReceive(&frame)) {
            node_hdl.onCanFrameReceived(frame);
        }

        node_hdl.spinSome();

        static uint_fast64_t prev = 0;
        uint_fast64_t now = time_us_64();

        /* Publish the heartbeat once/second */
        if(now - prev > 1'000'000)
        {
            prev = now;

            uavcan::node::Heartbeat_1_0 msg;

            msg.uptime = now / 1'000'000UL;
            msg.health.value = uavcan::node::Health_1_0::NOMINAL;
            msg.mode.value = uavcan::node::Mode_1_0::OPERATIONAL;
            msg.vendor_specific_status_code = 0;

            heartbeat_pub->publish(msg);
        }

        __wfe();
    }
}

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

void onBit_1_0_Received(Bit_1_0 const & msg)
{
    gpio_put(PICO_DEFAULT_LED_PIN, msg.value);
}

ExecuteCommand::Response_1_1 onExecuteCommand_1_1_Request_Received(ExecuteCommand::Request_1_1 const & req, cyphal::TransferMetadata const & metadata)
{
    ExecuteCommand::Response_1_1 rsp;

    std::string parameter;

    if (req.command == uavcan::node::ExecuteCommand::Request_1_1::COMMAND_BEGIN_SOFTWARE_UPDATE) {
        ArgumentsFromApplication args = {
            .args_from_app_version_major = 0,
            .cyphal_serial_node_id = 0xFF,
            .cyphal_can_node_id = node_hdl.getNodeId(),
            .file_server_node_id = metadata.remote_node_id
        };

        std::copy(req.parameter.begin(), req.parameter.end(), args.remote_file_path.begin());

        VolatileStorage<ArgumentsFromApplication> volatileStore(reinterpret_cast<std::uint8_t*>(ARGS_ADDR));
        volatileStore.store(args);

        picoRestart();
    } else if (req.command == 0xCAFE) {
        rsp.status = ExecuteCommand::Response_1_1::STATUS_SUCCESS;
    } else {
        rsp.status = uavcan::node::ExecuteCommand_1_1::Response::STATUS_BAD_COMMAND;
    }
    return rsp;
}

GetInfo::Response_1_0 onGetInfo_1_0_Request_Received(GetInfo::Request_1_0 const & req, cyphal::TransferMetadata const & metadata)
{
    GetInfo::Response_1_0 rsp = {
        .protocol_version = {
            .major = CANARD_CYPHAL_SPECIFICATION_VERSION_MAJOR,
            .minor = CANARD_CYPHAL_SPECIFICATION_VERSION_MINOR
        },
        .hardware_version = {
            .major = 0,
            .minor = 1
        },
        .software_version = {
            .major = SOFTWARE_VERSION_MAJOR,
            .minor = SOFTWARE_VERSION_MINOR
        },
        .software_vcs_revision_id = GIT_HASH,
        .unique_id = {
            2, 2, 2, 2, 2, 2, 2, 2,
            2, 2, 2, 2, 2, 2, 2, 2
        }, // #TODO: Use Flash Unique ID
        // "org.cwrubaja.pico.testboard"
        // .name = {'o', 'r', 'g', '.', 'c', 'w', 'r', 'u', 'b', 'a', 'j', 'a', '.', 'p', 'i', 'c', 'o', '.', 't', 'e', 's', 't', 'b', 'o', 'a', 'r', 'd', '\0'},
        // .software_image_crc = {g_app_descriptor.image_crc}
    };

    return rsp;
}


