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

extern "C" {
    #include "crossCoreCan2040.h"
}

#include "RP2040.h"

#include <stdlib.h>
#include <stdint.h>

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

void onBit_1_0_Received (Bit_1_0 const & msg);

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

cyphal::Node::Heap<cyphal::Node::DEFAULT_O1HEAP_SIZE> node_heap;
cyphal::Node node_hdl(node_heap.data(), node_heap.size(), time_us_64, [] (CanardFrame const & frame) { crossCoreCanSend(&frame); return true; });

cyphal::Publisher<Heartbeat_1_0> heartbeat_pub = node_hdl.create_publisher<Heartbeat_1_0>
  (1*1000*1000UL /* = 1 sec in usecs. */);
cyphal::Subscription bit_subscription = node_hdl.create_subscription<Bit_1_0>
  (BIT_PORT_ID, onBit_1_0_Received);

static const volatile AppDescriptor g_app_descriptor __attribute__((used, section(".app_descriptor")));
// VolatileStorage<ArgumentsFromApplication> storage(0x2040'0000U - 0x0800);


/******************************************5********************************************
 * MAIN
 **************************************************************************************/

int main() {
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    crossCoreCanInit(1'000'000, CAN2040_RX_PIN, CAN2040_TX_PIN);

    while (true) {
        /* Process all pending OpenCyphal actions.
        */
        CanardFrame frame;

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

        busy_wait_us_32(10U);
    }
}

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

void onBit_1_0_Received(Bit_1_0 const & msg)
{
    gpio_put(PICO_DEFAULT_LED_PIN, msg.value);
}
