/*
 * This example creates a OpenCyphal node. The builtin LED can be switched on and off using OpenCyphal.
 * It also shows periodic transmission of a OpenCyphal heartbeat message via CAN.
 *
 * switch built in LED on with
 *   yakut pub 1620:uavcan.primitive.scalar.Bit.1.0 'value: true'
 *
 * switch built in LED off with
 *   yakut pub 1620:uavcan.primitive.scalar.Bit.1.0 'value: false'
 *
 * make LED blink at 0.125 Hz with
 *  yakut pub 1620:uavcan.primitive.scalar.Bit.1.0 'value: !$ "n % 2"' -T 4
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <107-Arduino-Cyphal.h>

#include "appDesc.hpp"
#include "argsFromApp.hpp"

extern "C"
{
#include "crossCoreCan2040.h"
}

#define PICO_USE_STACK_GUARDS 1

#define PICO_STACK_SIZE _u(0x10000)
#define PICO_CORE1_STACK_SIZE _u(0x1000)

#define PICO_USE_MALLOC_MUTEX 0

#include "RP2040.h"

#include <stdlib.h>
#include <stdint.h>

#include "hardware/watchdog.h"
#include "hardware/sync.h"
#include "hardware/flash.h"
#include "hardware/irq.h"
#include "hardware/exception.h"

#include "pico/stdlib.h"
#include "pico/unique_id.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

using namespace uavcan::node;
using namespace uavcan::primitive::scalar;

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static int const MEGA = 1'000'000;

static int const CAN2040_TX_PIN = 1;
static int const CAN2040_RX_PIN = 2;
static int const CAN_BITRATE = MEGA;
static int const SPEED_CTRL = 3;

static CanardPortID const BIT_PORT_ID = 1620U;

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

ExecuteCommand::Response_1_1 onExecuteCommand_1_1_Request_Received(ExecuteCommand::Request_1_1 const &, cyphal::TransferMetadata const &);

void onBit_1_0_Received(Bit_1_0 const &msg);

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

static const volatile AppDescriptor g_app_descriptor __attribute__((used, section(".app_descriptor")));

static bool reboot = false;

cyphal::Node::Heap<cyphal::Node::DEFAULT_O1HEAP_SIZE> node_heap __attribute__((used, section(".o1heap")));
cyphal::Node node_hdl(
    node_heap.data(), node_heap.size(), time_us_64, [](CanardFrame const &frame)
    { return crossCoreCanSend(&frame); },
    111);

cyphal::Publisher<Heartbeat_1_0> heartbeat_pub = node_hdl.create_publisher<Heartbeat_1_0>(1 * 1000 * 1000UL /* = 1 sec in usecs. */);
cyphal::Subscription bit_subscription = node_hdl.create_subscription<Bit_1_0>(BIT_PORT_ID, onBit_1_0_Received);
cyphal::ServiceServer execute_command_srv = node_hdl.create_service_server<ExecuteCommand::Request_1_1, ExecuteCommand::Response_1_1>(
    2 * 1000 * 1000UL,
    onExecuteCommand_1_1_Request_Received);

void ex_handler_hard_fault(void)
{
  // Shut off the CAN transceiver with the standby/speed control pin
  gpio_put(SPEED_CTRL, 1);
  gpio_deinit(CAN2040_RX_PIN);
  gpio_deinit(CAN2040_TX_PIN);

  watchdog_reboot(0U, 0U, 0x7fffff);
}

/**************************************************************************************
 * MAIN
 **************************************************************************************/

int main()
{
  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

  pico_unique_board_id_t flash_unique_id;
  pico_get_unique_board_id(&flash_unique_id);

  std::array<uint8_t, 16> unique_id;

  for (uint8_t i = 0; i < 8; i++)
  {
    unique_id[2 * i] = flash_unique_id.id[2];
  }

  cyphal::NodeInfo node_info = node_hdl.create_node_info(
      CANARD_CYPHAL_SPECIFICATION_VERSION_MAJOR, CANARD_CYPHAL_SPECIFICATION_VERSION_MINOR,
      0, 1,
      SOFTWARE_VERSION_MAJOR, SOFTWARE_VERSION_MINOR,
      GIT_HASH,
      unique_id,
      "org.cwrubaja.pico.testboard", g_app_descriptor.image_crc);

  // Bring the CAN Speed Control pin to ground for high speed operation
  gpio_init(SPEED_CTRL);
  gpio_set_dir(SPEED_CTRL, GPIO_OUT);
  gpio_put(SPEED_CTRL, 0);

  // Register the CAN Shutdown handler
  exception_set_exclusive_handler(HARDFAULT_EXCEPTION, ex_handler_hard_fault);

  crossCoreCanInit(CAN_BITRATE, CAN2040_RX_PIN, CAN2040_TX_PIN);

  // watchdog_enable(500, 1);

  while (true)
  {
    /* Process all pending OpenCyphal actions.
     */
    CanardFrame frame;

    uint8_t payload[8];

    frame.payload = payload;

    // Empty Queue of received frames
    while (crossCoreCanReceive(&frame))
    {
      node_hdl.onCanFrameReceived(frame);
    }

    node_hdl.spinSome();

    if (reboot)
    {
      gpio_put(SPEED_CTRL, 1);
      gpio_deinit(CAN2040_RX_PIN);
      gpio_deinit(CAN2040_TX_PIN);

      watchdog_reboot(0U, 0U, 0x7fffff);

      while (1)
      {
        tight_loop_contents();
      }
    }

    static uint_fast64_t prev = 0;
    uint_fast64_t now = time_us_64();

    /* Publish the heartbeat once/second */
    if (now - prev > MEGA)
    {
      prev = now;

      uavcan::node::Heartbeat_1_0 msg;

      msg.uptime = now / MEGA;
      msg.health.value = uavcan::node::Health_1_0::NOMINAL;
      msg.mode.value = uavcan::node::Mode_1_0::OPERATIONAL;
      msg.vendor_specific_status_code = gpio_get(PICO_DEFAULT_LED_PIN);

      heartbeat_pub->publish(msg);
    }

    watchdog_update();

    absolute_time_t deadline = make_timeout_time_us(100);
    best_effort_wfe_or_timeout(deadline);
  }
}

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

void onBit_1_0_Received(Bit_1_0 const &msg)
{
  gpio_put(PICO_DEFAULT_LED_PIN, msg.value);
}

ExecuteCommand::Response_1_1 onExecuteCommand_1_1_Request_Received(ExecuteCommand::Request_1_1 const &req, cyphal::TransferMetadata const &metadata)
{
  ExecuteCommand::Response_1_1 rsp;

  std::string parameter;

  switch (req.command)
  {
  case uavcan::node::ExecuteCommand::Request_1_1::COMMAND_BEGIN_SOFTWARE_UPDATE:
  {
    ArgumentsFromApplication args = {
        .args_from_app_version_major = 0,
        .cyphal_serial_node_id = 0xFF,
        .cyphal_can_node_id = node_hdl.getNodeId(),
        .file_server_node_id = metadata.remote_node_id};

    std::copy(req.parameter.begin(), req.parameter.end(), args.remote_file_path.begin());

    VolatileStorage<ArgumentsFromApplication> volatileStore(reinterpret_cast<std::uint8_t *>(ARGS_ADDR));
    volatileStore.store(args);
  }
  case uavcan::node::ExecuteCommand::Request_1_1::COMMAND_RESTART:
    rsp.status = uavcan::node::ExecuteCommand::Response_1_1::STATUS_SUCCESS;
    reboot = true;
    break;

  case uavcan::node::ExecuteCommand::Request_1_1::COMMAND_STORE_PERSISTENT_STATES:
    rsp.status = uavcan::node::ExecuteCommand::Response_1_1::STATUS_SUCCESS;
    break;
  default:
    rsp.status = uavcan::node::ExecuteCommand_1_1::Response::STATUS_BAD_COMMAND;
    break;
  }

  return rsp;
}
