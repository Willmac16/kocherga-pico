#ifndef CAN2040_DRIVER_HPP
#define CAN2040_DRIVER_HPP

#define CAN_MAILBOX_SIZE 64U

static int const CAN2040_TX_PIN = 1;
static int const CAN2040_RX_PIN = 2;
static int const SPEED_CTRL = 3;

void coreOneWork();

void multicore_can2040Stop(void);

#include <cstdint>

#include "kocherga_can.hpp"

#define CAN_MAILBOX_SIZE 64U

class Can2040Driver final : public kocherga::can::ICANDriver
{
public:
  auto configure(const Bitrate &bitrate,
                 const bool silent,
                 const kocherga::can::CANAcceptanceFilterConfig &filter) -> std::optional<Mode> override;

  auto push(const bool force_classic_can,
            const std::uint32_t extended_can_id,
            const std::uint8_t payload_size,
            const void *const payload) -> bool override;

  auto pop(PayloadBuffer &payload_buffer) -> std::optional<std::pair<std::uint32_t, std::uint8_t>> override;

private:
  int can2040Enqueue(const std::uint32_t extended_can_id,
                     const std::uint8_t payload_size,
                     const void *const payload);

  auto unloadCanMailbox(void *const payload) -> std::optional<std::pair<std::uint32_t, std::uint8_t>>;

  void pollTxQueue(const std::chrono::microseconds now);

  kocherga::can::TxQueue<void *(*)(std::size_t), void (*)(void *)> tx_queue_ = {o1malloc, o1free};
};

#endif // CAN2040_DRIVER_HPP
