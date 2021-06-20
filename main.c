/* minimal slcan @ 15-jun-2021 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/can.h>
#include <libopencm3/cm3/nvic.h>
#include <stdlib.h>

/* types and vars */

typedef enum {
  state_init,
  state_closed,
  state_normal,
  state_silent
} state_e;

state_e can_state = state_init;

typedef struct {
  uint32_t baudrate;
  uint32_t sjw;
  uint32_t ts1;
  uint32_t ts2;
  uint32_t brp;
} can_timings_t;

can_timings_t can_timings[] = {
  { 10000,   CAN_BTR_SJW_1TQ, CAN_BTR_TS1_14TQ, CAN_BTR_TS2_5TQ, 180 },
  { 20000,   CAN_BTR_SJW_1TQ, CAN_BTR_TS1_14TQ, CAN_BTR_TS2_5TQ, 90  },
  { 50000,   CAN_BTR_SJW_1TQ, CAN_BTR_TS1_14TQ, CAN_BTR_TS2_5TQ, 36  },
  { 100000,  CAN_BTR_SJW_1TQ, CAN_BTR_TS1_8TQ,  CAN_BTR_TS2_2TQ, 36  },
  { 125000,  CAN_BTR_SJW_1TQ, CAN_BTR_TS1_13TQ, CAN_BTR_TS2_2TQ, 18  },
  { 250000,  CAN_BTR_SJW_1TQ, CAN_BTR_TS1_13TQ, CAN_BTR_TS2_2TQ, 9   },
  { 500000,  CAN_BTR_SJW_1TQ, CAN_BTR_TS1_7TQ,  CAN_BTR_TS2_1TQ, 9   },
  { 800000,  CAN_BTR_SJW_1TQ, CAN_BTR_TS1_12TQ, CAN_BTR_TS2_2TQ, 3   },
  { 1000000, CAN_BTR_SJW_1TQ, CAN_BTR_TS1_15TQ, CAN_BTR_TS2_2TQ, 2   }
};

uint8_t num_can_timings = (sizeof(can_timings) / sizeof(can_timings_t));

uint8_t can_speed = 0;

typedef struct {
  uint8_t *data;
  size_t   size;
  uint32_t head;
  uint32_t tail;
} ring_t;

uint8_t tx_buffer[1024] = {};
uint8_t rx_buffer[1024] = {};
ring_t  tx_ring;
ring_t  rx_ring;

uint32_t num_commands_pending = 0;

const uint8_t version[4] = "0001";

/* prototypes */

void clock_setup(void);
void uart_setup(void);
void can_setup(void);
void update(void);

void ring_init(ring_t *ring, uint8_t *buf, size_t size);
bool ring_push(ring_t *ring, uint8_t data);
bool ring_pop(ring_t *ring, uint8_t *data);

uint8_t nib2hex(uint32_t nib);
bool hex2nib(uint8_t hex, uint8_t *nib);
bool bcd2nib(uint8_t bcd, uint8_t *nib);

bool read_hex(uint8_t *data);
bool read_bcd(uint8_t *data);
bool read_nib(uint8_t *data);

void purge_command(void);

void command_can_speed(void);
void command_open(bool silent);
void command_close(void);
void command_transmit(bool ext, bool rtr);
void command_status_flags(void);
void command_uart_speed(void);
void command_version(void);
void command_serial_number(void);
void command_unsupported(void);

/* helpers */

void ring_init(ring_t *ring, uint8_t *buf, size_t size)
{
  ring->data = buf;
  ring->size = size;
  ring->head = 0;
  ring->tail = 0; 
}

bool ring_push(ring_t *ring, uint8_t data)
{
  if (((ring->tail + 1) % ring->size) != ring->head)
  {
    ring->data[ring->tail++] = data;
    ring->tail %= ring->size;
    return true;
  }
  return false;
}

bool ring_pop(ring_t *ring, uint8_t *data)
{
  if (ring->head != ring->tail)
  {
    *data = ring->data[ring->head++];
    ring->head %= ring->size;
    return true;
  }
  return false;
}

uint8_t nib2hex(uint32_t nib)
{
  const uint8_t hex[] = "0123456789ABCDEF";
  return hex[nib & 0x0000000f];
}

bool hex2nib(uint8_t hex, uint8_t *nib)
{
  if (NULL == nib)
  {
    return false;
  }
  else if (('0' <= hex) && ('9' >= hex))
  {
    *nib = hex - '0';
    return true;
  }
  else if (('a' <= hex) && ('f' >= hex))
  {
    *nib = hex - 'a' + 10;
    return true;
  }
  else if (('A' <= hex) && ('F' >= hex))
  {
    *nib = hex - 'A' + 10;
    return true;
  }
  else
  {
    return false;
  }
}

bool bcd2nib(uint8_t bcd, uint8_t *nib)
{
  if ((NULL != nib) && ('0' <= bcd) && ('9' >= bcd))
  {
    *nib = bcd - '0';
    return true;
  }
  else
  {
    return false;
  }
}

bool read_hex(uint8_t *data)
{
  uint8_t nibble_ch = 0;
  uint8_t nibble_value = 0;

  if (NULL != data)
  {
    if (true == ring_pop(&rx_ring, &nibble_ch))
    {
      if (true == hex2nib(nibble_ch, &nibble_value))
      {
        *data = (nibble_value << 4);
        if (true == ring_pop(&rx_ring, &nibble_ch))
        {
          if (true == hex2nib(nibble_ch, &nibble_value))
          {
            *data |= nibble_value;
            return true;
          }
        }
      }
    }
  }

  return false;
}

bool read_bcd(uint8_t *data)
{
  uint8_t nibble_ch = 0;
  uint8_t nibble_value = 0;

  if (NULL != data)
  {
    if (true == ring_pop(&rx_ring, &nibble_ch))
    {
      if (true == bcd2nib(nibble_ch, &nibble_value))
      {
        *data = nibble_value;
        return true;
      }
    }
  }

  return false;
}

bool read_nib(uint8_t *data)
{
  uint8_t nibble_ch = 0;
  uint8_t nibble_value = 0;

  if (NULL != data)
  {
    if (true == ring_pop(&rx_ring, &nibble_ch))
    {
      if (true == hex2nib(nibble_ch, &nibble_value))
      {
        *data = nibble_value;
        return true;
      }
    }
  }

  return false;
}

void purge_command(void)
{
  bool popped = false;
  uint8_t ch = 0;
  do
  {
    popped = ring_pop(&rx_ring, &ch);
  }
  while ((true == popped) && ('\r' != ch));
}

/* setup */

void clock_setup(void)
{
  rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_AFIO);
  rcc_periph_clock_enable(RCC_USART2);
  rcc_periph_clock_enable(RCC_CAN);
}

void uart_setup(void)
{
  ring_init(&tx_ring, tx_buffer, sizeof(tx_buffer));
  ring_init(&rx_ring, rx_buffer, sizeof(rx_buffer));

  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART2_TX);
  gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO_USART2_RX);

  usart_set_baudrate(USART2, 115200);
  usart_set_databits(USART2, 8);
  usart_set_parity(USART2, USART_PARITY_NONE);
  usart_set_stopbits(USART2, USART_STOPBITS_1);
  usart_set_mode(USART2, USART_MODE_TX_RX);
  usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

  nvic_enable_irq(NVIC_USART2_IRQ);
  nvic_set_priority(NVIC_USART2_IRQ, 1);

  usart_enable_tx_interrupt(USART2);
  usart_enable_rx_interrupt(USART2);

  usart_enable(USART2);
}

void can_setup(void)
{
  gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON, AFIO_MAPR_CAN1_REMAP_PORTB);
  gpio_set_mode(GPIO_BANK_CAN1_PB_TX, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_CAN1_PB_TX);
  gpio_set_mode(GPIO_BANK_CAN1_PB_RX, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO_CAN1_PB_RX);
  gpio_set(GPIO_BANK_CAN1_PB_RX, GPIO_CAN1_PB_RX);

  nvic_enable_irq(NVIC_USB_LP_CAN_RX0_IRQ);
  nvic_set_priority(NVIC_USB_LP_CAN_RX0_IRQ, 1);
}

/* interrupt handlers */

void usart2_isr(void)
{
  uint8_t ch = 0;

  /* TX empty */
  if (true == usart_get_flag(USART2, USART_SR_TXE))
  {
    if (true == ring_pop(&tx_ring, &ch))
    {
      usart_send(USART2, ch);
    }
    else
    {
      usart_disable_tx_interrupt(USART2);
    }
  }

  /* RX not empty */
  if (true == usart_get_flag(USART2, USART_SR_RXNE))
  {
    ch = (uint8_t)usart_recv(USART2);
    if ('\n' != ch)
    {
      ring_push(&rx_ring, ch);
      if ('\r' == ch)
      {
        num_commands_pending += 1;
      }
    }
  }
}

void usb_lp_can_rx0_isr(void)
{
  int8_t i = 0;

  uint32_t msg_id = 0;
  bool ext = false;
  bool rtr = false;
  uint8_t fmi = 0;
  uint8_t dlc = 0;
  uint8_t payload[8] = {};

  can_receive(CAN1, 0, true, &msg_id, &ext, &rtr, &fmi, &dlc, payload, NULL);

  if (true == ext)
  {
    if (true == rtr)
    {
      ring_push(&tx_ring, 'R');
    }
    else
    {
      ring_push(&tx_ring, 'T');
    }

    for (i = 28; i >= 0; i -= 4)
    {
      ring_push(&tx_ring, nib2hex(msg_id >> i));
    }
  }
  else
  {
    if (true == rtr)
    {
      ring_push(&tx_ring, 'r');
    }
    else
    {
      ring_push(&tx_ring, 't');
    }

    for (i = 8; i >= 0; i -= 4)
    {
      ring_push(&tx_ring, nib2hex(msg_id >> i));
    }
  }

  ring_push(&tx_ring, nib2hex(dlc));

  if (false == rtr)
  {
    for (i = 0; i < dlc; i++)
    {
      ring_push(&tx_ring, nib2hex(payload[i] >> 4));
      ring_push(&tx_ring, nib2hex(payload[i]));
    }
  }

  ring_push(&tx_ring, '\r');

  usart_enable_tx_interrupt(USART2);
}

/* command loop */

void command_can_speed(void)
{
  uint8_t speed_id = 0;
  bool has_valid_data = false;

  if ((state_init == can_state) || (state_closed == can_state))
  {
    if (true == read_bcd(&speed_id))
    {
      if (num_can_timings > speed_id)
      {
        has_valid_data = true;
        can_speed = speed_id;
        can_state = state_closed;
      }
    }
  }

  purge_command();
  ring_push(&tx_ring, ((true == has_valid_data) ? '\r' : '\a'));
}

void command_open(bool silent)
{
  int32_t can_result = -1;

  purge_command();

  if ((state_closed == can_state) && (num_can_timings > can_speed))
  {
    can_reset(CAN1);
    can_result = can_init(CAN1, false, true, false, false, false, false,
                          can_timings[can_speed].sjw,
                          can_timings[can_speed].ts1,
                          can_timings[can_speed].ts2,
                          can_timings[can_speed].brp,
                          false, silent);
    if (0 == can_result)
    {
      can_filter_id_mask_32bit_init(0, 0, 0, 0, true);
      can_enable_irq(CAN1, CAN_IER_FMPIE0);

      can_state = ((true == silent) ? state_silent : state_normal);
      ring_push(&tx_ring, '\r');

      return;
    }
  }

  ring_push(&tx_ring, '\a');
}

void command_close(void)
{
  purge_command();

  if ((state_normal == can_state) || (state_silent == can_state))
  {
    can_disable_irq(CAN1, CAN_IER_FMPIE0);
    can_reset(CAN1);

    can_state = state_closed;

    ring_push(&tx_ring, '\r');
  }
  else
  {
    ring_push(&tx_ring, '\a');
  }
}

void command_transmit(bool ext, bool rtr)
{
  int8_t i = 0;
  uint8_t nibble = 0;
  uint8_t num_id_nibbles = ((true == ext) ? 8 : 3);
  uint32_t max_valid_msg_id = ((true == ext) ? 0x1fffffff : 0x7ff);
  int32_t can_result = -1;

  uint32_t msg_id = 0;
  uint8_t dlc = 0;
  uint8_t payload[8] = {};

  if (state_normal == can_state)
  {
    for (i = 0; i < num_id_nibbles; i++)
    {
      if (0 < i)
      {
        msg_id <<= 4;
      }

      if (true == read_nib(&nibble))
      {
        msg_id |= nibble;
      }
      else
      {
        purge_command();
        ring_push(&tx_ring, '\a');
        return;
      }
    }

    if (msg_id <= max_valid_msg_id)
    {
      if (true == read_nib(&dlc))
      {
        if (8 >= dlc)
        {
          if (false == rtr)
          {
            for (i = 0; i < dlc; i++)
            {
              if (false == read_hex(&payload[i]))
              {
                purge_command();
                ring_push(&tx_ring, '\a');
                return;
              }
            }
          }

          can_result = can_transmit(CAN1, msg_id, ext, rtr, dlc, payload);
          if (-1 != can_result)
          {
            purge_command();
            ring_push(&tx_ring, 'z');
            ring_push(&tx_ring, '\r');
            return;
          }
        }
      }
    }
  }

  purge_command();
  ring_push(&tx_ring, '\a');
}

void command_status_flags(void)
{
  purge_command();

  if (state_normal == can_state)
  {
    ring_push(&tx_ring, 'F');

    /* TODO: get CAN status flags */
    ring_push(&tx_ring, nib2hex(0x00));
    ring_push(&tx_ring, nib2hex(0x00));

    ring_push(&tx_ring, '\r');
  }
  else
  {
    ring_push(&tx_ring, '\a');
  }
}

void command_uart_speed(void)
{
  purge_command();
  /* TODO: set UART speed */
  ring_push(&tx_ring, '\a');
}

void command_version(void)
{
  int8_t i = 0;

  purge_command();
  ring_push(&tx_ring, 'V');
  for (i = 0; i < 4; i++)
  {
    ring_push(&tx_ring, version[i]);
  }
  ring_push(&tx_ring, '\r');
}

void command_serial_number(void)
{
  int8_t i = 0;

  purge_command();
  ring_push(&tx_ring, 'N');
  for (i = 0; i < 4; i++)
  {
    /* TODO: serial number */
    ring_push(&tx_ring, '0');
  }
  ring_push(&tx_ring, '\r');
}

void command_unsupported(void)
{
  purge_command();
  ring_push(&tx_ring, '\a');
}

void update(void)
{
  uint8_t command = 0;

  if (0 < num_commands_pending)
  {
    if (true == ring_pop(&rx_ring, &command))
    {
      switch(command)
      {
        case 'S':
        {
          command_can_speed();
          break;
        }

        case 'O':
        {
          command_open(false);
          break;
        }

        case 'L':
        {
          command_open(true);
          break;
        }

        case 'C':
        {
          command_close();
          break;
        }

        case 't':
        {
          command_transmit(false, false);
          break;
        }

        case 'T':
        {
          command_transmit(true, false);
          break;
        }

        case 'r':
        {
          command_transmit(false, true);
          break;
        }

        case 'R':
        {
          command_transmit(true, true);
          break;
        }

        case 'F':
        {
          command_status_flags();
          break;
        }

        case 'U':
        {
          command_uart_speed();
          break;
        }

        case 'V':
        {
          command_version();
          break;
        }

        case 'N':
        {
          command_serial_number();
          break;
        }

        case '\r':
        {
          break;
        }

        default:
        {
          command_unsupported();
          break;
        }
      }

      num_commands_pending--;
      usart_enable_tx_interrupt(USART2);
    }
    else
    {
      /* RX buffer is empty and can't have a command */
      num_commands_pending = 0;
    }
  }
}

/* start point */

int main(void)
{
  clock_setup();
  uart_setup();
  can_setup();

  while (1)
  {
    update();
  }

  return 0;
}
