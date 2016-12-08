#include "stm32f0xx_hal.h"
#include <string.h>

#include "i2c_slave.h"
#include "uart.h"

struct i2c_registers_type i2c_registers;

void i2c_data_xmt(I2C_HandleTypeDef *hi2c);

void i2c_slave_start() {
  memset(&i2c_registers, '\0', sizeof(i2c_registers));

  i2c_registers.frequency_MHz = 48;

  HAL_I2C_EnableListen_IT(&hi2c1);
}

#undef COUNT_I2C

#ifdef COUNT_I2C
static uint32_t counter_addr, counter_data_rcv, counter_rxcplt, counter_txcplt, counter_listen, counter_error, counter_abort;
#define ADD_COUNT_I2C(counter) counter++
#else
#define ADD_COUNT_I2C(counter)
#endif


static uint8_t i2c_transfer_position;
static enum {STATE_WAITING, STATE_GET_ADDR, STATE_GET_DATA, STATE_SEND_DATA, STATE_DROP_DATA} i2c_transfer_state;
static uint8_t i2c_data;

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode) {
  ADD_COUNT_I2C(counter_addr);

  if(TransferDirection == I2C_DIRECTION_TRANSMIT) { // master transmit
    i2c_transfer_state = STATE_GET_ADDR;
    HAL_I2C_Slave_Sequential_Receive_IT(hi2c, &i2c_data, 1, I2C_FIRST_FRAME);
  } else {
    i2c_transfer_state = STATE_SEND_DATA;
    i2c_data_xmt(hi2c);
  }
}

void i2c_data_rcv(uint8_t position, uint8_t data) {
  uint8_t *p = (uint8_t *)&i2c_registers;

  ADD_COUNT_I2C(counter_data_rcv);
  p[position] = data;
}

void i2c_data_xmt(I2C_HandleTypeDef *hi2c) {
  HAL_I2C_Slave_Sequential_Transmit_IT(hi2c, (uint8_t *)&i2c_registers, sizeof(i2c_registers), I2C_FIRST_FRAME);
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c) {
  ADD_COUNT_I2C(counter_txcplt);

  if(i2c_transfer_state == STATE_DROP_DATA) { // encountered a stop/error condition
    return;
  }

  i2c_data = 0;
  HAL_I2C_Slave_Sequential_Transmit_IT(hi2c, &i2c_data, 1, I2C_NEXT_FRAME);
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
  ADD_COUNT_I2C(counter_rxcplt);

  switch(i2c_transfer_state) {
    case STATE_GET_ADDR:
      i2c_transfer_position = i2c_data;
      i2c_transfer_state = STATE_GET_DATA;
      HAL_I2C_Slave_Sequential_Receive_IT(hi2c, &i2c_data, 1, I2C_LAST_FRAME);
      break;
    case STATE_GET_DATA:
      i2c_data_rcv(i2c_transfer_position, i2c_data);
      i2c_transfer_position++;
      HAL_I2C_Slave_Sequential_Receive_IT(hi2c, &i2c_data, 1, I2C_LAST_FRAME);
      break;
    default:
      HAL_I2C_Slave_Sequential_Receive_IT(hi2c, &i2c_data, 1, I2C_LAST_FRAME);
      break;
  }

  if(i2c_transfer_position > sizeof(struct i2c_registers_type)) {
    i2c_transfer_state = STATE_DROP_DATA;
  }
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c) {
  ADD_COUNT_I2C(counter_listen);
  HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
  ADD_COUNT_I2C(counter_error);

  i2c_transfer_state = STATE_DROP_DATA;

  if(hi2c->ErrorCode != HAL_I2C_ERROR_AF) {
    HAL_I2C_EnableListen_IT(hi2c);
  }
}

void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c) {
  ADD_COUNT_I2C(counter_abort);

  i2c_transfer_state = STATE_DROP_DATA;
}

void i2c_show_data() {
#ifdef COUNT_I2C
  uint8_t printed = 0;

  if(counter_addr > 0) {
    write_uart_s("addr ");
    write_uart_u(counter_addr);
    write_uart_s(" ");
    counter_addr = 0;
    printed++;
  }
  if(counter_data_rcv > 0) {
    write_uart_s("data_rcv ");
    write_uart_u(counter_data_rcv);
    write_uart_s(" ");
    counter_data_rcv = 0;
    printed++;
  }
  if(counter_rxcplt > 0) {
    write_uart_s("rxcplt ");
    write_uart_u(counter_rxcplt);
    write_uart_s(" ");
    counter_rxcplt = 0;
    printed++;
  }
  if(counter_txcplt > 0) {
    write_uart_s("txcplt ");
    write_uart_u(counter_txcplt);
    write_uart_s(" ");
    counter_txcplt = 0;
    printed++;
  }
  if(counter_listen > 0) {
    write_uart_s("listen ");
    write_uart_u(counter_listen);
    write_uart_s(" ");
    counter_listen = 0;
    printed++;
  }
  if(counter_error > 0) {
    write_uart_s("error ");
    write_uart_u(counter_error);
    write_uart_s(" ");
    counter_error = 0;
    printed++;
  }
  if(counter_abort > 0) {
    write_uart_s("abort ");
    write_uart_u(counter_abort);
    write_uart_s(" ");
    counter_abort = 0;
    printed++;
  }

  if(printed > 0) {
    write_uart_s("\n");
  }
#endif
}
