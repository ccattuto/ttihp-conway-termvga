# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-License-Identifier: MIT

import cocotb
from cocotb.clock import Clock
import cocotb.result
from cocotb.triggers import Timer, Edge, with_timeout
import numpy as np

BOARD_WIDTH = 8
BOARD_HEIGHT = 8


@cocotb.test(timeout_time=100, timeout_unit='ms')
async def test(dut):
    dut._log.info("Start")

    # Set the clock period to 24 MHz
    clock = Clock(dut.clk, 41666, units="ps")
    cocotb.start_soon(clock.start())

    # UART signals
    uart_rx = dut.ui_in[3]
    uart_tx = dut.uo_out[4]

    # GPIO config
    do_gpio_config(dut)

    # reset
    await do_reset(dut)
    assert uart_tx == 1

    # send CR over UART to trigger init message
    f = cocotb.start_soon(send_cmd(dut, uart_rx, 13))
    init_str = await get_uart_str(dut, uart_tx)
    await f

    # check for correct init string
    assert init_str == INIT_STRING
    dut._log.info("Received correct init string")

    # send '0' and receive board update
    await Timer(0.7, units="ms")
    f = cocotb.start_soon(send_cmd(dut, uart_rx, ord('0')))
    board_state_str = await get_uart_str(dut, uart_tx)
    await f

    # check board update
    board_state = parse_board_state(board_state_str)
    dut._log.info("RANDOMIZED")
    dut._log.info(board_state)

    # advance step by step for 5 steps and check correct state evolution
    for count in range(5):
        board_state_correct = next_board_state(board_state)

        # send '1' and receive board update
        await Timer(0.25, units="ms")
        f = cocotb.start_soon(send_cmd(dut, uart_rx, ord('1')))
        await Edge(dut.uo_out)
        board_state_str = await get_uart_str(dut, uart_tx)
        await f

        board_state = parse_board_state(board_state_str)
        dut._log.info(f"STEP #{count+1}")
        dut._log.info(board_state)

        assert np.array_equal(board_state, board_state_correct)

    dut._log.info("All good!")

@cocotb.test(timeout_time=100, timeout_unit='ms')
async def test2(dut):
    dut._log.info("Start")

    # Set the clock period to 24 MHz
    clock = Clock(dut.clk, 41666, units="ps")
    cocotb.start_soon(clock.start())

    # UART signals
    uart_rx = dut.ui_in[3]
    uart_tx = dut.uo_out[4]

    # GPIO config
    do_gpio_config(dut)

    # reset
    await do_reset(dut)
    assert uart_tx == 1

    # send CR over UART to trigger init message
    f = cocotb.start_soon(send_cmd(dut, uart_rx, 13))
    init_str = await get_uart_str(dut, uart_tx)
    await f

    # check for correct init string
    assert init_str == INIT_STRING
    dut._log.info("Received correct init string")

    # compute correct 1-step evolution of the initial state
    board_state = np.array([1 if c == "1" else 0 for c in "1100000111010000100010100100011001111110100110011101110110010111"[::-1]], dtype=int).reshape(8,8)
    board_state_correct = next_board_state(board_state)

    # send '1' and receive board update
    await Timer(0.25, units="ms")
    f = cocotb.start_soon(send_cmd(dut, uart_rx, ord('1')))
    await Edge(dut.uo_out)
    board_state_str = await get_uart_str(dut, uart_tx)
    await f

    board_state = parse_board_state(board_state_str)
    dut._log.info(board_state)

    assert np.array_equal(board_state, board_state_correct)

    dut._log.info("All good!")


# HELPER FUNCTIONS

async def send_cmd(dut, uart_rx, cmd=13):
    dut._log.info("Sending: 0x%02X" % cmd)
    await do_tx(uart_rx, 115200, cmd)

async def do_reset(dut):
    dut._log.info("Reset")
    dut.ena.value = 1
    dut.rst_n.value = 0
    await Timer(1, units="us")
    dut.rst_n.value = 1
    await Timer(5, units="us")

def do_gpio_config(dut):
    dut._log.info("GPIO config")
    # GPIO IN
    dut.ui_in.value = 0
    # set RX high
    dut.ui_in[3].value = 1

    # GPIO IN/OUT
    dut.uio_in.value = 0

async def do_tx(uart_rx, baud, data):
    # prepare random test data
    TEST_BITS_LSB = [(data >> s) & 1 for s in range(8)]

    # send start bit (0), 8 data bits, stop bit (1)
    for tx_bit in [0] + TEST_BITS_LSB + [1]:
        uart_rx.value = tx_bit
        await Timer(int(1.0 / baud * 1e12), units="ps")

async def do_rx(dut, uart_tx, baud, timeout_us=0):
    if timeout_us > 0:
        count = 0
        did_timeout = False
        while uart_tx.value == 1 and count < timeout_us:
            count += 1
            try:
                await with_timeout(Edge(dut.uo_out), 1, 'us')
                did_timeout = False
            except cocotb.result.SimTimeoutError:
                did_timeout = True

        if did_timeout:
            return None
    else:
        while uart_tx.value == 1:
            await Edge(dut.uo_out)
    
    assert uart_tx.value == 0

    # wait 1/2 bit
    await Timer(int(0.5 / baud * 1e12), units="ps")
    # check start bit
    assert uart_tx.value == 0

    # 8 data bits
    data = 0
    for i in range(8):
        await Timer(int(1.0 / baud * 1e12), units="ps")
        data |= (1 << i) if uart_tx.value == 1 else 0

    # check stop bit
    await Timer(int(1.0 / baud * 1e12), units="ps")
    assert uart_tx.value == 1

    return data

async def get_uart_str(dut, uart_tx):
    blist = []

    while True:
        rx_byte = await do_rx(dut, uart_tx, 115200, timeout_us=100)
        if rx_byte == None:
            break
        dut._log.info("Received [%d]: 0x%02X" % (len(blist), rx_byte))
        blist.append(rx_byte)

    return bytes(blist).decode()

INIT_STRING = "\x1bc" + "\x1b[92m" + "Hello!\r\nspace: start/stop\r\n0: randomize\r\n1: step\r\n"
ESC_STRING = "\x1b[;H"

def parse_board_state(state_string):
    assert state_string[:4] == ESC_STRING
    assert len(state_string) == 4 + BOARD_HEIGHT * BOARD_WIDTH + (BOARD_HEIGHT-1)*2
    assert len( set(state_string[4:]).difference(['O',' ','\r','\n']) ) == 0
    rows = state_string[4:].split("\r\n")
    assert len(rows) == BOARD_HEIGHT
    assert [len(r) for r in rows] == BOARD_HEIGHT * [BOARD_WIDTH]
    return np.array( [[1 if c=='O' else 0 for c in r] for r in rows] )

def num_neighbors(state, i, j):
    i_above = (i-1) if (i > 0) else (BOARD_HEIGHT-1)
    i_below = (i+1) if (i < BOARD_HEIGHT-1) else (0)
    j_left = (j-1) if (j > 0) else (BOARD_WIDTH-1)
    j_right = (j+1) if (j < BOARD_WIDTH-1) else (0)

    neighs = 0
    neighs += state[i_above, j_left] + state[i_above, j] + state[i_above, j_right]
    neighs += state[i, j_left] + state[i, j_right]
    neighs += state[i_below, j_left] + state[i_below, j] + state[i_below, j_right]

    return neighs

# Conway's Game of Life update step
def next_board_state(state):
    next_state = np.zeros((BOARD_HEIGHT, BOARD_WIDTH), dtype=int)
    for i in range(BOARD_HEIGHT):
        for j in range(BOARD_WIDTH):
            n = num_neighbors(state, i, j)
            if (state[i,j] != 0 and (n == 2)) or (n == 3):
                next_state[i,j] = 1
    return next_state

