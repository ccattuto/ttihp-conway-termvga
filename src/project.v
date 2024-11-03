/*
 * Copyright (c) 2024 Ciro Cattuto <ciro.cattuto@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module tt_um_ccattuto_conway (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered, so you can ignore it
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

// -------------- I/O PINS ---------------------------

// All output pins must be assigned. If not used, assign to 0.
assign uo_out[3:0] = 0;
assign uo_out[7:5] = 0;
assign uio_oe  = 8'b1;

// UART signals
wire uart_rx, uart_tx;
assign uart_rx = ui_in[3];
assign uo_out[4] = uart_tx;

// clock
localparam CLOCK_FREQ = 24000000;

// reset
wire boot_reset;
assign boot_reset = ~rst_n;

// GPIO simulation control (as opposed to control via UART)
wire ctrl_running, ctrl_randomize;
assign ctrl_running = ui_in[0];
assign ctrl_randomize = ui_in[1];

// TinyVGA PMOD
assign uio_out = {hsync, B[0], G[0], R[0], vsync, B[1], G[1], R[1]};


// -------------- UART TRANSMITTER ---------------------------

wire uart_tx_en;
assign uart_tx_en = 1;

reg [7:0]   uart_tx_data;
reg         uart_tx_valid;
wire        uart_tx_ready;

UARTTransmitter #(
    .CLOCK_RATE(CLOCK_FREQ),
    .BAUD_RATE(115200)
) uart_tx_inst (
    .clk(clk),
    .reset(boot_reset),       // reset
    .enable(uart_tx_en),      // TX enable
    .valid(uart_tx_valid),    // start of TX
    .in(uart_tx_data),        // data to transmit
    .out(uart_tx),            // TX signal
    .ready(uart_tx_ready)     // read for TX data
);


// -------------- UART RECEIVER ---------------------------

wire uart_rx_en;
assign uart_rx_en = 1;

wire [7:0]  uart_rx_data;
wire        uart_rx_valid;
wire        uart_rx_error;
wire        uart_rx_overrun;
reg         uart_rx_ready;

UARTReceiver #(
    .CLOCK_RATE(CLOCK_FREQ),
    .BAUD_RATE(115200)
) uart_rx_inst (
    .clk(clk),
    .reset(boot_reset),       // reset
    .enable(uart_rx_en),      // RX enable
    .in(uart_rx),             // RX signal
    .ready(uart_rx_ready),    // ready to consume RX data
    .out(uart_rx_data),       // RX wires
    .valid(uart_rx_valid),    // RX completed
    .error(uart_rx_error),    // RX error
    .overrun(uart_rx_overrun) // RX overrun
);


// ---------------- VGA ---------------

wire hsync;
wire vsync;
wire [1:0] R;
wire [1:0] G;
wire [1:0] B;
wire video_active;
wire [9:0] pix_x;
wire [9:0] pix_y;

hvsync_generator hvsync_inst (
  .clk(clk),
  .reset(boot_reset),
  .hsync(hsync),
  .vsync(vsync),
  .display_on(video_active),
  .hpos(pix_x),
  .vpos(pix_y)
);

// high when the pixel belongs to the simulation rectangle
wire frame_active;
assign frame_active = (pix_x >= 64 && pix_x < 640-64 && pix_y >= 112 && pix_y < 480-112) ? 1 : 0;

// look up into the 16x16 icon bitmap for live cells
wire icon_pixel;
assign icon_pixel = icon[pix_y[3:0]][pix_x[3:0]];

// compute index into board state (hardcoded logic for 32x16 grid)
wire [8:0] cell_index;
assign cell_index = (pix_y[7:4] << 5) | pix_x[8:4];

// generate RGB signals
assign R = (video_active & frame_active) ? {board_state[cell_index] & icon_pixel, 1'b1} : 2'b00;
assign G = (video_active & frame_active) ? {board_state[cell_index] & icon_pixel, 1'b1} : 2'b00;
assign B = 2'b01;


// ----------------- RNG ----------------------

wire rng;

lfsr_rng lfsr(
  .clk(clk),
  .reset(boot_reset),
  .random_bit(rng)
);


// ----------------- SIMULATION PARAMS -------------------------

localparam logWIDTH = 5, logHEIGHT = 4;         // 32x16 board
localparam UPDATE_INTERVAL = CLOCK_FREQ / 5;    // 5 Hz simulation update

localparam WIDTH = 2 ** logWIDTH;
localparam HEIGHT = 2 ** logHEIGHT;
localparam BOARD_SIZE = WIDTH * HEIGHT;

reg board_state [0:BOARD_SIZE-1];         // current state of the simulation
reg [BOARD_SIZE-1:0] board_state_next;    // next state of the simulation


// ----------------- SIMULATION CONTROL VIA UART RX --------------------

localparam ACTION_IDLE = 0, ACTION_UPDATE = 1, ACTION_COPY = 2, ACTION_DISPLAY = 3, ACTION_RND = 4, ACTION_INIT = 5, ACTION_LOAD_INIT = 6, ACTION_DISPLAY_INIT = 7;
reg [2:0] action;

reg running;  // high when simulation is advancing automatically based on timer
reg [31:0] timer;

reg [logWIDTH+logHEIGHT-1:0] index;    // index of cell being updated
wire [logWIDTH-1:0] cell_x;             // x-coordinate (column) of cell being updated
wire [logHEIGHT-1:0] cell_y;            // y coordinate (row) of cell being updated
assign cell_x = index[logWIDTH-1:0];
assign cell_y = index[logWIDTH+logHEIGHT-1:logWIDTH];

reg [3:0] neigh_index;                  // index of neighboring cell (0 to 7)
reg [3:0] num_neighbors;                // number of neighbors of current cell

localparam HEIGHT_MASK = {logHEIGHT{1'b1}};
localparam WIDTH_MASK = {logWIDTH{1'b1}};

localparam TX_IDLE = 0, TX_SEND = 1, TX_WAIT = 2, TX_SEND_CRLF = 3, TX_WAIT_CRLF = 4, TX_SEND_HOME = 5, TX_WAIT_HOME = 6, TX_INIT = 7, TX_WAIT_INIT = 8;
reg [3:0] txstate;
reg [logWIDTH:0] colindex;            // column index of current cell
reg [5:0] txindex;                    // index into strings to be printed (welcome message, formatting)

localparam CELL_ALIVE_CHAR = 79;      // "O" is used to display alive cells
localparam CELL_DEAD_CHAR = 32;       // " " is used to display dead cells


// STATE TRANSITION LOGIC
//
// - ACTION_UPDATE computes board_state_next based on board_state
// - ACTION_COPY copies board_state_next over board_state
// - ACTION_DISPLAY prints out board_state over UART (ANSI terminal)
// - ACTION_RND randomizes board_state

always @(posedge clk) begin
  if (boot_reset) begin
    action <= ACTION_INIT;
    running <= 0;
    timer <= 0;

    uart_rx_ready <= 0;
    uart_tx_data <= 0;
    uart_tx_valid <= 0;

    index <= 0;
    neigh_index <= 0;
    num_neighbors <= 0;

    txstate <= TX_INIT;
    colindex <= 0;
    txindex <= 0;
  end else begin
    case (action)
      // at init, any received character triggers randomized initialization and refresh over UART
      ACTION_INIT: begin
        if (!(uart_rx_valid & uart_rx_ready)) begin
          uart_rx_ready <= 1;
        end else begin
          action <= ACTION_LOAD_INIT;
          uart_rx_ready <= 0;
        end
      end

      // idle loop: simulation controlled via characters received over UART 
      ACTION_IDLE: begin
        if (uart_rx_valid & uart_rx_ready) begin
          uart_rx_ready <= 0;
          
          case (uart_rx_data)
            // '0' randomizes the state of the simulation
            48: begin
              action <= ACTION_RND;
            end

            // '1' advances the simulation by 1 step
            49: begin
              if (~running) begin
                action <= ACTION_UPDATE;
              end else begin
                running <= 0;
                timer <= 0;
              end
            end

            // ' ' toggles simulation timer-based advancing
            32: begin
              running <= ~running;
              timer <= 0;
            end

            default: begin
              action <= ACTION_IDLE;
            end
          endcase
        end else if (uart_rx_valid) begin
          uart_rx_ready <= 1;
        end else if (running) begin // timer-based update trigger
          if (timer < UPDATE_INTERVAL) begin
            timer <= timer + 1;
          end else begin
            timer <= 0;
            uart_rx_ready <= 0;
            action <= (~ctrl_randomize) ? ACTION_UPDATE : ACTION_RND;
          end
        end else if (ctrl_running) begin
          running <= 1;
          timer <= 0;
        end
      end

      // ----------------- ACTION: RANDOMIZE SIMULATION STATE --------------------

      ACTION_RND: begin
        board_state[index] <= rng;
        if (index < BOARD_SIZE - 1) begin
          index <= index + 1;
        end else  begin
          index <= 0;
          txstate <= TX_SEND_HOME;
          action <= ACTION_DISPLAY;
        end
      end

      // ----------------- ACTION: LOAD INITIAL PATTERN --------------------

      ACTION_LOAD_INIT: begin
        board_state[index] <= board_state_init[index];
        if (index < BOARD_SIZE - 1) begin
          index <= index + 1;
        end else  begin
          index <= 0;
          action <= ACTION_DISPLAY_INIT;
        end
      end

      // ----------------- ACTION: COMPUTE SIMULATION'S NEXT STATE --------------------

      ACTION_UPDATE: begin
        // loop over the 8 neighbors of the current cell
        case (neigh_index)
          0: begin // (-1, +1)
            num_neighbors <= num_neighbors + board_state[((cell_y + 1) & HEIGHT_MASK) << logWIDTH | ((cell_x - 1) & WIDTH_MASK)];
            neigh_index <= neigh_index + 1;
          end

          1: begin // (0, +1)
            num_neighbors <= num_neighbors + board_state[((cell_y + 1) & HEIGHT_MASK) << logWIDTH | ((cell_x + 0) & WIDTH_MASK)];
            neigh_index <= neigh_index + 1; 
          end

          2: begin // (+1, +1)
            num_neighbors <= num_neighbors + board_state[((cell_y + 1) & HEIGHT_MASK) << logWIDTH | ((cell_x + 1) & WIDTH_MASK)];
            neigh_index <= neigh_index + 1;
          end

          3: begin // (-1, 0)
            num_neighbors <= num_neighbors + board_state[((cell_y + 0) & HEIGHT_MASK) << logWIDTH | ((cell_x - 1) & WIDTH_MASK)];
            neigh_index <= neigh_index + 1;
          end

          4: begin // (+1, 0)
            num_neighbors <= num_neighbors + board_state[((cell_y + 0) & HEIGHT_MASK) << logWIDTH | ((cell_x + 1) & WIDTH_MASK)];
            neigh_index <= neigh_index + 1;
          end

          5: begin // (-1, -1)
            num_neighbors <= num_neighbors + board_state[((cell_y - 1) & HEIGHT_MASK) << logWIDTH | ((cell_x - 1) & WIDTH_MASK)];
            neigh_index <= neigh_index + 1;
          end

          6: begin // (0, -1)
            num_neighbors <= num_neighbors + board_state[((cell_y - 1) & HEIGHT_MASK) << logWIDTH | ((cell_x + 0) & WIDTH_MASK)];
            neigh_index <= neigh_index + 1;
          end

          7: begin // (+1, -1)
            num_neighbors <= num_neighbors + board_state[((cell_y - 1) & HEIGHT_MASK) << logWIDTH | ((cell_x + 1) & WIDTH_MASK)];
            neigh_index <= neigh_index + 1;
          end

          // this state (neigh_index = 8) is used to compute the new state of the current cell
          // according to the rules of Conway's Game of Life
          8: begin
            board_state_next <= { board_state_next[BOARD_SIZE-2:0], (board_state[index] && (num_neighbors == 2)) | (num_neighbors == 3) };
            //board_state_next[index] <= (board_state[index] && (num_neighbors == 2)) | (num_neighbors == 3);

            neigh_index <= 0;
            num_neighbors <= 0;

            // advance to next cell to be updated, or terminate
            if (index < BOARD_SIZE - 1) begin
              index <= index + 1;
            end else begin
              index <= 0;
              action <= ACTION_COPY;
            end
          end

          default: begin
            neigh_index <= 0;
          end
        endcase
      end

      // --------------- ACTION: COPY NEW SIMULATION STATE OVER OLD ONE --------------------

      ACTION_COPY: begin
        if (vsync) begin // synchronize to vsync
          // board_state[index] <= board_state_next[index];
          board_state[index] <= board_state_next[BOARD_SIZE-1];
          board_state_next <= {board_state_next[BOARD_SIZE-2:0], 1'b0};
          if (index < BOARD_SIZE - 1) begin
            index <= index + 1;
          end else begin
            index <= 0;
            action <= ACTION_DISPLAY;
            txstate <= TX_SEND_HOME;
          end
        end
      end

      // --------------- ACTION: DISPLAY SIMULATION STATE OVER UART --------------------

      ACTION_DISPLAY: begin
        case (txstate)
          // Sends ASCII character corresponding to current cell state,
          // then waits (TX_WAIT) for UART transmitted to be ready
          // and advances to the next cell.
          TX_SEND: begin
            if (colindex < WIDTH) begin
              uart_tx_data <= board_state[index] ? CELL_ALIVE_CHAR : CELL_DEAD_CHAR; // cell state to ASCII char
              index <= index + 1;
              colindex <= colindex + 1;
              txstate <= TX_WAIT;
            end else begin
              colindex <= 0;
              txstate <= TX_SEND_CRLF;
            end
          end

          TX_WAIT: begin
            if (uart_tx_ready && !uart_tx_valid ) begin
              uart_tx_valid <= 1;
            end else if (uart_tx_valid && !uart_tx_ready) begin
              uart_tx_valid <= 0;
              if (index != 0) begin
                txstate <= TX_SEND;
              end else begin
                txstate <= TX_IDLE;      
                action <= ACTION_IDLE;
              end
            end
          end        

          // Sends CR+LF (used at the end of every row)
          TX_SEND_CRLF: begin
            if (txindex < 2) begin
              uart_tx_data <= (txindex == 0) ? 13 : 10;
              txindex <= txindex + 1;
              txstate <= TX_WAIT_CRLF;
            end else begin
              txindex <= 0;
              txstate <= TX_SEND;              
            end
          end

          TX_WAIT_CRLF: begin
            if (uart_tx_ready && !uart_tx_valid ) begin
              uart_tx_valid <= 1;
            end else if (uart_tx_valid && !uart_tx_ready) begin
              uart_tx_valid <= 0;
              txstate <= TX_SEND_CRLF;
            end
          end        

          // Sends ANSI "home" escape sequence [0x1b, '[', ';', 'H']
          // moving the cursor to the top left of the terminal (0,0) 
          TX_SEND_HOME: begin
            if (txindex < 4) begin
              case (txindex)
                0: begin
                  uart_tx_data <= 27;
                end
                1: begin
                  uart_tx_data <= 91;
                end
                2: begin
                  uart_tx_data <= 59;
                end
                3: begin
                  uart_tx_data <= 72;
                end
                default:
                  uart_tx_data <= 0;
              endcase
              txindex <= txindex + 1;
              txstate <= TX_WAIT_HOME;
            end else begin
              txindex <= 0;
              colindex <= 0;
              txstate <= TX_SEND;
            end
          end

          TX_WAIT_HOME: begin
            if (uart_tx_ready && !uart_tx_valid ) begin
              uart_tx_valid <= 1;
            end else if (uart_tx_valid && !uart_tx_ready) begin
              uart_tx_valid <= 0;
              txstate <= TX_SEND_HOME;
            end
          end

          default: begin
          end
        endcase
      end

      ACTION_DISPLAY_INIT: begin
        case (txstate)
          // prints out welcome message and usage instructions stored in string_init (ROM below)
          TX_INIT: begin
            if (txindex < STRING_INIT_LEN) begin
              uart_tx_data <= string_init[txindex];
              txindex <= txindex + 1;
              txstate <= TX_WAIT_INIT;    
            end else begin
              txstate <= TX_IDLE;
              txindex <= 0;
              action <= ACTION_IDLE;
            end
          end

          TX_WAIT_INIT: begin
            if (uart_tx_ready && !uart_tx_valid ) begin
              uart_tx_valid <= 1;
            end else if (uart_tx_valid && !uart_tx_ready) begin
              uart_tx_valid <= 0;
              txstate <= TX_INIT;
            end
          end

          default: begin
          end
        endcase
      end

      default: begin
        action <= ACTION_IDLE;
      end
    endcase
  end
end 


// ROM containing the welcome message with usage instructions
localparam STRING_INIT_LEN = 57;
reg [7:0] string_init [0:STRING_INIT_LEN-1];
initial begin
  $readmemh("string_init.hex", string_init);
end

reg [511:0] board_state_init;
initial begin
  board_state_init = 512'b10100110111101010110100111101110100111110001100111110000000010100101111010111010001011011000101000001100011111011111110101010011101101000101110000100111111011011011111011001101010110100010101001101111100100001100101110111001101111000000010111111010110000100000000110111100110111011110100101111011110001111111111101011011011101000101110001010100010111101101111010011010000111010101001100001110010001101100110011100111010101000111010100110010000000101100100111100010000000110010100011001101111111010100011000011111;
end

// Icon for live cell
reg [15:0] icon[0:15];
initial begin
  icon[0]  = 16'b0000000000000000;
  icon[1]  = 16'b0000000000000000;
  icon[2]  = 16'b0000001111000000;
  icon[3]  = 16'b0000111111110000;
  icon[4]  = 16'b0001111111111000;
  icon[5]  = 16'b0001111111111000;
  icon[6]  = 16'b0011111111111100;
  icon[7]  = 16'b0011111111111100;
  icon[8]  = 16'b0011111111111100;
  icon[9]  = 16'b0011111111111100;
  icon[10] = 16'b0001111111111000;
  icon[11] = 16'b0001111111111000;
  icon[12] = 16'b0000111111110000;
  icon[13] = 16'b0000001111000000;
  icon[14] = 16'b0000000000000000;
  icon[15] = 16'b0000000000000000;
end

endmodule
