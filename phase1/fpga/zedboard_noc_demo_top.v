//=====================================================================
// zedboard_noc_top.v  -  ZedBoard Wrapper for Direct Manual NoC Testing
//=====================================================================
`timescale 1ns/1ps

module zedboard_noc_demo_top (
    input  wire        clk_100,
    output wire [7:0]  led        // LD0-LD7 physical ZedBoard LEDs
);

    //-----------------------------------------------------------------
    // 1. Clock Generation : 100 MHz Single-Ended -> 50 MHz Clock
    //-----------------------------------------------------------------
    wire clk100_ibuf;
    wire clk50_bufr;
    wire clk;

    IBUF u_ibuf (
        .I (clk_100),
        .O (clk100_ibuf)
    );

    BUFR #(
        .BUFR_DIVIDE("2"),
        .SIM_DEVICE("7SERIES")
    ) u_bufr_div (
        .I   (clk100_ibuf),
        .CE  (1'b1),
        .CLR (1'b0),
        .O   (clk50_bufr)
    );

    BUFG u_bufg (
        .I (clk50_bufr),
        .O (clk)
    );

    //-----------------------------------------------------------------
    // 2. VIO Wires & Readouts
    //-----------------------------------------------------------------
    wire        vio_rst;
    wire [1:0]  vio_src, vio_dest, vio_qos;
    wire [3:0]  vio_len;
    wire        vio_corrupt, vio_send, vio_sweep;

    reg  [15:0] pass_cnt, fail_cnt, pkt_cnt;
    reg  [7:0]  out_cnt [0:3];
    wire [31:0] out_cnt_flat = {out_cnt[3], out_cnt[2], out_cnt[1], out_cnt[0]};
    wire [7:0]  status;

    reg  [15:0] last_rx_data;
    reg  [1:0]  last_rx_port;

    //-----------------------------------------------------------------
    // 3. Reset Generator
    //-----------------------------------------------------------------
    reg [7:0] por_cnt = 8'd0;
    reg       resetn_r = 1'b0;

    always @(posedge clk) begin
        if (por_cnt != 8'hFF) por_cnt <= por_cnt + 8'd1;
        resetn_r <= (por_cnt == 8'hFF) & ~vio_rst;
    end

    //-----------------------------------------------------------------
    // 4. Router Core (DUT)
    //-----------------------------------------------------------------
    reg  [3:0]  valid_in;
    reg  [3:0]  is_header_in;
    reg  [63:0] data_in;
    wire [3:0]  read_enb_out = 4'hF;      // Continuous output FIFO draining

    wire [63:0] data_out;
    wire [3:0]  out_valid;
    wire        busy, parity_error, acl_drop_flag;
    wire [1:0]  invalid_addr_flag;

    secure_noc_router_top u_router (
        .clk               (clk),
        .resetn            (resetn_r),
        .valid_in          (valid_in),
        .is_header_in      (is_header_in),
        .data_in           (data_in),
        .read_enb_out      (read_enb_out),
        .data_out          (data_out),
        .out_valid         (out_valid),
        .busy              (busy),
        .parity_error      (parity_error),
        .acl_drop_flag     (acl_drop_flag),
        .invalid_addr_flag (invalid_addr_flag)
    );

    //-----------------------------------------------------------------
    // 5. Packet Helper Functions
    //-----------------------------------------------------------------
    function [15:0] mk_payload;
        input [1:0] s;
        input [1:0] d;
        input [3:0] i;
        begin
            mk_payload = 16'h1000 + {6'd0, s, 8'd0} + {9'd0, d, 5'd0} + {12'd0, i};
        end
    endfunction

    //-----------------------------------------------------------------
    // 6. Packet Generator FSM
    //-----------------------------------------------------------------
    localparam [2:0] S_IDLE = 3'd0,
                     S_HDR  = 3'd1,
                     S_PAY  = 3'd2,
                     S_PAR  = 3'd3,
                     S_END  = 3'd4,
                     S_WAIT = 3'd5,
                     S_CHK  = 3'd6,
                     S_NEXT = 3'd7;

    reg [2:0]  state;
    reg [1:0]  j_src, j_dest, j_qos;
    reg [3:0]  j_len;
    reg        j_corrupt;
    reg [3:0]  k;
    reg [15:0] parity_acc;
    reg [15:0] tx_parity;
    reg [7:0]  wait_cnt;
    reg        sweep_mode;
    reg [4:0]  sweep_idx;
    reg        sweep_done;
    reg        send_d, sweep_d;
    reg        last_valid, last_pass;
    reg        mismatch;

    wire j_blocked = ((j_src == 2'd0) && (j_dest == 2'd3)) ||   // N -> W
                     ((j_src == 2'd2) && (j_dest == 2'd1));     // E -> S

    wire [15:0] hdr_word = {8'h00, j_len, j_dest, j_qos};

    wire [7:0] exp_cnt     = j_blocked ? 8'd0 : ({4'd0, j_len} + 8'd1);
    wire       others_zero = ((j_dest == 2'd0) || (out_cnt[0] == 8'd0)) &&
                             ((j_dest == 2'd1) || (out_cnt[1] == 8'd0)) &&
                             ((j_dest == 2'd2) || (out_cnt[2] == 8'd0)) &&
                             ((j_dest == 2'd3) || (out_cnt[3] == 8'd0));
    wire       chk_ok      = !mismatch &&
                             (out_cnt[j_dest] == exp_cnt) &&
                             others_zero &&
                             (acl_drop_flag == j_blocked) &&
                             (parity_error  == j_corrupt);

    always @(posedge clk) begin
        send_d  <= vio_send;
        sweep_d <= vio_sweep;

        if (!resetn_r) begin
            state        <= S_IDLE;
            valid_in     <= 4'd0;
            is_header_in <= 4'd0;
            data_in      <= 64'd0;
            j_src <= 2'd0; j_dest <= 2'd0; j_qos <= 2'd0;
            j_len <= 4'd0; j_corrupt <= 1'b0;
            k <= 4'd0; parity_acc <= 16'd0; tx_parity <= 16'd0;
            wait_cnt <= 8'd0;
            sweep_mode <= 1'b0; sweep_idx <= 5'd0; sweep_done <= 1'b1;
            pass_cnt <= 16'd0; fail_cnt <= 16'd0; pkt_cnt <= 16'd0;
            last_valid <= 1'b0; last_pass <= 1'b0;
        end else begin
            case (state)

            S_IDLE: begin
                valid_in     <= 4'd0;
                is_header_in <= 4'd0;
                data_in      <= 64'd0;

                if (vio_sweep && !sweep_d) begin
                    sweep_mode <= 1'b1;
                    sweep_idx  <= 5'd0;
                    sweep_done <= 1'b0;
                    pass_cnt   <= 16'd0;
                    fail_cnt   <= 16'd0;
                    pkt_cnt    <= 16'd0;
                    state      <= S_NEXT;
                end else if (vio_send && !send_d) begin
                    sweep_mode <= 1'b0;
                    sweep_done <= 1'b0;
                    j_src      <= vio_src;
                    j_dest     <= vio_dest;
                    j_qos      <= vio_qos;
                    j_len      <= vio_len;
                    j_corrupt  <= vio_corrupt;
                    state      <= S_HDR;
                end
            end

            S_NEXT: begin
                if (sweep_idx[4]) begin
                    sweep_done <= 1'b1;
                    sweep_mode <= 1'b0;
                    state      <= S_IDLE;
                end else if (sweep_idx[3:2] == sweep_idx[1:0]) begin
                    sweep_idx <= sweep_idx + 5'd1;
                end else begin
                    j_src     <= sweep_idx[3:2];
                    j_dest    <= sweep_idx[1:0];
                    j_qos     <= 2'd1;
                    j_len     <= 4'd3;
                    j_corrupt <= 1'b0;
                    sweep_idx <= sweep_idx + 5'd1;
                    state     <= S_HDR;
                end
            end

            S_HDR: begin
                valid_in     <= 4'b0001 << j_src;
                is_header_in <= 4'b0001 << j_src;
                data_in      <= {48'd0, hdr_word} << {j_src, 4'b0000};
                parity_acc   <= hdr_word;
                k            <= 4'd0;
                state        <= (j_len == 4'd0) ? S_PAR : S_PAY;
            end

            S_PAY: begin
                is_header_in <= 4'd0;
                data_in      <= {48'd0, mk_payload(j_src, j_dest, k)} << {j_src, 4'b0000};
                parity_acc   <= parity_acc ^ mk_payload(j_src, j_dest, k);
                k            <= k + 4'd1;
                if (k == j_len - 4'd1) state <= S_PAR;
            end

            S_PAR: begin
                is_header_in <= 4'd0;
                tx_parity    <= j_corrupt ? ~parity_acc : parity_acc;
                data_in      <= {48'd0, (j_corrupt ? ~parity_acc : parity_acc)} << {j_src, 4'b0000};
                state        <= S_END;
            end

            S_END: begin
                valid_in     <= 4'd0;
                is_header_in <= 4'd0;
                data_in      <= 64'd0;
                wait_cnt     <= 8'd0;
                state        <= S_WAIT;
            end

            S_WAIT: begin
                wait_cnt <= wait_cnt + 8'd1;
                if (wait_cnt == 8'd80) state <= S_CHK;
            end

            S_CHK: begin
                last_valid <= 1'b1;
                last_pass  <= chk_ok;
                pkt_cnt    <= pkt_cnt + 16'd1;
                if (chk_ok) pass_cnt <= pass_cnt + 16'd1;
                else        fail_cnt <= fail_cnt + 16'd1;
                state      <= sweep_mode ? S_NEXT : S_IDLE;
            end

            default: state <= S_IDLE;
            endcase
        end
    end

    //-----------------------------------------------------------------
    // 7. Output Monitor & Checker
    //-----------------------------------------------------------------
    reg [3:0] data_valid;
    integer   p;

    always @(posedge clk) begin
        data_valid <= out_valid & read_enb_out;

        if (!resetn_r || state == S_HDR) begin
            for (p = 0; p < 4; p = p + 1) out_cnt[p] <= 8'd0;
            mismatch     <= 1'b0;
            last_rx_data <= 16'd0;
            last_rx_port <= 2'd0;
        end else begin
            for (p = 0; p < 4; p = p + 1) begin
                if (data_valid[p]) begin
                    last_rx_data <= data_out[p*16 +: 16];
                    last_rx_port <= p[1:0];

                    if (out_cnt[p] != 8'hFF) out_cnt[p] <= out_cnt[p] + 8'd1;

                    if (j_blocked || (p != j_dest)) begin
                        mismatch <= 1'b1;
                    end else if (out_cnt[p] < {4'd0, j_len}) begin
                        if (data_out[p*16 +: 16] != mk_payload(j_src, j_dest, out_cnt[p][3:0]))
                            mismatch <= 1'b1;
                    end else if (out_cnt[p] == {4'd0, j_len}) begin
                        if (data_out[p*16 +: 16] != tx_parity)
                            mismatch <= 1'b1;
                    end else begin
                        mismatch <= 1'b1;
                    end
                end
            end
        end
    end

    //-----------------------------------------------------------------
    // 8. Status Register
    //-----------------------------------------------------------------
    assign status = {sweep_done,
                     (state != S_IDLE),
                     last_valid & last_pass,
                     last_valid & ~last_pass,
                     parity_error,
                     acl_drop_flag,
                     mismatch,
                     busy};

    //-----------------------------------------------------------------
    // 9. Hardware LED Drive Logic (LD0 - LD7) STICKY FIX
    //-----------------------------------------------------------------
    reg [25:0] hb;
    always @(posedge clk) hb <= hb + 26'd1;

    // LED[0] : Heartbeat (~0.75 Hz) - Blinks to prove the FPGA clock is running
    assign led[0] = hb[25];

    // Latch the short pulses so they remain visible to the human eye on the board
    reg [7:1] sticky_leds;

    always @(posedge clk) begin
        // Clear the LEDs when the board resets, or when you trigger a new packet from VIO
        if (!resetn_r || vio_rst || (vio_send && !send_d) || (vio_sweep && !sweep_d)) begin
            sticky_leds <= 7'b0000000;
        end else begin
            // Turn ON if there is valid traffic on a specific port
            if (valid_in[0] | out_valid[0]) sticky_leds[1] <= 1'b1; // Port 0
            if (valid_in[1] | out_valid[1]) sticky_leds[2] <= 1'b1; // Port 1
            if (valid_in[2] | out_valid[2]) sticky_leds[3] <= 1'b1; // Port 2
            if (valid_in[3] | out_valid[3]) sticky_leds[4] <= 1'b1; // Port 3
            
            // Turn ON if a specific error or success event happens
            if (acl_drop_flag)          sticky_leds[5] <= 1'b1; // ACL Drop Event
            if (parity_error)           sticky_leds[6] <= 1'b1; // Parity Error 
            if (last_valid & last_pass) sticky_leds[7] <= 1'b1; // Transaction PASS
        end
    end

    // Assign the latched values directly to the physical ZedBoard pins
    assign led[7:1] = sticky_leds;

    //-----------------------------------------------------------------
    // 10. Vivado Hardware Debug IPs
    //-----------------------------------------------------------------
    vio_0 u_vio (
        .clk       (clk),
        .probe_in0 (pass_cnt),
        .probe_in1 (fail_cnt),
        .probe_in2 (pkt_cnt),
        .probe_in3 (out_cnt_flat),
        .probe_in4 (status),
        .probe_out0(vio_rst),
        .probe_out1(vio_src),
        .probe_out2(vio_dest),
        .probe_out3(vio_qos),
        .probe_out4(vio_len),
        .probe_out5(vio_corrupt),
        .probe_out6(vio_send),
        .probe_out7(vio_sweep)
    );

    ila_0 u_ila (
        .clk    (clk),
        .probe0 (valid_in),        // [3:0]
        .probe1 (is_header_in),    // [3:0]
        .probe2 (data_in),         // [63:0]
        .probe3 (out_valid),       // [3:0]
        .probe4 (data_out),        // [63:0]
        .probe5 (status),          // [7:0]
        .probe6 (state)            // [2:0]
    );

endmodule