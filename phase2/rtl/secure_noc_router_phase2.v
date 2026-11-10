// secure_noc_router_phase2.v
//
// PHASE II : Pipelined Secure NoC Router (4-port)
//
// Per-port pipeline (4 identical lanes, each with its own parser + RCU):
//
//   S0 Input FIFO -> S1 Packet Parser -> S2 Routing Computation Unit
//        (ACL + parity)  ->  S3 4x4 Crossbar (per-output QoS arbiter,
//        packet-level lock)  ->  S4 Output FIFO
//
// Differences vs Phase I:
//   * No central arbiter / shared parser. Parser + RCU are replicated per
//     input port, so all 4 ports can be processed in parallel.
//   * Arbitration moves to the crossbar outputs (one arbiter per output,
//     QoS priority High > Med > Low, round-robin tie-break). Only
//     real output contention stalls a lane.
//   * Registered stages with valid/ready backpressure between all stages.
//   * Wormhole style: once an output accepts the first flit of a packet it
//     is locked to that input until the parity word passes, so packets
//     never interleave.
//
// Interface is identical to Phase I (module secure_noc_router_top), so the
// existing wrapper (zedboard / ZCU104 top) and testbench ports still match.
// Do NOT add secure_noc_router_top.v (Phase I) to the same project.
//
// Config : DW=16, NPORTS=4, DEPTH=8, AW=3
// Header : {8'h00, payload_len[3:0], dest[1:0], qos[1:0]}
// Packet : HEADER -> PAYLOAD[0..len-1] -> PARITY (running XOR, incl. header)
// Output : PAYLOAD[0..len-1] -> PARITY   (header is consumed, as in Phase I)

`timescale 1ns/1ps

`define DEPTH   8
`define AW      3
`define NPORTS  4
`define DW      16

//====================================================================
// FIFO (same as Phase I: registered read, 1-cycle read latency)
//====================================================================
module router_fifo(
    input  wire             clk,
    input  wire             resetn,
    input  wire             soft_reset,
    input  wire             write_enb,
    input  wire             read_enb,
    input  wire             is_header,
    input  wire [`DW-1:0]   datain,
    output reg              full,
    output reg              empty,
    output reg  [`DW-1:0]   dataout,
    output reg              dataout_hdr
);
    reg [`DW:0]   fifo [0:`DEPTH-1];
    reg [`AW-1:0] wptr, rptr;
    reg [`AW:0]   count;
    integer i;

    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            for (i = 0; i < `DEPTH; i = i + 1) fifo[i] <= 0;
            wptr <= 0;
        end else if (soft_reset) begin
            for (i = 0; i < `DEPTH; i = i + 1) fifo[i] <= 0;
            wptr <= 0;
        end else if (write_enb && !full) begin
            fifo[wptr] <= {is_header, datain};
            wptr <= wptr + 1'b1;
        end
    end

    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            rptr <= 0; dataout <= 0; dataout_hdr <= 0;
        end else if (soft_reset) begin
            rptr <= 0; dataout <= 0; dataout_hdr <= 0;
        end else if (read_enb && !empty) begin
            {dataout_hdr, dataout} <= fifo[rptr];
            rptr <= rptr + 1'b1;
        end
    end

    always @(posedge clk or negedge resetn) begin
        if (!resetn) count <= 0;
        else if (soft_reset) count <= 0;
        else case ({(write_enb && !full), (read_enb && !empty)})
            2'b10:   count <= count + 1'b1;
            2'b01:   count <= count - 1'b1;
            default: count <= count;
        endcase
    end

    always @(*) begin
        empty = (count == 0);
        full  = (count == `DEPTH);
    end
endmodule

//====================================================================
// OUTPUT BUFFER (S4)
//====================================================================
module output_buffer(
    input  wire            clk,
    input  wire            resetn,
    input  wire            write_enb,
    input  wire [`DW-1:0]  datain,
    input  wire            read_enb,
    output wire            full,
    output wire            empty,
    output wire [`DW-1:0]  dataout
);
    router_fifo u_fifo(
        .clk(clk), .resetn(resetn), .soft_reset(1'b0),
        .write_enb(write_enb), .read_enb(read_enb),
        .is_header(1'b0), .datain(datain),
        .full(full), .empty(empty),
        .dataout(dataout), .dataout_hdr()
    );
endmodule

//====================================================================
// PIPELINE LANE : S0 Input FIFO -> S1 Packet Parser -> S2 RCU
// One instance per input port.
//
// Output flit (registered, S2 output register):
//   r_valid, r_data, r_dest, r_qos, r_first, r_last
// r_take (from crossbar) pops the flit.
//====================================================================
module pipe_lane #(
    parameter [1:0] SRC = 2'd0
)(
    input  wire             clk,
    input  wire             resetn,

    // input interface
    input  wire             valid_in,
    input  wire             is_header_in,
    input  wire [`DW-1:0]   data_in,
    output wire             fifo_full,

    // flit to crossbar
    output reg              r_valid,
    output reg  [`DW-1:0]   r_data,
    output reg  [1:0]       r_dest,
    output reg  [1:0]       r_qos,
    output reg              r_first,
    output reg              r_last,
    input  wire             r_take,

    // status events (consumed by the top-level flag registers)
    output wire             hdr_evt,     // a header was processed this cycle
    output wire             hdr_acl,     // ... and the ACL blocks it
    output wire             par_evt,     // a parity word was checked this cycle
    output wire             par_err      // ... and it mismatched
);

    //----------------------------------------------------------------
    // S0 : Input FIFO + output holding stage (s0_valid)
    //----------------------------------------------------------------
    wire             f_empty;
    wire [`DW-1:0]   f_data;
    wire             f_hdr;
    wire             s0_consume;
    reg              s0_valid;
    wire             rd_en = !f_empty && (!s0_valid || s0_consume);

    router_fifo u_fifo(
        .clk(clk), .resetn(resetn), .soft_reset(1'b0),
        .write_enb(valid_in), .read_enb(rd_en),
        .is_header(is_header_in), .datain(data_in),
        .full(fifo_full), .empty(f_empty),
        .dataout(f_data), .dataout_hdr(f_hdr)
    );

    always @(posedge clk or negedge resetn) begin
        if (!resetn)         s0_valid <= 1'b0;
        else if (rd_en)      s0_valid <= 1'b1;
        else if (s0_consume) s0_valid <= 1'b0;
    end

    //----------------------------------------------------------------
    // S1 : Packet Parser  (classifies each flit as HDR / PAY / PAR)
    //----------------------------------------------------------------
    localparam [1:0] T_HDR = 2'd0, T_PAY = 2'd1, T_PAR = 2'd2;

    reg              p_valid;
    reg  [1:0]       p_type;
    reg  [`DW-1:0]   p_data;
    reg              in_pkt;
    reg  [3:0]       p_len, p_cnt;
    wire             p_take;                       // from S2

    assign s0_consume = s0_valid && (!p_valid || p_take);

    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            p_valid <= 1'b0; p_type <= T_HDR; p_data <= 0;
            in_pkt  <= 1'b0; p_len  <= 4'd0;  p_cnt  <= 4'd0;
        end else begin
            if (p_take) p_valid <= 1'b0;

            if (s0_consume) begin
                if (f_hdr) begin
                    p_valid <= 1'b1; p_type <= T_HDR; p_data <= f_data;
                    in_pkt  <= 1'b1; p_len  <= f_data[7:4]; p_cnt <= 4'd0;
                end else if (in_pkt && (p_cnt < p_len)) begin
                    p_valid <= 1'b1; p_type <= T_PAY; p_data <= f_data;
                    p_cnt   <= p_cnt + 1'b1;
                end else if (in_pkt) begin
                    p_valid <= 1'b1; p_type <= T_PAR; p_data <= f_data;
                    in_pkt  <= 1'b0;
                end
                // else: stray flit outside a packet -> discarded
            end
        end
    end

    //----------------------------------------------------------------
    // S2 : Routing Computation Unit  (dest, ACL, running parity)
    //
    // ACL_TABLE addressed by {src_port, dest}: 1 = allowed, 0 = blocked.
    // Same table as Phase I: N(00)->W(11) blocked, E(10)->S(01) blocked.
    //----------------------------------------------------------------
    localparam [15:0] ACL_TABLE = 16'hFDF7;

    wire is_hdr      = (p_type == T_HDR);
    wire acl_allowed = ACL_TABLE[{SRC, p_data[3:2]}];

    reg [1:0]       cur_dest, cur_qos;
    reg             cur_drop;
    reg             first_pend;
    reg [`DW-1:0]   run_par;

    // header never writes the output register, so it can always advance
    assign p_take = p_valid && (is_hdr || !r_valid || r_take);

    assign hdr_evt = p_take && (p_type == T_HDR);
    assign hdr_acl = !acl_allowed;
    assign par_evt = p_take && (p_type == T_PAR);
    assign par_err = (run_par != p_data);

    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            r_valid <= 1'b0; r_data <= 0; r_dest <= 0; r_qos <= 0;
            r_first <= 1'b0; r_last <= 1'b0;
            cur_dest <= 0; cur_qos <= 0; cur_drop <= 1'b0;
            first_pend <= 1'b1; run_par <= 0;
        end else begin
            if (r_take) r_valid <= 1'b0;

            if (p_take) begin
                case (p_type)
                    T_HDR: begin
                        cur_dest    <= p_data[3:2];
                        cur_qos     <= p_data[1:0];
                        cur_drop    <= !acl_allowed;
                        run_par     <= p_data;
                        first_pend  <= 1'b1;
                    end
                    T_PAY: begin
                        run_par    <= run_par ^ p_data;
                        first_pend <= 1'b0;
                        if (!cur_drop) begin
                            r_valid <= 1'b1;  r_data <= p_data;
                            r_dest  <= cur_dest; r_qos <= cur_qos;
                            r_first <= first_pend; r_last <= 1'b0;
                        end
                    end
                    default: begin   // T_PAR : trailing parity word
                        first_pend  <= 1'b1;
                        if (!cur_drop) begin
                            r_valid <= 1'b1;  r_data <= p_data;
                            r_dest  <= cur_dest; r_qos <= cur_qos;
                            r_first <= first_pend; r_last <= 1'b1;
                        end
                    end
                endcase
            end
        end
    end
endmodule

//====================================================================
// OUTPUT ARBITER (one per crossbar output, S3)
//
// - Unlocked : among lanes requesting this output with a FIRST flit, pick
//              highest QoS; ties broken round-robin.
// - Locked   : only the owning lane may write until its LAST flit passes.
// - Writes only when the output FIFO has room (out_ready).
//====================================================================
module output_arbiter(
    input  wire                 clk,
    input  wire                 resetn,
    input  wire [`NPORTS-1:0]   req,        // lane has valid flit for this output
    input  wire [`NPORTS-1:0]   first,
    input  wire [`NPORTS-1:0]   last,
    input  wire [2*`NPORTS-1:0] qos,
    input  wire                 out_ready,
    output reg  [`NPORTS-1:0]   grant,
    output reg  [1:0]           sel_idx,
    output reg                  wr_en
);
    reg       locked;
    reg [1:0] lock_idx;
    reg [1:0] rr_ptr;
    integer p, idx, prio, best_prio, chosen;

    always @(*) begin
        grant = 0; sel_idx = 0; wr_en = 1'b0;
        best_prio = -1; chosen = -1; idx = 0; prio = 0;

        if (locked) begin
            if (req[lock_idx] && out_ready) begin
                grant[lock_idx] = 1'b1;
                sel_idx = lock_idx;
                wr_en   = 1'b1;
            end
        end else begin
            for (p = 0; p < `NPORTS; p = p + 1) begin
                idx = (rr_ptr + p) % `NPORTS;
                if (req[idx] && first[idx]) begin
                    prio = qos[idx*2 +: 2];
                    if (prio > best_prio) begin
                        best_prio = prio;
                        chosen    = idx;
                    end
                end
            end
            if (chosen != -1 && out_ready) begin
                grant[chosen] = 1'b1;
                sel_idx = chosen[1:0];
                wr_en   = 1'b1;
            end
        end
    end

    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            locked <= 1'b0; lock_idx <= 2'd0; rr_ptr <= 2'd0;
        end else if (wr_en) begin
            if (last[sel_idx]) begin
                locked <= 1'b0;
                rr_ptr <= sel_idx + 1'b1;   // rotate priority after each packet
            end else begin
                locked   <= 1'b1;
                lock_idx <= sel_idx;
            end
        end
    end
endmodule

//====================================================================
// TOP : secure_noc_router_top  (Phase II, pipelined)
//====================================================================
module secure_noc_router_top(
    input  wire clk,
    input  wire resetn,

    input  wire [`NPORTS-1:0]      valid_in,
    input  wire [`NPORTS-1:0]      is_header_in,
    input  wire [`NPORTS*`DW-1:0]  data_in,

    input  wire [`NPORTS-1:0]      read_enb_out,

    output wire [`NPORTS*`DW-1:0]  data_out,
    output wire [`NPORTS-1:0]      out_valid,

    output wire busy,
    output wire parity_error,
    output wire acl_drop_flag,
    output wire [1:0] invalid_addr_flag   // dest of the oldest-port valid flit (debug)
);

    //----------------------------------------------------------------
    // 4 input lanes : FIFO -> Parser -> RCU
    //----------------------------------------------------------------
    wire [`NPORTS-1:0]      l_valid, l_first, l_last, l_take;
    wire [`NPORTS*`DW-1:0]  l_data;
    wire [2*`NPORTS-1:0]    l_dest, l_qos;
    wire [`NPORTS-1:0]      l_full;
    wire [`NPORTS-1:0]      l_hdr_evt, l_hdr_acl, l_par_evt, l_par_err;

    genvar g, o;
    generate
        for (g = 0; g < `NPORTS; g = g + 1) begin : lanes
            pipe_lane #(.SRC(g)) u_lane (
                .clk(clk), .resetn(resetn),
                .valid_in(valid_in[g]),
                .is_header_in(is_header_in[g]),
                .data_in(data_in[g*`DW +: `DW]),
                .fifo_full(l_full[g]),
                .r_valid(l_valid[g]),
                .r_data(l_data[g*`DW +: `DW]),
                .r_dest(l_dest[g*2 +: 2]),
                .r_qos(l_qos[g*2 +: 2]),
                .r_first(l_first[g]),
                .r_last(l_last[g]),
                .r_take(l_take[g]),
                .hdr_evt(l_hdr_evt[g]),
                .hdr_acl(l_hdr_acl[g]),
                .par_evt(l_par_evt[g]),
                .par_err(l_par_err[g])
            );
        end
    endgenerate

    //----------------------------------------------------------------
    // 4x4 crossbar : per-output arbiter + output FIFO
    //----------------------------------------------------------------
    wire [`NPORTS*`NPORTS-1:0] grant_all;     // [o*4 +: 4] = grant for output o
    wire [`NPORTS*2-1:0]       sel_all;
    wire [`NPORTS-1:0]         wr_all;
    wire [`NPORTS-1:0]         ob_full, ob_empty;
    wire [`DW-1:0]             ob_dout [0:`NPORTS-1];

    generate
        for (o = 0; o < `NPORTS; o = o + 1) begin : out_ports
            // which lanes want this output
            wire [`NPORTS-1:0] req_o;
            for (g = 0; g < `NPORTS; g = g + 1) begin : rq
                assign req_o[g] = l_valid[g] && (l_dest[g*2 +: 2] == o);
            end

            output_arbiter u_arb (
                .clk(clk), .resetn(resetn),
                .req(req_o), .first(l_first), .last(l_last),
                .qos(l_qos), .out_ready(!ob_full[o]),
                .grant(grant_all[o*`NPORTS +: `NPORTS]),
                .sel_idx(sel_all[o*2 +: 2]),
                .wr_en(wr_all[o])
            );

            output_buffer u_ob (
                .clk(clk), .resetn(resetn),
                .write_enb(wr_all[o]),
                .datain(l_data[sel_all[o*2 +: 2]*`DW +: `DW]),
                .read_enb(read_enb_out[o]),
                .full(ob_full[o]), .empty(ob_empty[o]),
                .dataout(ob_dout[o])
            );

            assign data_out[o*`DW +: `DW] = ob_dout[o];
            assign out_valid[o]           = !ob_empty[o];
        end

        // a lane's flit is popped when any output granted it
        for (g = 0; g < `NPORTS; g = g + 1) begin : takes
            assign l_take[g] = grant_all[0*`NPORTS + g] | grant_all[1*`NPORTS + g]
                             | grant_all[2*`NPORTS + g] | grant_all[3*`NPORTS + g];
        end
    endgenerate

    //----------------------------------------------------------------
    // Status
    //----------------------------------------------------------------
    reg [1:0] dbg_dest;
    integer k;
    always @(*) begin
        dbg_dest = 2'b00;
        for (k = `NPORTS-1; k >= 0; k = k - 1)
            if (l_valid[k]) dbg_dest = l_dest[k*2 +: 2];
    end

    // Flags follow the most recently processed packet (same behaviour as
    // Phase I's single parser): a new header clears parity_error and
    // loads acl_drop; the parity word loads parity_error.
    // If two lanes hit in the same cycle, the lower port index wins.
    reg acl_q, par_q;
    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            acl_q <= 1'b0; par_q <= 1'b0;
        end else begin
            for (k = `NPORTS-1; k >= 0; k = k - 1) begin
                if (l_hdr_evt[k]) begin acl_q <= l_hdr_acl[k]; par_q <= 1'b0; end
                if (l_par_evt[k]) par_q <= l_par_err[k];
            end
        end
    end

    assign busy              = |l_valid | (|wr_all);
    assign parity_error      = par_q;
    assign acl_drop_flag     = acl_q;
    assign invalid_addr_flag = dbg_dest;

endmodule
