// secure_noc_router_phase1.v
//
// PHASE I DELIVERABLE - Baseline Centralized Secure NoC Router
//
// Part A : Synthesizable RTL
//          Input Interface -> Central QoS Arbiter -> Packet Parser +
//          Routing Computation + ACL Module (2-rule table) -> 4x4
//          Crossbar -> Output Buffer, with Parity Generator/Checker

//
// Config : DW=16, NPORTS=4, DEPTH=8, AW=3
// Header : {8'h00, payload_len[3:0], dest[1:0], qos[1:0]}
// Packet : HEADER -> PAYLOAD[0..len-1] -> PARITY (running XOR)


`timescale 1ns/1ps

// secure_noc_router_top.v
// PHASE I : Baseline centralized Secure NoC Router (non-pipelined)
// 4-port router with QoS arbitration, ACL filtering, parity checking
//
// Team: always @(last_min) / Mridula, Harini, Venkatesh

`define DEPTH   8
`define AW      3
`define NPORTS  4
`define DW      16

// FIFO  (per-port input / output buffer)

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

    reg [`DW:0] fifo [0:`DEPTH-1];
    reg [`AW-1:0] wptr, rptr;
    reg [`AW:0] count;
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
            2'b10: count <= count + 1'b1;
            2'b01: count <= count - 1'b1;
            default: count <= count;
        endcase
    end

    always @(*) begin
        empty = (count == 0);
        full  = (count == `DEPTH);
    end

endmodule



// INPUT INTERFACE  (FIFO wrapper + QoS extraction + request generation)

module input_port(
    input  wire             clk,
    input  wire             resetn,
    input  wire             valid_in,
    input  wire [`DW-1:0]   data_in,
    input  wire             is_header_in,
    input  wire             read_enb,
    output wire             full,
    output wire             empty,
    output wire [`DW-1:0]   dataout,
    output wire             dataout_hdr,
    output reg  [1:0]       qos_level,
    output wire             req
);

    router_fifo u_fifo(
        .clk(clk), .resetn(resetn), .soft_reset(1'b0),
        .write_enb(valid_in), .read_enb(read_enb),
        .is_header(is_header_in), .datain(data_in),
        .full(full), .empty(empty),
        .dataout(dataout), .dataout_hdr(dataout_hdr)
    );

    assign req = !empty;

    // QoS field = header bits [1:0]
    always @(posedge clk or negedge resetn) begin
        if (!resetn) qos_level <= 2'b00;
        else if (is_header_in && valid_in) qos_level <= data_in[1:0];
    end

endmodule

// CENTRAL / QoS PRIORITY ARBITER
// Priority: High(2'b10) > Medium(2'b01) > Low(2'b00)
// Round-robin tiebreak among equal priority requests

module qos_arbiter(
    input  wire clk,
    input  wire resetn,
    input  wire [`NPORTS-1:0]   req,
    input  wire [2*`NPORTS-1:0] qos,
    output reg  [`NPORTS-1:0]   grant,
    output reg  [1:0]           grant_idx,
    output reg                  grant_valid
);

    reg [1:0] rr_ptr;
    integer p, best_prio, chosen;

    always @(posedge clk or negedge resetn) begin
        if (!resetn) rr_ptr <= 0;
        else if (grant_valid) rr_ptr <= grant_idx + 1'b1;
    end

    always @(*) begin
        grant = 0; grant_valid = 0; grant_idx = 0;
        best_prio = -1; chosen = -1;

        for (p = 0; p < `NPORTS; p = p + 1) begin : arb_loop
            integer idx, prio;
            idx = (rr_ptr + p) % `NPORTS;
            if (req[idx]) begin
                prio = qos[idx*2 +: 2];
                if (prio > best_prio) begin
                    best_prio = prio;
                    chosen = idx;
                end
            end
        end

        if (chosen != -1) begin
            grant[chosen] = 1'b1;
            grant_idx = chosen[1:0];
            grant_valid = 1'b1;
        end
    end

endmodule

// PACKET PARSER + ROUTING COMPUTATION + ACL MODULE + PARITY GENERATOR
//
// Header flit : {8'h00, payload_len[3:0], dest[1:0], qos[1:0]}
// Packet      : HEADER -> PAYLOAD[0 .. len-1] -> PARITY (running XOR)
//
// ACL_TABLE   : 16-entry lookup, addressed by {src_port, dest_port}.
//               1 = allowed, 0 = blocked. Default = all allowed,
//               except N(0) -> W(3), which the security policy blocks.
//               Synthesizable as a flat bit-vector (avoids the
//               variable-indexed 2D-array issue in Vivado's synth).

module parser_acl_route(
    input  wire             clk,
    input  wire             resetn,
    input  wire             grant_valid,
    input  wire [1:0]       src_port,
    input  wire             is_header,
    input  wire [`DW-1:0]   data,

    output reg  [1:0]       dest_port,
    output reg              acl_drop,
    output reg              parity_err,
    output reg  [`DW-1:0]   parity_calc,
    output reg              route_valid,
    output reg  [`DW-1:0]   data_out
);

    // ACL Module 
    // index = {src_port, hdr_dest} : 4-bit address (0..15), 16 entries
    // bit[idx] = 1 -> allowed, 0 -> blocked.
    // Security policy (2 rules): N(00)->W(11) blocked, E(10)->S(01) blocked
    localparam [15:0] ACL_TABLE = 16'hFDF7;  // bit3=0 (N->W), bit9=0 (E->S)

    reg [1:0] hdr_dest;
    wire      acl_allowed;

    always @(*) hdr_dest = data[3:2];

    assign acl_allowed = ACL_TABLE[{src_port, hdr_dest}];

    //  Parser + Parity 
    reg [`DW-1:0] running_parity;
    reg [3:0] payload_len;
    reg [3:0] payload_count;

    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            dest_port      <= 2'b00;
            acl_drop       <= 1'b0;
            parity_err     <= 1'b0;
            running_parity <= 0;
            payload_len    <= 0;
            payload_count  <= 0;
            route_valid    <= 1'b0;
            data_out       <= 0;
        end else begin
            route_valid <= 1'b0;

            if (grant_valid) begin
                if (is_header) begin
                    //  Routing Computation 
                    dest_port      <= data[3:2];
                    acl_drop       <= !acl_allowed;
                    payload_len    <= data[7:4];
                    payload_count  <= 0;
                    running_parity <= data;
                    parity_err     <= 1'b0;
                    route_valid    <= 1'b0;
                end else if (payload_count < payload_len) begin
                    running_parity <= running_parity ^ data;
                    payload_count  <= payload_count + 1'b1;
                    route_valid    <= 1'b1;
                    data_out       <= data;
                end else begin
                    // trailing parity word
                    parity_err    <= (running_parity != data);
                    route_valid   <= 1'b1;
                    data_out      <= data;
                    payload_count <= 0;
                end
            end
        end
    end

    always @(*) parity_calc = running_parity;

endmodule

// 4x4 CROSSBAR SWITCH

module crossbar(
    input  wire [1:0]      dest_port,
    input  wire            grant_valid,
    input  wire            acl_drop,
    input  wire [`DW-1:0]  data_in,

    output reg  [`DW-1:0]  out_n,
    output reg  [`DW-1:0]  out_s,
    output reg  [`DW-1:0]  out_e,
    output reg  [`DW-1:0]  out_w,
    output reg  [3:0]      out_valid
);
    always @(*) begin
        out_n = 0; out_s = 0; out_e = 0; out_w = 0;
        out_valid = 4'b0000;
        if (grant_valid && !acl_drop) begin
            case (dest_port)
                2'd0: begin out_n = data_in; out_valid[0] = 1'b1; end
                2'd1: begin out_s = data_in; out_valid[1] = 1'b1; end
                2'd2: begin out_e = data_in; out_valid[2] = 1'b1; end
                2'd3: begin out_w = data_in; out_valid[3] = 1'b1; end
            endcase
        end
    end
endmodule

// OUTPUT BUFFER

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

// TOP MODULE : secure_noc_router_top
// Input Interface -> Central QoS Arbiter -> Packet Parser/ACL/Route
//   -> 4x4 Crossbar -> Output Buffer
// (single-cycle-per-stage, centralized, NOT pipelined -> Phase I)

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
    output wire [1:0] invalid_addr_flag   // computed dest port (monitor/debug)
);

    wire [`NPORTS-1:0] fifo_full, fifo_empty;
    wire [`DW-1:0] fifo_dout [0:`NPORTS-1];
    wire fifo_dout_hdr [0:`NPORTS-1];
    wire [1:0] qos [0:`NPORTS-1];
    wire [`NPORTS-1:0] req;
    wire [`NPORTS-1:0] read_enb_in;

    genvar g;
    generate
        for (g = 0; g < `NPORTS; g = g + 1) begin : in_ports
            input_port u_ip(
                .clk(clk), .resetn(resetn),
                .valid_in(valid_in[g]),
                .data_in(data_in[g*`DW +: `DW]),
                .is_header_in(is_header_in[g]),
                .read_enb(read_enb_in[g]),
                .full(fifo_full[g]), .empty(fifo_empty[g]),
                .dataout(fifo_dout[g]), .dataout_hdr(fifo_dout_hdr[g]),
                .qos_level(qos[g]), .req(req[g])
            );
        end
    endgenerate

    wire [`NPORTS-1:0] grant;
    wire [1:0] grant_idx;
    wire grant_valid;

    qos_arbiter u_arb(
        .clk(clk), .resetn(resetn),
        .req(req), .qos({qos[3], qos[2], qos[1], qos[0]}),
        .grant(grant), .grant_idx(grant_idx), .grant_valid(grant_valid)
    );

    reg grant_valid_d;
    reg [1:0] grant_idx_d;
    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            grant_valid_d <= 1'b0; grant_idx_d <= 2'b00;
        end else begin
            grant_valid_d <= grant_valid; grant_idx_d <= grant_idx;
        end
    end

    assign read_enb_in = grant_valid ? grant : {`NPORTS{1'b0}};

    reg [`DW-1:0] sel_data;
    reg sel_hdr;
    always @(*) begin
        sel_data = fifo_dout[grant_idx_d];
        sel_hdr  = fifo_dout_hdr[grant_idx_d];
    end

    wire [1:0] dest_port;
    wire acl_drop;
    wire [`DW-1:0] calc_parity;
    wire route_valid;
    wire [`DW-1:0] parsed_data;
    wire parser_parity_error;

    parser_acl_route u_parse(
        .clk(clk), .resetn(resetn),
        .grant_valid(grant_valid_d), .src_port(grant_idx_d),
        .is_header(sel_hdr), .data(sel_data),
        .dest_port(dest_port), .acl_drop(acl_drop),
        .parity_err(parser_parity_error), .parity_calc(calc_parity),
        .route_valid(route_valid), .data_out(parsed_data)
    );

    wire [`DW-1:0] xbar_n, xbar_s, xbar_e, xbar_w;
    wire [`NPORTS-1:0] xbar_valid;

    crossbar u_xbar(
        .dest_port(dest_port),
        .grant_valid(route_valid), .acl_drop(acl_drop),
        .data_in(parsed_data),
        .out_n(xbar_n), .out_s(xbar_s), .out_e(xbar_e), .out_w(xbar_w),
        .out_valid(xbar_valid)
    );

    wire [`NPORTS-1:0] ob_full, ob_empty;
    wire [`DW-1:0] ob_dout [0:`NPORTS-1];
    wire [`DW-1:0] xbar_arr [0:`NPORTS-1];
    assign xbar_arr[0] = xbar_n;
    assign xbar_arr[1] = xbar_s;
    assign xbar_arr[2] = xbar_e;
    assign xbar_arr[3] = xbar_w;

    generate
        for (g = 0; g < `NPORTS; g = g + 1) begin : out_ports
            output_buffer u_ob(
                .clk(clk), .resetn(resetn),
                .write_enb(xbar_valid[g]), .datain(xbar_arr[g]),
                .read_enb(read_enb_out[g]),
                .full(ob_full[g]), .empty(ob_empty[g]),
                .dataout(ob_dout[g])
            );
            assign data_out[g*`DW +: `DW] = ob_dout[g];
            assign out_valid[g] = !ob_empty[g];
        end
    endgenerate

    assign busy              = grant_valid;
    assign parity_error      = parser_parity_error;
    assign acl_drop_flag     = acl_drop;
    assign invalid_addr_flag = dest_port;

endmodule
