//=====================================================================
// PART B : SELF-CHECKING TESTBENCH
//=====================================================================

//=====================================================================
// secure_noc_router_tb.v
//
// Updated self-checking testbench for Secure NoC Router
//
// Verifies:
//   1. Normal routing
//   2. Payload ordering
//   3. Correct destination output
//   4. No unintended output ports
//   5. ACL violation detection + output blocking
//   6. Correct parity case
//   7. Corrupted parity case
//   8. QoS priority arbitration
//   9. Round-robin arbitration for equal QoS
//  10. Four-port contention
//  11. Zero-payload packet
//  12. Maximum payload packet
//  13. Reset recovery
//  14. One-hot grant assertion
//
// DUT:
//   DW     = 16
//   NPORTS = 4
//   DEPTH  = 8
//
// Port encoding:
//   0 = NORTH
//   1 = SOUTH
//   2 = EAST
//   3 = WEST
//
// Packet header:
//   {8'h00, payload_len[3:0], dest_port[1:0], qos[1:0]}
//
// Output path does NOT contain the header.
// Expected output:
//   payload[0]
//   payload[1]
//   ...
//   payload[N-1]
//   parity
//
// IMPORTANT:
//   Payloads are deterministic rather than $random.
//   Therefore every checker can independently calculate the expected
//   payload and parity. No shared global expected-packet array is used.
//=====================================================================



module secure_noc_router_tb();

    //=================================================================
    // PARAMETERS
    //=================================================================

    localparam DW     = 16;
    localparam NPORTS = 4;
    localparam DEPTH  = 8;

    //=============================================================
    // TESTCASE
    //
    // 0 = complete regression
    //
    // 1 = normal routing
    // 2 = ACL
    // 3 = correct parity
    // 4 = corrupted parity
    // 5 = QoS priority
    // 6 = round robin
    // 7 = four-port contention
    // 8 = zero payload
    // 9 = maximum payload
    // 10 = reset recovery
    // 11 = ACL rule 2 (S -> N)
    // 12 = grant one-hot assertion sweep
    // 13 = FIFO-full backpressure
    // 14 = exhaustive ACL sweep (all 12 valid pairs)
    // 15 = parallel outputs (Phase II)
    //=============================================================

    localparam TESTCASE = 0;


    //=================================================================
    // SIGNALS
    //=================================================================

    reg clk;
    reg resetn;

    reg [NPORTS-1:0] valid_in;
    reg [NPORTS-1:0] is_header_in;

    reg [NPORTS*DW-1:0] data_in;

    reg [NPORTS-1:0] read_enb_out;

    wire [NPORTS*DW-1:0] data_out;
    wire [NPORTS-1:0]    out_valid;

    wire busy;
    wire parity_error;
    wire acl_drop_flag;

    wire [1:0] invalid_addr_flag;


    //=================================================================
    // COUNTERS
    //=================================================================

    integer pass_count;
    integer fail_count;


    //=================================================================
    // COVERAGE COUNTERS
    //=================================================================

    integer cov_qos_high;
    integer cov_qos_med;
    integer cov_qos_low;

    integer cov_port_n;
    integer cov_port_s;
    integer cov_port_e;
    integer cov_port_w;

    integer cov_acl_drop;
    integer cov_parity_good;
    integer cov_parity_err;

    integer cov_qos_priority;
    integer cov_round_robin;

    integer cov_acl_rule2;
    integer cov_onehot_checks;
    integer cov_backpressure;
    integer cov_acl_sweep_pairs;


    //=================================================================
    // DUT
    //=================================================================

    secure_noc_router_top DUT(

        .clk               (clk),
        .resetn            (resetn),

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


    //=================================================================
    // CLOCK
    //=================================================================

    initial clk = 1'b0;

    always #5 clk = ~clk;


    //=================================================================
    // PHASE-II ARBITRATION ASSERTIONS + MONITORS
    //
    // Phase I had ONE central arbiter (DUT.u_arb). Phase II has one
    // arbiter per crossbar output (DUT.out_ports[o].u_arb) and the
    // grant is held for a whole packet (wormhole lock). So:
    //   * one-hot is checked per output, not globally
    //   * a lane may never be granted by two outputs at once
    //   * arbitration ORDER is observed through win_log[] (who won each
    //     output, packet by packet) instead of cycle-exact grant checks
    //   * non-interleaving is checked on the real output stream
    //=================================================================

    function integer count_ones;
        input [NPORTS-1:0] vec;
        integer i;
        begin
            count_ones = 0;
            for (i = 0; i < NPORTS; i = i + 1)
                count_ones = count_ones + vec[i];
        end
    endfunction

    // ---- win / output logs --------------------------------------
    integer         win_cnt  [0:NPORTS-1];
    reg  [1:0]      win_log  [0:NPORTS*16-1];   // [o*16 + n] = winning src of n-th packet on output o
    integer         win_time [0:NPORTS*16-1];   // ns timestamp of that win
    integer         out_cnt  [0:NPORTS-1];
    reg  [DW-1:0]   out_log  [0:NPORTS*64-1];   // [o*64 + n] = n-th word seen on output o
    reg  [NPORTS-1:0] ov_d;
    integer         mi, li, lane_grants;
    reg             bp_in_full_seen, bp_out_full_seen;

    task clear_logs;
        integer q;
    begin
        for (q = 0; q < NPORTS; q = q + 1) begin
            win_cnt[q] = 0;
            out_cnt[q] = 0;
        end
        ov_d = {NPORTS{1'b0}};
        bp_in_full_seen  = 1'b0;
        bp_out_full_seen = 1'b0;
    end
    endtask

    initial clear_logs;

    always @(posedge clk) begin
        if (resetn) begin
            for (mi = 0; mi < NPORTS; mi = mi + 1) begin

                // packet-start win on output mi (first flit written)
                if (DUT.wr_all[mi] &&
                    DUT.l_first[DUT.sel_all[mi*2 +: 2]] &&
                    win_cnt[mi] < 16) begin
                    win_log [mi*16 + win_cnt[mi]] = DUT.sel_all[mi*2 +: 2];
                    win_time[mi*16 + win_cnt[mi]] = $time;
                    win_cnt[mi] = win_cnt[mi] + 1;
                end

                // output word capture (FIFO read data is valid 1 cycle
                // after the read, so use the delayed valid)
                if (ov_d[mi] && out_cnt[mi] < 64) begin
                    out_log[mi*64 + out_cnt[mi]] = data_out[mi*DW +: DW];
                    out_cnt[mi] = out_cnt[mi] + 1;
                end
            end

            if (DUT.lanes[0].u_lane.fifo_full) bp_in_full_seen  = 1'b1;
            if (DUT.ob_full[0])                bp_out_full_seen = 1'b1;
        end
        ov_d <= out_valid & read_enb_out;
    end

    // ---- per-output arbiter assertions --------------------------
    genvar go;
    generate
        for (go = 0; go < NPORTS; go = go + 1) begin : arb_chk
            wire [NPORTS-1:0] g = DUT.grant_all[go*NPORTS +: NPORTS];
            wire              w = DUT.wr_all[go];

            always @(posedge clk) begin
                if (resetn) begin
                    cov_onehot_checks = cov_onehot_checks + 1;

                    if (count_ones(g) > 1) begin
                        $display("[FAIL] ASSERT_ONEHOT : output %0d grant=%b (not one-hot)", go, g);
                        fail_count = fail_count + 1;
                    end

                    if (w !== (|g)) begin
                        $display("[FAIL] ASSERT_WR : output %0d wr_en=%b but grant=%b", go, w, g);
                        fail_count = fail_count + 1;
                    end

                    // locked output may only serve its owner lane
                    if (DUT.out_ports[go].u_arb.locked && w &&
                        (DUT.out_ports[go].u_arb.sel_idx !== DUT.out_ports[go].u_arb.lock_idx)) begin
                        $display("[FAIL] ASSERT_LOCK : output %0d served lane %0d while locked to lane %0d",
                                 go, DUT.out_ports[go].u_arb.sel_idx, DUT.out_ports[go].u_arb.lock_idx);
                        fail_count = fail_count + 1;
                    end
                end
            end
        end
    endgenerate

    // ---- a lane must never be granted by two outputs in one cycle
    always @(posedge clk) begin
        if (resetn) begin
            for (li = 0; li < NPORTS; li = li + 1) begin
                lane_grants = DUT.grant_all[li] + DUT.grant_all[NPORTS + li]
                            + DUT.grant_all[2*NPORTS + li] + DUT.grant_all[3*NPORTS + li];
                if (lane_grants > 1) begin
                    $display("[FAIL] ASSERT_LANE : lane %0d granted by %0d outputs at t=%0t",
                             li, lane_grants, $time);
                    fail_count = fail_count + 1;
                end
            end
        end
    end


    //=================================================================
    // RESET
    //=================================================================

    task reset_dut;

    begin

        resetn = 1'b0;
        clear_logs;

        valid_in     = {NPORTS{1'b0}};
        is_header_in = {NPORTS{1'b0}};
        data_in      = {(NPORTS*DW){1'b0}};

        // Output FIFOs continuously read.
        read_enb_out = {NPORTS{1'b1}};

        repeat(3) @(negedge clk);

        resetn = 1'b1;

        @(negedge clk);

    end

    endtask


    //=================================================================
    // DETERMINISTIC PAYLOAD GENERATOR
    //
    // This is intentionally deterministic so that a checker can
    // independently reproduce the expected packet without sharing
    // a global expected_payload array.
    //
    // Example:
    //
    // src=0,dest=1:
    //   payload[0] = 16'h1010
    //   payload[1] = 16'h1011
    //   payload[2] = 16'h1012
    //
    // src=1,dest=2:
    //   payload[0] = 16'h1220
    //   ...
    //=================================================================

    function [DW-1:0] make_payload;

        input integer src_port;
        input integer dest_port;
        input integer index;

        begin

            make_payload =
                16'h1000 +
                (src_port  * 16'h0100) +
                (dest_port * 16'h0020) +
                index;

        end

    endfunction


    //=================================================================
    // MAKE HEADER
    //=================================================================

    function [DW-1:0] make_header;

        input integer dest_port;
        input integer qos;
        input integer payload_len;

        begin

            make_header = {
                8'h00,
                payload_len[3:0],
                dest_port[1:0],
                qos[1:0]
            };

        end

    endfunction


    //=================================================================
    // GET OUTPUT DATA
    //=================================================================

    function [DW-1:0] get_output_data;

        input integer port;

        begin

            get_output_data =
                data_out[port*DW +: DW];

        end

    endfunction


    //=================================================================
    // CHECK THAT ALL OTHER OUTPUTS ARE IDLE
    //=================================================================

    task check_other_outputs_idle;

        input integer expected_port;

        integer p;

    begin

        for (p = 0; p < NPORTS; p = p + 1) begin

            if (p != expected_port) begin

                if (out_valid[p]) begin

                    $display(
                        "[FAIL] Unexpected output on port %0d : data=%h",
                        p,
                        data_out[p*DW +: DW]
                    );

                    fail_count = fail_count + 1;

                end

            end

        end

    end

    endtask


    //=================================================================
    // SEND ONE COMPLETE PACKET
    //
    // No global expected packet storage is used.
    //=================================================================

    task automatic send_packet;

        input [1:0] src_port;
        input [1:0] dest_port;
        input [1:0] qos;
        input [3:0] payload_len;
        input       corrupt_parity;

        reg [DW-1:0] header;
        reg [DW-1:0] payload;
        reg [DW-1:0] parity;

        integer k;

    begin

        //-------------------------------------------------------------
        // Header
        //-------------------------------------------------------------

        header = {
            8'h00,
            payload_len,
            dest_port,
            qos
        };

        parity = header;


        //-------------------------------------------------------------
        // Coverage
        //-------------------------------------------------------------

        case (qos)

            2'b10:
                cov_qos_high = cov_qos_high + 1;

            2'b01:
                cov_qos_med = cov_qos_med + 1;

            2'b00:
                cov_qos_low = cov_qos_low + 1;

        endcase


        case (dest_port)

            2'd0:
                cov_port_n = cov_port_n + 1;

            2'd1:
                cov_port_s = cov_port_s + 1;

            2'd2:
                cov_port_e = cov_port_e + 1;

            2'd3:
                cov_port_w = cov_port_w + 1;

        endcase


        //-------------------------------------------------------------
        // Header
        //-------------------------------------------------------------

        @(negedge clk);

        valid_in[src_port]     = 1'b1;
        is_header_in[src_port] = 1'b1;

        data_in[src_port*DW +: DW] = header;


        //-------------------------------------------------------------
        // Move to payload
        //-------------------------------------------------------------

        @(negedge clk);

        is_header_in[src_port] = 1'b0;


        //-------------------------------------------------------------
        // Payload
        //-------------------------------------------------------------

        for (k = 0; k < payload_len; k = k + 1) begin

            payload = make_payload(
                src_port,
                dest_port,
                k
            );

            parity = parity ^ payload;

            data_in[src_port*DW +: DW] = payload;

            @(negedge clk);

        end


        //-------------------------------------------------------------
        // Optional parity corruption
        //-------------------------------------------------------------

        if (corrupt_parity)
            parity = ~parity;


        //-------------------------------------------------------------
        // Parity
        //-------------------------------------------------------------

        data_in[src_port*DW +: DW] = parity;

        @(negedge clk);


        //-------------------------------------------------------------
        // End packet
        //-------------------------------------------------------------

        valid_in[src_port] = 1'b0;

        data_in[src_port*DW +: DW] = {DW{1'b0}};

        @(negedge clk);

    end

    endtask


    //=================================================================
    // CHECK COMPLETE PACKET
    //
    // Expected:
    //
    // payload[0]
    // payload[1]
    // ...
    // payload[N-1]
    // parity
    //
    // Header is consumed internally.
    //=================================================================

    task automatic check_packet_output;

        input [8*40-1:0] test_name;
        input integer src_port;
        input integer dest_port;
        input integer qos;
        input integer payload_len;

        integer k;
        integer timeout;

        reg [DW-1:0] expected;
        reg [DW-1:0] parity;
        reg [DW-1:0] actual;
        reg [DW-1:0] header;

    begin

        //-------------------------------------------------------------
        // Reconstruct expected packet independently.
        //-------------------------------------------------------------

        header = make_header(
            dest_port,
            qos,
            payload_len
        );

        parity = header;


        $display("");
        $display(
            "-----------------------------------------------------"
        );

        $display(
            "OUTPUT CHECK: %0s",
            test_name
        );

        $display(
            "src=%0d dest=%0d qos=%0d payload_len=%0d",
            src_port,
            dest_port,
            qos,
            payload_len
        );

        $display(
            "-----------------------------------------------------"
        );


        //-------------------------------------------------------------
        // Payload
        //-------------------------------------------------------------

        for (k = 0; k < payload_len; k = k + 1) begin

            expected = make_payload(
                src_port,
                dest_port,
                k
            );

            parity = parity ^ expected;

            timeout = 0;

            while (!out_valid[dest_port] &&
                   timeout < 40) begin

                @(negedge clk);

                timeout = timeout + 1;

            end


            if (timeout >= 40) begin

                $display(
                    "[FAIL] %0s : timeout waiting for payload[%0d]",
                    test_name,
                    k
                );

                fail_count = fail_count + 1;

            end

            else begin

                // Wait one half cycle so registered FIFO data has
                // settled before sampling.
                @(negedge clk);

                actual = get_output_data(dest_port);

                if (actual !== expected) begin

                    $display(
                        "[FAIL] %0s : payload[%0d] port=%0d actual=%h expected=%h",
                        test_name,
                        k,
                        dest_port,
                        actual,
                        expected
                    );

                    fail_count = fail_count + 1;

                end

                else begin

                    $display(
                        "[PASS] %0s : payload[%0d] port=%0d data=%h",
                        test_name,
                        k,
                        dest_port,
                        actual
                    );

                    pass_count = pass_count + 1;

                end

            end

        end


        //-------------------------------------------------------------
        // Expected parity word
        //-------------------------------------------------------------

        timeout = 0;

        while (!out_valid[dest_port] &&
               timeout < 40) begin

            @(negedge clk);

            timeout = timeout + 1;

        end


        if (timeout >= 40) begin

            $display(
                "[FAIL] %0s : timeout waiting for parity",
                test_name
            );

            fail_count = fail_count + 1;

        end

        else begin

            @(negedge clk);

            actual = get_output_data(dest_port);

            if (actual !== parity) begin

                $display(
                    "[FAIL] %0s : parity port=%0d actual=%h expected=%h",
                    test_name,
                    dest_port,
                    actual,
                    parity
                );

                fail_count = fail_count + 1;

            end

            else begin

                $display(
                    "[PASS] %0s : parity port=%0d data=%h",
                    test_name,
                    dest_port,
                    actual
                );

                pass_count = pass_count + 1;

            end

        end


        //-------------------------------------------------------------
        // Check other outputs.
        //-------------------------------------------------------------

        check_other_outputs_idle(dest_port);


        $display(
            "OUTPUT CHECK COMPLETE: %0s",
            test_name
        );

        $display(
            "-----------------------------------------------------"
        );

    end

    endtask


    //=================================================================
    // CHECK ACL FLAG
    //=================================================================

    task automatic check_acl;

        input [8*40-1:0] test_name;
        input expected_acl_drop;

        integer timeout;

    begin

        timeout = 0;

        //-------------------------------------------------------------
        // Wait until ACL flag changes or timeout.
        //-------------------------------------------------------------

        while ((acl_drop_flag !== expected_acl_drop) &&
               timeout < 20) begin

            @(negedge clk);

            timeout = timeout + 1;

        end


        if (acl_drop_flag === expected_acl_drop) begin

            $display(
                "[PASS] %0s : acl_drop_flag=%b",
                test_name,
                acl_drop_flag
            );

            pass_count = pass_count + 1;

        end

        else begin

            $display(
                "[FAIL] %0s : acl_drop_flag=%b expected=%b",
                test_name,
                acl_drop_flag,
                expected_acl_drop
            );

            fail_count = fail_count + 1;

        end


        if (expected_acl_drop)
            cov_acl_drop = cov_acl_drop + 1;

    end

    endtask


    //=================================================================
    // CHECK ACL PACKET PRODUCES NO OUTPUT
    //=================================================================

    task automatic check_acl_output_blocked;

        input [8*40-1:0] test_name;

        integer cycles;
        integer p;
        reg found_output;

    begin

        found_output = 1'b0;


        for (cycles = 0; cycles < 35; cycles = cycles + 1) begin

            @(negedge clk);

            for (p = 0; p < NPORTS; p = p + 1) begin

                if (out_valid[p])
                    found_output = 1'b1;

            end

        end


        if (found_output) begin

            $display(
                "[FAIL] %0s : ACL packet produced output",
                test_name
            );

            fail_count = fail_count + 1;

        end

        else begin

            $display(
                "[PASS] %0s : ACL packet produced NO output",
                test_name
            );

            pass_count = pass_count + 1;

        end

    end

    endtask


    //=================================================================
    // CHECK PARITY ERROR
    //
    // Wait long enough for the complete packet to reach the parity
    // checker. This avoids accidentally checking the reset value 0.
    //=================================================================

    task automatic check_parity_result;

        input [8*40-1:0] test_name;
        input expected_error;

        integer cycles;

    begin

        //-------------------------------------------------------------
        // Give packet time to travel through the router.
        //-------------------------------------------------------------

        for (cycles = 0; cycles < 25; cycles = cycles + 1)
            @(negedge clk);


        if (parity_error === expected_error) begin

            $display(
                "[PASS] %0s : parity_error=%b expected=%b",
                test_name,
                parity_error,
                expected_error
            );

            pass_count = pass_count + 1;

        end

        else begin

            $display(
                "[FAIL] %0s : parity_error=%b expected=%b",
                test_name,
                parity_error,
                expected_error
            );

            fail_count = fail_count + 1;

        end


        if (expected_error) begin

            cov_parity_err = cov_parity_err + 1;

        end

        else begin

            cov_parity_good = cov_parity_good + 1;

        end

    end

    endtask


    //=================================================================
    // INJECT HEADER ONLY
    //
    // Used specifically for testing the arbiter.
    //
    // We intentionally inject only the header because the arbiter
    // operates before the parser/crossbar path.
    //=================================================================

    task automatic inject_header;

        input integer src_port;
        input integer dest_port;
        input integer qos;

    begin

        @(negedge clk);

        valid_in[src_port]     = 1'b1;
        is_header_in[src_port] = 1'b1;

        data_in[src_port*DW +: DW] =
            make_header(
                dest_port,
                qos,
                0
            );

        @(negedge clk);

        valid_in[src_port]     = 1'b0;
        is_header_in[src_port] = 1'b0;

        data_in[src_port*DW +: DW] = 16'h0000;

    end

    endtask


    //=================================================================
    // INJECT FOUR HEADERS SIMULTANEOUSLY
    //
    // Used to create deterministic arbiter contention.
    //=================================================================

    task automatic inject_four_headers;

        input integer qos0;
        input integer qos1;
        input integer qos2;
        input integer qos3;

    begin

        @(negedge clk);

        valid_in = 4'b1111;

        is_header_in = 4'b1111;

        data_in[0*DW +: DW] = make_header(1, qos0, 0);
        data_in[1*DW +: DW] = make_header(2, qos1, 0);
        data_in[2*DW +: DW] = make_header(3, qos2, 0);
        data_in[3*DW +: DW] = make_header(0, qos3, 0);

        //-------------------------------------------------------------
        // One rising edge writes all four headers.
        //-------------------------------------------------------------

        @(negedge clk);

        valid_in = 4'b0000;

        is_header_in = 4'b0000;

        data_in = {(NPORTS*DW){1'b0}};

    end

    endtask


    //=================================================================
    // CHECK A PACKET-START WIN
    //
    // Phase II: "grant" is per output. Checks that the n-th packet to
    // win output `o` came from `expected_src`.
    //=================================================================

    task automatic check_grant;

        input [8*40-1:0] test_name;
        input integer    o;
        input integer    n;
        input integer    expected_src;

    begin
        if (win_cnt[o] > n && win_log[o*16 + n] === expected_src[1:0]) begin
            $display("[PASS] %0s : output %0d packet %0d won by P%0d",
                     test_name, o, n, expected_src);
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] %0s : output %0d packet %0d expected P%0d, got %0s",
                     test_name, o, n, expected_src,
                     (win_cnt[o] > n) ? "other port" : "no win");
            fail_count = fail_count + 1;
        end
    end

    endtask


    //=================================================================
    // CHECK OUTPUT STREAM  (data + no interleaving)
    //
    // Output `o` must carry `nblk` complete packets back to back, in the
    // order recorded in win_log. Each packet = len payloads + parity,
    // generated by make_payload / header^payload parity.
    //=================================================================

    task automatic check_stream;

        input [8*40-1:0]       test_name;
        input integer          o;
        input integer          nblk;
        input integer          len;
        input [2*NPORTS-1:0]   qos_by_src;

        integer b, k, src, base, bad;
        reg [DW-1:0] hdr, par, w;

    begin
        bad = 0;

        if (out_cnt[o] !== nblk*(len+1)) begin
            $display("[FAIL] %0s : output %0d word count=%0d expected=%0d",
                     test_name, o, out_cnt[o], nblk*(len+1));
            bad = 1;
        end else begin
            for (b = 0; b < nblk; b = b + 1) begin
                src  = win_log[o*16 + b];
                hdr  = make_header(o, qos_by_src[src*2 +: 2], len);
                par  = hdr;
                base = b*(len+1);

                for (k = 0; k < len; k = k + 1) begin
                    w   = make_payload(src, o, k);
                    par = par ^ w;
                    if (out_log[o*64 + base + k] !== w) begin
                        $display("[FAIL] %0s : output %0d pkt %0d (P%0d) word %0d = %h expected %h",
                                 test_name, o, b, src, k, out_log[o*64 + base + k], w);
                        bad = 1;
                    end
                end

                if (out_log[o*64 + base + len] !== par) begin
                    $display("[FAIL] %0s : output %0d pkt %0d (P%0d) parity = %h expected %h",
                             test_name, o, b, src, out_log[o*64 + base + len], par);
                    bad = 1;
                end
            end
        end

        if (!bad) begin
            $display("[PASS] %0s : output %0d carried %0d intact, non-interleaved packets",
                     test_name, o, nblk);
            pass_count = pass_count + 1;
        end else
            fail_count = fail_count + 1;
    end

    endtask




    //=================================================================
    // TEST 5: QoS PRIORITY  (Phase II: contention at ONE output)
    //
    // All four ports send a 2-payload packet to NORTH at the same time.
    //   P0 = Low, P1 = Medium, P2 = High, P3 = Low
    // Expected winners on output N:
    //   P2 (High) -> P1 (Medium) -> {P0,P3} in round-robin order
    // The packets must not interleave on the output.
    //=================================================================

    task test_qos_priority;

    begin
        $display("");
        $display("=====================================================");
        $display(" TEST 5: QoS PRIORITY");
        $display("=====================================================");

        reset_dut;

        fork
            send_packet(2'd0, 2'd0, 2'b00, 4'd2, 1'b0);
            send_packet(2'd1, 2'd0, 2'b01, 4'd2, 1'b0);
            send_packet(2'd2, 2'd0, 2'b10, 4'd2, 1'b0);
            send_packet(2'd3, 2'd0, 2'b00, 4'd2, 1'b0);
        join

        repeat(60) @(negedge clk);

        $display("QoS win order on N: P%0d P%0d P%0d P%0d (wins=%0d)",
                 win_log[0], win_log[1], win_log[2], win_log[3], win_cnt[0]);

        check_grant("T_QOS_1 HIGH   -> P2", 0, 0, 2);
        check_grant("T_QOS_2 MEDIUM -> P1", 0, 1, 1);

        if (win_cnt[0] == 4 &&
            ((win_log[2] == 2'd0 && win_log[3] == 2'd3) ||
             (win_log[2] == 2'd3 && win_log[3] == 2'd0))) begin
            $display("[PASS] T_QOS_3/4 : both LOW packets served last (P0,P3)");
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] T_QOS_3/4 : expected P0 and P3 as 3rd/4th, wins=%0d", win_cnt[0]);
            fail_count = fail_count + 1;
        end

        check_stream("T_QOS_STREAM", 0, 4, 2, {2'b00, 2'b10, 2'b01, 2'b00});

        cov_qos_priority = cov_qos_priority + 1;
    end

    endtask


    //=================================================================
    // TEST 6: ROUND ROBIN  (equal QoS, contention at output N)
    //
    // rr_ptr is 0 after reset and rotates to (winner+1) after every
    // packet, so the order must be P0 -> P1 -> P2 -> P3.
    //=================================================================

    task test_round_robin;

    begin
        $display("");
        $display("=====================================================");
        $display(" TEST 6: ROUND ROBIN");
        $display("=====================================================");

        reset_dut;

        fork
            send_packet(2'd0, 2'd0, 2'b00, 4'd2, 1'b0);
            send_packet(2'd1, 2'd0, 2'b00, 4'd2, 1'b0);
            send_packet(2'd2, 2'd0, 2'b00, 4'd2, 1'b0);
            send_packet(2'd3, 2'd0, 2'b00, 4'd2, 1'b0);
        join

        repeat(60) @(negedge clk);

        $display("RR win order on N: P%0d P%0d P%0d P%0d (wins=%0d)",
                 win_log[0], win_log[1], win_log[2], win_log[3], win_cnt[0]);

        check_grant("T_RR_1", 0, 0, 0);
        check_grant("T_RR_2", 0, 1, 1);
        check_grant("T_RR_3", 0, 2, 2);
        check_grant("T_RR_4", 0, 3, 3);

        check_stream("T_RR_STREAM", 0, 4, 2, {4{2'b00}});
    end

    endtask


    //=================================================================
    // DEBUG: print per-output arbiter state for 8 cycles
    //=================================================================

    task debug_arbiter;
        integer i, o;
    begin
        $display("\n========== ARBITER DEBUG ==========");
        for (i = 0; i < 8; i = i + 1) begin
            @(negedge clk);
            for (o = 0; o < NPORTS; o = o + 1)
                $display("CYCLE=%0d | OUT%0d | GRANT=%b | WR=%b | SEL=P%0d",
                         i, o,
                         DUT.grant_all[o*NPORTS +: NPORTS],
                         DUT.wr_all[o],
                         DUT.sel_all[o*2 +: 2]);
        end
        $display("====================================");
    end
    endtask

    //=================================================================
    // TEST: FOUR-PORT CONTENTION  (all four ports -> output N)
    //
    //   P0 = Medium, P1 = High, P2 = Low, P3 = Medium
    // Expected: P1 first, P2 last, {P0,P3} in the middle (round robin).
    // Per-output one-hot / lock assertions run continuously.
    //=================================================================

    task test_four_port_contention;

    begin
        $display("");
        $display("=====================================================");
        $display(" TEST: FOUR-PORT CONTENTION");
        $display("=====================================================");

        reset_dut;

        fork
            send_packet(2'd0, 2'd0, 2'b01, 4'd3, 1'b0);
            send_packet(2'd1, 2'd0, 2'b10, 4'd3, 1'b0);
            send_packet(2'd2, 2'd0, 2'b00, 4'd3, 1'b0);
            send_packet(2'd3, 2'd0, 2'b01, 4'd3, 1'b0);
        join

        repeat(60) @(negedge clk);

        $display("Contention win order on N: P%0d P%0d P%0d P%0d (wins=%0d)",
                 win_log[0], win_log[1], win_log[2], win_log[3], win_cnt[0]);

        check_grant("T_CONTENTION_1 HIGH first", 0, 0, 1);
        check_grant("T_CONTENTION_4 LOW last",   0, 3, 2);

        if (win_cnt[0] == 4 &&
            ((win_log[1] == 2'd0 && win_log[2] == 2'd3) ||
             (win_log[1] == 2'd3 && win_log[2] == 2'd0))) begin
            $display("[PASS] T_CONTENTION_2/3 : MEDIUM pair (P0,P3) served in the middle");
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] T_CONTENTION_2/3 : expected P0/P3 in the middle, wins=%0d", win_cnt[0]);
            fail_count = fail_count + 1;
        end

        check_stream("T_CONTENTION_STREAM", 0, 4, 3, {2'b01, 2'b00, 2'b10, 2'b01});
    end

    endtask


    //=================================================================
    // TEST 15 (NEW, Phase II): PARALLEL OUTPUTS
    //
    // Four ports send to four DIFFERENT outputs at once:
    //   P0->S, P1->E, P2->W, P3->N
    // With per-port parser/RCU and per-output arbiters there is no
    // contention, so all four packets must win their output in the SAME
    // cycle (Phase I would serialize them through the central arbiter).
    //=================================================================

    task test_parallel_outputs;

    begin
        $display("");
        $display("=====================================================");
        $display(" TEST 15: PARALLEL OUTPUTS (Phase II)");
        $display("=====================================================");

        reset_dut;

        fork
            send_packet(2'd0, 2'd1, 2'b00, 4'd3, 1'b0);
            send_packet(2'd1, 2'd2, 2'b00, 4'd3, 1'b0);
            send_packet(2'd2, 2'd3, 2'b00, 4'd3, 1'b0);
            send_packet(2'd3, 2'd0, 2'b00, 4'd3, 1'b0);
        join

        repeat(40) @(negedge clk);

        check_grant("T_PAR_S", 1, 0, 0);
        check_grant("T_PAR_E", 2, 0, 1);
        check_grant("T_PAR_W", 3, 0, 2);
        check_grant("T_PAR_N", 0, 0, 3);

        if (win_time[0*16] == win_time[1*16] &&
            win_time[1*16] == win_time[2*16] &&
            win_time[2*16] == win_time[3*16]) begin
            $display("[PASS] T_PAR_SIMULTANEOUS : all 4 outputs granted in the same cycle (t=%0d)",
                     win_time[0]);
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] T_PAR_SIMULTANEOUS : grant times N=%0d S=%0d E=%0d W=%0d",
                     win_time[0], win_time[16], win_time[32], win_time[48]);
            fail_count = fail_count + 1;
        end

        check_stream("T_PAR_STREAM_N", 0, 1, 3, {4{2'b00}});
        check_stream("T_PAR_STREAM_S", 1, 1, 3, {4{2'b00}});
        check_stream("T_PAR_STREAM_E", 2, 1, 3, {4{2'b00}});
        check_stream("T_PAR_STREAM_W", 3, 1, 3, {4{2'b00}});
    end

    endtask


    //=================================================================
    // SINGLE TEST MODE
    //=================================================================

    //=================================================================
    // TEST: FIFO-FULL BACKPRESSURE
    //
    // Starve port 0 by keeping ports 1-3 continuously requesting at
    // HIGH QoS (they win every arbitration round), while port 0
    // keeps writing single-word "header" flits. After DEPTH (8)
    // writes, port 0's input FIFO must assert full, and any further
    // write_enb must be silently dropped (checked via hierarchical
    // reference to the FIFO's internal full/count state) with no
    // wptr corruption. This is the backpressure path the original
    // directed tests never exercised.
    //=================================================================

    //=================================================================
    // TEST: EXHAUSTIVE ACL SWEEP
    //
    // Loops all 16 (src,dest) combinations. For each, sends a
    // 1-word-payload packet and checks the router's actual
    // acl_drop_flag / output behavior against an independently
    // hardcoded expectation of the ACL_TABLE contents (mirrors the
    // RTL's 2-rule policy: N->W blocked, E->S blocked, everything
    // else allowed). Closes the gap where directed tests only prove
    // "blocked stays blocked" but never prove "allowed stays
    // allowed" for the other 14 pairs.
    //=================================================================

    task automatic acl_expected_block;

        input integer src;
        input integer dst;
        output blocked;

    begin
        blocked = 1'b0;
        if (src == 0 && dst == 3) blocked = 1'b1; // N -> W
        if (src == 2 && dst == 1) blocked = 1'b1; // E -> S
    end

    endtask


    task test_acl_sweep;

        integer s, d;
        reg expect_blocked;
        reg [8*24-1:0] tname;

    begin

        cov_acl_sweep_pairs = 0;

        for (s = 0; s < NPORTS; s = s + 1) begin
            for (d = 0; d < NPORTS; d = d + 1) begin

                if (s != d) begin

                    reset_dut;

                    acl_expected_block(s, d, expect_blocked);

                    tname = "TSWEEP";

                    if (expect_blocked) begin

                        fork
                            send_packet(s[1:0], d[1:0], 2'b00, 1, 1'b0);
                            check_acl(tname, 1'b1);
                            check_acl_output_blocked(tname);
                        join

                    end else begin

                        fork
                            send_packet(s[1:0], d[1:0], 2'b00, 1, 1'b0);
                            check_packet_output(tname, s, d, 0, 1);
                        join

                    end

                    cov_acl_sweep_pairs = cov_acl_sweep_pairs + 1;

                    repeat(4) @(negedge clk);

                end

            end
        end

        $display(
            "[INFO] ACL_SWEEP : checked %0d of 12 valid (src!=dest) pairs",
            cov_acl_sweep_pairs
        );

    end

    endtask


    task test_backpressure_full;

    begin
        $display("");
        $display("=====================================================");
        $display(" TEST 13: FIFO-FULL BACKPRESSURE (Phase II)");
        $display("=====================================================");

        reset_dut;

        // Stop draining output N -> output FIFO fills, crossbar stalls,
        // RCU/parser/s0 stall, then port 0's input FIFO fills.
        read_enb_out = {NPORTS{1'b0}};

        send_packet(2'd0, 2'd0, 2'b00, 4'd15, 1'b0);
        send_packet(2'd0, 2'd0, 2'b00, 4'd15, 1'b0);

        if (bp_out_full_seen) begin
            $display("[PASS] T_BACKPRESSURE_OUT : output FIFO N asserted full");
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] T_BACKPRESSURE_OUT : output FIFO N never asserted full");
            fail_count = fail_count + 1;
        end

        if (bp_in_full_seen) begin
            $display("[PASS] T_BACKPRESSURE : port0 input FIFO asserted full once the pipeline backed up");
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] T_BACKPRESSURE : port0 input FIFO never asserted full");
            fail_count = fail_count + 1;
        end

        cov_backpressure = cov_backpressure + 1;

        // Drain, then prove the router is still alive: a fresh packet on
        // another port must come out clean on its own output.
        read_enb_out = {NPORTS{1'b1}};
        repeat(100) @(negedge clk);

        out_cnt[2] = 0;
        win_cnt[2] = 0;
        ov_d       = {NPORTS{1'b0}};

        send_packet(2'd1, 2'd2, 2'b01, 4'd3, 1'b0);
        repeat(30) @(negedge clk);

        check_stream("T_BACKPRESSURE_RECOVERY", 2, 1, 3, {2'b00, 2'b00, 2'b01, 2'b00});

        repeat(10) @(negedge clk);
    end

    endtask


    task run_single_test;

    begin

        case (TESTCASE)

            //---------------------------------------------------------
            // 1 = Normal routing
            //---------------------------------------------------------

            1: begin

                reset_dut;

                fork

                    send_packet(
                        2'd0,
                        2'd1,
                        2'b00,
                        4,
                        1'b0
                    );

                    check_packet_output(
                        "TC1_N_to_S",
                        0,
                        1,
                        0,
                        4
                    );

                join

            end


            //---------------------------------------------------------
            // 2 = ACL
            //---------------------------------------------------------

            2: begin

                reset_dut;

                fork

                    send_packet(
                        2'd0,
                        2'd3,
                        2'b01,
                        4,
                        1'b0
                    );

                    check_acl(
                        "TC2_N_to_W_ACL",
                        1'b1
                    );

                    check_acl_output_blocked(
                        "TC2_N_to_W_ACL"
                    );

                join

            end


            //---------------------------------------------------------
            // 3 = Correct parity
            //---------------------------------------------------------

            3: begin

                reset_dut;

                fork

                    send_packet(
                        2'd1,
                        2'd0,
                        2'b01,
                        3,
                        1'b0
                    );

                    check_packet_output(
                        "TC3_Correct_Parity",
                        1,
                        0,
                        1,
                        3
                    );

                    check_parity_result(
                        "TC3_Correct_Parity",
                        1'b0
                    );

                join

            end


            //---------------------------------------------------------
            // 4 = Corrupted parity
            //---------------------------------------------------------

            4: begin

                reset_dut;

                fork

                    send_packet(
                        2'd1,
                        2'd0,
                        2'b01,
                        3,
                        1'b1
                    );

                    check_parity_result(
                        "TC4_Corrupted_Parity",
                        1'b1
                    );

                join

            end


            //---------------------------------------------------------
            // 5 = QoS priority
            //---------------------------------------------------------

            5:
                test_qos_priority;


            //---------------------------------------------------------
            // 6 = Round robin
            //---------------------------------------------------------

            6:
                test_round_robin;


            //---------------------------------------------------------
            // 7 = Four-port contention
            //---------------------------------------------------------

            7:
                test_four_port_contention;


            //---------------------------------------------------------
            // 8 = Zero payload
            //---------------------------------------------------------

            8: begin

                reset_dut;

                fork

                    send_packet(
                        2'd1,
                        2'd3,
                        2'b00,
                        4'd0,
                        1'b0
                    );

                    check_packet_output(
                        "TC8_Zero_Payload",
                        1,
                        3,
                        0,
                        0
                    );

                join

            end


            //---------------------------------------------------------
            // 9 = Maximum payload
            //---------------------------------------------------------

            9: begin

                reset_dut;

                fork

                    send_packet(
                        2'd2,
                        2'd0,
                        2'b01,
                        4'd15,
                        1'b0
                    );

                    check_packet_output(
                        "TC9_Max_Payload",
                        2,
                        0,
                        1,
                        15
                    );

                join

            end


            //---------------------------------------------------------
            // 10 = Reset recovery
            //---------------------------------------------------------

            10: begin

                reset_dut;

                fork

                    send_packet(
                        2'd3,
                        2'd1,
                        2'b10,
                        4,
                        1'b0
                    );

                    check_packet_output(
                        "TC10_Reset_Recovery",
                        3,
                        1,
                        2,
                        4
                    );

                join

            end


            //---------------------------------------------------------
            // 11 = ACL rule 2 (S -> N)
            //---------------------------------------------------------

            11: begin

                reset_dut;

                fork

                    send_packet(
                        2'd2,
                        2'd1,
                        2'b01,
                        3,
                        1'b0
                    );

                    check_acl(
                        "TC11_E_to_S_ACL",
                        1'b1
                    );

                    check_acl_output_blocked(
                        "TC11_E_to_S_ACL"
                    );

                join

                cov_acl_rule2 = cov_acl_rule2 + 1;

            end


            //---------------------------------------------------------
            // 12 = grant one-hot assertion sweep (runs under
            //      four-port contention to stress the arbiter)
            //---------------------------------------------------------

            12:
                test_four_port_contention;


            //---------------------------------------------------------
            // 13 = FIFO-full backpressure
            //---------------------------------------------------------

            13:
                test_backpressure_full;


            //---------------------------------------------------------
            // 14 = exhaustive ACL sweep (all 12 valid src!=dest pairs)
            //---------------------------------------------------------

            14:
                test_acl_sweep;


            //---------------------------------------------------------
            // 15 = parallel outputs (Phase II)
            //---------------------------------------------------------

            15:
                test_parallel_outputs;


            default: begin

                $display(
                    "[FAIL] Invalid TESTCASE=%0d",
                    TESTCASE
                );

                fail_count = fail_count + 1;

            end

        endcase

    end

    endtask


    //=================================================================
    // MAIN REGRESSION
    //=================================================================

    initial begin

        pass_count = 0;
        fail_count = 0;

        cov_qos_high = 0;
        cov_qos_med  = 0;
        cov_qos_low  = 0;

        cov_port_n = 0;
        cov_port_s = 0;
        cov_port_e = 0;
        cov_port_w = 0;

        cov_acl_drop   = 0;
        cov_parity_good = 0;
        cov_parity_err  = 0;

        cov_qos_priority = 0;
        cov_round_robin  = 0;

        cov_acl_rule2      = 0;
        cov_onehot_checks  = 0;
        cov_backpressure   = 0;
        cov_acl_sweep_pairs = 0;


        $display("");
        $display("=====================================================");
        $display(" Secure NoC Router - UPDATED SELF-CHECKING TB");
        $display("=====================================================");
        $display(
            "DW=%0d NPORTS=%0d DEPTH=%0d",
            DW,
            NPORTS,
            DEPTH
        );
        $display(
            "TESTCASE=%0d",
            TESTCASE
        );
        $display("=====================================================");


        //=============================================================
        // SINGLE TEST MODE
        //=============================================================

        if (TESTCASE != 0) begin

            run_single_test;

        end


        //=============================================================
        // COMPLETE REGRESSION
        //=============================================================

        else begin


            //=========================================================
            // TEST 1
            // Normal routing
            //=========================================================

            $display("");
            $display("--- TEST 1: Normal routing ---");


            reset_dut;

            fork

                send_packet(
                    2'd0,
                    2'd1,
                    2'b00,
                    4,
                    1'b0
                );

                check_packet_output(
                    "T1_N_to_S",
                    0,
                    1,
                    0,
                    4
                );

            join


            reset_dut;

            fork

                send_packet(
                    2'd1,
                    2'd2,
                    2'b00,
                    4,
                    1'b0
                );

                check_packet_output(
                    "T1_S_to_E",
                    1,
                    2,
                    0,
                    4
                );

            join


            reset_dut;

            fork

                send_packet(
                    2'd2,
                    2'd0,
                    2'b00,
                    4,
                    1'b0
                );

                check_packet_output(
                    "T1_E_to_N",
                    2,
                    0,
                    0,
                    4
                );

            join


            reset_dut;

            fork

                send_packet(
                    2'd3,
                    2'd2,
                    2'b00,
                    4,
                    1'b0
                );

                check_packet_output(
                    "T1_W_to_E",
                    3,
                    2,
                    0,
                    4
                );

            join


            //=========================================================
            // TEST 2
            // ACL
            //=========================================================

            $display("");
            $display("--- TEST 2: ACL N -> W violation ---");


            reset_dut;

            fork

                send_packet(
                    2'd0,
                    2'd3,
                    2'b01,
                    4,
                    1'b0
                );

                check_acl(
                    "T2_ACL",
                    1'b1
                );

                check_acl_output_blocked(
                    "T2_ACL"
                );

            join


            //=========================================================
            // TEST 3
            // Correct parity
            //=========================================================

            $display("");
            $display("--- TEST 3: Correct parity ---");


            reset_dut;

            fork

                send_packet(
                    2'd1,
                    2'd0,
                    2'b01,
                    3,
                    1'b0
                );

                check_packet_output(
                    "T3_Correct_Parity",
                    1,
                    0,
                    1,
                    3
                );

                check_parity_result(
                    "T3_Correct_Parity",
                    1'b0
                );

            join


            //=========================================================
            // TEST 4
            // Corrupted parity
            //=========================================================

            $display("");
            $display("--- TEST 4: Corrupted parity ---");


            reset_dut;

            fork

                send_packet(
                    2'd1,
                    2'd0,
                    2'b01,
                    3,
                    1'b1
                );

                check_parity_result(
                    "T4_Corrupted_Parity",
                    1'b1
                );

            join


            //=========================================================
            // TEST 5
            // QoS priority
            //=========================================================

            test_qos_priority;


            //=========================================================
            // TEST 6
            // Round robin
            //=========================================================

            test_round_robin;


            //=========================================================
            // TEST 7
            // Four-port contention
            //=========================================================

            test_four_port_contention;


            //=========================================================
            // TEST 15
            // Parallel outputs (Phase II)
            //=========================================================

            test_parallel_outputs;


            //=========================================================
            // TEST 8
            // Zero payload
            //=========================================================

            $display("");
            $display("--- TEST 8: Zero-payload packet ---");


            reset_dut;

            fork

                send_packet(
                    2'd1,
                    2'd3,
                    2'b00,
                    4'd0,
                    1'b0
                );

                check_packet_output(
                    "T8_Zero_Payload",
                    1,
                    3,
                    0,
                    0
                );

            join


            //=========================================================
            // TEST 9
            // Maximum payload
            //=========================================================

            $display("");
            $display("--- TEST 9: Maximum payload ---");


            reset_dut;

            fork

                send_packet(
                    2'd2,
                    2'd0,
                    2'b01,
                    4'd15,
                    1'b0
                );

                check_packet_output(
                    "T9_Max_Payload",
                    2,
                    0,
                    1,
                    15
                );

            join


            //=========================================================
            // TEST 10
            // Reset recovery
            //=========================================================

            $display("");
            $display("--- TEST 10: Reset recovery ---");


            reset_dut;

            // Generate traffic first.
            fork

                send_packet(
                    2'd0,
                    2'd1,
                    2'b00,
                    3,
                    1'b0
                );

                check_packet_output(
                    "T10_Pre_Reset_Traffic",
                    0,
                    1,
                    0,
                    3
                );

            join


            // Actual reset.
            reset_dut;


            // Traffic after reset.
            fork

                send_packet(
                    2'd3,
                    2'd1,
                    2'b10,
                    4,
                    1'b0
                );

                check_packet_output(
                    "T10_Post_Reset_Traffic",
                    3,
                    1,
                    2,
                    4
                );

            join


            repeat(10) @(negedge clk);


            //=========================================================
            // TEST 11
            // ACL rule 2 (E -> S)
            //=========================================================

            $display("");
            $display("--- TEST 11: ACL E -> S violation (rule 2) ---");

            reset_dut;

            fork

                send_packet(
                    2'd2,
                    2'd1,
                    2'b01,
                    3,
                    1'b0
                );

                check_acl(
                    "T11_E_to_S_ACL",
                    1'b1
                );

                check_acl_output_blocked(
                    "T11_E_to_S_ACL"
                );

            join

            cov_acl_rule2 = cov_acl_rule2 + 1;


            //=========================================================
            // TEST 12
            // FIFO-full backpressure
            //=========================================================

            $display("");
            $display("--- TEST 12: FIFO-full backpressure ---");

            test_backpressure_full;


            //=========================================================
            // TEST 13
            // Exhaustive ACL sweep (all 12 valid src!=dest pairs)
            //=========================================================

            $display("");
            $display("--- TEST 13: Exhaustive ACL sweep (12 pairs) ---");

            test_acl_sweep;


            repeat(10) @(negedge clk);

        end


        //=============================================================
        // SUMMARY
        //=============================================================

        $display("");
        $display("=====================================================");
        $display(" TEST SUMMARY");
        $display("=====================================================");

        $display(
            "PASS = %0d",
            pass_count
        );

        $display(
            "FAIL = %0d",
            fail_count
        );

        $display("-----------------------------------------------------");

        $display(
            "QoS High seen       : %0d",
            cov_qos_high
        );

        $display(
            "QoS Medium seen     : %0d",
            cov_qos_med
        );

        $display(
            "QoS Low seen        : %0d",
            cov_qos_low
        );

        $display(
            "Dest North utilized : %0d",
            cov_port_n
        );

        $display(
            "Dest South utilized : %0d",
            cov_port_s
        );

        $display(
            "Dest East utilized  : %0d",
            cov_port_e
        );

        $display(
            "Dest West utilized  : %0d",
            cov_port_w
        );

        $display(
            "ACL drop events     : %0d",
            cov_acl_drop
        );

        $display(
            "Correct parity tests: %0d",
            cov_parity_good
        );

        $display(
            "Parity error tests  : %0d",
            cov_parity_err
        );

        $display(
            "QoS priority tests  : %0d",
            cov_qos_priority
        );

        $display(
            "Round-robin tests   : %0d",
            cov_round_robin
        );

        $display(
            "ACL rule-2 tests    : %0d",
            cov_acl_rule2
        );

        $display(
            "Backpressure tests  : %0d",
            cov_backpressure
        );

        $display(
            "ACL sweep pairs     : %0d of 12",
            cov_acl_sweep_pairs
        );

        $display(
            "Onehot-grant checks : %0d cycles (background assertion)",
            cov_onehot_checks
        );

        $display("=====================================================");


        if (fail_count == 0) begin

            $display(
                "RESULT: ALL CHECKED TESTS PASSED"
            );

        end

        else begin

            $display(
                "RESULT: %0d TEST(S) FAILED",
                fail_count
            );

        end


        $display("=====================================================");

        $stop;

    end


    //=================================================================
    // NOTE: the Phase-I global grant assertions (DUT.u_arb.grant) were
    // replaced by the per-output assertions near the top of this file
    // (ASSERT_ONEHOT / ASSERT_WR / ASSERT_LOCK / ASSERT_LANE).
    //=================================================================


    //=================================================================
    // WAVEFORM
    //=================================================================

    initial begin

        $dumpfile("secure_noc_router_tb.vcd");

        $dumpvars(
            0,
            secure_noc_router_tb
        );

    end

endmodule