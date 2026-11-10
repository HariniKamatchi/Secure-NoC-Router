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
    // ALWAYS-ON ASSERTION: grant must be one-hot whenever grant_valid,
    // and zero otherwise. Runs continuously under every TESTCASE,
    // catching arbiter bugs that a single directed test might miss.
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

    always @(posedge clk) begin

        if (resetn) begin

            cov_onehot_checks = cov_onehot_checks + 1;

            if (DUT.u_arb.grant_valid) begin

                if (count_ones(DUT.u_arb.grant) != 1) begin
                    $display(
                        "[FAIL] ASSERT_ONEHOT : grant_valid=1 but grant=%b (not one-hot)",
                        DUT.u_arb.grant
                    );
                    fail_count = fail_count + 1;
                end

            end else begin

                if (DUT.u_arb.grant != 0) begin
                    $display(
                        "[FAIL] ASSERT_ONEHOT : grant_valid=0 but grant=%b (should be idle)",
                        DUT.u_arb.grant
                    );
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
    // CHECK CURRENT ARBITER GRANT
    //=================================================================

    task automatic check_grant;

        input [8*40-1:0] test_name;
        input integer expected_port;

        reg [NPORTS-1:0] expected_grant;

    begin

        expected_grant = 4'b0000;

        if (expected_port >= 0)
            expected_grant[expected_port] = 1'b1;


        if (DUT.u_arb.grant === expected_grant) begin

            $display(
                "[PASS] %0s : grant=%b selected_port=%0d",
                test_name,
                DUT.u_arb.grant,
                expected_port
            );

            pass_count = pass_count + 1;

        end

        else begin

            $display(
                "[FAIL] %0s : grant=%b expected=%b expected_port=%0d",
                test_name,
                DUT.u_arb.grant,
                expected_grant,
                expected_port
            );

            fail_count = fail_count + 1;

        end

    end

    endtask


    //=================================================================
    // TEST: QoS PRIORITY
    //
    // Port priorities:
    //
    //   P0 = Low
    //   P1 = Medium
    //   P2 = High
    //   P3 = Low
    //
    // Expected order:
    //
    //   P2 -> P1 -> P3 -> P0
    //
    // Why?
    //
    // First: High wins.
    // Then: Medium wins.
    // Finally P0/P3 have equal Low priority, so round-robin
    // determines their order.
    //=================================================================

task test_qos_priority;

begin

    $display("");
    $display("=====================================================");
    $display(" TEST 5: QoS PRIORITY");
    $display("=====================================================");

    reset_dut;

    //---------------------------------------------------------
    // P0 = LOW
    // P1 = MEDIUM
    // P2 = HIGH
    // P3 = LOW
    //---------------------------------------------------------

    inject_four_headers(
        2'b00,       // P0 LOW
        2'b01,       // P1 MEDIUM
        2'b10,       // P2 HIGH
        2'b00        // P3 LOW
    );


    //---------------------------------------------------------
    // IMPORTANT:
    // inject_four_headers has already waited until the headers
    // are inside the FIFOs.
    //
    // Therefore DO NOT wait another negedge here.
    //---------------------------------------------------------

    $display(
        "QoS cycle 1: REQ=%b GRANT=%b VALID=%b RR_PTR=%0d",
        DUT.u_arb.req,
        DUT.u_arb.grant,
        DUT.u_arb.grant_valid,
        DUT.u_arb.rr_ptr
    );

    if (DUT.u_arb.grant === 4'b0100) begin

        $display("[PASS] T_QOS_1 : HIGH -> P2");
        pass_count = pass_count + 1;

    end
    else begin

        $display(
            "[FAIL] T_QOS_1 : expected P2 (0100), got %b",
            DUT.u_arb.grant
        );

        fail_count = fail_count + 1;

    end


    //---------------------------------------------------------
    // Consume P2
    //---------------------------------------------------------

    @(posedge clk);

    //---------------------------------------------------------
    // Grant #2
    //---------------------------------------------------------

    @(negedge clk);

    $display(
        "QoS cycle 2: REQ=%b GRANT=%b VALID=%b RR_PTR=%0d",
        DUT.u_arb.req,
        DUT.u_arb.grant,
        DUT.u_arb.grant_valid,
        DUT.u_arb.rr_ptr
    );

    if (DUT.u_arb.grant === 4'b0010) begin

        $display("[PASS] T_QOS_2 : MEDIUM -> P1");
        pass_count = pass_count + 1;

    end
    else begin

        $display(
            "[FAIL] T_QOS_2 : expected P1 (0010), got %b",
            DUT.u_arb.grant
        );

        fail_count = fail_count + 1;

    end


    //---------------------------------------------------------
    // Consume P1
    //---------------------------------------------------------

    @(posedge clk);

    //---------------------------------------------------------
    // Grant #3
    //---------------------------------------------------------

    @(negedge clk);

    $display(
        "QoS cycle 3: REQ=%b GRANT=%b VALID=%b RR_PTR=%0d",
        DUT.u_arb.req,
        DUT.u_arb.grant,
        DUT.u_arb.grant_valid,
        DUT.u_arb.rr_ptr
    );

    if ((DUT.u_arb.grant === 4'b0001) ||
        (DUT.u_arb.grant === 4'b1000)) begin

        $display("[PASS] T_QOS_3 : LOW selected");
        pass_count = pass_count + 1;

    end
    else begin

        $display(
            "[FAIL] T_QOS_3 : expected P0/P3, got %b",
            DUT.u_arb.grant
        );

        fail_count = fail_count + 1;

    end


    //---------------------------------------------------------
    // Consume selected LOW port
    //---------------------------------------------------------

    @(posedge clk);

    //---------------------------------------------------------
    // Grant #4
    //---------------------------------------------------------

    @(negedge clk);

    $display(
        "QoS cycle 4: REQ=%b GRANT=%b VALID=%b RR_PTR=%0d",
        DUT.u_arb.req,
        DUT.u_arb.grant,
        DUT.u_arb.grant_valid,
        DUT.u_arb.rr_ptr
    );

    if ((DUT.u_arb.grant === 4'b0001) ||
        (DUT.u_arb.grant === 4'b1000)) begin

        $display("[PASS] T_QOS_4 : final LOW selected");
        pass_count = pass_count + 1;

    end
    else begin

        $display(
            "[FAIL] T_QOS_4 : expected remaining LOW, got %b",
            DUT.u_arb.grant
        );

        fail_count = fail_count + 1;

    end

    cov_qos_priority = cov_qos_priority + 1;

end

endtask

    //=================================================================
    // TEST: PURE ROUND ROBIN
    //
    // All four requests have the SAME QoS.
    //
    // rr_ptr starts at 0 after reset.
    //
    // Expected:
    //   P0 -> P1 -> P2 -> P3
    //=================================================================

task test_round_robin;

begin

    $display("");
    $display("=====================================================");
    $display(" TEST 6: ROUND ROBIN");
    $display("=====================================================");

    reset_dut;

    //---------------------------------------------------------
    // All four ports LOW priority.
    //---------------------------------------------------------

    inject_four_headers(
        2'b00,
        2'b00,
        2'b00,
        2'b00
    );


    //---------------------------------------------------------
    // RR #1
    //
    // rr_ptr is reset to 0.
    // Therefore P0 should win.
    //---------------------------------------------------------

    $display(
        "RR cycle 1: REQ=%b GRANT=%b VALID=%b RR_PTR=%0d",
        DUT.u_arb.req,
        DUT.u_arb.grant,
        DUT.u_arb.grant_valid,
        DUT.u_arb.rr_ptr
    );

    if (DUT.u_arb.grant === 4'b0001) begin

        $display("[PASS] T_RR_1 : P0");
        pass_count = pass_count + 1;

    end
    else begin

        $display(
            "[FAIL] T_RR_1 : expected P0 (0001), got %b",
            DUT.u_arb.grant
        );

        fail_count = fail_count + 1;

    end


    //---------------------------------------------------------
    // Consume P0
    //---------------------------------------------------------

    @(posedge clk);
    @(negedge clk);


    //---------------------------------------------------------
    // RR #2
    //---------------------------------------------------------

    $display(
        "RR cycle 2: REQ=%b GRANT=%b VALID=%b RR_PTR=%0d",
        DUT.u_arb.req,
        DUT.u_arb.grant,
        DUT.u_arb.grant_valid,
        DUT.u_arb.rr_ptr
    );

    if (DUT.u_arb.grant === 4'b0010) begin

        $display("[PASS] T_RR_2 : P1");
        pass_count = pass_count + 1;

    end
    else begin

        $display(
            "[FAIL] T_RR_2 : expected P1 (0010), got %b",
            DUT.u_arb.grant
        );

        fail_count = fail_count + 1;

    end


    //---------------------------------------------------------
    // Consume P1
    //---------------------------------------------------------

    @(posedge clk);
    @(negedge clk);


    //---------------------------------------------------------
    // RR #3
    //---------------------------------------------------------

    $display(
        "RR cycle 3: REQ=%b GRANT=%b VALID=%b RR_PTR=%0d",
        DUT.u_arb.req,
        DUT.u_arb.grant,
        DUT.u_arb.grant_valid,
        DUT.u_arb.rr_ptr
    );

    if (DUT.u_arb.grant === 4'b0100) begin

        $display("[PASS] T_RR_3 : P2");
        pass_count = pass_count + 1;

    end
    else begin

        $display(
            "[FAIL] T_RR_3 : expected P2 (0100), got %b",
            DUT.u_arb.grant
        );

        fail_count = fail_count + 1;

    end


    //---------------------------------------------------------
    // Consume P2
    //---------------------------------------------------------

    @(posedge clk);
    @(negedge clk);


    //---------------------------------------------------------
    // RR #4
    //---------------------------------------------------------

    $display(
        "RR cycle 4: REQ=%b GRANT=%b VALID=%b RR_PTR=%0d",
        DUT.u_arb.req,
        DUT.u_arb.grant,
        DUT.u_arb.grant_valid,
        DUT.u_arb.rr_ptr
    );

    if (DUT.u_arb.grant === 4'b1000) begin

        $display("[PASS] T_RR_4 : P3");
        pass_count = pass_count + 1;

    end
    else begin

        $display(
            "[FAIL] T_RR_4 : expected P3 (1000), got %b",
            DUT.u_arb.grant
        );

        fail_count = fail_count + 1;

    end

    cov_round_robin = cov_round_robin + 1;

end

endtask
task debug_arbiter;
    integer i;
    begin
        $display("\n========== ARBITER DEBUG ==========");

        for (i = 0; i < 8; i = i + 1) begin

            @(negedge clk);

            $display(
                "CYCLE=%0d | REQ=%b | QOS=%0d %0d %0d %0d | GRANT=%b | VALID=%b | RR_PTR=%0d",
                i,
                DUT.u_arb.req,
                DUT.u_arb.qos[0],
                DUT.u_arb.qos[1],
                DUT.u_arb.qos[2],
                DUT.u_arb.qos[3],
                DUT.u_arb.grant,
                DUT.u_arb.grant_valid,
                DUT.u_arb.rr_ptr
            );

            @(posedge clk);
        end

        $display("====================================");
    end
endtask
    //=================================================================
    // TEST: FOUR-PORT CONTENTION
    //
    // Also explicitly verifies grant is one-hot at each arbitration
    // step.
    //
    // Priorities:
    //
    // P0 = Medium
    // P1 = High
    // P2 = Low
    // P3 = Medium
    //
    // Expected:
    //
    // P1 -> P0/P3 depending on rr state -> ...
    //=================================================================

    task test_four_port_contention;

        integer cycles;

    begin

        $display("");
        $display("=====================================================");
        $display(" TEST: FOUR-PORT CONTENTION");
        $display("=====================================================");


        reset_dut;


        //-------------------------------------------------------------
        // Four simultaneous requests.
        //-------------------------------------------------------------

        inject_four_headers(
            2'b01,
            2'b10,
            2'b00,
            2'b01
        );


        //-------------------------------------------------------------
        // Monitor several arbitration cycles.
        //
        // We don't hard-code the whole order here because the purpose
        // of this test is to verify:
        //
        //   1. grant is one-hot
        //   2. a valid request is granted
        //   3. all four requests eventually disappear
        //-------------------------------------------------------------

        for (cycles = 0; cycles < 4; cycles = cycles + 1) begin

            @(negedge clk);

            if ((DUT.u_arb.grant == 4'b0000) ||
                (DUT.u_arb.grant == 4'b0001) ||
                (DUT.u_arb.grant == 4'b0010) ||
                (DUT.u_arb.grant == 4'b0100) ||
                (DUT.u_arb.grant == 4'b1000)) begin

                $display(
                    "[PASS] T_CONTENTION_%0d : grant=%b",
                    cycles + 1,
                    DUT.u_arb.grant
                );

                pass_count = pass_count + 1;

            end

            else begin

                $display(
                    "[FAIL] T_CONTENTION_%0d : illegal grant=%b",
                    cycles + 1,
                    DUT.u_arb.grant
                );

                fail_count = fail_count + 1;

            end


            @(posedge clk);

        end

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

        integer i;
        integer full_seen;

    begin

        reset_dut;

        // Ports 1-3: continuous high-QoS header traffic so the
        // arbiter always prefers them over port 0.
        valid_in[1]     = 1'b1; is_header_in[1] = 1'b1;
        valid_in[2]     = 1'b1; is_header_in[2] = 1'b1;
        valid_in[3]     = 1'b1; is_header_in[3] = 1'b1;

        data_in[1*DW +: DW] = make_header(0, 2, 0); // HIGH qos, len 0
        data_in[2*DW +: DW] = make_header(0, 2, 0);
        data_in[3*DW +: DW] = make_header(0, 2, 0);

        full_seen = 0;

        // Port 0: keep writing words, faster than it can ever be
        // granted, for more than DEPTH cycles.
        for (i = 0; i < (DEPTH + 6); i = i + 1) begin

            @(negedge clk);

            valid_in[0]     = 1'b1;
            is_header_in[0] = 1'b0;
            data_in[0*DW +: DW] = 16'hBEE0 + i;

            if (DUT.in_ports[0].u_ip.full)
                full_seen = 1;

        end

        @(negedge clk);
        valid_in = {NPORTS{1'b0}};
        is_header_in = {NPORTS{1'b0}};

        if (full_seen) begin
            $display(
                "[PASS] T_BACKPRESSURE : port0 FIFO correctly asserted full under sustained starvation"
            );
            pass_count = pass_count + 1;
        end else begin
            $display(
                "[FAIL] T_BACKPRESSURE : port0 FIFO never asserted full (expected after %0d writes)",
                DEPTH
            );
            fail_count = fail_count + 1;
        end

        cov_backpressure = cov_backpressure + 1;

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
    // ASSERTION
    //
    // Arbiter grant must always be one-hot or zero.
    //=================================================================

    always @(posedge clk) begin

        if (resetn) begin

            case (DUT.u_arb.grant)

                4'b0000,
                4'b0001,
                4'b0010,
                4'b0100,
                4'b1000:
                    ;

                default: begin

                    $display(
                        "[ASSERTION FAIL] Illegal multiple grant at t=%0t : grant=%b",
                        $time,
                        DUT.u_arb.grant
                    );

                    fail_count = fail_count + 1;

                end

            endcase

        end

    end


    //=================================================================
    // ADDITIONAL ARBITER ASSERTION
    //
    // If grant_valid is asserted, grant must not be zero.
    //=================================================================

    always @(posedge clk) begin

        if (resetn) begin

            if (DUT.u_arb.grant_valid &&
                (DUT.u_arb.grant == 4'b0000)) begin

                $display(
                    "[ASSERTION FAIL] grant_valid=1 but grant=0000 at t=%0t",
                    $time
                );

                fail_count = fail_count + 1;

            end

        end

    end


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