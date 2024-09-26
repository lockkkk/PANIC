`timescale 1ns / 1ps
`include "panic_define.v"
module packet_gen_pipeline;

reg  clk;
reg rst;
reg start;

reg                        config_mat_en;
reg [`MATCH_KEY_WIDTH-1:0] config_mat_key;
reg [128-1:0]              config_mat_value;
reg [`MAT_ADDR_WIDTH-1:0]  config_mat_addr;

reg [`PANIC_DESC_LEN_SIZE-1:0] desc_len;
reg [`PANIC_DESC_CELL_ID_SIZE-1:0] desc_cell_id;
reg [`PANIC_DESC_CHAIN_SIZE-1:0] desc_chain;
reg [`PANIC_DESC_PRIO_SIZE-1:0] desc_prio;
reg [`PANIC_DESC_TIME_SIZE-1:0] desc_time;
reg                             desc_drop;
reg [`PANIC_DESC_FLOW_SIZE-1:0] desc_flow_id;
reg [`PANIC_DESC_TS_SIZE-1:0]   desc_ts;

localparam REG_DATA_WIDTH = 32;
localparam REG_ADDR_WIDTH = 16;
reg [REG_ADDR_WIDTH-1:0]            ctrl_reg_wr_addr;
reg [REG_DATA_WIDTH-1:0]            ctrl_reg_wr_data;
reg                                 ctrl_reg_wr_en;
 
always begin
    clk = ~clk; 
    #2;
end

initial begin
    $display("   ___                              ");
    $display("  / __\\_ __  ___  _ __   ____ _   _ ");
    $display(" / _\\ | '__|/ _ \\| '_ \\ |_  /| | | |");
    $display("/ /   | |  |  __/| | | | / / | |_| |");
    $display("\\/    |_|   \\___||_| |_|/___| \\__, |");
    $display("                              |___/ ");
    $display("--------Simulation Parameter--------");
    $display("|* Simulation Frequency: 250Mhz    |");
    $display("|* Logging Interval: 4K cycles/log |");
    $display("|* Packet Size Range: [64B, 2048B] |");
    $display("|* Offload engine variance: 40%    |");
    $display("|* Chain: -> A(A1, A2, A3) ->      |");
    $display("|                                  |");
    $display("|* More detail about the chainning |");
    $display("|  model please reference Fig.7(b) |");
    $display("|  in Frenzy paper                 |");
    $display("|----------------------------------|");
    clk = 0;
    rst = 1;
    start = 0;
    config_mat_en = 0;
    packet_len = 4;
    pattern_id = 0;
    ctrl_reg_wr_en = 0;
    #1000;
    rst = 0;
    #800;
    // config monitor 
    ctrl_reg_wr_en = 1;
    ctrl_reg_wr_data = {16'h4, 8'h40, 4'h1, 4'h0};
    ctrl_reg_wr_addr = 16'h0094;
    #4;
    ctrl_reg_wr_en = 0;
    #4;
    // config monitor 
    ctrl_reg_wr_en = 1;
    ctrl_reg_wr_data = {16'h4, 8'h40, 4'h1, 4'h1};
    ctrl_reg_wr_addr = 16'h0094;
    #4;
    ctrl_reg_wr_en = 0;
    #600;
    start = 1;

    $display("--------------\nRST, Start Sending %4dB packet", packet_len * 64 );
end


localparam AXIS_DATA_WIDTH = 512;
localparam AXIS_KEEP_WIDTH = AXIS_DATA_WIDTH / 8;
localparam PORTS = 1;

wire [PORTS*AXIS_DATA_WIDTH-1:0]    rx_axis_tdata;
wire [PORTS*AXIS_KEEP_WIDTH-1:0]    rx_axis_tkeep;
wire [PORTS-1:0]                    rx_axis_tvalid;
wire [PORTS-1:0]                    rx_axis_tready;
wire [PORTS-1:0]                    rx_axis_tlast;
wire [PORTS-1:0]                    rx_axis_tuser;


reg  [PORTS*AXIS_DATA_WIDTH-1:0]    fifo_rx_axis_tdata;
reg  [PORTS*AXIS_KEEP_WIDTH-1:0]    fifo_rx_axis_tkeep;
reg  [PORTS-1:0]                    fifo_rx_axis_tvalid;
wire [PORTS-1:0]                    fifo_rx_axis_tready;
reg  [PORTS-1:0]                    fifo_rx_axis_tlast;
reg  [PORTS-1:0]                    fifo_rx_axis_tuser;


/*
* Receive data input
*/
wire [PORTS*AXIS_DATA_WIDTH-1:0]    panic_rx_axis_tdata;
wire [PORTS*AXIS_KEEP_WIDTH-1:0]    panic_rx_axis_tkeep;
wire [PORTS-1:0]                    panic_rx_axis_tvalid;
reg  [PORTS-1:0]                    panic_rx_axis_tready;
wire [PORTS-1:0]                    panic_rx_axis_tlast;
wire [PORTS-1:0]                    panic_rx_axis_tuser;


panic #
(   
    /* MEMORY PARAMETER */
    // Width of AXI memory data bus in bits, normal is 512
    .AXI_DATA_WIDTH(AXIS_DATA_WIDTH),
    // Width of panic memory address bus in bits
    .AXI_ADDR_WIDTH(18),

    /*AXIS INTERFACE PARAMETER*/
    // Width of AXI stream interfaces in bits, normal is 512
    .AXIS_DATA_WIDTH(AXIS_DATA_WIDTH),
    .AXIS_KEEP_WIDTH(AXIS_KEEP_WIDTH),
    .AXIS_LAST_ENABLE(1),
    .AXIS_ID_ENABLE(0),
    .AXIS_DEST_ENABLE(0),
    .AXIS_USER_ENABLE(0),
    .LEN_WIDTH(16),
    .TAG_WIDTH(8),
    .ENABLE_UNALIGNED(1),
    .ENABLE_SG(0),
    .REG_DATA_WIDTH(REG_DATA_WIDTH),
    .REG_ADDR_WIDTH(REG_ADDR_WIDTH),
    /*CROSSBAR PARAMETER*/
    // crossbar data width
    .SWITCH_DATA_WIDTH(512),
    // crossbar dest width, if it is 3, then we have 2^3 ports for the corssbar
    .SWITCH_DEST_WIDTH(3),
    .SWITCH_USER_ENABLE(1),  
    .SWITCH_USER_WIDTH(1),

    /*ENGINE PARAMETER*/
    .INIT_CREDIT_NUM(8),
    .ENGINE_NUM(4),
    .TEST_MODE(3)

)
panic_inst
(
    .clk(clk),
    .rst(rst),

    .ctrl_reg_wr_addr(ctrl_reg_wr_addr),
    .ctrl_reg_wr_data(ctrl_reg_wr_data),
    .ctrl_reg_wr_en(ctrl_reg_wr_en),
    .ctrl_reg_wr_ack(),
    .ctrl_reg_rd_addr(0),
    .ctrl_reg_rd_en(0),
    .ctrl_reg_rd_data(),
    .ctrl_reg_rd_ack(),
    /*
    * Receive data from the wire
    */
    .s_rx_axis_tdata(rx_axis_tdata),
    .s_rx_axis_tkeep(rx_axis_tkeep),
    .s_rx_axis_tvalid(rx_axis_tvalid),
    .s_rx_axis_tready(rx_axis_tready),
    .s_rx_axis_tlast(rx_axis_tlast),
    .s_rx_axis_tuser(rx_axis_tuser),

    /*
    * Send data output to the dma
    */
    .m_rx_axis_tdata(panic_rx_axis_tdata),
    .m_rx_axis_tkeep(panic_rx_axis_tkeep),
    .m_rx_axis_tvalid(panic_rx_axis_tvalid),
    .m_rx_axis_tready(1),
    .m_rx_axis_tlast(panic_rx_axis_tlast)
);


axis_fifo_old #(
    .DEPTH(1024 * AXIS_KEEP_WIDTH),
    .DATA_WIDTH(AXIS_DATA_WIDTH),
    .KEEP_ENABLE(1),
    .KEEP_WIDTH(AXIS_KEEP_WIDTH),
    .LAST_ENABLE(1),
    .USER_ENABLE(0),
    .ID_ENABLE(0),
    .DEST_ENABLE(0),
    .FRAME_FIFO(0)
)
input_buffer_fifo (
    .clk(clk),
    .rst(rst),

    // AXI input
    .s_axis_tdata(fifo_rx_axis_tdata),
    .s_axis_tkeep(fifo_rx_axis_tkeep),
    .s_axis_tvalid(fifo_rx_axis_tvalid),
    .s_axis_tready(fifo_rx_axis_tready),
    .s_axis_tlast(fifo_rx_axis_tlast),

    // AXI output
    .m_axis_tdata(rx_axis_tdata),
    .m_axis_tkeep(rx_axis_tkeep),
    .m_axis_tvalid(rx_axis_tvalid),
    .m_axis_tready(rx_axis_tready),
    .m_axis_tlast(rx_axis_tlast)
);
reg [31:0] test_counter = 0;

reg [63:0] counter;
reg [63:0] c_counter;
reg [63:0] total_counter;
reg [63:0] cycle_counter;
reg [63:0] byte_counter;

reg [4:0] pattern_id;

reg [15:0] packet_len;
wire [15:0] header_length;
assign header_length = (packet_len)*64 - 14;

reg [7:0] flow_id;

reg if_next_valid;
always@(*) begin

    fifo_rx_axis_tvalid = 0;
    fifo_rx_axis_tlast = 0;
    if(start) begin
        if(if_next_valid) begin
            fifo_rx_axis_tvalid = 1;
            if(c_counter == 0) begin
                fifo_rx_axis_tdata = 512'h1514131211100F0E0D0C0B0A0908070605040302010081B90801020001006501A8C06401A8C0B7B5114000400000F2050045000855545352515AD5D4D3D20000; // udp header
                fifo_rx_axis_tdata[16*8 +: 8] = header_length[15:8];
                fifo_rx_axis_tdata[17*8 +: 8] = header_length[7:0];
                fifo_rx_axis_tdata[35*8 +: 8] = flow_id;
                fifo_rx_axis_tdata[0 +: 9] = counter;
                fifo_rx_axis_tkeep = {64{1'b1}};
            end
            else begin
                fifo_rx_axis_tdata = c_counter + counter;  
                fifo_rx_axis_tkeep = {64{1'b1}};
            end
            if(c_counter == packet_len-1) begin
                fifo_rx_axis_tkeep = {64{1'b1}};
                fifo_rx_axis_tlast <= 1;
            end
        end
    end
end
reg [63:0] int_hash;

// assign int_hash = ((counter >> 16 )^counter) * 16'h45d9f3b;

always@(posedge clk) begin
    if(rst) begin
        counter <= 1;
        c_counter <= 0;
        cycle_counter <= 0;
        flow_id <= 0;
        if_next_valid <= 0;
        total_counter <= 0;
    end
    else begin
        if(start) begin
            cycle_counter <= cycle_counter + 1;
            int_hash <=  $urandom;
            if_next_valid <=1;
        end
        if(start && fifo_rx_axis_tready && fifo_rx_axis_tvalid) begin
            c_counter<= c_counter+1;
            if(c_counter == packet_len-1) begin
                c_counter <= 0;
                counter <= counter + 1;
            end
        end
        if(start && fifo_rx_axis_tlast && fifo_rx_axis_tvalid && fifo_rx_axis_tready) begin
            if(counter%10 < 4) begin
                flow_id <= 0;
            end
            else begin
                flow_id <= 1;
            end
            // flow_id <= 1234;
        end
        
        if( (counter <= 5000)) begin
            if_next_valid <= 1;
        end
        else begin
            // $display($urandom % 128);
            if_next_valid <= 0;
        end
    end
end

reg [63:0] check_seq_counter;
reg [63:0] check_counter;
reg [2:0] pk_start;
always@(posedge clk) begin
    if(rst) begin
        check_counter <= 0;
        check_seq_counter <= 1;
        pk_start <= 1;
        byte_counter <= 0;
        panic_rx_axis_tready = 1;
    end
    // panic_rx_axis_tready = (($urandom % 128) > 100);
    if(panic_rx_axis_tvalid && panic_rx_axis_tready) begin
        check_counter <= check_counter +1;
        byte_counter <= byte_counter + 512/8;
        if(pk_start == 1) begin
            pk_start <= 2;
        end
        else if (pk_start == 2) begin
            // check_seq_counter = panic_rx_axis_tdata - check_counter;
            pk_start <= 0;
        end
        else if(!panic_rx_axis_tlast) begin
            if( panic_rx_axis_tkeep != {64{1'b1}}) begin
                $display("ERROR in compare %x with  %x",panic_rx_axis_tdata, check_counter + check_seq_counter);
            end
        end
        else if(panic_rx_axis_tlast) begin
            if(panic_rx_axis_tkeep != {64{1'b1}}) begin
                $display("ERROR in compare %x with  %x",panic_rx_axis_tdata, check_counter + check_seq_counter);
            end
            // $display("DEBUG, received: %d",panic_rx_axis_tdata);
            pk_start <= 1;
            check_counter <= 0;
            check_seq_counter <=check_seq_counter + 1; 
        end
    end

end

always @* begin
    // if(byte_counter >= 160000 * 64 || counter > 40000) begin
    //     rst = 1;
    //     start = 0;
        
    //     #1000;
    //     rst = 0;
    //     #600;
    //     start = 1;

    //     pattern_id = pattern_id + 1;
    //     if(pattern_id >= 4)
    //         $finish;
    //     // packet_len = packet_len * 2;
        
    //     $display("--------------\nRST, Start Sending %4dB packet", pattern_id );
    // end
    if(byte_counter >= 30000 * 64 || counter > 8000) begin
        rst = 1;
        start = 0;
        
        #1000;
        rst = 0;
        #600;
        start = 1;

        pattern_id = pattern_id + 1;
        if(pattern_id >= 7)
            $finish;
        // packet_len = packet_len * 2;
        
        $display("--------------\nRST, Start Sending %4dB packet", pattern_id );
    end
end

endmodule
