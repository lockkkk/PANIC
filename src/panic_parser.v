
`timescale 1ns / 1ps
`include "panic_define.v"

module panic_parser #
(
    // Width of AXI stream interfaces in bits
    parameter AXIS_DATA_WIDTH = 512,
    // Use AXI stream tkeep signal
    parameter AXIS_KEEP_ENABLE = (AXIS_DATA_WIDTH>8),
    // AXI stream tkeep signal width (words per cycle)
    parameter AXIS_KEEP_WIDTH = (AXIS_DATA_WIDTH/8),
    // Use AXI stream tlast signal
    parameter AXIS_LAST_ENABLE = 1,
    // Propagate AXI stream tid signal
    parameter AXIS_ID_ENABLE = 0,
    // AXI stream tid signal width
    parameter AXIS_ID_WIDTH = 8,
    // Propagate AXI stream tdest signal
    parameter AXIS_DEST_ENABLE = 0,
    // AXI stream tdest signal width
    parameter AXIS_DEST_WIDTH = 8,
    // Propagate AXI stream tuser signal
    parameter AXIS_USER_ENABLE = 0,
    // AXI stream tuser signal width
    parameter AXIS_USER_WIDTH = 1,
    
    // parameter AXI_ADDR_WIDTH = 16,
    parameter LEN_WIDTH = 16,

    parameter CELL_ID_WIDTH = 16,

    // crossbar data width, current 512, but need refine
    parameter SWITCH_DATA_WIDTH = 512,
    parameter SWITCH_KEEP_WIDTH = (SWITCH_DATA_WIDTH/8),
    // crossbar address width, if 5 is 32*32 crossbar
    parameter SWITCH_DEST_WIDTH = 3,
    // crossbar vcs number
    parameter SWITCH_USER_ENABLE = 1,  

    parameter SWITCH_USER_WIDTH = 1

)
(
    input  wire                       clk,
    input  wire                       rst,
    /*
    * Receive data from the wire
    */
    input  wire [AXIS_DATA_WIDTH-1:0]           s_rx_axis_tdata,
    input  wire [AXIS_KEEP_WIDTH-1:0]           s_rx_axis_tkeep,
    input  wire                                 s_rx_axis_tvalid,
    output wire                                 s_rx_axis_tready,
    input  wire                                 s_rx_axis_tlast,
    input  wire                                 s_rx_axis_tuser,

    /*
    * Send data output to the dma
    */
    output  wire [AXIS_DATA_WIDTH-1:0]          m_rx_axis_tdata,
    output  wire [AXIS_KEEP_WIDTH-1:0]          m_rx_axis_tkeep,
    output  wire                                m_rx_axis_tvalid,
    input   wire                                m_rx_axis_tready,
    output  wire                                m_rx_axis_tlast,
    output  wire                                m_rx_axis_tuser,

    /*memory address form the memory allocator*/
    output reg                                  alloc_mem_req,
    output reg  [LEN_WIDTH - 1 : 0]             alloc_mem_size,
    // input  wire [AXI_ADDR_WIDTH -1 : 0]         alloc_mem_addr,
    input  wire [CELL_ID_WIDTH - 1 : 0]         alloc_cell_id,
    input  wire                                 alloc_port_id,
    input  wire                                 alloc_mem_success,
    input  wire                                 alloc_mem_intense,

    /*
    * Crossbar interface input
    */
    input  wire [SWITCH_DATA_WIDTH-1:0]     s_switch_axis_tdata,
    input  wire [SWITCH_KEEP_WIDTH-1:0]     s_switch_axis_tkeep,
    input  wire                             s_switch_axis_tvalid,
    output wire                             s_switch_axis_tready,
    input  wire                             s_switch_axis_tlast,
    input  wire [SWITCH_DEST_WIDTH-1:0]     s_switch_axis_tdest,
    input  wire [SWITCH_USER_WIDTH-1:0]     s_switch_axis_tuser,


    /*
    * Crossbar interface output
    */
    output  wire [SWITCH_DATA_WIDTH-1:0]      m_switch_axis_tdata,
    output  wire [SWITCH_KEEP_WIDTH-1:0]      m_switch_axis_tkeep,
    output  wire                              m_switch_axis_tvalid,
    input   wire                              m_switch_axis_tready,
    output  wire                              m_switch_axis_tlast,
    output  wire [SWITCH_DEST_WIDTH-1:0]      m_switch_axis_tdest,
    output  wire [SWITCH_USER_WIDTH-1:0]      m_switch_axis_tuser,

    input  wire                        config_mat_en,
    input  wire [`MATCH_KEY_WIDTH-1:0] config_mat_key,
    input  wire [128-1:0]              config_mat_value,
    input  wire [`MAT_ADDR_WIDTH-1:0]  config_mat_addr,

    input   wire [`PANIC_DESC_TS_SIZE-1:0]    timestamp

);
reg [SWITCH_DATA_WIDTH-1:0]      s_output_fifo_tdata;
reg [SWITCH_KEEP_WIDTH-1:0]      s_output_fifo_tkeep;
reg                              s_output_fifo_tvalid;
wire                             s_output_fifo_tready;
reg                              s_output_fifo_tlast;
reg [SWITCH_DEST_WIDTH-1:0]      s_output_fifo_tdest;
reg [SWITCH_USER_WIDTH-1:0]      s_output_fifo_tuser;


axis_fifo #(
    .DEPTH(1024 * SWITCH_KEEP_WIDTH),
    .DATA_WIDTH(SWITCH_DATA_WIDTH),
    .KEEP_ENABLE(1),
    .KEEP_WIDTH(SWITCH_KEEP_WIDTH),
    .LAST_ENABLE(1),
    .ID_ENABLE(0),
    .DEST_ENABLE(1),
    .DEST_WIDTH(SWITCH_DEST_WIDTH),
    .USER_ENABLE(SWITCH_USER_ENABLE),
    .USER_WIDTH(SWITCH_USER_WIDTH),
    .FRAME_FIFO(0)
)
output_fifo (
    .clk(clk),
    .rst(rst),

    // AXI input
    .s_axis_tdata(s_output_fifo_tdata),
    .s_axis_tkeep(s_output_fifo_tkeep),
    .s_axis_tvalid(s_output_fifo_tvalid),
    .s_axis_tready(s_output_fifo_tready),
    .s_axis_tlast(s_output_fifo_tlast),
    .s_axis_tdest(s_output_fifo_tdest),
    .s_axis_tuser(s_output_fifo_tuser),
    
    // AXI output
    .m_axis_tdata(m_switch_axis_tdata),
    .m_axis_tkeep(m_switch_axis_tkeep),
    .m_axis_tvalid(m_switch_axis_tvalid),
    .m_axis_tready(m_switch_axis_tready),
    .m_axis_tlast(m_switch_axis_tlast),
    .m_axis_tdest(m_switch_axis_tdest),
    .m_axis_tuser(m_switch_axis_tuser)
);

wire [AXIS_DATA_WIDTH-1:0]          after_parse_axis_tdata;
wire [AXIS_KEEP_WIDTH-1:0]          after_parse_axis_tkeep;
wire                                after_parse_axis_tvalid;
reg                                 after_parse_axis_tready;
wire                                after_parse_axis_tlast;


wire [`PANIC_DESC_WIDTH-1:0]          packet_desc;
wire                                  packet_desc_valid;
reg                                   packet_desc_ready;

header_parser_wrapper #(
    .AXIS_DATA_WIDTH(AXIS_DATA_WIDTH),
    .AXIS_KEEP_WIDTH(AXIS_KEEP_WIDTH),
    .AXIS_LAST_ENABLE(AXIS_LAST_ENABLE),
    .AXIS_ID_ENABLE(AXIS_ID_ENABLE),
    .AXIS_DEST_ENABLE(AXIS_DEST_ENABLE),
    .AXIS_USER_ENABLE(AXIS_USER_ENABLE)
) header_parser_wrapper_inst (
    .clk(clk),
    .rst(rst),

    .config_mat_en(config_mat_en),
    .config_mat_key(config_mat_key),
    .config_mat_value(config_mat_value),
    .config_mat_addr(config_mat_addr),

    .s_rx_axis_tdata(s_rx_axis_tdata),
    .s_rx_axis_tkeep(s_rx_axis_tkeep),
    .s_rx_axis_tvalid(s_rx_axis_tvalid),
    .s_rx_axis_tready(s_rx_axis_tready),
    .s_rx_axis_tlast(s_rx_axis_tlast),

    .m_rx_axis_tdata(after_parse_axis_tdata),
    .m_rx_axis_tkeep(after_parse_axis_tkeep),
    .m_rx_axis_tvalid(after_parse_axis_tvalid),
    .m_rx_axis_tready(after_parse_axis_tready),
    .m_rx_axis_tlast(after_parse_axis_tlast),

    .m_packet_desc(packet_desc),
    .m_packet_desc_valid(packet_desc_valid),
    .m_packet_desc_ready(packet_desc_ready)
);

assign s_switch_axis_tready = 1;

reg [1:0] parser_state;
reg if_bypass;
reg [4:0] next_dest_port;
reg if_drop;

localparam PK_DESC_STATE = 1;
localparam PK_DATA_STATE = 2;
reg [`PANIC_DESC_CHAIN_SIZE -1 : 0]         desc_chain;
reg                                         parse_req; 
always @(posedge clk) begin
    if(rst) begin
        parser_state <= PK_DESC_STATE;
        if_bypass <= 0;
        if_drop <= 0;
        next_dest_port <= 0;
    end
    else begin
        if(parser_state == PK_DESC_STATE) begin
            if(packet_desc_ready && packet_desc_valid ) begin // TODO: bypass path no allocation
                parser_state <= PK_DATA_STATE;
                next_dest_port <= s_output_fifo_tdest;
                if(desc_chain == 0 )
                    if_bypass <= 1;
                else begin
                    if_bypass <= 0;

                    if(!alloc_mem_success) begin
                        if_drop <= 1;
                        $display("DROP from packet parser, no bypass path");
                    end
                end
            end
        end
        else if (parser_state == PK_DATA_STATE) begin
            if(after_parse_axis_tvalid && after_parse_axis_tready && after_parse_axis_tlast) begin
                parser_state <= PK_DESC_STATE;
            end
        end
    end
end 

integer i;
always @* begin
    s_output_fifo_tdata = 0;
    s_output_fifo_tvalid = 0;
    s_output_fifo_tkeep = 0;
    s_output_fifo_tuser = 0; 
    s_output_fifo_tdest = 0;
    s_output_fifo_tlast = 0;

    after_parse_axis_tready = 0;

    alloc_mem_req = 0;
    alloc_mem_size = 0;
    parse_req = 0;
    desc_chain = 0;

    packet_desc_ready = ((parser_state == PK_DESC_STATE) && s_output_fifo_tready);
    if(packet_desc_ready && packet_desc_valid) begin // packet header
        s_output_fifo_tdata = packet_desc;
        
        desc_chain = packet_desc[`PANIC_DESC_CHAIN_OF +: `PANIC_DESC_CHAIN_SIZE];
        if(desc_chain == 0 && s_output_fifo_tready) begin // bypass path
            s_output_fifo_tdata[`PANIC_DESC_CELL_ID_OF  +: `PANIC_DESC_CELL_ID_SIZE]  = alloc_cell_id;
            s_output_fifo_tdata[`PANIC_DESC_DROP_OF]                          = !alloc_mem_success;
            s_output_fifo_tvalid = 1;
            s_output_fifo_tkeep = {{(SWITCH_DATA_WIDTH - `PANIC_DESC_WIDTH)/8{1'd0}},{`PANIC_DESC_WIDTH/8{1'd1}}};
            s_output_fifo_tuser = 1;            // user = 1 means it does not alloc mem
            s_output_fifo_tdest = 1;
            s_output_fifo_tdata[`PANIC_DESC_TS_OF +: `PANIC_DESC_TS_SIZE]= timestamp;
        end
        else if(desc_chain !=0 && s_output_fifo_tready) begin // no bypass, go to scheduler
            alloc_mem_req = 1;
            alloc_mem_size = packet_desc[`PANIC_DESC_LEN_OF   +: `PANIC_DESC_LEN_SIZE];

            if(alloc_mem_success) begin
                s_output_fifo_tdata[`PANIC_DESC_CELL_ID_OF  +: `PANIC_DESC_CELL_ID_SIZE]  = alloc_cell_id;
                s_output_fifo_tdata[`PANIC_DESC_DROP_OF]                          = !alloc_mem_success;
                s_output_fifo_tdata[`PANIC_DESC_TS_OF +: `PANIC_DESC_TS_SIZE]= timestamp;
                s_output_fifo_tdata[`PANIC_DESC_PORT_OF]                           = alloc_port_id;

                s_output_fifo_tvalid = 1;
                s_output_fifo_tkeep = {{(SWITCH_DATA_WIDTH - `PANIC_DESC_WIDTH)/8{1'd0}},{`PANIC_DESC_WIDTH/8{1'd1}}};
                s_output_fifo_tuser = 0;            // user = 0 means it is data, not credit
                if(alloc_port_id == 0) begin
                    s_output_fifo_tdest = 0;
                end
                else begin
                    s_output_fifo_tdest = 3;
                end
            end

        end  
    end
    else if(parser_state ==PK_DATA_STATE) begin
        if(if_drop) begin
            after_parse_axis_tready = 1;
        end
        else begin
            s_output_fifo_tvalid = after_parse_axis_tvalid;
            s_output_fifo_tdata = after_parse_axis_tdata;
            s_output_fifo_tlast = after_parse_axis_tlast;
            if(if_bypass) begin
                s_output_fifo_tdest = next_dest_port;
                s_output_fifo_tuser = 1;
            end
            else begin
                s_output_fifo_tdest = next_dest_port;
                s_output_fifo_tuser = 0;
            end

            s_output_fifo_tkeep = after_parse_axis_tkeep;
            after_parse_axis_tready = s_output_fifo_tready;
        end

    end
end

// ila_panic_parser ila_panic_parser_inst (
// 	.clk(clk), // input wire clk


// 	.probe0(s_parser_desc_fifo_tready), // input wire [0:0] probe0  
// 	.probe1( {s_parser_desc_fifo_tdata[`PANIC_DESC_WIDTH-1:0],alloc_mem_intense,alloc_mem_success,desc_pk_len,desc_prio,desc_chain,desc_time}), // input wire [511:0]  probe1 
// 	.probe2( 0), // input wire [63:0]  probe2 
// 	.probe3( s_parser_desc_fifo_tvalid), // input wire [0:0]  probe3 
// 	.probe4( 1), // input wire [0:0]  probe4 
// 	.probe5( 0), // input wire [0:0]  probe5 
// 	.probe6( {{64{1'b1}}}), // input wire [63:0]  probe6 
// 	.probe7( {{3{1'b1}}}), // input wire [2:0]  probe7  
// 	.probe8( 0) // input wire [0:0]  probe8
// );


endmodule
