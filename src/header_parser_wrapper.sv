
`timescale 1ns / 1ps
`include "panic_define.v"

module header_parser_wrapper #
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
    parameter AXIS_USER_WIDTH = 1
)
(
    input  wire                       clk,
    input  wire                       rst,

    input  wire                        config_mat_en,
    input  wire [`MATCH_KEY_WIDTH-1:0] config_mat_key,
    input  wire [128-1:0]              config_mat_value,
    input  wire [`MAT_ADDR_WIDTH-1:0]  config_mat_addr,

    /*
    * Receive data 
    */
    input  wire [AXIS_DATA_WIDTH-1:0]           s_rx_axis_tdata,
    input  wire [AXIS_KEEP_WIDTH-1:0]           s_rx_axis_tkeep,
    input  wire                                 s_rx_axis_tvalid,
    output reg                                  s_rx_axis_tready,
    input  wire                                 s_rx_axis_tlast,

    /*
    * Send data 
    */
    output  wire [AXIS_DATA_WIDTH-1:0]          m_rx_axis_tdata,
    output  wire [AXIS_KEEP_WIDTH-1:0]          m_rx_axis_tkeep,
    output  wire                                m_rx_axis_tvalid,
    input   wire                                m_rx_axis_tready,
    output  wire                                m_rx_axis_tlast,

    /* out put packet descriptor*/
    output reg [`PANIC_DESC_WIDTH-1:0]          m_packet_desc,
    output reg                                  m_packet_desc_valid,
    input  wire                                  m_packet_desc_ready
);


reg [AXIS_DATA_WIDTH-1:0]      s_output_fifo_tdata;
reg [AXIS_KEEP_WIDTH-1:0]      s_output_fifo_tkeep;
reg                            s_output_fifo_tvalid;
wire                           s_output_fifo_tready;
reg                            s_output_fifo_tlast;


axis_fifo #(
    .DEPTH(64 * AXIS_KEEP_WIDTH),
    .DATA_WIDTH(AXIS_DATA_WIDTH),
    .KEEP_ENABLE(1),
    .KEEP_WIDTH(AXIS_KEEP_WIDTH),
    .LAST_ENABLE(1),
    .ID_ENABLE(0),
    .DEST_ENABLE(0),
    .USER_ENABLE(0),
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

    
    // AXI output
    .m_axis_tdata(m_rx_axis_tdata),
    .m_axis_tkeep(m_rx_axis_tkeep),
    .m_axis_tvalid(m_rx_axis_tvalid),
    .m_axis_tready(m_rx_axis_tready),
    .m_axis_tlast(m_rx_axis_tlast)
);


reg                                          parse_req;
wire                                         parse_ready;
wire                                         desc_valid;
wire                                         desc_ready;
wire [`PANIC_DESC_PRIO_SIZE -1 : 0]          desc_prio;
wire [`PANIC_DESC_CHAIN_SIZE -1 : 0]         desc_chain;
wire [`PANIC_DESC_TIME_SIZE - 1 : 0]         desc_time;
wire [`PANIC_DESC_LEN_SIZE - 1 : 0]          desc_pk_len;
wire [`PANIC_DESC_FLOW_SIZE -1 : 0]          desc_flow_id;

header_parser #
(
    .DATA_WIDTH(AXIS_DATA_WIDTH),
    .KEEP_WIDTH(AXIS_KEEP_WIDTH)
)
panic_header_parser(
    .clk(clk),
    .rst(rst),

    .config_mat_en(config_mat_en),
    .config_mat_key(config_mat_key),
    .config_mat_value(config_mat_value),
    .config_mat_addr(config_mat_addr),

    .s_axis_tdata(s_rx_axis_tdata),
    .s_axis_tkeep(s_rx_axis_tkeep),
    .s_axis_tvalid(parse_req),
    .s_axis_tlast(parse_req),
    .s_axis_tready(parse_ready),

    .m_desc_valid(desc_valid),
    .m_desc_prio(desc_prio),
    .m_desc_chain(desc_chain),
    .m_desc_time(desc_time),
    .m_desc_pk_len(desc_pk_len),
    .m_desc_flow_id(desc_flow_id),
    .m_desc_ready(desc_ready)

);

reg [1:0] parser_state;
localparam PK_PARSE_STATE = 0;
localparam PK_DATA_STATE = 1;

always @(posedge clk) begin
    if(rst) begin
        parser_state <= PK_PARSE_STATE;
    end
    else begin
        if(parser_state == PK_PARSE_STATE && s_rx_axis_tvalid && s_rx_axis_tready) begin
            if(s_rx_axis_tlast) begin
                parser_state <= PK_PARSE_STATE;
            end
            else begin
                parser_state <= PK_DATA_STATE;
            end
        end
        else if(parser_state == PK_DATA_STATE && s_rx_axis_tlast && s_rx_axis_tvalid && s_rx_axis_tready) begin
            parser_state <= PK_PARSE_STATE;
        end
    end
end

assign desc_ready = m_packet_desc_ready;
always @* begin
    s_output_fifo_tvalid = s_rx_axis_tvalid;
    s_output_fifo_tdata = s_rx_axis_tdata;
    s_output_fifo_tkeep = s_rx_axis_tkeep;
    s_output_fifo_tlast = s_rx_axis_tlast;
    parse_req = 0;
    if(parser_state == PK_PARSE_STATE) begin
        s_rx_axis_tready = s_output_fifo_tready && parse_ready;
        if(s_rx_axis_tvalid && s_rx_axis_tready) begin
            parse_req = 1;
        end
    end
    else begin
        s_rx_axis_tready = s_output_fifo_tready;
    end

    m_packet_desc_valid = 0;
    m_packet_desc = 0;
    if(desc_valid && desc_ready) begin
        m_packet_desc_valid = 1;
        m_packet_desc[`PANIC_DESC_LEN_OF   +: `PANIC_DESC_LEN_SIZE]  = desc_pk_len;
        m_packet_desc[`PANIC_DESC_PRIO_OF  +: `PANIC_DESC_PRIO_SIZE] =  desc_prio; // calculate priority, which is time stamp here
        m_packet_desc[`PANIC_DESC_CHAIN_OF +: `PANIC_DESC_CHAIN_SIZE]  = desc_chain;   // current service  to node 1 then node 2
        m_packet_desc[`PANIC_DESC_TIME_OF  +: `PANIC_DESC_TIME_SIZE]  = desc_time;   // current service just to node 1
        m_packet_desc[`PANIC_DESC_FLOW_OF +: `PANIC_DESC_FLOW_SIZE] = desc_flow_id;
    end
end


endmodule
