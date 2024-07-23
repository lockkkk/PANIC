`timescale 1ns / 1ps
`include "panic_define.v"

module header_parser #
(
    // Width of AXI stream interfaces in bits
    parameter DATA_WIDTH = 256,
    // AXI stream tkeep signal width (words per cycle)
    parameter KEEP_WIDTH = (DATA_WIDTH/8)
)
(
    input  wire                       clk,
    input  wire                       rst,

    input  wire                        config_mat_en,
    input  wire [`MATCH_KEY_WIDTH-1:0] config_mat_key,
    input  wire [128-1:0]              config_mat_value,
    input  wire [`MAT_ADDR_WIDTH-1:0]  config_mat_addr,

    /*
     * AXI input
     */
    input  wire [DATA_WIDTH-1:0]  s_axis_tdata,
    input  wire [KEEP_WIDTH-1:0]  s_axis_tkeep,
    input  wire                   s_axis_tvalid,
    input  wire                   s_axis_tlast,
    output wire                   s_axis_tready,

    output reg [`PANIC_DESC_PRIO_SIZE -1 : 0]          m_desc_prio,
    output reg [`PANIC_DESC_CHAIN_SIZE -1 : 0]         m_desc_chain,
    output reg [`PANIC_DESC_TIME_SIZE - 1 : 0]         m_desc_time,
    output reg [`PANIC_DESC_LEN_SIZE - 1 : 0]          m_desc_pk_len,
    output reg [`PANIC_DESC_FLOW_SIZE -1 : 0]          m_desc_flow_id,
    output wire                                        m_desc_ready,
    output wire                                        m_desc_valid
);

/*
TCP/UDP Frame (IPv4)
 Field                       Length
 Destination MAC address     6 octets
 Source MAC address          6 octets
 Ethertype (0x0800)          2 octets
 Version (4)                 4 bits
 IHL (5-15)                  4 bits
 DSCP (0)                    6 bits
 ECN (0)                     2 bits
 length                      2 octets
 identification (0?)         2 octets
 flags (010)                 3 bits
 fragment offset (0)         13 bits
 time to live (64?)          1 octet
 protocol (6 or 17)          1 octet
 header checksum             2 octets
 source IP                   4 octets
 destination IP              4 octets
 options                     (IHL-5)*4 octets
 source port                 2 octets
 desination port             2 octets
 other fields + payload
TCP/UDP Frame (IPv6)
 Field                       Length
 Destination MAC address     6 octets
 Source MAC address          6 octets
 Ethertype (0x86dd)          2 octets
 Version (4)                 4 bits
 Traffic class               8 bits
 Flow label                  20 bits
 length                      2 octets
 next header (6 or 17)       1 octet
 hop limit                   1 octet
 source IP                   16 octets
 destination IP              16 octets
 source port                 2 octets
 desination port             2 octets
 other fields + payload
*/
parameter CYCLE_COUNT = (38+KEEP_WIDTH-1)/KEEP_WIDTH;


reg [15:0] eth_type_next;
reg [3:0] ihl_next;

reg [4*8-1:0] ip_src_next;
reg [4*8-1:0] ip_dest_next;
reg [4*8-1:0] ip_len_next;
reg [2*8-1:0] port_src_next;
reg [2*8-1:0] port_dest_next;


reg ipv4_next;
reg tcp_next;
reg udp_next;

always @* begin
    eth_type_next = 0;
    ipv4_next = 0;
    tcp_next = 0;
    udp_next = 0;
    
    port_src_next = 0;
    port_dest_next = 0;
    ip_src_next   = 0;
    ip_dest_next   = 0;
    ip_len_next   = 0;
    ihl_next = 0;

    
    if(s_axis_tvalid) begin
        eth_type_next = 1'b0;
        ipv4_next = 1'b0;
        tcp_next = 1'b0;
        udp_next = 1'b0;
        
        port_src_next = 0;
        port_dest_next = 0;
        ip_src_next   = 0;
        ip_dest_next   = 0;
        ihl_next = 0;

        eth_type_next[15:8] = s_axis_tdata[(12%KEEP_WIDTH)*8 +: 8];
        eth_type_next[7:0] = s_axis_tdata[(13%KEEP_WIDTH)*8 +: 8];
        if (eth_type_next == 16'h0800) begin
            // ipv4
            ipv4_next = 1'b1;
        end 
        ihl_next = s_axis_tdata[(14%KEEP_WIDTH)*8 +: 8];
        if (ipv4_next) begin

            ip_len_next[15:8] = s_axis_tdata[(16%KEEP_WIDTH)*8 +: 8];
            ip_len_next[7:0]  = s_axis_tdata[(17%KEEP_WIDTH)*8 +: 8];

            if (s_axis_tdata[(23%KEEP_WIDTH)*8 +: 8] == 8'h06 && ihl_next == 5) begin
                // TCP
                tcp_next = 1'b1;
            end else if (s_axis_tdata[(23%KEEP_WIDTH)*8 +: 8] == 8'h11 && ihl_next == 5) begin
                // UDP
                udp_next = 1'b1;
            end
            ip_src_next[31:24] = s_axis_tdata[(26%KEEP_WIDTH)*8 +: 8];
            ip_src_next[23:16] = s_axis_tdata[(27%KEEP_WIDTH)*8 +: 8];
            ip_src_next[15:8] = s_axis_tdata[(28%KEEP_WIDTH)*8 +: 8];
            ip_src_next[7:0] = s_axis_tdata[(29%KEEP_WIDTH)*8 +: 8];

            ip_dest_next[31:24] = s_axis_tdata[(30%KEEP_WIDTH)*8 +: 8];
            ip_dest_next[23:16] = s_axis_tdata[(31%KEEP_WIDTH)*8 +: 8];
            ip_dest_next[15:8] = s_axis_tdata[(32%KEEP_WIDTH)*8 +: 8];
            ip_dest_next[7:0] = s_axis_tdata[(33%KEEP_WIDTH)*8 +: 8];

            if (tcp_next || udp_next) begin
               port_src_next[15:8] = s_axis_tdata[(34%KEEP_WIDTH)*8 +: 8];
               port_src_next[7:0] = s_axis_tdata[(35%KEEP_WIDTH)*8 +: 8];
               port_dest_next[15:8] = s_axis_tdata[(36%KEEP_WIDTH)*8 +: 8];
               port_dest_next[7:0] = s_axis_tdata[(37%KEEP_WIDTH)*8 +: 8];
            end

        end
    end
end

wire lookup_value_valid;
wire [`MAT_VALUE_SIZE-1 : 0] lookup_value_data;
wire [32-1 : 0] lookup_value_user;

cam_wrapper_fifo#(
.TABLE_SIZE(`MAT_SIZE),
.KEY_SIZE(`MATCH_KEY_WIDTH),
.VALUE_SIZE(`MAT_VALUE_SIZE),
.LOOKUP_USER_WIDTH(32)
)mat_table(
	 .clk(clk), 
	 .rst(rst) ,
	//lookup port 
	.s_lookup_req_index(port_src_next),
	.s_lookup_req_valid((s_axis_tvalid && udp_next)),
	.s_lookup_req_ready(s_axis_tready), 
    .s_lookup_req_user(ip_len_next + 14),
	.m_lookup_value_valid(m_desc_valid),
	.m_lookup_value_data(lookup_value_data),
	.m_lookup_value_ready(m_desc_ready),
    .m_lookup_value_user(lookup_value_user),


	//update port 
	.s_update_req_index(config_mat_key),
	.s_update_req_data(config_mat_value),
	.s_update_req_valid(config_mat_en),
	.s_update_req_ready() // should never be low
);

always @*begin
    m_desc_prio = 1; // any value
    m_desc_time = 2; // any value
    m_desc_chain = 0;
    m_desc_pk_len = lookup_value_user; // add eth packet header
    if(m_desc_valid && m_desc_ready) begin
        m_desc_prio = lookup_value_data[`PANIC_DESC_PRIO_OF +: `PANIC_DESC_PRIO_SIZE];
        m_desc_time = lookup_value_data[`PANIC_DESC_TIME_OF +: `PANIC_DESC_TIME_SIZE];
        m_desc_chain = lookup_value_data[`PANIC_DESC_CHAIN_OF +: `PANIC_DESC_CHAIN_SIZE];
        m_desc_flow_id = lookup_value_data[`PANIC_DESC_FLOW_OF +: `PANIC_DESC_FLOW_SIZE];
    end
end

// ila_parser hearder_parser_debug (
// 	.clk(clk), // input wire clk

// 	.probe0(1), // input wire [0:0] probe0  
// 	.probe1({s_axis_tvalid,s_axis_tlast,m_desc_prio, m_desc_time, m_desc_chain, ip_src_next, ip_dest_next, port_src_next, port_dest_next }), // input wire [127:0]  probe1 
// 	.probe2( 0), // input wire [15:0]  probe2 
// 	.probe3(s_axis_tlast && s_axis_tvalid), // input wire [0:0]  probe3 
// 	.probe4( 1), // input wire [0:0]  probe4 
// 	.probe5( ipv4_next), // input wire [0:0]  probe5 
// 	.probe6( {{16{1'b1}}}), // input wire [15:0]  probe6 
// 	.probe7( tcp_next), // input wire [0:0]  probe7  
// 	.probe8( udp_next) // input wire [0:0]  probe8
// );

endmodule
