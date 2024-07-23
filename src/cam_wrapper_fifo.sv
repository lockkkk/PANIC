module cam_wrapper_fifo #(
    parameter TABLE_SIZE = 16,
    parameter KEY_SIZE = 8,
    parameter VALUE_SIZE = 32,
    parameter UPDATE_USER_WIDTH = 4,
    parameter LOOKUP_USER_WIDTH = 4
)(

    input  wire                          clk,
    input  wire                          rst,
    // UPDATE = 1 MATCH + 1 SET
    input  wire  [KEY_SIZE -1:0]                 s_update_req_index,
    input  wire  [VALUE_SIZE-1:0]                s_update_req_data,
    input  wire                                  s_update_req_valid,
    output wire                                  s_update_req_ready,
    input  wire  [UPDATE_USER_WIDTH-1:0]         s_update_req_user,


    // LOOKUP = 1 MATCH 
    input  wire  [KEY_SIZE-1:0]                  s_lookup_req_index,
    input  wire                                  s_lookup_req_valid,
    output  wire                                 s_lookup_req_ready,
    input  wire  [LOOKUP_USER_WIDTH-1:0]         s_lookup_req_user,
    output  wire [VALUE_SIZE-1:0]                m_lookup_value_data,
    output  wire                                 m_lookup_value_valid,
    input wire                                   m_lookup_value_ready,
    output  wire  [LOOKUP_USER_WIDTH-1:0]        m_lookup_value_user
);

    wire  [KEY_SIZE -1:0]                 fifo_update_req_index;
    wire  [VALUE_SIZE-1:0]                fifo_update_req_data;
    wire                                  fifo_update_req_valid;
    wire                                  fifo_update_req_ready;
    wire  [UPDATE_USER_WIDTH-1:0]         fifo_update_req_user;

    wire  [KEY_SIZE-1:0]                  fifo_lookup_req_index;
    wire                                  fifo_lookup_req_valid;
    wire                                  fifo_lookup_req_ready;
    wire  [LOOKUP_USER_WIDTH-1:0]         fifo_lookup_req_user;

    wire  [VALUE_SIZE-1:0]                fifo_lookup_value_data;
    wire                                  fifo_lookup_value_valid;
    wire                                  fifo_lookup_value_ready;
    wire  [LOOKUP_USER_WIDTH-1:0]         fifo_lookup_value_user;

    axis_fifo #(
        .DEPTH(4),
        .DATA_WIDTH(KEY_SIZE + VALUE_SIZE + UPDATE_USER_WIDTH),
        .KEEP_ENABLE(0),
        .LAST_ENABLE(0),
        .ID_ENABLE(0),
        .DEST_ENABLE(0),
        .USER_ENABLE(0),
        .FRAME_FIFO(0)
    )
    update_fifo_index (
        .clk(clk),
        .rst(rst),
        // AXI input
        .s_axis_tdata({s_update_req_index, s_update_req_data, s_update_req_user}),
        .s_axis_tvalid(s_update_req_valid),
        .s_axis_tready(s_update_req_ready),

        // AXI output
        .m_axis_tdata({fifo_update_req_index, fifo_update_req_data, fifo_update_req_user}),
        .m_axis_tvalid(fifo_update_req_valid),
        .m_axis_tready(fifo_update_req_ready)
    );

    axis_fifo #(
        .DEPTH(4),
        .DATA_WIDTH(KEY_SIZE + LOOKUP_USER_WIDTH),
        .KEEP_ENABLE(0),
        .LAST_ENABLE(0),
        .ID_ENABLE(0),
        .DEST_ENABLE(0),
        .USER_ENABLE(0),
        .FRAME_FIFO(0)
    )
    lookupreq_fifo (
        .clk(clk),
        .rst(rst),
        // AXI input
        .s_axis_tdata({s_lookup_req_index, s_lookup_req_user}),
        .s_axis_tvalid(s_lookup_req_valid),
        .s_axis_tready(s_lookup_req_ready),

        // AXI output
        .m_axis_tdata({fifo_lookup_req_index, fifo_lookup_req_user}),
        .m_axis_tvalid(fifo_lookup_req_valid),
        .m_axis_tready(fifo_lookup_req_ready)
    );

    axis_fifo #(
        .DEPTH(4),
        .DATA_WIDTH(VALUE_SIZE + LOOKUP_USER_WIDTH),
        .KEEP_ENABLE(0),
        .LAST_ENABLE(0),
        .ID_ENABLE(0),
        .DEST_ENABLE(0),
        .USER_ENABLE(0),
        .FRAME_FIFO(0)
    )lookup_value_fifo (
        .clk(clk),
        .rst(rst),
        // AXI input
        .s_axis_tdata({fifo_lookup_value_data, fifo_lookup_value_user}),
        .s_axis_tvalid(fifo_lookup_value_valid),
        .s_axis_tready(fifo_lookup_value_ready),

        // AXI output
        .m_axis_tdata({m_lookup_value_data, m_lookup_value_user}),
        .m_axis_tvalid(m_lookup_value_valid),
        .m_axis_tready(m_lookup_value_ready)
    );
cam_wrapper #(
    .TABLE_SIZE(TABLE_SIZE),
    .KEY_SIZE(KEY_SIZE),
    .VALUE_SIZE(VALUE_SIZE),
    .UPDATE_USER_WIDTH(UPDATE_USER_WIDTH),
    .LOOKUP_USER_WIDTH(LOOKUP_USER_WIDTH)
)
cam_wrapper (
    .clk(clk),
    .rst(rst),
    // AXI input
    .update_req_index(fifo_update_req_index),
    .update_req_data(fifo_update_req_data),
    .update_req_valid(fifo_update_req_valid),
    .update_req_ready(fifo_update_req_ready),
    .update_req_user(fifo_update_req_user),

    // AXI output
    .lookup_req_index(fifo_lookup_req_index),
    .lookup_req_valid(fifo_lookup_req_valid),
    .lookup_req_ready(fifo_lookup_req_ready),
    .lookup_req_user(fifo_lookup_req_user),
    .lookup_value_data(fifo_lookup_value_data),
    .lookup_value_valid(fifo_lookup_value_valid),
    .lookup_value_ready(fifo_lookup_value_ready),
    .lookup_value_user(fifo_lookup_value_user)
);


endmodule