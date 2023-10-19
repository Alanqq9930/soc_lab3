module fir 
#(  parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32,
    parameter Tape_Num    = 11
)
(
    output  wire                     awready,
    output  wire                     wready,
    input   wire                     awvalid,
    input   wire [(pADDR_WIDTH-1):0] awaddr,
    input   wire                     wvalid,
    input   wire [(pDATA_WIDTH-1):0] wdata,
    output  wire                     arready,
    input   wire                     rready,
    input   wire                     arvalid,
    input   wire [(pADDR_WIDTH-1):0] araddr,
    output  wire                     rvalid,
    output  wire [(pDATA_WIDTH-1):0] rdata,    

    input   wire                     ss_tvalid, 
    input   wire [(pDATA_WIDTH-1):0] ss_tdata, 
    input   wire                     ss_tlast, 
    output  wire                     ss_tready, 

    input   wire                     sm_tready, 
    output  wire                     sm_tvalid, 
    output  wire [(pDATA_WIDTH-1):0] sm_tdata, 
    output  wire                     sm_tlast, 
    
    // bram for tap RAM
    output  wire [3:0]               tap_WE,
    output  wire                     tap_EN,
    output  wire [(pDATA_WIDTH-1):0] tap_Di,
    output  wire [(pADDR_WIDTH-1):0] tap_A,
    input   wire [(pDATA_WIDTH-1):0] tap_Do,

    // bram for data RAM
    output  wire [3:0]               data_WE,
    output  wire                     data_EN,
    output  wire [(pDATA_WIDTH-1):0] data_Di,
    output  wire [(pADDR_WIDTH-1):0] data_A,
    input   wire [(pDATA_WIDTH-1):0] data_Do,

    input   wire                     axis_clk,
    input   wire                     axis_rst_n
);

////FSM
parameter idle = 2'd0;
parameter start = 2'd1;
parameter complete = 2'd2;
parameter done = 2'd3;

reg [1:0] state_current,state_next;
reg [3:0] cnt_cal;
reg [9:0] cnt_fir;
reg [9:0] data_length;
reg buffer_empty;

////axi_lite control
reg AW_ready,W_ready,AR_ready,R_valid;
reg AW_handshake,W_handshake,AR_handshake,R_handshake;
reg [pADDR_WIDTH-1:0] AW_addr,AR_addr;
reg [pDATA_WIDTH-1:0] W_data,R_data; 

////bram control
reg [3:0] TAP_WE_w;
reg TAP_EN_w;
reg [(pDATA_WIDTH-1):0] TAP_DI_w;
reg [(pDATA_WIDTH-1):0] TAP_ADDR_w;

reg [(pADDR_WIDTH-1):0] DATA_ADDR_w;
reg DATA_EN_w;
reg [3:0] DATA_WE_w;
reg [(pDATA_WIDTH-1):0] DATA_DI_w;
reg [pADDR_WIDTH-1:0] DATA_ADDR_read;

////fir control
reg signed [(pDATA_WIDTH-1):0] cal_out;
wire signed [(pDATA_WIDTH-1):0] data_cal;

////output buffer
reg [(pDATA_WIDTH-1):0] buffer_data; 

////axi_stream control
reg SS_ready_w;
reg SM_valid_w,SM_last_w;
reg [pDATA_WIDTH-1 : 0] SM_data_w;
reg [3:0] cnt_clear;
reg signed [(pDATA_WIDTH-1):0] SS_data;
assign ss_tready = SS_ready_w;

////register
reg [2:0] ap_coe_r,ap_coe_w;
wire ap_start = ap_coe_r[0];
wire ap_idle = ap_coe_r[1];
wire ap_done = ap_coe_r[2];

assign data_cal = (cnt_cal == 1) ? SS_data : (cnt_cal < 12) ? data_Do : 0;
assign awready = AW_ready;
assign wready = W_ready;
assign arready = AR_ready;
assign rvalid = R_valid;
assign rdata = R_data;
assign sm_tvalid = SM_valid_w;
assign sm_tdata = SM_data_w;
assign sm_tlast = SM_last_w;

////FSM
always @(posedge axis_clk or negedge axis_rst_n) begin
    if(!axis_rst_n)
        state_current <= 2'd0;
    else
        state_current <= state_next;
end

always @(*) begin
    state_next = state_current;
    case(state_current)
    idle : begin
        if(ap_start)
            state_next = start;
        end
    start : begin
        if(cnt_fir == 10'd600)
            state_next = complete;
    end
    complete : begin
        if(buffer_empty)
            state_next = done;
    end
    done : begin
        state_next = idle;
    end
    endcase
end

////axi lite
always@(posedge axis_clk or negedge axis_rst_n) begin
    if(!axis_rst_n) begin
        AW_ready <= 0;
        W_ready <= 0;
        AR_ready <= 0;
        R_valid <= 0;
        AW_handshake <= 0;
        W_handshake <= 0;
        AR_handshake <= 0;
        R_handshake <= 0;
        AW_addr <= 0;
        W_data <= 0;
        AR_addr <= 0;
    end
    else begin
        if(!(awvalid && awready) && !AW_handshake)
            AW_ready <= 1;
        else 
            AW_ready <= 0;

        if(!(wvalid && wready) && !W_handshake)
            W_ready <= 1;
        else
            W_ready <= 0;
        
        if(!(arvalid && arready) && !AR_handshake)
            AR_ready <= 1;
        else 
            AR_ready <= 0;

        if(AR_handshake && R_handshake)
            R_valid <= 0;
        else if(AR_handshake && !rvalid)
            R_valid <= 1;
        else if(rvalid && rready)
            R_valid <= 0;
        else 
            R_valid <= R_valid;

        if(AW_handshake && W_handshake)
            AW_handshake <= 0;
        else if(awvalid && awready)
            AW_handshake <= 1;
        
        if(AW_handshake && W_handshake)
            W_handshake <= 0;
        else if(wvalid && wready)
            W_handshake <= 1;
        
        if(AR_handshake && R_handshake)
            AR_handshake <= 0;
        else if(arvalid && arready)
            AR_handshake <= 1;

        if(AR_handshake && R_handshake)
            R_handshake <= 0;
        else if(rvalid && rready)
            R_handshake <= 1;

        if(awvalid && awready)
            AW_addr <= awaddr;

        if(wvalid && wready)
            W_data <= wdata;
        else if(state_current == done)
            W_data <= 0;

        if(arvalid && arready)
            AR_addr <= araddr;
    end
end

always @(*) begin
    R_data = 0;
    if(AR_handshake) begin
        case(AR_addr)
        12'h00 : begin
            R_data = {29'd0,ap_coe_r};
        end
        12'h10,
        12'h11,
        12'h12,
        12'h13,
        12'h14 : begin
            R_data = {22'd0,data_length};
        end
        default : begin
            R_data = tap_Do;
        end
        endcase
    end
end

////register
always @(posedge axis_clk or negedge axis_rst_n) begin
    if(!axis_rst_n) begin
        ap_coe_r <= 3'b100;
        data_length <= 0;
    end
    else begin
        ap_coe_r <= ap_coe_w;
        if(AW_handshake && W_handshake) begin
            if(AW_addr > 12'h9 && AW_addr < 12'h15)
                data_length <= W_data;
        end
    end
end

always @(*) begin
    ap_coe_w = ap_coe_r;
    if(AW_handshake && W_handshake && (AW_addr == 12'h0) && (state_current == idle)) begin //ap_start
        ap_coe_w[0] = W_data;
    end
    else begin
        ap_coe_w[0] = 0;
    end

    if(state_current == done) begin //ap_done
        if(rvalid && rready && AR_addr == 12'h00 && ap_coe_r[1])
            ap_coe_w[1] = 0;
        else
            ap_coe_w[1] = 1;
    end
    else if(state_current == idle) begin
        if(rvalid && rready && AR_addr == 12'h00 && ap_coe_r[1])
            ap_coe_w[1] = 0;
        else
            ap_coe_w[1] = ap_coe_r[1];
    end
    else 
        ap_coe_w[1] = 0;

    if(cnt_fir == 599 && cnt_cal == 10) //ap_idle
        ap_coe_w[2] = 1;
    else if(rvalid && rready && AR_addr == 12'h00 && ap_coe_r[2])
        ap_coe_w[2] = 0;
    else if(state_current == idle && state_next == start)
        ap_coe_w[2] = 0;
     
end

////bram
assign tap_WE = TAP_WE_w;
assign tap_EN = TAP_EN_w;
assign tap_Di = TAP_DI_w;
assign tap_A = TAP_ADDR_w;
// assign tap_Do = TAP_DO_w;
assign data_Di = DATA_DI_w;
assign data_A = DATA_ADDR_w;
assign data_WE = DATA_WE_w;
assign data_EN = DATA_EN_w;

reg [pADDR_WIDTH-1:0] DATA_ADDR_WP,DATA_ADDR_RP;

always @(posedge axis_clk or negedge axis_rst_n) begin
    if(!axis_rst_n) begin
        DATA_ADDR_WP <= 12'd40;
    end
    else if(cnt_cal == 5'd11) begin
        if(DATA_ADDR_WP == 12'd0) begin
            DATA_ADDR_WP <= 40;
        end
        else begin
            DATA_ADDR_WP <= DATA_ADDR_WP - 4;
        end
    end
end

always @(*) begin
    if(DATA_ADDR_WP + ( (cnt_cal) << 2) > 40)
        DATA_ADDR_read = DATA_ADDR_WP + ( (cnt_cal ) << 2) - 44;
    else
        DATA_ADDR_read = DATA_ADDR_WP + ( (cnt_cal ) << 2);
end

always @(*) begin
    TAP_WE_w = 0;
    TAP_EN_w = 0;
    TAP_ADDR_w = 0;
    TAP_DI_w = 0;
    DATA_WE_w = 0;
    DATA_EN_w = 0;
    DATA_DI_w = 0;
    DATA_ADDR_w = 0;

    if(state_current == start) begin
        TAP_ADDR_w = cnt_cal << 2;
        TAP_EN_w = 1;

        if(cnt_clear < 4'd11) begin
            DATA_ADDR_w = cnt_clear << 2 ;
            DATA_DI_w = 0;
            DATA_WE_w = 4'b1111;
            DATA_EN_w = 1;
        end
        else if(cnt_cal == 5'd11) begin
            DATA_ADDR_w = DATA_ADDR_WP;
            DATA_DI_w = SS_data;
            DATA_WE_w = 4'b1111;
            DATA_EN_w = 1;
        end
        else begin
            DATA_ADDR_w = DATA_ADDR_read;
            DATA_EN_w = 1;
        end
    end
    else begin
        TAP_EN_w = 1;
        if(AW_handshake && W_handshake && awaddr[1:0] == 2'd0 && awaddr >= 12'h20) begin ///coe write
            TAP_WE_w = 4'b1111;
            TAP_ADDR_w = AW_addr - 12'h20;
            TAP_DI_w = W_data;
        end
        else if(AR_handshake && araddr[1:0] == 2'd0 && araddr >= 12'h20) begin
            TAP_ADDR_w = AR_addr - 12'h20;
        end
    end
end
    
////fir
always @(posedge axis_clk or negedge axis_rst_n) begin
    if(!axis_rst_n) begin
        cal_out <= 0;
    end
    else if(cnt_cal == 0) begin
        cal_out <= 0;
    end
    else if(cnt_cal < 11) begin
        cal_out <= cal_out + tap_Do * data_cal;
    end
    else 
        cal_out <= 0;
end

////axi stream
always @(*) begin
    SS_ready_w = 0;
    SM_valid_w = 0;
    SM_data_w = 0;
    SM_last_w = 0;
    
    if(state_current == start && cnt_cal == 5'd0 && cnt_clear == 4'd11) begin
        SS_ready_w = 1;
    end

    if(!buffer_empty)
        SM_valid_w = 1;

    if(!buffer_empty)
        SM_data_w = buffer_data;

    // if(cnt_fir == 600 && sm_tvalid)
    //     SM_last_w = 1;
    if((cnt_fir == 600 || cnt_fir == 0) && sm_tvalid)
        SM_last_w = sm_tvalid;
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if(!axis_rst_n) begin
        SS_data <= 0;
    end
    else if(cnt_cal == 0 && ss_tready && ss_tvalid) begin
        SS_data <= ss_tdata;
    end
end

////counter
always @(posedge axis_clk or negedge axis_rst_n) begin
    if(!axis_rst_n) begin
        cnt_cal <= 0;
        cnt_fir <= 0;
    end
    else if(state_current != start) begin
        cnt_cal <= 0;
        cnt_fir <= 0;
    end
    else if(cnt_cal == 0 && cnt_clear != 4'd11) begin
        cnt_cal <= 0;
    end
    else if(cnt_cal == 0 && !ss_tvalid ) begin
        cnt_cal <= 0;
    end
    else if(cnt_cal == 5'd11 && buffer_empty) begin
        cnt_cal <= 0;
        cnt_fir <= cnt_fir + 1;
    end
    else if(cnt_cal == 5'd11 && !buffer_empty) begin
        cnt_cal <= 5'd11;
    end
    else begin
        cnt_cal <= cnt_cal + 1;
    end
end


always @(posedge axis_clk or negedge axis_rst_n) begin
    if(!axis_rst_n) begin
        cnt_clear <= 0;
    end
    else if(state_current == idle && state_next == start) begin
        cnt_clear <= 0;
    end
    else if(state_current == idle) begin
        cnt_clear <= 0;
    end
    else if(cnt_clear == 4'd11) begin
        cnt_clear <= cnt_clear;
    end
    else begin
        cnt_clear <= cnt_clear + 1;
    end
end

////output buffer
always @(posedge axis_clk or negedge axis_rst_n) begin
    if(!axis_rst_n) begin
        buffer_data <= 0;
        buffer_empty <= 1;
    end
    else begin
        if(cnt_cal == 11 && buffer_empty) begin
            buffer_data <= cal_out;
            buffer_empty <= 0;
        end
        else if(sm_tready && !buffer_empty) begin //withdraw
            buffer_empty <= 1;
        end
    end
end

endmodule