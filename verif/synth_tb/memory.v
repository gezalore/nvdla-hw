
`include "syn_tb_defines.vh"

module slave_mem #(
  parameter logic [`AXI_ADDR_WIDTH-1:0] MEM_ADDR_START, // Base address
  parameter logic [`AXI_ADDR_WIDTH-1:0] TOTAL_MEM_SIZE  // Size
) (
  input wire logic clk,
  input wire logic reset,

  input wire logic                          slave2mem_cmd_rd,
  input wire logic [`AXI_ADDR_WIDTH-1:0]    slave2mem_addr_rd,
  input wire logic [`AXI_LEN_WIDTH-1:0]     slave2mem_len_rd,
  input wire logic [`AXI_SIZE_WIDTH-1:0]    slave2mem_size_rd,
  output logic                              mem2slave_rd_ready,

  input wire logic                          slave2mem_cmd_wr,
  input wire logic [`AXI_ADDR_WIDTH-1:0]    slave2mem_addr_wr,
  input wire logic [`WORD_SIZE-1:0]         slave2mem_data,
  input wire logic [`WORD_BYTES-1:0]        slave2mem_wr_mask,
  input wire logic [`AXI_LEN_WIDTH-1:0]     slave2mem_len_wr,
  input wire logic [`AXI_SIZE_WIDTH-1:0]    slave2mem_size_wr,
  output logic                              mem2slave_wr_ready,

  output logic                              mem2slave_rdresp_vld,
  output logic [`WORD_SIZE-1:0]             mem2slave_rdresp_data,

  output logic                              mem2slave_wrresp_vld
);

logic [`MEM_WIDTH-1:0] memory[TOTAL_MEM_SIZE-1:0];

reg [`AXI_LEN_WIDTH-1:0] rd_remaining = '0;

assign mem2slave_rd_ready = rd_remaining == '0;
assign mem2slave_wr_ready = 1'b1;

logic [`AXI_ADDR_WIDTH-1:0] rd_addr;
logic [`AXI_ADDR_WIDTH-1:0] wr_addr;

always @(posedge clk) begin
  // Read
  mem2slave_rdresp_vld <= 1'b0;
  if (slave2mem_cmd_rd | (rd_remaining != '0)) begin
    if (rd_remaining == '0) begin
      rd_remaining <= slave2mem_len_rd;
      rd_addr = slave2mem_addr_rd - MEM_ADDR_START;
    end else begin
      rd_remaining <= rd_remaining - 1;
      rd_addr += `WORD_SIZE/8;
    end
    if (rd_addr >= TOTAL_MEM_SIZE) $error("Read address out of range");
    if (rd_addr[5:0] != 6'b0) $error("Read addres is unaligned");
    if (slave2mem_size_rd != `AXI_SIZE_WIDTH'd6) $error("Non 6 rd size");
    mem2slave_rdresp_vld <= 1'b1;
    for (longint unsigned i = 0; i < `WORD_SIZE/`MEM_WIDTH; ++i) begin
      mem2slave_rdresp_data[`MEM_WIDTH*i +: `MEM_WIDTH] <= memory[rd_addr/`MEM_BYTES + i];
    end
  end

  // Write
  mem2slave_wrresp_vld <= 1'b0;
  if (slave2mem_cmd_wr) begin
    // Ignore len. We get a command for each beat...
    wr_addr = slave2mem_addr_wr - MEM_ADDR_START;
    if (wr_addr >= TOTAL_MEM_SIZE) $error("Write address out of range");
    if (wr_addr[5:0] != 6'b0) $error("Write addres is unaligned");
    if (slave2mem_size_wr != `AXI_SIZE_WIDTH'd6) $error("Non 6 wr size");
    mem2slave_wrresp_vld <= 1'b1;
    for (longint unsigned i = 0; i < `WORD_SIZE/`MEM_WIDTH; ++i) begin
      for (longint unsigned j = 0; j < `MEM_BYTES; ++j) begin
        if (slave2mem_wr_mask[`MEM_BYTES*i + j]) begin
          memory[wr_addr/`MEM_BYTES + i][8*j +: 8] = slave2mem_data[`MEM_WIDTH*i + 8*j +: 8];
        end
      end
    end
  end
end

endmodule


