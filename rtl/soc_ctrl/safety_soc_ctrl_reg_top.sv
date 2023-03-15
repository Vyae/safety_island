// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Register Top module auto-generated by `reggen`


`include "common_cells/assertions.svh"

module safety_soc_ctrl_reg_top #(
  parameter type reg_req_t = logic,
  parameter type reg_rsp_t = logic,
  parameter int AW = 4,
  parameter int unsigned BootAddrDefault = 32'h0
) (
  input logic clk_i,
  input logic rst_ni,
  input  reg_req_t reg_req_i,
  output reg_rsp_t reg_rsp_o,
  // To HW
  output safety_soc_ctrl_reg_pkg::safety_soc_ctrl_reg2hw_t reg2hw, // Write


  // Config
  input devmode_i // If 1, explicit error return for unmapped register access
);

  import safety_soc_ctrl_reg_pkg::* ;

  localparam int DW = 32;
  localparam int DBW = DW/8;                    // Byte Width

  // register signals
  logic           reg_we;
  logic           reg_re;
  logic [AW-1:0]  reg_addr;
  logic [DW-1:0]  reg_wdata;
  logic [DBW-1:0] reg_be;
  logic [DW-1:0]  reg_rdata;
  logic           reg_error;

  logic          addrmiss, wr_err;

  logic [DW-1:0] reg_rdata_next;

  // Below register interface can be changed
  reg_req_t  reg_intf_req;
  reg_rsp_t  reg_intf_rsp;


  assign reg_intf_req = reg_req_i;
  assign reg_rsp_o = reg_intf_rsp;


  assign reg_we = reg_intf_req.valid & reg_intf_req.write;
  assign reg_re = reg_intf_req.valid & ~reg_intf_req.write;
  assign reg_addr = reg_intf_req.addr;
  assign reg_wdata = reg_intf_req.wdata;
  assign reg_be = reg_intf_req.wstrb;
  assign reg_intf_rsp.rdata = reg_rdata;
  assign reg_intf_rsp.error = reg_error;
  assign reg_intf_rsp.ready = 1'b1;

  assign reg_rdata = reg_rdata_next ;
  assign reg_error = (devmode_i & addrmiss) | wr_err;


  // Define SW related signals
  // Format: <reg>_<field>_{wd|we|qs}
  //        or <reg>_{wd|we|qs} if field == 1 or 0
  logic [31:0] bootaddr_qs;
  logic [31:0] bootaddr_wd;
  logic bootaddr_we;
  logic fetchen_qs;
  logic fetchen_wd;
  logic fetchen_we;
  logic [31:0] corestatus_qs;
  logic [31:0] corestatus_wd;
  logic corestatus_we;

  // Register instances
  // R[bootaddr]: V(False)

  prim_subreg #(
    .DW      (32),
    .SWACCESS("RW"),
    .RESVAL  (BootAddrDefault)
  ) u_bootaddr (
    .clk_i   (clk_i    ),
    .rst_ni  (rst_ni  ),

    // from register interface
    .we     (bootaddr_we),
    .wd     (bootaddr_wd),

    // from internal hardware
    .de     (1'b0),
    .d      ('0  ),

    // to internal hardware
    .qe     (),
    .q      (reg2hw.bootaddr.q ),

    // to register interface (read)
    .qs     (bootaddr_qs)
  );


  // R[fetchen]: V(False)

  prim_subreg #(
    .DW      (1),
    .SWACCESS("RW"),
    .RESVAL  (1'h0)
  ) u_fetchen (
    .clk_i   (clk_i    ),
    .rst_ni  (rst_ni  ),

    // from register interface
    .we     (fetchen_we),
    .wd     (fetchen_wd),

    // from internal hardware
    .de     (1'b0),
    .d      ('0  ),

    // to internal hardware
    .qe     (),
    .q      (reg2hw.fetchen.q ),

    // to register interface (read)
    .qs     (fetchen_qs)
  );


  // R[corestatus]: V(False)

  prim_subreg #(
    .DW      (32),
    .SWACCESS("RW"),
    .RESVAL  (32'h0)
  ) u_corestatus (
    .clk_i   (clk_i    ),
    .rst_ni  (rst_ni  ),

    // from register interface
    .we     (corestatus_we),
    .wd     (corestatus_wd),

    // from internal hardware
    .de     (1'b0),
    .d      ('0  ),

    // to internal hardware
    .qe     (),
    .q      (reg2hw.corestatus.q ),

    // to register interface (read)
    .qs     (corestatus_qs)
  );




  logic [2:0] addr_hit;
  always_comb begin
    addr_hit = '0;
    addr_hit[0] = (reg_addr == SAFETY_SOC_CTRL_BOOTADDR_OFFSET);
    addr_hit[1] = (reg_addr == SAFETY_SOC_CTRL_FETCHEN_OFFSET);
    addr_hit[2] = (reg_addr == SAFETY_SOC_CTRL_CORESTATUS_OFFSET);
  end

  assign addrmiss = (reg_re || reg_we) ? ~|addr_hit : 1'b0 ;

  // Check sub-word write is permitted
  always_comb begin
    wr_err = (reg_we &
              ((addr_hit[0] & (|(SAFETY_SOC_CTRL_PERMIT[0] & ~reg_be))) |
               (addr_hit[1] & (|(SAFETY_SOC_CTRL_PERMIT[1] & ~reg_be))) |
               (addr_hit[2] & (|(SAFETY_SOC_CTRL_PERMIT[2] & ~reg_be)))));
  end

  assign bootaddr_we = addr_hit[0] & reg_we & !reg_error;
  assign bootaddr_wd = reg_wdata[31:0];

  assign fetchen_we = addr_hit[1] & reg_we & !reg_error;
  assign fetchen_wd = reg_wdata[0];

  assign corestatus_we = addr_hit[2] & reg_we & !reg_error;
  assign corestatus_wd = reg_wdata[31:0];

  // Read data return
  always_comb begin
    reg_rdata_next = '0;
    unique case (1'b1)
      addr_hit[0]: begin
        reg_rdata_next[31:0] = bootaddr_qs;
      end

      addr_hit[1]: begin
        reg_rdata_next[0] = fetchen_qs;
      end

      addr_hit[2]: begin
        reg_rdata_next[31:0] = corestatus_qs;
      end

      default: begin
        reg_rdata_next = '1;
      end
    endcase
  end

  // Unused signal tieoff

  // wdata / byte enable are not always fully used
  // add a blanket unused statement to handle lint waivers
  logic unused_wdata;
  logic unused_be;
  assign unused_wdata = ^reg_wdata;
  assign unused_be = ^reg_be;

  // Assertions for Register Interface
  `ASSERT(en2addrHit, (reg_we || reg_re) |-> $onehot0(addr_hit))

endmodule

module safety_soc_ctrl_reg_top_intf
#(
  parameter int AW = 4,
  localparam int DW = 32
) (
  input logic clk_i,
  input logic rst_ni,
  REG_BUS.in  regbus_slave,
  // To HW
  output safety_soc_ctrl_reg_pkg::safety_soc_ctrl_reg2hw_t reg2hw, // Write
  // Config
  input devmode_i // If 1, explicit error return for unmapped register access
);
 localparam int unsigned STRB_WIDTH = DW/8;

`include "register_interface/typedef.svh"
`include "register_interface/assign.svh"

  // Define structs for reg_bus
  typedef logic [AW-1:0] addr_t;
  typedef logic [DW-1:0] data_t;
  typedef logic [STRB_WIDTH-1:0] strb_t;
  `REG_BUS_TYPEDEF_ALL(reg_bus, addr_t, data_t, strb_t)

  reg_bus_req_t s_reg_req;
  reg_bus_rsp_t s_reg_rsp;
  
  // Assign SV interface to structs
  `REG_BUS_ASSIGN_TO_REQ(s_reg_req, regbus_slave)
  `REG_BUS_ASSIGN_FROM_RSP(regbus_slave, s_reg_rsp)

  

  safety_soc_ctrl_reg_top #(
    .reg_req_t(reg_bus_req_t),
    .reg_rsp_t(reg_bus_rsp_t),
    .AW(AW)
  ) i_regs (
    .clk_i,
    .rst_ni,
    .reg_req_i(s_reg_req),
    .reg_rsp_o(s_reg_rsp),
    .reg2hw, // Write
    .devmode_i
  );
  
endmodule


