

`timescale  1ps/1ps

module  ahci_top #(
parameter   PCIE_FUNC   = 1,
parameter   PORT_NUM    = 1
)
(
input                           hba_clk         ,
input                           hba_rst         ,
input                           pcie_clk        ,
input                           pcie_rst        ,
input                           pcie_lnk_up     ,

input       [PORT_NUM-1:0]      sata_ref_clk    ,
input       [PORT_NUM-1:0]      sata_gth_rxn    ,
input       [PORT_NUM-1:0]      sata_gth_rxp    ,
output      [PORT_NUM-1:0]      sata_gth_txn    ,
output      [PORT_NUM-1:0]      sata_gth_txp    ,


input                           cbus_req            ,
input                           cbus_rw             ,
input       [13: 0]             cbus_addr           ,
input       [31: 0]             cbus_wdata          ,
output                          cbus_ack            ,
output      [31: 0]             cbus_rdata          ,

output                          ahci_irq            ,

input                           sata_wr_ready       ,
output      [15: 0]             sata_wr_data_ex     ,
output      [255:0]             sata_wr_data        ,
output                          sata_wr_wen         ,
output                          sata_cpl_ready      ,
input       [15:0]              sata_cpl_data_ex    ,
input       [255:0]             sata_cpl_data       ,
input                           sata_cpl_wen

);

wire                            jlbus_req_ghc       ;
wire                            jlbus_rw_ghc        ;
wire        [ 7: 0]             jlbus_addr_ghc      ;
wire        [31: 0]             jlbus_wdata_ghc     ;
wire                            jlbus_ack_ghc       ;
wire        [31: 0]             jlbus_rdata_ghc     ;

wire        [PORT_NUM-1:0]      jlbus_req_port      ;
wire                            jlbus_rw_port       ;
wire        [ 6: 0]             jlbus_addr_port     ;
wire        [31: 0]             jlbus_wdata_port    ;
wire        [PORT_NUM-1:0]      jlbus_ack_port      ;
wire        [PORT_NUM*32-1:0]   jlbus_rdata_port    ;

wire                            jlbus_req_dfx       ;
wire                            jlbus_rw_dfx        ;
wire        [11: 0]             jlbus_addr_dfx      ;
wire        [31: 0]             jlbus_wdata_dfx     ;
wire                            jlbus_ack_dfx       ;
wire        [31: 0]             jlbus_rdata_dfx     ;
//wire                            jlbus_req_gb        ;
//wire                            jlbus_rw_gb         ;
//wire        [ 7: 0]             jlbus_addr_gb       ;
//wire        [31: 0]             jlbus_wdata_gb      ;
//wire                            jlbus_ack_gb        ;
//wire        [31: 0]             jlbus_rdata_gb      ;
//
//wire        [PORT_NUM-1:0]      jlbus_req_pt        ;
//wire                            jlbus_rw_pt         ;
//wire        [ 7: 0]             jlbus_addr_pt       ;
//wire        [31: 0]             jlbus_wdata_pt      ;
//wire        [PORT_NUM-1:0]      jlbus_ack_pt        ;
//wire        [PORT_NUM*32-1:0]   jlbus_rdata_pt      ;

wire        [PORT_NUM-1:0]      port_soft_reset     ;
wire                            clear               ;
wire        [ 4: 0]             port_sel            ;
wire                            hba_reset           ;
wire        [PORT_NUM-1:0]      ahci_rst            ;
reg                             hba_hw_rst          ;
reg         [PORT_NUM-1:0]      hba_suspend         ;
wire        [31: 0]             ghc_is_pending      ;
wire        [31: 0]             ghc_is_clear        ;
wire        [31: 0]             offline_set         ;



wire        [ 4: 0]             ghc_cap_ncs         ;       // Number of Command Slots
wire                            ghc_ghc_ae          ;       // ahci_enable

wire        [ 1: 0]             ahci_ghc_reg_inc    ;


wire        [PORT_NUM*4-1:0]    dma_op_id           ;
wire        [PORT_NUM*64-1:0]   dma_op_adrs         ;
wire        [PORT_NUM*4-1:0]    dma_op_user         ;
wire        [PORT_NUM*64-1:0]   dma_op_data         ;
wire        [PORT_NUM-1:0]      dma_op_wen          ;
wire        [PORT_NUM-1:0]      dma_op_ready        ;

//wire        [PORT_NUM*4-1:0]    dma_rd_hid          ;
//wire        [PORT_NUM*64-1:0]   dma_rd_adrs         ;
//wire        [PORT_NUM*14-1:0]   dma_rd_length       ;
//wire        [PORT_NUM-1:0]      dma_rd_wen          ;
//wire        [PORT_NUM-1:0]      dma_rd_ready        ;

wire        [PORT_NUM*4-1:0]    dma_cpl_id          ;
wire        [PORT_NUM*4-1:0]    dma_cpl_user        ;
wire        [PORT_NUM*64-1:0]   dma_cpl_data        ;
wire        [PORT_NUM-1:0]      dma_cpl_wen         ;
wire        [PORT_NUM-1:0]      dma_cpl_ready       ;

wire        [PORT_NUM*4-1:0]    cmd_rd_id           ;
wire        [PORT_NUM*64-1:0]   cmd_rd_adrs         ;
wire        [PORT_NUM*12-1:0]   cmd_rd_length       ;
wire        [PORT_NUM-1:0]      cmd_rd_wen          ;
wire        [PORT_NUM-1:0]      cmd_rd_ready        ;

wire        [PORT_NUM*4-1:0]    cmd_cpl_id          ;
wire        [PORT_NUM*4-1:0]    cmd_cpl_user        ;
wire        [PORT_NUM*64-1:0]   cmd_cpl_data        ;
wire        [PORT_NUM-1:0]      cmd_cpl_wen         ;


wire        [PORT_NUM-1:0]      phy_clk_tx          ;
wire        [PORT_NUM-1:0]      phy_rst_tx          ;
wire        [PORT_NUM-1:0]      phy_clk_rx          ;
wire        [PORT_NUM-1:0]      phy_rst_rx          ;
//wire        [PORT_NUM-1:0]      rxbyteisaligned     ;   // RX byte alignment completed
wire        [PORT_NUM-1:0]      rxcominitdet        ;
wire        [PORT_NUM-1:0]      rxcomwakedet        ;
wire        [PORT_NUM-1:0]      rxelecidle          ;   // RX electrical idle
wire        [PORT_NUM*2-1:0]    rx_charerr          ;
wire        [PORT_NUM*2-1:0]    rx_charisk          ;
wire        [PORT_NUM*16-1:0]   rx_datain           ;
wire        [PORT_NUM-1:0]      rxpmareset          ;
wire        [PORT_NUM-1:0]      rxuserrdy           ;
wire        [PORT_NUM-1:0]      rxcdrhold           ;
wire        [PORT_NUM-1:0]      txcominit           ;   // TX OOB enable
wire        [PORT_NUM-1:0]      txcomwake           ;   // TX OOB type select
wire        [PORT_NUM-1:0]      txcomfinish         ;
wire        [PORT_NUM-1:0]      txelecidle          ;   // TX electircal idel
wire        [PORT_NUM*2-1:0]    tx_charisk          ;   // TX byted is K character
wire        [PORT_NUM*16-1:0]   tx_dataout          ;   // Outgoing TX data
wire        [PORT_NUM-1:0]      rx_data_valid       ;

wire        [ 4: 0]             txdiffctrl          ;
wire        [ 4: 0]             txpostcursor        ;
wire        [ 4: 0]             txprecursor         ;
wire        [15: 0]             ncq_int_delay       ;
wire        [ 9: 0]             ncq_again_gap       ;
wire        [ 5: 0]             ncq_again_num       ;


wire        [223:0]             u_ahci_dma_op_dbg   ;
//wire        [95: 0]             u_ahci_dma_rd_dbg   ;
wire        [159:0]             u_ahci_dma_cpl_dbg  ;
wire        [PORT_NUM*32-1:0]   u_transition_dbg    ;
wire        [PORT_NUM*16-1:0]   u_action_dbg        ;
wire        [PORT_NUM*16-1:0]   u_clist_ctab_dbg    ;
wire        [PORT_NUM*4-1:0]    u_receive_dbg       ;
wire        [PORT_NUM*12-1:0]   u_transmit_dbg      ;
wire        [PORT_NUM*16-1:0]   u_port_dma_dbg      ;
wire        [PORT_NUM*32-1:0]   u_link_top_dbg      ;
wire        [PORT_NUM*160-1:0]  log_status          ;
wire        [PORT_NUM*16-1:0]   rx_charerr_cnt      ;

wire        [23: 0]             u_ahci_dma_op_inc   ;
wire        [18: 0]             u_ahci_dma_cpl_inc  ;
wire        [PORT_NUM-1:0]      phyrdy_fall_inc     ;
wire        [PORT_NUM*3-1:0]    u_action_inc        ;
wire        [PORT_NUM*4-1:0]    u_clist_ctab_inc    ;
wire        [PORT_NUM*8-1:0]    u_receive_inc       ;
wire        [PORT_NUM*8-1:0]    u_transmit_inc      ;
wire        [PORT_NUM*4-1:0]    u_port_dma_inc      ;
wire        [PORT_NUM*6-1:0]    u_link_tx_inc       ;
wire        [PORT_NUM*6-1:0]    u_link_rx_inc       ;
wire        [PORT_NUM*4-1:0]    u_link_fsm_inc      ;

(* ASYNC_REG = "TRUE" *)reg                     pcie_lnk_up_meta    ;
(* ASYNC_REG = "TRUE" *)reg                     pcie_lnk_up_syn     ;
//debug
wire        [PORT_NUM*4-1:0]    dbg_h2d_user        ;
wire        [PORT_NUM*32-1:0]   dbg_h2d_data        ;
wire        [PORT_NUM-1:0]      dbg_h2d_valid       ;
wire        [PORT_NUM-1:0]      dbg_h2d_ready       ;
wire        [PORT_NUM*4-1:0]    dbg_d2h_user        ;
wire        [PORT_NUM*32-1:0]   dbg_d2h_data        ;
wire        [PORT_NUM-1:0]      dbg_d2h_valid       ;
wire        [PORT_NUM-1:0]      dbg_d2h_ready       ;
wire        [PORT_NUM*6-1:0]    dbg_p_state         ;




always@(posedge hba_clk)
begin
    pcie_lnk_up_meta <= pcie_lnk_up;
    pcie_lnk_up_syn  <= pcie_lnk_up_meta;
end


ahci_lbus_ctl #(
    .PORT_NUM   ( PORT_NUM )
)
u_ahci_lbus_ctl(
    .hba_clk            ( hba_clk           ),
    .hba_rst            ( hba_rst           ),

    .ilbus_req          ( cbus_req          ),
    .ilbus_rw           ( cbus_rw           ),   //1:rd 0:wr
    .ilbus_addr         ( cbus_addr         ),
    .ilbus_wdata        ( cbus_wdata        ),
    .olbus_ack          ( cbus_ack          ),
    .olbus_rdata        ( cbus_rdata        ),

    .olbus_req_ghc      ( jlbus_req_ghc     ),
    .olbus_rw_ghc       ( jlbus_rw_ghc      ),
    .olbus_addr_ghc     ( jlbus_addr_ghc    ),
    .olbus_wdata_ghc    ( jlbus_wdata_ghc   ),
    .ilbus_ack_ghc      ( jlbus_ack_ghc     ),
    .ilbus_rdata_ghc    ( jlbus_rdata_ghc   ),

    .olbus_req_port     ( jlbus_req_port    ),
    .olbus_rw_port      ( jlbus_rw_port     ),
    .olbus_addr_port    ( jlbus_addr_port   ),
    .olbus_wdata_port   ( jlbus_wdata_port  ),
    .ilbus_ack_port     ( jlbus_ack_port    ),
    .ilbus_rdata_port   ( jlbus_rdata_port  ),

    .olbus_req_dfx      ( jlbus_req_dfx     ),
    .olbus_rw_dfx       ( jlbus_rw_dfx      ),
    .olbus_addr_dfx     ( jlbus_addr_dfx    ),
    .olbus_wdata_dfx    ( jlbus_wdata_dfx   ),
    .ilbus_ack_dfx      ( jlbus_ack_dfx     ),
    .ilbus_rdata_dfx    ( jlbus_rdata_dfx   )
);

ahci_ghc_reg u_ahci_ghc_reg(
    .hba_clk            ( hba_clk           ),
    .hba_rst            ( hba_rst           ),
    .pcie_lnk_up        ( pcie_lnk_up_syn   ),

    .ilbus_req          ( jlbus_req_ghc     ),
    .ilbus_rw           ( jlbus_rw_ghc      ),   //1:rd 0:wr
    .ilbus_addr         ( jlbus_addr_ghc    ),
    .ilbus_wdata        ( jlbus_wdata_ghc   ),
    .olbus_ack          ( jlbus_ack_ghc     ),
    .olbus_rdata        ( jlbus_rdata_ghc   ),

    .hba_reset          ( hba_reset         ),
    .ghc_is_pending     ( ghc_is_pending    ),
    .ghc_is_clear       ( ghc_is_clear      ),
    .ahci_irq           ( ahci_irq          ),

    .ostat_inc          ( ahci_ghc_reg_inc  )
);
always@(posedge hba_clk)
begin
    if(hba_rst)
        hba_hw_rst <= 1'b1;
    else if(hba_reset)
        hba_hw_rst <= 1'b1;
    else
        hba_hw_rst <= 1'b0;
end



ahci_global_dfx #(
    .PORT_NUM   ( PORT_NUM  )
)
u_ahci_global_dfx(
    .cbus_clk           ( hba_clk           ),
    .cbus_rst           ( hba_rst           ),
    .pcie_clk           ( pcie_clk          ),

    .ilbus_req          ( jlbus_req_dfx     ),
    .ilbus_rw           ( jlbus_rw_dfx      ),   //1:rd 0:wr
    .ilbus_addr         ( jlbus_addr_dfx    ),
    .ilbus_wdata        ( jlbus_wdata_dfx   ),
    .olbus_ack          ( jlbus_ack_dfx     ),
    .olbus_rdata        ( jlbus_rdata_dfx   ),

    .soft_reset         ( port_soft_reset   ),
    .oclear             ( clear             ),
    .port_sel           ( port_sel          ),
    .txdiffctrl         ( txdiffctrl        ),
    .txpostcursor       ( txpostcursor      ),
    .txprecursor        ( txprecursor       ),
    .ncq_int_delay      ( ncq_int_delay     ),
    .ncq_again_gap      ( ncq_again_gap     ),
    .ncq_again_num      ( ncq_again_num     ),
    .u_ahci_dma_op_dbg  ( u_ahci_dma_op_dbg ),
//    .u_ahci_dma_rd_dbg  ( u_ahci_dma_rd_dbg ),
    .u_ahci_dma_cpl_dbg ( u_ahci_dma_cpl_dbg),

    .u_transition_dbg   ( u_transition_dbg  ),
    .u_action_dbg       ( u_action_dbg      ),
    .u_clist_ctab_dbg   ( u_clist_ctab_dbg  ),
    .u_receive_dbg      ( u_receive_dbg     ),
    .u_transmit_dbg     ( u_transmit_dbg    ),
    .u_port_dma_dbg     ( u_port_dma_dbg    ),
    .u_link_top_dbg     ( u_link_top_dbg    ),
    .log_status         ( log_status        ),
    .rx_charerr_cnt     ( rx_charerr_cnt    ),

    .u_ahci_dma_op_inc  ( u_ahci_dma_op_inc ),
    .u_ahci_dma_cpl_inc ( u_ahci_dma_cpl_inc),
//    .phy_clk_tx         ( phy_clk_tx        ),
//    .phy_clk_rx         ( phy_clk_rx        ),
    .phyrdy_fall_inc    ( phyrdy_fall_inc   ),
    .u_action_inc       ( u_action_inc      ),
    .u_clist_ctab_inc   ( u_clist_ctab_inc  ),
    .u_receive_inc      ( u_receive_inc     ),
    .u_transmit_inc     ( u_transmit_inc    ),
    .u_port_dma_inc     ( u_port_dma_inc    ),
    .u_link_tx_inc      ( u_link_tx_inc     ),
    .u_link_rx_inc      ( u_link_rx_inc     ),
    .u_link_fsm_inc     ( u_link_fsm_inc    )
);

ahci_dma_op #(
    .PCIE_FUNC  ( PCIE_FUNC ),
    .PORT_NUM   ( PORT_NUM  )
)
u_ahci_dma_op(
    .pcie_clk           ( pcie_clk          ),
    .pcie_rst           ( pcie_rst          ),
    .hba_clk            ( hba_clk           ),
    .hba_rst            ( ahci_rst          ),

    .dma_op_id          ( dma_op_id         ),
    .dma_op_adrs        ( dma_op_adrs       ),
    .dma_op_user        ( dma_op_user       ),
    .dma_op_data        ( dma_op_data       ),
    .dma_op_wen         ( dma_op_wen        ),
    .dma_op_ready       ( dma_op_ready      ),

    .cmd_rd_id          ( cmd_rd_id         ),
    .cmd_rd_adrs        ( cmd_rd_adrs       ),
    .cmd_rd_length      ( cmd_rd_length     ),
    .cmd_rd_wen         ( cmd_rd_wen        ),
    .cmd_rd_ready       ( cmd_rd_ready      ),

    .sata_tx_wr_ready   ( sata_wr_ready     ),
    .sata_tx_wr_data_ex ( sata_wr_data_ex   ),
    .sata_tx_wr_data    ( sata_wr_data      ),
    .sata_tx_wr_wen     ( sata_wr_wen       ),

    .ostat_inc          ( u_ahci_dma_op_inc ),
    .ostatus_dbg        ( u_ahci_dma_op_dbg )
);

ahci_dma_cpl #(
    .PORT_NUM   ( PORT_NUM  )
)
u_ahci_dma_cpl(
    .pcie_clk           ( pcie_clk          ),
    .pcie_rst           ( pcie_rst          ),
    .hba_clk            ( hba_clk           ),
    .hba_rst            ( ahci_rst          ),

    .sata_rx_cpl_ready  ( sata_cpl_ready    ),
    .sata_rx_cpl_data_ex( sata_cpl_data_ex  ),
    .sata_rx_cpl_data   ( sata_cpl_data     ),
    .sata_rx_cpl_wen    ( sata_cpl_wen      ),

    .dma_cpl_id         ( dma_cpl_id        ),
    .dma_cpl_user       ( dma_cpl_user      ),
    .dma_cpl_data       ( dma_cpl_data      ),
    .dma_cpl_wen        ( dma_cpl_wen       ),
    .dma_cpl_ready      ( dma_cpl_ready     ),

    .cmd_cpl_id         ( cmd_cpl_id        ),
    .cmd_cpl_user       ( cmd_cpl_user      ),
    .cmd_cpl_data       ( cmd_cpl_data      ),
    .cmd_cpl_wen        ( cmd_cpl_wen       ),

    .ostat_inc          ( u_ahci_dma_cpl_inc),
    .ostatus_dbg        ( u_ahci_dma_cpl_dbg)
);

genvar port_i;
generate
    for(port_i=0;port_i<PORT_NUM;port_i=port_i+1)
    begin : gen_port

    always@(posedge hba_clk)
    begin
        if(hba_rst)
            hba_suspend[port_i] <= 1'b1;
        else if(hba_reset | port_soft_reset[port_i])
            hba_suspend[port_i] <= 1'b1;
        else
            hba_suspend[port_i] <= 1'b0;
    end

    ahci_port_top #(
        .PORT_ID    ( port_i    )
    )
    u_ahci_port_top(
        .hba_clk            ( hba_clk                       ),
        .hba_rst            ( hba_hw_rst                    ),
        .ahci_rst           ( ahci_rst[port_i]              ),
        .port_suspend       ( hba_suspend[port_i]           ),
        .ncq_int_delay      ( ncq_int_delay                 ),
        .ncq_again_gap      ( ncq_again_gap                 ),
        .ncq_again_num      ( ncq_again_num                 ),

        .cbus_req           ( jlbus_req_port[port_i]        ),
        .cbus_rw            ( jlbus_rw_port                 ),
        .cbus_addr          ( jlbus_addr_port               ),
        .cbus_wdata         ( jlbus_wdata_port              ),
        .cbus_ack           ( jlbus_ack_port[port_i]        ),
        .cbus_rdata         ( jlbus_rdata_port[port_i*32+:32]),

        .phy_clk_tx         ( phy_clk_tx[port_i]            ),
        .phy_rst_tx         ( phy_rst_tx[port_i]            ),
        .phy_clk_rx         ( phy_clk_rx[port_i]            ),
        .phy_rst_rx         ( phy_rst_rx[port_i]            ),
//        .rxbyteisaligned    ( rxbyteisaligned[port_i]       ),   // RX byte alignment completed
        .rx_data_valid      ( rx_data_valid[port_i]         ),
        .rxcominitdet       ( rxcominitdet[port_i]          ),
        .rxcomwakedet       ( rxcomwakedet[port_i]          ),
        .rxelecidle         ( rxelecidle[port_i]            ),   // RX electrical idle
        .rx_charerr         ( rx_charerr[port_i*2+:2]       ),
        .rx_charisk         ( rx_charisk[port_i*2+:2]       ),
        .rx_datain          ( rx_datain[port_i*16+:16]      ),
        .rxpmareset         ( rxpmareset[port_i]            ),
        .rxuserrdy          ( rxuserrdy[port_i]             ),
        .rxcdrhold          ( rxcdrhold[port_i]             ),
        .txcominit          ( txcominit[port_i]             ),   // TX OOB enable
        .txcomwake          ( txcomwake[port_i]             ),   // TX OOB type select
        .txcomfinish        ( txcomfinish[port_i]           ),
        .txelecidle         ( txelecidle[port_i]            ),   // TX electircal idel
        .tx_charisk         ( tx_charisk[port_i*2+:2]       ),   // TX byted is K character
        .tx_dataout         ( tx_dataout[port_i*16+:16]     ),   // Outgoing TX data

        .dma_op_id          ( dma_op_id[port_i*4+:4]        ),
        .dma_op_adrs        ( dma_op_adrs[port_i*64+:64]    ),
        .dma_op_user        ( dma_op_user[port_i*4+:4]      ),
        .dma_op_data        ( dma_op_data[port_i*64+:64]    ),
        .dma_op_wen         ( dma_op_wen[port_i]            ),
        .dma_op_ready       ( dma_op_ready[port_i]          ),

//        .dma_rd_hid         ( dma_rd_hid[port_i*4+:4]       ),
//        .dma_rd_adrs        ( dma_rd_adrs[port_i*64+:64]    ),
//        .dma_rd_length      ( dma_rd_length[port_i*14+:14]  ),
//        .dma_rd_wen         ( dma_rd_wen[port_i]            ),
//        .dma_rd_ready       ( dma_rd_ready[port_i]          ),

        .dma_cpl_id         ( dma_cpl_id[port_i*4+:4]       ),
        .dma_cpl_user       ( dma_cpl_user[port_i*4+:4]     ),
        .dma_cpl_data       ( dma_cpl_data[port_i*64+:64]   ),
        .dma_cpl_wen        ( dma_cpl_wen[port_i]           ),
        .dma_cpl_ready      ( dma_cpl_ready[port_i]         ),

        .cmd_rd_id          ( cmd_rd_id[port_i*4+:4]        ),
        .cmd_rd_adrs        ( cmd_rd_adrs[port_i*64+:64]    ),
        .cmd_rd_length      ( cmd_rd_length[port_i*12+:12]  ),
        .cmd_rd_wen         ( cmd_rd_wen[port_i]            ),
        .cmd_rd_ready       ( cmd_rd_ready[port_i]          ),

        .cmd_cpl_id         ( cmd_cpl_id[port_i*4+:4]       ),
        .cmd_cpl_user       ( cmd_cpl_user[port_i*4+:4]     ),
        .cmd_cpl_data       ( cmd_cpl_data[port_i*64+:64]   ),
        .cmd_cpl_wen        ( cmd_cpl_wen[port_i]           ),

        .clear              ( clear                         ),
        .ghc_is_pending     ( ghc_is_pending[port_i]        ),
        .ghc_is_clear       ( ghc_is_clear[port_i]          ),
        .offline_set        ( offline_set[port_i]           ),

        .u_transition_dbg   ( u_transition_dbg[port_i*32+:32]),
        .u_action_dbg       ( u_action_dbg[port_i*16+:16]    ),
        .u_clist_ctab_dbg   ( u_clist_ctab_dbg[port_i*16+:16]),
        .u_receive_dbg      ( u_receive_dbg[port_i*4+:4]     ),
        .u_transmit_dbg     ( u_transmit_dbg[port_i*12+:12]  ),
        .u_port_dma_dbg     ( u_port_dma_dbg[port_i*16+:16]  ),
        .u_link_top_dbg     ( u_link_top_dbg[port_i*32+:32]  ),
        .log_status         ( log_status[port_i*160+:160]    ),
        .rx_charerr_cnt     ( rx_charerr_cnt[port_i*16+:16] ),

        .phyrdy_fall_inc    ( phyrdy_fall_inc[port_i]       ),
        .u_action_inc       ( u_action_inc[port_i*3+:3]     ),
        .u_clist_ctab_inc   ( u_clist_ctab_inc[port_i*4+:4] ),
        .u_receive_inc      ( u_receive_inc[port_i*8+:8]    ),
        .u_transmit_inc     ( u_transmit_inc[port_i*8+:8]   ),
        .u_port_dma_inc     ( u_port_dma_inc[port_i*4+:4]   ),
        .u_link_tx_inc      ( u_link_tx_inc[port_i*6+:6]    ),
        .u_link_rx_inc      ( u_link_rx_inc[port_i*6+:6]    ),
        .u_link_fsm_inc     ( u_link_fsm_inc[port_i*4+:4]   ),

        .h2d_user           ( dbg_h2d_user[port_i*4+:4]     ),
        .h2d_data           ( dbg_h2d_data[port_i*32+:32]   ),
        .h2d_valid          ( dbg_h2d_valid[port_i]         ),
        .h2d_ready          ( dbg_h2d_ready[port_i]         ),
        .d2h_user           ( dbg_d2h_user[port_i*4+:4]     ),
        .d2h_data           ( dbg_d2h_data[port_i*32+:32]   ),
        .d2h_valid          ( dbg_d2h_valid[port_i]         ),
        .d2h_ready          ( dbg_d2h_ready[port_i]         ),
        .p_state            ( dbg_p_state[port_i*6+:6]      )
    );

    sata_phy_top #(
        .PORT_ID    ( port_i    )
    )
    u_sata_phy_top(
        .sata_ref_clk                   ( sata_ref_clk[port_i]      ),
        .sata_gth_rxn                   ( sata_gth_rxn[port_i]      ),
        .sata_gth_rxp                   ( sata_gth_rxp[port_i]      ),
        .sata_gth_txn                   ( sata_gth_txn[port_i]      ),
        .sata_gth_txp                   ( sata_gth_txp[port_i]      ),
        .hb_gtwiz_reset_clk_freerun_in  ( hba_clk                   ),
        .hb_gtwiz_reset_all_in          ( hba_hw_rst|offline_set[port_i]),

        .phy_clk_tx                     ( phy_clk_tx[port_i]        ),
        .phy_rst_tx                     ( phy_rst_tx[port_i]        ),
        .phy_clk_rx                     ( phy_clk_rx[port_i]        ),
        .phy_rst_rx                     ( phy_rst_rx[port_i]        ),
//        .rxbyteisaligned                ( rxbyteisaligned[port_i]   ),   // RX byte alignment completed
        .rx_data_valid                  ( rx_data_valid[port_i]     ),
        .rxcominitdet                   ( rxcominitdet[port_i]      ),
        .rxcomwakedet                   ( rxcomwakedet[port_i]      ),
        .rxelecidle                     ( rxelecidle[port_i]        ),   // RX electrical idle
        .rx_charerr                     ( rx_charerr[port_i*2+:2]   ),
        .rx_charisk                     ( rx_charisk[port_i*2+:2]   ),
        .rx_datain                      ( rx_datain[port_i*16+:16]  ),
        .rxpmareset                     ( rxpmareset[port_i]        ),
        .rxuserrdy                      ( rxuserrdy[port_i]         ),
        .rxcdrhold                      ( rxcdrhold[port_i]         ),
        .txcominit                      ( txcominit[port_i]         ),   // TX OOB enable
        .txcomwake                      ( txcomwake[port_i]         ),   // TX OOB type select
        .txcomfinish                    ( txcomfinish[port_i]       ),
        .txelecidle                     ( txelecidle[port_i]        ),   // TX electircal idel
        .tx_charisk                     ( tx_charisk[port_i*2+:2]   ),   // TX byted is K character
        .tx_dataout                     ( tx_dataout[port_i*16+:16] ),   // Outgoing TX data
        .txdiffctrl                     ( txdiffctrl                ),
        .txpostcursor                   ( txpostcursor              ),
        .txprecursor                    ( txprecursor               )
    );

//    ahci_port_dfx u_ahci_port_dfx(
//        .cbus_clk           ( hba_clk                       ),
//        .cbus_rst           ( hba_hw_rst                    ),
//        .pcie_clk           ( pcie_clk                      ),
////        .pcie_rst           ( pcie_rst                      ),
//
//        .ilbus_req          ( jlbus_req_pt[port_i]          ),
//        .ilbus_rw           ( jlbus_rw_pt                   ),   //1:rd 0:wr
//        .ilbus_addr         ( jlbus_addr_pt                 ),
//        .ilbus_wdata        ( jlbus_wdata_pt                ),
//        .olbus_ack          ( jlbus_ack_pt[port_i]          ),
//        .olbus_rdata        ( jlbus_rdata_pt[port_i*32+:32] ),
//
//        .u_transition_dbg   ( u_transition_dbg[port_i*32+:32]),
//        .u_action_dbg       ( u_action_dbg[port_i*16+:16]    ),
//        .u_clist_ctab_dbg   ( u_clist_ctab_dbg[port_i*16+:16]),
//        .u_receive_dbg      ( u_receive_dbg[port_i*8+:8]     ),
//        .u_transmit_dbg     ( u_transmit_dbg[port_i*8+:8]    ),
//        .u_port_dma_dbg     ( u_port_dma_dbg[port_i*16+:16]  ),
//        .u_link_top_dbg     ( u_link_top_dbg[port_i*32+:32]  ),
//        .log_status         ( log_status[port_i*128+:128]   ),
//
//        .phy_clk_tx         ( phy_clk_tx[port_i]            ),
//        .phy_clk_rx         ( phy_clk_rx[port_i]            ),
//        .phyrdy_fall_inc    ( phyrdy_fall_inc[port_i]       ),
//        .u_action_inc       ( u_action_inc[port_i*2+:2]     ),
//        .u_clist_ctab_inc   ( u_clist_ctab_inc[port_i*4+:4] ),
//        .u_receive_inc      ( u_receive_inc[port_i*8+:8]    ),
//        .u_transmit_inc     ( u_transmit_inc[port_i*8+:8]   ),
//        .u_port_dma_inc     ( u_port_dma_inc[port_i*4+:4]   ),
//        .u_link_tx_inc      ( u_link_tx_inc[port_i*6+:6]    ),
//        .u_link_rx_inc      ( u_link_rx_inc[port_i*6+:6]    ),
//        .u_link_fsm_inc     ( u_link_fsm_inc[port_i*4+:4]   )
//
//    );
    end
endgenerate


//assign cbus_port_rdata[32*PORT_NUM+:32*(32-PORT_NUM)] = {(32-PORT_NUM){32'b0}};
//assign cbus_ack_port[PORT_NUM+:32-PORT_NUM] = {(32-PORT_NUM){1'b0}};

assign ghc_is_pending[PORT_NUM+:32-PORT_NUM] = {(32-PORT_NUM){1'b0}};
assign offline_set[PORT_NUM+:32-PORT_NUM] = {(32-PORT_NUM){1'b0}};




////debug
//reg [35:0]  curr_time;
//always@(posedge hba_clk)
//begin
//    if(clear)
//        curr_time <= 36'b0;
//    else
//        curr_time <= curr_time + 1'b1;
//end
//
//reg [ 3: 0]     h2d_user        ;
//reg [31: 0]     h2d_data        ;
//reg             h2d_valid       ;
//reg             h2d_ready       ;
//reg [ 3: 0]     d2h_user        ;
//reg [31: 0]     d2h_data        ;
//reg             d2h_valid       ;
//reg             d2h_ready       ;
//reg [ 5: 0]     p_state         ;
//
//reg             dbg_cbus_req    ;
//reg             dbg_cbus_rw     ;
//reg [ 6: 0]     dbg_cbus_addr   ;
//reg [31: 0]     dbg_cbus_wdata  ;
//reg             dbg_cbus_ack    ;
//reg [31: 0]     dbg_cbus_rdata  ;
//
//
//always@(posedge hba_clk)
//begin
//    h2d_user     <= dbg_h2d_user[port_sel*4+:4];
//    h2d_data     <= dbg_h2d_data[port_sel*32+:32];
//    h2d_valid    <= dbg_h2d_valid[port_sel];
//    h2d_ready    <= dbg_h2d_ready[port_sel];
//    d2h_user     <= dbg_d2h_user[port_sel*4+:4];
//    d2h_data     <= dbg_d2h_data[port_sel*32+:32];
//    d2h_valid    <= dbg_d2h_valid[port_sel];
//    d2h_ready    <= dbg_d2h_ready[port_sel];
//    p_state      <= dbg_p_state[port_sel*6+:6];
//
//    dbg_cbus_req   <= jlbus_req_port[port_sel];
//    dbg_cbus_rw    <= jlbus_rw_port;
//    dbg_cbus_addr  <= jlbus_addr_port;
//    dbg_cbus_wdata <= jlbus_wdata_port;
//    dbg_cbus_ack   <= jlbus_ack_port[port_sel];
//    dbg_cbus_rdata <= jlbus_rdata_port[port_sel*32+:32];
//end
//
//
//
//
//
//
//reg                             tx_data_flg;
//reg [12:0]                      tx_data_cnt;
//reg [35:0]                      tx_cap_din;
//reg                             tx_cap_wen;
//(* keep = "true" *)reg  [9:0]   tx_cap_wadrs;
//(* keep = "true" *)reg  [9:0]   cap_radrs;
//wire[35:0]                      tx_cap_dout;
//(* keep = "true" *)reg [35:0]   tx_cap_dout_1d;
//reg [35:0]                      tx_data;
//reg                             tx_wen;
//reg [35:0]                      tx_data_1d;
//reg                             tx_wen_1d;
//sdp_ram #(
//    .DATA_WIDTH     ( 36        ),
//    .ADDR_WIDTH     ( 10        ),
//    .READ_LATENCY   ( 2         ),
//    .MEMORY_TYPE    ( "block"   )
//)
//tx_cap_ram(
//    .clka           ( hba_clk       ),
//    .addra          ( tx_cap_wadrs  ),
//    .dina           ( tx_cap_din    ),
//    .wea            ( tx_cap_wen    ),
//    .clkb           ( hba_clk       ),
//    .addrb          ( cap_radrs     ),
//    .doutb          ( tx_cap_dout   )
//);
//
//always@(posedge hba_clk)
//begin
//    if(clear)
//        cap_radrs <= 10'b0;
//    else
//        cap_radrs <= cap_radrs + 1'b1;
//end
//
//
//always@(posedge hba_clk)
//begin
//    if(h2d_valid & h2d_ready & h2d_user[3])
//        tx_data_flg <= (h2d_data[7:0]==8'h46)? 1'b1 : 1'b0;
//    else
//        ;
//end
//always@(posedge hba_clk)
//begin
//    if(tx_wen)
//        tx_data_cnt <= (tx_data[35])? 13'd1 :
//                       (tx_data[34])? 13'b0 :
//                                      tx_data_cnt + 1'b1;
//    else
//        ;
//end
//always@(posedge hba_clk)
//begin
//    if(h2d_valid & h2d_ready)
//    begin
////        tx_data <= {h2d_user,h2d_data};
//        tx_data <= {h2d_user[3:1],1'b0,h2d_data};
//        tx_wen  <= 1'b1;
//    end
//    else
//        tx_wen  <= 1'b0;
//end
//always@(posedge hba_clk)
//begin
//    tx_data_1d <= tx_data;
//    tx_wen_1d  <= tx_wen;
//end
//always@(posedge hba_clk)
//begin
//    if(h2d_valid & h2d_ready & h2d_user[3])
//    begin
//        tx_cap_din <= {2'b10,curr_time[35:2]};
//        tx_cap_wen <= 1'b1;
//    end
//    else if(tx_wen & (tx_data[35] | ~tx_data_flg))
////    else if(tx_wen)
//    begin
//        tx_cap_din <= {1'b0,tx_data[34:0]};
//        tx_cap_wen <= 1'b1;
//    end
//    else if(tx_wen & tx_data_flg & tx_data[34])
//    begin
//        tx_cap_din <= {2'b01,21'b0,tx_data_cnt};
//        tx_cap_wen <= 1'b1;
//    end
////    else if(tx_wen_1d & tx_data_1d[34])
////    begin
////        tx_cap_din <= {4'hf,pxsact};
////        tx_cap_wen <= 1'b1;
////    end
//    else
//        tx_cap_wen <= 1'b0;
//end
//always@(posedge hba_clk)
//begin
//    if(clear)
//        tx_cap_wadrs <= 10'b0;
//    else if(tx_cap_wen)
////        tx_cap_wadrs <= (tx_cap_wadrs==10'h3ff)? 10'h3ff : tx_cap_wadrs + 1'b1;
//        tx_cap_wadrs <= tx_cap_wadrs + 1'b1;
//    else
//        ;
//end
//
//always@(posedge hba_clk)
//    tx_cap_dout_1d <= tx_cap_dout;
//
//
//
//
//reg                             rx_data_flg;
//reg [12:0]                      rx_data_cnt;
//reg [35:0]                      rx_cap_din;
//reg                             rx_cap_wen;
//(* keep = "true" *)reg  [9:0]   rx_cap_wadrs;
////(* keep = "true" *)reg  [9:0]   rx_cap_radrs;
//wire[35:0]                      rx_cap_dout;
//(* keep = "true" *)reg [35:0]   rx_cap_dout_1d;
//reg [35:0]                      rx_data;
//reg                             rx_wen;
//reg [35:0]                      rx_data_1d;
//reg                             rx_wen_1d;
//sdp_ram #(
//    .DATA_WIDTH     ( 36        ),
//    .ADDR_WIDTH     ( 10        ),
//    .READ_LATENCY   ( 2         ),
//    .MEMORY_TYPE    ( "block"   )
//)
//rx_cap_ram(
//    .clka           ( hba_clk       ),
//    .addra          ( rx_cap_wadrs  ),
//    .dina           ( rx_cap_din    ),
//    .wea            ( rx_cap_wen    ),
//    .clkb           ( hba_clk       ),
//    .addrb          ( cap_radrs     ),
//    .doutb          ( rx_cap_dout   )
//);
//always@(posedge hba_clk)
//begin
//    if(d2h_valid & d2h_ready & d2h_user[3])
//        rx_data_flg <= (d2h_data[7:0]==8'h46)? 1'b1 : 1'b0;
//    else
//        ;
//end
//always@(posedge hba_clk)
//begin
//    if(rx_wen)
//        rx_data_cnt <= (rx_data[35])? 13'd1 :
//                       (rx_data[34])? 13'b0 :
//                                      rx_data_cnt + 1'b1;
//    else
//        ;
//end
//always@(posedge hba_clk)
//begin
//    if(d2h_valid & d2h_ready)
//    begin
//        rx_data <= {d2h_user,d2h_data};
//        rx_wen  <= 1'b1;
//    end
//    else
//        rx_wen  <= 1'b0;
//end
//always@(posedge hba_clk)
//begin
//    rx_data_1d <= rx_data;
//    rx_wen_1d  <= rx_wen;
//end
//always@(posedge hba_clk)
//begin
//    if(d2h_valid & d2h_ready & d2h_user[3])
//    begin
//        rx_cap_din <= {2'b10,curr_time[35:2]};
//        rx_cap_wen <= 1'b1;
//    end
//    else if(rx_wen & (rx_data[35] | ~rx_data_flg))
////    else if(rx_wen)
//    begin
//        rx_cap_din <= {1'b0,rx_data[34:0]};
//        rx_cap_wen <= 1'b1;
//    end
//    else if(rx_wen & rx_data_flg & rx_data[34])
//    begin
//        rx_cap_din <= {2'b01,21'b0,rx_data_cnt};
//        rx_cap_wen <= 1'b1;
//    end
////    else if(rx_wen_1d & rx_data_1d[34])
////    begin
////        rx_cap_din <= {4'hf,pxsact};
////        rx_cap_wen <= 1'b1;
////    end
//    else
//        rx_cap_wen <= 1'b0;
//end
////always@(posedge hba_clk)
////begin
////    if(d2h_valid & d2h_ready & d2h_user[3] & tx_identify_flg)
////    begin
////        rx_cap_din <= {2'b10,curr_time[35:2]};
////        rx_cap_wen <= 1'b1;
////    end
////    else if(rx_wen & tx_identify_flg)
////    begin
////        rx_cap_din <= {1'b0,rx_data[34:0]};
////        rx_cap_wen <= 1'b1;
////    end
//////    else if(rx_wen & rx_data_flg & rx_data[34])
//////    begin
//////        rx_cap_din <= {2'b01,21'b0,rx_data_cnt};
//////        rx_cap_wen <= 1'b1;
//////    end
////    else
////        rx_cap_wen <= 1'b0;
////end
//always@(posedge hba_clk)
//begin
//    if(clear)
//        rx_cap_wadrs <= 10'b0;
//    else if(rx_cap_wen)
////        rx_cap_wadrs <= (rx_cap_wadrs==10'h3ff)? 10'h3ff : rx_cap_wadrs + 1'b1;
//        rx_cap_wadrs <= rx_cap_wadrs + 1'b1;
//    else
//        ;
//end
//always@(posedge hba_clk)
//    rx_cap_dout_1d <= rx_cap_dout;
//
//
////debug
//reg [41:0]                      p_state_cap_din;
//reg                             p_state_cap_wen;
//(* keep = "true" *)reg  [9:0]   p_state_cap_wadrs;
////(* keep = "true" *)reg  [9:0]   rx_cap_radrs;
//wire[41:0]                      p_state_cap_dout;
//(* keep = "true" *)reg [41:0]   p_state_cap_dout_1d;
//
//reg [5:0]   p_state_1d;
//always@(posedge hba_clk)
//    p_state_1d <= p_state;
//sdp_ram #(
//    .DATA_WIDTH     ( 42        ),
//    .ADDR_WIDTH     ( 10        ),
//    .READ_LATENCY   ( 2         ),
//    .MEMORY_TYPE    ( "block"   )
//)
//p_state_cap_ram(
//    .clka           ( hba_clk           ),
//    .addra          ( p_state_cap_wadrs ),
//    .dina           ( p_state_cap_din   ),
//    .wea            ( p_state_cap_wen   ),
//    .clkb           ( hba_clk           ),
//    .addrb          ( cap_radrs         ),
//    .doutb          ( p_state_cap_dout  )
//);
//always@(posedge hba_clk)
//begin
//    if(p_state_1d != p_state)
//    begin
//        p_state_cap_din <= {curr_time[35:2],2'b0,p_state[5:0]};
//        p_state_cap_wen <= 1'b1;
//    end
//    else
//        p_state_cap_wen <= 1'b0;
//end
//always@(posedge hba_clk)
//begin
//    if(clear)
//        p_state_cap_wadrs <= 10'b0;
//    else if(p_state_cap_wen)
//        p_state_cap_wadrs <= p_state_cap_wadrs + 1'b1;
//    else
//        ;
//end
//always@(posedge hba_clk)
//    p_state_cap_dout_1d <= p_state_cap_dout;
//
//
//always@(posedge hba_clk)
//begin
//    if(d2h_valid & d2h_ready)
//        $display(" host rx user = %h, data = %h",d2h_user,d2h_data);
//    else
//        ;
//end
//
//
//
//
//
//reg [76:0]                      cbus_cap_din;
//reg                             cbus_cap_wen;
//(* keep = "true" *)reg  [9:0]   cbus_cap_wadrs;
//wire[76:0]                      cbus_cap_dout;
//(* keep = "true" *)reg [76:0]   cbus_cap_dout_1d;
//reg cbus_req_1d;
//always@(posedge hba_clk)
//    cbus_req_1d <= dbg_cbus_req;
//always@(posedge hba_clk)
//begin
//    if(cbus_req_1d & ~dbg_cbus_req & dbg_cbus_ack)
//    begin
//        cbus_cap_din[76]    <= dbg_cbus_rw;
//        cbus_cap_din[75:40] <= {2'b0,curr_time[35:2]};
//        cbus_cap_din[39:32] <= {1'b0,dbg_cbus_addr};
//        cbus_cap_din[31:0]  <= (dbg_cbus_rw)? dbg_cbus_rdata : dbg_cbus_wdata;
//        cbus_cap_wen <= 1'b1;
//    end
//    else
//        cbus_cap_wen <= 1'b0;
//end
//always@(posedge hba_clk)
//begin
//    if(clear)
//        cbus_cap_wadrs <= 10'b0;
//    else if(cbus_cap_wen)
//        cbus_cap_wadrs <= cbus_cap_wadrs + 1'b1;
//    else
//        ;
//end
//sdp_ram #(
//    .DATA_WIDTH     ( 77        ),
//    .ADDR_WIDTH     ( 10        ),
//    .READ_LATENCY   ( 2         ),
//    .MEMORY_TYPE    ( "block"   )
//)
//cbus_cap_ram(
//    .clka           ( hba_clk       ),
//    .addra          ( cbus_cap_wadrs),
//    .dina           ( cbus_cap_din  ),
//    .wea            ( cbus_cap_wen  ),
//    .clkb           ( hba_clk       ),
//    .addrb          ( cap_radrs     ),
//    .doutb          ( cbus_cap_dout )
//);
//always@(posedge hba_clk)
//    cbus_cap_dout_1d <= cbus_cap_dout;
//
//
//
//
//(* keep = "true" *)reg          p_state_error;
//(* keep = "true" *)reg [7:0]    p_state_err_dly;
//always@(posedge hba_clk)
//    p_state_1d <= p_state;
//always@(posedge hba_clk)
//begin
//    if(p_state==6'h07 & p_state_1d==6'h31)
//        p_state_error <= 1'b1;
//    else if(p_state==6'h31 & p_state_1d==6'h07)
//        p_state_error <= 1'b1;
//    else
//        p_state_error <= 1'b0;
//end
//always@(posedge hba_clk)
//begin
//    if(p_state_error)
//        p_state_err_dly <= p_state_err_dly[7]? p_state_err_dly : p_state_err_dly + 1'b1;
//    else
//        p_state_err_dly <= 8'b0;
//end


endmodule
