`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Behzad Dah Dahee 
// bvd5281@psu.edu
// CMPEN 331 Section 001
//////////////////////////////////////////////////////////////////////////////////


module system(
input clk,
output out
//output [31:0] pcin,pcout,adderin,adderout
    );
assign out = 0;
wire [31:0] addertopc, pctoadder, a, memtoif_id, if_idtocontrol, if_idtomux, if_idtoreg,
if_idtoextender, wregtoid, m2regtoid, wmemtoid, aluctoid, aluimmtoid, regrttomux, muxtoid, qatoid, qbtoid, 
extendertoid, aluimmtomux2, qbtomux2, extendtomux2, aluctoalu, qatomux4, mux2tomux5, ewregtoexe, em2regtoexe,
ewmemtoexe, alutoexe, qbtoexe, emux1resulttoexe, mrtodata, qbtodata, mrtowb, mmux1resulttowb,
mm2regtowb, mwregtowb, datatowb, mux3toreg, wbtoreg1, wbtomux3_1, wbtomux3_2, wbtomux3_3, wbtoreg2
,if_idtoid, exetoforward1, exetoforward2, exetomem, memtoforward1, memtoforward2, wbtoforward1, wbtoforward2,
memtowb, memtomux4, memtomux5, mux3tomux4, mux3tomux5, mux4toalu, mux5toalu, forwardtomux4, forwardtomux5;
PC pc_instance(
.clk(clk),
.pcin(addertopc),
.pcout(pctoadder),
.pctomem(a)
);

adder adder_instance(
.adderin(pctoadder),
.adderout(addertopc)
);

instmem instmem_instance(
.pc(a),
.do(memtoif_id)
);

IF_ID if_idinstance(
.clk(clk),
.if_idin(memtoif_id),
.if_idoutcontrol(if_idtocontrol),
.if_idoutmux(if_idtomux),
.if_idoutreg(if_idtoreg),
.if_idoutextender(if_idtoextender),
.if_idout(if_idtoid)
);

control_unit controlu_instance(
.controlin(if_idtocontrol),
.wreg(wregtoid),
.m2reg(m2regtoid),
.wmem(wmemtoid),
.aluc(aluctoid),
.aluimm(aluimmtoid),
.regrt(regrttomux)
);

mux1 mux_instance1(
.regrt(regrttomux),
.muxin(if_idtomux),
.muxout(muxtoid)

);

regfile regfile_intstance(
.regfilein(if_idtoreg),
.d(mux3toreg),
.wn(wbtoreg2),
.we(wbtoreg1),
.qa(qatoid),
.qb(qbtoid)
);

signextender extender_instance(
.signextenderin(if_idtoextender),
.signextenderout(extendertoid)
);

ID_EXE id_exe_instance(
.clk(clk),
.wregin(wregtoid),
.m2regin(m2regtoid),
.wmemin(wmemtoid),
.alucin(aluctoid),
.aluimmin(aluimmtoid),
.muxresultin(muxtoid),
.qain(qatoid),
.qbin(qbtoid),
.immextendin(extendertoid),
.wregout(ewregtoexe),
.m2regout(em2regtoexe),
.wmemout(ewmemtoexe),
.aluimmout(aluimmtomux2),
.qbout(qbtomux2),
.qaout(qatomux4),
.immextendout(extendtomux2),
.qbout2(qbtoexe),
.muxresultout(emux1resulttoexe),
.instruction(if_idtoid),
.rsout(exetoforward1),
.rtout(exetoforward2),
.rdout(exetomem),
.alucout(aluctoalu)
);

mux2 mux2_instance(
.aluimm_mux(aluimmtomux2),
.qbmux(qbtomux2),
.extendermux(extendtomux2),
.muxout(mux2tomux5)
);

alu alu_instance(
.aluc(aluctoalu),
.a(mux4toalu),
.b(mux5toalu),
.aluout(alutoexe)
);


EXE_MEM exe_mem_instance(
.ewreg(ewregtoexe),
.em2reg(em2regtoexe),
.ewmem(ewmemtoexe),
.clk(clk),
.emux1result(emux1resulttoexe),
.r(alutoexe),
.eqb(qbtoexe),
.mr(mrtodata),
.mqb(qbtodata),
.mwreg(mwregtowb),
.mm2reg(mm2regtowb),
.mmux1result(mmux1resulttowb),
.mr2(mrtowb),
.rd(exetomem),
.mrd(memtowb),
.mmux1result2(memtoforward1),
.mwreg2(memtoforward2),
.mr3(memtomux4),
.mr4(memtomux5)
);


datamemory datamemory_instance(
.a(mrtodata),
.writedata(qbtodata),
.do(datatowb)
);


MEM_WB mem_wb_instance(
.clk(clk),
.mwreg(mwregtowb),
.mm2reg(mm2regtowb),
.mmux1result(mmux1resulttowb),
.mr(mrtowb),
.do(datatowb),
.wwreg(wbtoreg1),
.wm2reg(wbtomux3_3),
.wmux1result(wbtoreg2),
.wr(wbtomux3_1),
.wdo(wbtomux3_2),
.mrd(memtowb),
.wmux1result2(wbtoforward1),
.wwreg2(wbtoforward2)
);


mux3 mux3_instance(
.wr(wbtomux3_1),
.wdo(wbtomux3_2),
.wm2reg(wbtomux3_3),
.mux3out(mux3toreg),
.mux3out2(mux3tomux4),
.mux3out3(mux3tomux5)
);


forwarding_unit funit_instance(
.rs(exetoforward1),
.rt(exetoforward2),
.ex_rd(memtoforward1),
.wb_rd(wbtoforward1),
.mwreg(memtoforward2),
.wwreg(wbtoforward2),
.forwarda(forwardtomux4),
.forwardb(forwardtomux5)
);


mux4 mux4_instance(
.eqa(qatomux4),
.mr(memtomux4),
.mux3out(mux3tomux4),
.mux4out(mux4toalu),
.forwardA(forwardtomux4)
);


mux5 mux5_instance(
.mux2out(mux2tomux5),
.mr(memtomux5),
.mux3out(mux3tomux5),
.mux5out(mux5toalu),
.forwardB(forwardtomux5)
);
endmodule



module PC(
    input clk,
    input [31:0] pcin,
    output reg [31:0] pcout, pctomem
);
always @ (posedge clk)
begin
    pcout <= pcin;
    pctomem <=pcin;
end
endmodule



module adder(
    input [31:0] adderin,
    output reg [31:0] adderout
);
always @ (adderin)
begin
    adderout = adderin + 4;
end
endmodule


module instmem(
    input [31:0] pc,
    output reg [31:0] do
);
reg [31:0] im [1023:0];
initial begin
im[100] = 32'h00221820;
im[104] = 32'h01232022;
im[108] = 32'h00692825;
im[112] = 32'h00693026;
im[116] = 32'h00693824;
end
always @(pc)
begin
    do = im[pc];
end
endmodule



module IF_ID(
    input clk,
    input [31:0] if_idin,
    output reg [31:0] if_idoutcontrol,if_idoutmux, if_idoutreg, if_idoutextender,if_idout
);
always @ (posedge clk)
begin
    if_idoutcontrol <= if_idin;
    if_idoutmux <= if_idin;
    if_idoutreg <= if_idin;
    if_idoutextender <= if_idin;
    if_idout <= if_idin;
end
endmodule


module control_unit(
    input [31:0] controlin,
    output reg wreg, m2reg, wmem, aluimm, regrt,
    output reg [3:0] aluc
);
reg [5:0] op,func;
always @ (controlin)
begin
    op = controlin [31:26];
    func = controlin [5:0];
    if (op == 0)
    begin
        if (func == 32)
        begin
            wreg <= 1'b1;
            m2reg <= 1'b0;
            wmem <= 0;
            aluc <= 2;
            aluimm <= 1'b0;
            regrt <= 1'b0;
        end
        if (func == 34)
        begin
            wreg <= 1'b1;
            m2reg <= 1'b0;
            wmem <= 0;
            aluc <= 6;
            aluimm <= 1'b0;
            regrt <= 1'b0;
        end
        if (func == 36)
        begin
            wreg <= 1'b1;
            m2reg <= 1'b0;
            wmem <= 0;
            aluc <= 0;
            aluimm <= 1'b0;
            regrt <= 1'b0;
        end
        if (func == 37)
        begin
            wreg <= 1'b1;
            m2reg <= 1'b0;
            wmem <= 0;
            aluc <= 1;
            aluimm <= 1'b0;
            regrt <= 1'b0;
        end
        if (func == 38)
        begin
            wreg <= 1'b1;
            m2reg <= 1'b0;
            wmem <= 0;
            aluc <= 7;
            aluimm <= 1'b0;
            regrt <= 1'b0;
        end                  
    end
    else
    begin
        wreg <= 1'b1;
        m2reg <= 1'b1;
        wmem <= 0;
        aluc <= 2;
        aluimm <= 1'b1;
        regrt <= 1'b1;
    end
end

endmodule


module mux1(
    input regrt,
    input [31:0] muxin,
    output reg [4:0] muxout
);
reg [4:0] rd, rt;
always @ (muxin)
begin
    rd = muxin [15:11];
    rt = muxin [20:16];
    if ( regrt == 0 )
    begin
    muxout = rd;
    end
    else
    begin
    muxout = rt;
    end
end
endmodule



module regfile(
    input [31:0] regfilein,d,
    input [4:0] wn,
    input we,
    output reg [31:0] qa, qb
);
reg [4:0] rs, rt;
reg [31:0] regs [31:0];
integer i;
initial begin
    for (i=0; i<32; i=i+1)
    begin
        regs[i] = 0;
    end
    regs [0] = 32'ha00000aa;
    regs [1] = 32'h10000011;
    regs [2] = 32'h20000022;
    regs [3] = 32'h30000033;
    regs [4] = 32'h40000044;
    regs [5] = 32'h50000055;
    regs [6] = 32'h60000066;
    regs [7] = 32'h70000077;
    regs [8] = 32'h80000088;
    regs [9] = 32'h90000099;
end
always @(regfilein or wn or d)
begin
    rs = regfilein[25:21];
    rt = regfilein[20:16];
    if (we == 1)
    begin
        regs[wn] = d;
    end
    qa = regs[rs];
    qb = regs[rt];
end
endmodule


module signextender(
    input [31:0] signextenderin,
    output reg [31:0] signextenderout
);
reg [15:0] imm;
always @ (signextenderin)
begin
    imm = signextenderin [15:0];
    signextenderout = {{16{imm[15]}},imm};
end
endmodule


module ID_EXE(
    input wregin, m2regin, wmemin, aluimmin,clk,
    input [3:0] alucin,
    input [4:0] muxresultin,
    input [31:0] qain, qbin, immextendin,instruction,
    output reg wregout, m2regout, wmemout, aluimmout,
    output reg [3:0] alucout,
    output reg [4:0] muxresultout,rsout,rtout,rdout,
    output reg [31:0] qaout, qbout, qbout2, immextendout
);
reg [4:0] rs, rt,rd;
always @ (posedge clk)
begin
    rs = instruction [25:21];
    rt = instruction [20:16];
    rd = instruction [15:11];
    wregout <= wregin;
    m2regout <= m2regin;
    wmemout <= wmemin;
    alucout <= alucin;
    aluimmout <= aluimmin;
    muxresultout <= muxresultin;
    qaout <= qain;
    qbout <= qbin;
    qbout2 <= qbin;
    immextendout <= immextendin;
    rsout <= rs;
    rtout <= rt;
    rdout <= rd;
end
endmodule 


module forwarding_unit(
    input [4:0] rs, rt, ex_rd, wb_rd,
    input mwreg, wwreg,
    output reg [1:0] forwarda, forwardb
);
    integer x;
    integer y;
    always @ (rs or rt or ex_rd or wb_rd)
    begin
        x = 0;
        y = 0;
        if(mwreg != 0 && ex_rd !=0)
        begin
            if (rs == ex_rd)
            begin
                forwarda <= 2;
                x <= 2;            
            end
            if (rt == ex_rd)
            begin
                forwardb <= 2;
                y <= 2;
            end
        end
        
        if(wwreg != 0 && wb_rd !=0)
        begin
            if (rs == wb_rd)
            begin
                forwarda <= 1;
                x <= 1;            
            end
            if (rt == wb_rd)
            begin
                forwardb = 1;
                y <= 1;
            end
        end
        if (x == 0)
        begin
            forwarda = 0;
        end
        if (y == 0)
        begin
            forwardb = 0;
        end       
    end
endmodule



module mux4(
    input [31:0] eqa, mr,mux3out,
    input [1:0] forwardA,
    output reg [31:0] mux4out
);
    always @(eqa or mr or mux3out or forwardA)
    begin
        if(forwardA == 0)
        begin
            mux4out = eqa;
        end
        if(forwardA == 1)
        begin
            mux4out = mr;
        end
        if(forwardA == 2)
        begin
            mux4out = mux3out;
        end
    end
endmodule



module mux5(
    input [31:0] mux2out,mr,mux3out,
    input [1:0] forwardB,
    output reg [31:0] mux5out
);
    always @(mux2out or mr or mux3out or forwardB)
    begin
        if(forwardB == 0)
        begin
            mux5out = mux2out;
        end
        if(forwardB == 1)
        begin
            mux5out = mr;
        end
        if(forwardB == 2)
        begin
            mux5out = mux3out;
        end
    end
endmodule


module mux2 (
    input aluimm_mux,
    input [31:0] qbmux, extendermux,
    output reg [31:0] muxout
);
always @(qbmux or extendermux)
begin
    if (aluimm_mux == 0)
    begin
        muxout = qbmux;
    end
    else
    begin
        muxout = extendermux;
    end
end
endmodule


module alu(
    input[3:0] aluc,
    input [31:0] a, b,
    output reg [31:0] aluout
);
always @(a or b or aluc)
begin
    if (aluc == 2)
    begin
    aluout = a + b;
    end
    if (aluc == 6)
    begin
    aluout = a - b;
    end
    if (aluc == 0)
    begin
    aluout = a & b;
    end
    if (aluc == 1)
    begin
    aluout = a | b;
    end
    if (aluc == 7)
    begin
    aluout = a ^ b;
    end
end
endmodule


module EXE_MEM(
    input ewreg, em2reg, ewmem, clk,
    input [4:0] emux1result,rd,
    input [31:0] r,eqb,
    output reg mwreg, mwreg2, mm2reg, mwmem,
    output reg [4:0] mmux1result,mmux1result2,mrd,mrd2,
    output reg [31:0] mr, mqb, mr2, mr3, mr4
);
always @ (posedge clk)
begin
    mwreg <= ewreg;
    mwreg2 <= ewreg;
    mm2reg <= em2reg;
    mwmem <= ewmem;
    mmux1result <= emux1result;
    mmux1result2 <= emux1result;
    mr <= r;
    mr2 <= r;
    mqb <= eqb;
    mrd <= rd;
    mrd2 <= rd;
    mr3 <= r;
    mr4 <= r;  
end
endmodule



module datamemory (
    input [31:0] a, writedata,
    output reg [31:0] do    
);
reg [31:0] data [31:0];
integer i;
initial begin
    for (i=0; i<32; i=i+1)
    begin
        data[i] = 0;
    end
    data [0] = 32'ha00000aa;
    data [4] = 32'h10000011;
    data [8] = 32'h20000022;
    data [12] = 32'h30000033;
    data [16] = 32'h40000044;
    data [20] = 32'h50000055;
    data [24] = 32'h60000066;
    data [38] = 32'h70000077;
    data [32] = 32'h80000088;
    data [36] = 32'h90000099;
end
always @(a)
begin
    do = data[a];
end
endmodule


module MEM_WB(
    input clk, mwreg, mm2reg,
    input [4:0] mmux1result,mrd,
    input [31:0] mr, do,
    output reg wwreg, wwreg2, wm2reg,
    output reg [4:0] wmux1result,wrd,wmux1result2,
    output reg [31:0] wr, wdo
);
always @ (posedge clk)
begin
    wwreg <= mwreg;
    wwreg2 <= mwreg;
    wm2reg <= mm2reg;
    wmux1result <= mmux1result;
    wmux1result2 <= mmux1result;
    wr <= mr;
    wdo <= do;
    wrd <= mrd;
end
endmodule


module mux3(
    input [31:0] wr, wdo,
    input wm2reg,
    output reg [31:0] mux3out, mux3out2, mux3out3     
);
always @(wr or wdo)
begin
    if (wm2reg == 0)
    begin
        mux3out <= wr;
        mux3out2 <= wr;
        mux3out3 <= wr;
    end
    else
    begin
        mux3out <= wdo;
        mux3out2 <= wdo;
        mux3out3 <= wdo;
    end
end
endmodule