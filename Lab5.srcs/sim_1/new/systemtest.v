`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Behzad Dah Dahee 
// bvd5281@psu.edu
// CMPEN 331 Section 001
//////////////////////////////////////////////////////////////////////////////////


module systemtest();
reg clk;
wire [31:0] pcin, pcout,adderin,adderout;
wire [31:0]  pctoadder,addertopc;
parameter clock_period = 2;
system uut(clk
);  
always  #(clock_period/2) clk = !clk;  
    initial  
	begin
	clk =0;
	uut.pc_instance.pcout = 96;
	#20 $finish;  
    end  

endmodule
