# SDC - VibraDistill-Edge, Tang Primer 20K (GW2A-LV18PG256)

create_clock -name clk_27m -period 37.037 -waveform {0 18.518} [get_ports {clk_27m}]

create_generated_clock -name clk_100m -source [get_ports {clk_27m}] -master_clock clk_27m -multiply_by 26 -divide_by 7 [get_pins {u_pll/CLKOUT}]

set_clock_uncertainty 0.5 -setup -from [get_clocks {clk_100m}] -to [get_clocks {clk_100m}]
set_clock_uncertainty 0.2 -hold  -from [get_clocks {clk_100m}] -to [get_clocks {clk_100m}]
