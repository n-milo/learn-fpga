
all:
	iverilog $(VFLAGS) -DBENCH -DBOARD_FREQ=10 bench_iverilog.v SOC.v

run:
	iverilog $(VFLAGS) -DBENCH -DBOARD_FREQ=10 bench_iverilog.v SOC.v && vvp a.out

