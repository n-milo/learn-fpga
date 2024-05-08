
all: cpu.v
	iverilog -DBENCH -DBOARD_FREQ=10 bench_iverilog.v $^

run: all
	vvp a.out

