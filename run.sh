iverilog -DBENCH -DBOARD_FREQ=10 bench_iverilog.v $1 && vvp a.out
