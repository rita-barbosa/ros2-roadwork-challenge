# Episode summary: results/d1

| run | result | finished | violations | sim time [s] | min clearance [m] | max speed [m/s] | stopped [s] | rejected cmds | GO latched | GO confirm latency [s] | state transitions |
|---|---|---|---|---:|---:|---:|---:|---:|---|---:|---:|
| s1_seed1 | PASS | True | none | 25.05 | 1.167 | 3.500 | 0.60 | 5 | t=0.55 x=5.00 | - | 7 |
| s1_seed2 | PASS | True | none | 25.00 | 1.167 | 3.500 | 0.55 | 1 | t=0.50 x=5.00 | - | 7 |
| s1_seed3 | PASS | True | none | 25.05 | 1.167 | 3.500 | 0.60 | 2 | t=0.50 x=5.00 | - | 7 |

rejected_commands includes startup ticks before the first valid command.
