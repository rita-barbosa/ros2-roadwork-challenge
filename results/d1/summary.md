# Episode summary: results/d1

| run | result | finished | violations | sim time [s] | min clearance [m] | max speed [m/s] | stopped [s] | rejected cmds | GO latched | GO confirm latency [s] | state transitions |
|---|---|---|---|---:|---:|---:|---:|---:|---|---:|---:|
| s1_seed1 | PASS | True | none | 25.05 | 1.167 | 3.500 | 0.60 | 2 | t=6.45 x=21.65 | - | 7 |
| s1_seed2 | PASS | True | none | 25.05 | 1.167 | 3.500 | 0.60 | 8 | t=6.45 x=21.65 | - | 9 |
| s1_seed3 | PASS | True | none | 25.00 | 1.167 | 3.500 | 0.55 | 1 | t=6.40 x=21.65 | - | 7 |
| s2_seed1 | PASS | True | none | 29.55 | 1.168 | 3.500 | 2.05 | 4 | t=11.05 x=23.69 | 0.20 | 7 |
| s2_seed2 | PASS | True | none | 29.55 | 1.168 | 3.500 | 2.05 | 22 | t=11.05 x=23.69 | 0.15 | 7 |
| s2_seed3 | PASS | True | none | 29.55 | 1.168 | 3.500 | 1.90 | 1 | t=11.05 x=23.70 | 0.20 | 8 |

rejected_commands includes startup ticks before the first valid command.
