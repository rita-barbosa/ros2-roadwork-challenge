# Episode summary: results/d2/var_shift_m2

```
date: 2026-11-25T17:19:13+00:00
software: d1-submission-4-g63c8bf9
ros_distro: jazzy  ROS_DOMAIN_ID: 37
host: Linux alf-Ubuntu-24-04-PC-Q35-ICH9-2009 7.0.0-42-generic #42~24.04.1-Ubuntu SMP PREEMPT_DYNAMIC Mon Oct 19 14:43:01 UTC 2 x86_64 x86_64 x86_64 GNU/Linux
cpu:  AMD Ryzen 5 7600X3D 6-Core Processor
cpus: 4  mem: 7.8Gi
visualization: off (rviz:=false visualize:=false)
scenarios: 1 2 3 4 5  seeds: 1 2 3  extra: work_shift:=-2.0  label: geometry variation
```

| run | result | violations | sim time [s] | min clearance [m] | max speed [m/s] | stopped [s] | rejected cmds | entry t [s] | GO latched | mean/max \|cte\| [m] | scenario measurements | transitions |
|---|---|---|---:|---:|---:|---:|---:|---:|---|---|---|---:|
| s1_seed1 | PASS | none | 25.15 | 1.179 | 3.500 | 0.55 | 1 | 7.15 | t=6.40 x=21.65 | 0.071 / 0.294 | unexplained stops 0 | 7 |
| s1_seed2 | PASS | none | 25.15 | 1.179 | 3.500 | 0.55 | 6 | 7.15 | t=6.40 x=21.65 | 0.071 / 0.295 | unexplained stops 0 | 7 |
| s1_seed3 | PASS | none | 25.15 | 1.179 | 3.500 | 0.55 | 3 | 7.15 | t=6.40 x=21.65 | 0.071 / 0.294 | unexplained stops 0 | 7 |
| s2_seed1 | PASS | none | 29.60 | 1.180 | 3.500 | 2.05 | 8 | 11.35 | t=11.05 x=23.69 | 0.089 / 0.304 | GO +0.20 s, resume +0.20 s | 8 |
| s2_seed2 | PASS | none | 29.60 | 1.180 | 3.500 | 2.05 | 1 | 11.35 | t=11.05 x=23.69 | 0.090 / 0.304 | GO +0.20 s, resume +0.20 s | 8 |
| s2_seed3 | PASS | none | 29.60 | 1.180 | 3.500 | 2.05 | 1 | 11.35 | t=11.05 x=23.69 | 0.088 / 0.304 | GO +0.20 s, resume +0.20 s | 8 |
| s3_seed1 | PASS | none | 37.80 | 1.180 | 3.500 | 10.25 | 2 | 19.55 | t=19.25 x=23.69 | 0.064 / 0.304 | yield 17.9 s, gap safe t=18.4, car clr 1.69 m | 6 |
| s3_seed2 | PASS | none | 37.80 | 1.180 | 3.500 | 10.25 | 4 | 19.55 | t=19.25 x=23.69 | 0.064 / 0.303 | yield 17.9 s, gap safe t=18.4, car clr 1.69 m | 8 |
| s3_seed3 | PASS | none | 37.80 | 1.180 | 3.500 | 10.25 | 9 | 19.55 | t=19.25 x=23.69 | 0.064 / 0.304 | yield 17.7 s, gap safe t=18.4, car clr 1.69 m | 9 |
| s4_seed1 | PASS | none | 27.35 | 1.180 | 3.500 | 0.65 | 9 | 9.15 | t=8.85 x=23.59 | 0.084 / 0.302 | beacon stale t=2.70, GO +0.20 s, resume +0.20 s | 9 |
| s4_seed2 | PASS | none | 27.35 | 1.180 | 3.500 | 0.65 | 3 | 9.15 | t=8.85 x=23.59 | 0.084 / 0.302 | beacon stale t=2.70, GO +0.20 s, resume +0.20 s | 9 |
| s4_seed3 | PASS | none | 27.35 | 1.181 | 3.500 | 0.65 | 9 | 9.15 | t=8.85 x=23.59 | 0.086 / 0.303 | beacon stale t=2.70, GO +0.20 s, resume +0.20 s | 9 |
| s5_seed1 | PASS | none | 32.05 | 1.179 | 3.500 | 5.50 | 6 | 7.15 | t=6.40 x=21.65 | 0.073 / 0.295 | detect +0.05 s, brake +1.50 s, stop x=51.54, obst clr 1.98 m | 8 |
| s5_seed2 | PASS | none | 32.05 | 1.179 | 3.500 | 5.50 | 2 | 7.15 | t=6.40 x=21.65 | 0.074 / 0.295 | detect +0.05 s, brake +1.50 s, stop x=51.54, obst clr 1.98 m | 8 |
| s5_seed3 | PASS | none | 32.05 | 1.179 | 3.500 | 5.50 | 10 | 7.20 | t=6.45 x=21.65 | 0.073 / 0.295 | detect +0.00 s, brake +1.50 s, stop x=51.54, obst clr 1.98 m | 9 |

**15/15 runs pass.** min clearance is the evaluator's disc-to-disc minimum over all objects; rejected_commands includes startup ticks before the first valid command.
