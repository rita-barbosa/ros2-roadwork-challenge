# Episode summary: results/d2/var_shift_p2

```
date: 2026-11-25T17:28:07+00:00
software: d1-submission-4-g63c8bf9
ros_distro: jazzy  ROS_DOMAIN_ID: 37
host: Linux alf-Ubuntu-24-04-PC-Q35-ICH9-2009 7.0.0-42-generic #42~24.04.1-Ubuntu SMP PREEMPT_DYNAMIC Mon Oct 19 14:43:01 UTC 2 x86_64 x86_64 x86_64 GNU/Linux
cpu:  AMD Ryzen 5 7600X3D 6-Core Processor
cpus: 4  mem: 7.8Gi
visualization: off (rviz:=false visualize:=false)
scenarios: 1 2 3 4 5  seeds: 1 2 3  extra: work_shift:=2.0  label: geometry variation
```

| run | result | violations | sim time [s] | min clearance [m] | max speed [m/s] | stopped [s] | rejected cmds | entry t [s] | GO latched | mean/max \|cte\| [m] | scenario measurements | transitions |
|---|---|---|---:|---:|---:|---:|---:|---:|---|---|---|---:|
| s1_seed1 | PASS | none | 25.05 | 1.155 | 3.500 | 0.60 | 11 | 7.15 | t=6.45 x=21.65 | 0.065 / 0.295 | unexplained stops 0 | 8 |
| s1_seed2 | PASS | none | 25.05 | 1.155 | 3.500 | 0.60 | 6 | 7.15 | t=6.45 x=21.65 | 0.065 / 0.294 | unexplained stops 0 | 7 |
| s1_seed3 | PASS | none | 25.05 | 1.156 | 3.500 | 0.60 | 3 | 7.15 | t=6.45 x=21.65 | 0.065 / 0.295 | unexplained stops 0 | 7 |
| s2_seed1 | PASS | none | 29.55 | 1.156 | 3.500 | 2.05 | 3 | 11.35 | t=11.05 x=23.69 | 0.082 / 0.295 | GO +0.20 s, resume +0.20 s | 8 |
| s2_seed2 | PASS | none | 29.55 | 1.156 | 3.500 | 2.05 | 1 | 11.35 | t=11.05 x=23.69 | 0.083 / 0.293 | GO +0.20 s, resume +0.20 s | 8 |
| s2_seed3 | PASS | none | 29.55 | 1.156 | 3.500 | 2.05 | 2 | 11.35 | t=11.05 x=23.69 | 0.082 / 0.295 | GO +0.20 s, resume +0.20 s | 8 |
| s3_seed1 | PASS | none | 37.25 | 1.156 | 3.500 | 9.75 | 2 | 19.05 | t=18.75 x=23.69 | 0.061 / 0.294 | yield 16.2 s, gap safe t=17.9, car clr 1.69 m | 8 |
| s3_seed2 | PASS | none | 37.25 | 1.156 | 3.500 | 9.75 | 6 | 19.05 | t=18.75 x=23.69 | 0.058 / 0.295 | yield 17.4 s, gap safe t=17.9, car clr 1.69 m | 8 |
| s3_seed3 | PASS | none | 37.25 | 1.156 | 3.500 | 9.75 | 3 | 19.05 | t=18.75 x=23.69 | 0.058 / 0.295 | yield 17.4 s, gap safe t=17.9, car clr 1.69 m | 8 |
| s4_seed1 | PASS | none | 27.25 | 1.155 | 3.500 | 0.60 | 3 | 9.15 | t=8.85 x=23.59 | 0.080 / 0.295 | beacon stale t=2.70, GO +0.20 s, resume +0.20 s | 9 |
| s4_seed2 | PASS | none | 27.25 | 1.156 | 3.500 | 0.60 | 5 | 9.15 | t=8.85 x=23.59 | 0.076 / 0.295 | beacon stale t=2.70, GO +0.20 s, resume +0.20 s | 9 |
| s4_seed3 | PASS | none | 27.15 | 1.155 | 3.500 | 1.05 | 10 | 9.15 | t=8.75 x=23.33 | 0.079 / 0.295 | beacon stale t=2.70, GO +0.20 s, resume +0.20 s | 9 |
| s5_seed1 | PASS | none | 31.90 | 1.156 | 3.500 | 5.50 | 1 | 7.15 | t=6.45 x=21.65 | 0.069 / 0.296 | detect +0.00 s, brake +1.50 s, stop x=55.53, obst clr 1.98 m | 8 |
| s5_seed2 | PASS | none | 31.90 | 1.156 | 3.500 | 5.50 | 1 | 7.15 | t=6.45 x=21.65 | 0.068 / 0.294 | detect +0.05 s, brake +1.50 s, stop x=55.53, obst clr 1.98 m | 8 |
| s5_seed3 | PASS | none | 31.90 | 1.155 | 3.500 | 5.50 | 3 | 7.15 | t=6.45 x=21.65 | 0.070 / 0.293 | detect +0.05 s, brake +1.50 s, stop x=55.53, obst clr 1.98 m | 8 |

**15/15 runs pass.** min clearance is the evaluator's disc-to-disc minimum over all objects; rejected_commands includes startup ticks before the first valid command.
