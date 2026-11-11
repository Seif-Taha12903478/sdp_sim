Demo attempt 1 (2026-11-11 08:43-12:44, renamed from demo_20261111_084304): 2/3 deliveries OK, Room 2 not run.
Room 1 OK (1.6 mm), Room 3 OK (1.5 mm), Room 4 FAILED at step 7 "arm home (carrying)": the delivery
process's /joint_states cache froze at 0.040 rad error while Gazebo showed the arm exactly home; the
wall-clock 3600 s arm-wait timeout then failed the delivery and the scheduler stopped. Partial video:
system_demo.mp4 (78 s). Fix: sim-time arm wait + stale-cache recovery + settled acceptance (2026-11-11).
