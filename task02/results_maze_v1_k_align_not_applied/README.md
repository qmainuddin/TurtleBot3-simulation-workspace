# First maze run of plans H and G (archived, 24 Oct 2026)

These 13 trials requested `k_align: 2.0` through `nav_params`, but the navigator node only accepted
a hand-written list of parameter names that did not include `k_align`, so it was silently ignored.

- Plan H here is therefore 4 more trials of the ORIGINAL Bug2 follower (same as plan F bug2): 3/4 success.
- Plan G here used the original follower with v_max 0.22 (2/3), d_follow 0.25 (3/3) and wheel-only odometry (0/3);
  those parameters WERE applied (they were in the list).

The navigator now derives its parameter list from the planner dataclasses, a unit test checks this,
and plans H and G were re-run. Kept for transparency; not used in the main tables.
