# Report outline (REP-01)

Framework for the semester report. Each section lists what it must contain, where
the material already is, and which issue criterion it covers. Write the text in the
report document; keep this file as the plan and tick sections off.

## Facts

| | |
|---|---|
| Deadline | **27.11.2026 kl. 14:00** (semesteroppgave, hvl.no course description 2026/2027) |
| Weight | Semester project 25 %, oral exam 75 % (30 min, no aids, starts 02.12.2026). Both parts must be passed. |
| Requirement (competition page) | "clearly explain how the robots communicate during S&R, which navigation algorithm is running and how you implemented those elements. The navigation algorithm needs to be one where you have done significant implementation yourself" (R7, R8) |
| Length, template, language | **Not on the course website or hvl.no: check Canvas** (ARCHITECTURE.md §15 Q4). Teaching language: Norwegian and English. |
| Also to deliver | Work log per member (R10, `docs/worklog/<firstname>.md`), AI-use declaration, repository with run instructions (R9) |

Until Canvas says otherwise: aim for about 12–15 pages plus appendices, and put the
weight on sections 3–5, which the requirement names.

---

## 0. Front matter
- Title, group 11, members, course DAT160, date.
- **Summary** (≈ 150 words): task, centralised design, distance-transform navigation,
  coverage viewpoints, result (5/5 and the meeting on all five worlds, average 1071).

## 1. Introduction (≈ 0.5 page)
- The task: two TurtleBot3, five ArUco markers, blueprint map known, marker
  positions unknown, 10 minutes, scoring service, meeting at the big fire.
- Scoring formula and its consequence: the clock only stops when everything is done,
  so finding every marker comes before speed (ARCHITECTURE.md §1, "Priority").
- Requirements R1–R10 in one short table.
- Structure of the report.

**Sources:** ARCHITECTURE.md §1, competition page.

## 2. System overview (≈ 1.5 pages)
- **Figure 1: system overview** (nodes and topics/services/actions). Source: the
  mermaid diagram in ARCHITECTURE.md §3, or the ASCII diagram in SYSTEM_GUIDE.md §1.
- The three nodes and their responsibilities (table, §3).
- Hybrid architecture: deliberative / executive / reactive layers (§3).
- Why centralised, and the decentralised alternative (§3 "Why centralised").
- What we measured before designing (spikes): odometry is ground truth, camera
  range 1.0–3.5 m, UDP versus shared memory, scoring behaviour (§2, SPK-01/02).

## 3. Communication (≈ 2 pages) — **R8, REP-02 criterion 1**
- Table of every topic, service and action (§4).
- **Why each pattern:** topic for streams, service for reporting, action for
  navigation (can be replaced or cancelled; the meeting replaces goals) (§4
  "Communication patterns").
- Custom interfaces: `NavigateTo.action`, `FoundMarker.msg`, with the result messages.
- Namespaces and the `robots` parameter: one class, no robot names in the code.
- QoS: transient-local `/map`; Fast-DDS over UDP and why (SPK-02: 0.5–4 versus
  15–20 detections/s).
- Robustness: goal ids, deadlines, ignored stale results; async calls in the leader
  (no `spin_until_future_complete` in callbacks) (§5.1, §10).
- **Figure 2: meeting sequence diagram** (§9, mermaid).

## 4. Navigation (≈ 3–4 pages) — **R7, R8, REP-02 criterion 2**
- 4.1 Grids: downsampling, inflation, configuration space (§6.1).
- 4.2 **Distance transform**: definition, relaxation, corner rule, steepest descent,
  line-of-sight simplification (§6.2, §6.3).
  **Figure 3: distance-transform example on a map** (cost field as a heat map on w1
  with a path drawn; script in `tools/`, image in `docs/report/`).
- 4.3 Why a distance transform and not D*, PRM, RRT, lattice planners or A*; why
  map-based planning and not wall following or Bug2 (§6.2, §6.9).
- 4.4 Path following: go-to-point controller with equations (§6.4).
- 4.5 Reactive layer: lidar safety stop, stuck recovery (§6.5).
- 4.6 Two robots: disc, conflict rule, right of way, timeout, the corridor-deadlock
  fix (§6.6, INT-02).
- 4.7 **What we implemented ourselves** (R7): table from CODE_STRUCTURE.md "Where the
  code comes from": used as-is / course templates / own assignments / written for
  the project.

## 5. Mission, exploration and markers (≈ 3 pages) — **R8, REP-02**
- 5.1 `Mission` state machine: events in, commands out; why it is pure Python
  (testable). **Figure 4: state machine** (§5, mermaid).
- 5.2 Viewpoints: the lattice (§8.1) and the coverage-based pass (§8.4 update,
  `generate_coverage_viewpoints`): wall faces, visibility, greedy set cover.
  Optional **Figure 5:** viewpoints of both methods on w3 with the markers.
- 5.3 Allocation: greedy, nearest by travel distance, reservation; why not an
  auction (§8.2).
- 5.4 Marker pipeline: `marker_detector` (TF at image time, retry queue), why the
  course `marker_map_pose` is replaced, **with the PER-03 table** (test_runs.md,
  PER-03) — **REP-02 criterion 3**.
- 5.5 Reporting: median, retries, `min_sightings` (§7.2).
- 5.6 Big-fire meeting: points A and B, the 1.8 m bound (§9).

## 6. Implementation and testing (≈ 1.5 pages) — **REP-02 criterion 4**
- Package structure, `core/` without ROS versus `nodes/` adapters (§12).
- Concurrency: MultiThreadedExecutor in `robot_handler`, single-threaded leader (§10).
- Parameters in one `params.yaml` (§13).
- Test strategy: 289 unit tests, flake8/pep257, spikes, full runs with two
  simulations in parallel (own ROS domain and Gazebo port).
- Workflow: issues with `impl:own` / `ai-allowed`, PRs with review, owners (§14).

## 7. Results (≈ 2 pages) — **REP-03 criterion 1, 2**
- **Table: scores per world**, final runs (test_runs.md, "Coverage viewpoints"):
  w1 937, w2 1070, w3 1185, w4 1125, w5 1037.
- Development of the score: first full run (INT-01, 3/5, 600) → tuning (INT-02, 4/5
  on three worlds, about 840 average) → coverage viewpoints (5/5 everywhere, 1071).
  Possible figure: score per world for the three stages.
- Tuning and why (INT-02 table): `viewpoint_spacing` 2.5, `spin_pause` 1.0,
  `min_sightings` 1, `spin_clearance` 0.9, `spin_speed` 1.5 justified with SPK-02.
- Robustness numbers: no collisions (smallest gap), no stuck, real-time factor.
- Start-order test (INT-03) and the competition-PC test (INT-04: not possible,
  see the issue).

## 8. Discussion (≈ 1.5 pages) — **REP-03 criterion 3**
- Fixed lattice versus coverage-based viewpoints: what the offline analysis showed
  (markers in narrow corridors had no viewpoint), and what changed.
- Limitations: one run per world (variance), markers whose front cannot be reached,
  a full turn at every viewpoint costs about 9.5 s, single point of failure (leader),
  results measured on the VM and not on the competition PC.
- Improvements: turn only towards the covered walls, revisit areas with uncertain
  sightings, faster speed now that the deadlock is fixed.

## 9. Conclusion (≈ 0.5 page)

## 10. Use of AI tools — **REP-04**
- Which tools, for what, how the output was checked (HVL guidelines).
- Per area, from the issue comments' "AI use" sections and the work logs:
  `impl:own` issues written by the owners with explanations/review from AI;
  `ai-allowed` issues (MIS-02, MIS-03, INT-03, INT-06) with AI-generated code reviewed
  by the owner; test runs and analysis; the coverage viewpoints (written by Claude,
  reviewed by jonkje-11, see the PR).
- Also fill in README, "Use of AI tools".

## References
- Corke, *Robotics, Vision and Control*, ch. 4–6; the competition page; the course
  repository; ROS 2 Foxy documentation; ArUco (OpenCV).

## Appendices
- A: work logs of all members (`docs/worklog/`).
- B: parameter table (`params.yaml`).
- C: full test-run tables (`docs/test_runs.md`).

---

## Figures to make (REP-01 criterion 2)

| # | Figure | From | Status |
|---|---|---|---|
| 1 | System overview | ARCHITECTURE.md §3 (mermaid) | to export |
| 2 | Meeting sequence | ARCHITECTURE.md §9 (mermaid) | to export |
| 3 | Distance transform on a map (cost field + path) | new script in `tools/` | to make |
| 4 | Mission state machine | ARCHITECTURE.md §5 (mermaid) | to export |
| 5 | Viewpoints, lattice versus coverage, on w3 (optional) | new script in `tools/` | to make |
| 6 | Score per world for INT-01 / INT-02 / coverage (optional) | test_runs.md | to make |

Mermaid diagrams can be exported with mermaid.live (paste, download PNG/SVG) into
`docs/report/`.

## Who writes what (suggestion, by ownership in ARCHITECTURE.md §14)

| Section | Owner |
|---|---|
| 4 Navigation, 5.2–5.3 viewpoints and allocation | A |
| 3 Communication, 4.4–4.5 robot node, 5.4 marker pipeline | B |
| 5.1 Mission, 5.5–5.6 reporting and meeting | C |
| 1, 2, 6–9, 10, figures | shared |
