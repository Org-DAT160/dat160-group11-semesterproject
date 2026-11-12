# Report outline (REP-01)

Framework for the semester report, following the course's required structure
("Report content" and "Report structure" in the assignment text). Each section lists
what it must contain, a word budget, and where the material already is.

## Rules from the assignment

| Rule | What it means for us |
|---|---|
| **3000 words ± 10 %** (2700–3300), one PDF, submitted as a group on **WiseFlow** | Word budget per section below. Figures, tables, references and appendices are presumably not counted: check Canvas. |
| **Deadline 27.11.2026 kl. 14:00** | Internal plan at the end of this file. |
| Sections 1–5, references, appendices, **as a minimum** | Exactly the headings below. |
| **Extensive, well-explained figures** | Figure list below. |
| **Links to supplementary material**, recommended through GitHub | ⚠️ Our repository is **private** (`Org-DAT160`). The examiners must be able to open the link: make it public before submission, or make a public copy. |
| **Good-quality videos are required** (e.g. YouTube), linked from the report | ⚠️ We have **no videos yet**. Record full runs (Gazebo window, plus a sped-up version) and upload them. |
| Only own content or openly licensed content, with attribution | Course worlds and models come from the course repository: say so, and do not re-upload them as our own. |
| **Plagiarism check** | Write the text ourselves; quote and cite. AI use is declared (see section 2 and the appendix). |
| **Existing libraries and code** must be documented: (1) what it does, (2) how it is integrated, with the code in an appendix and copied code clearly labelled, (3) why our own code is of suitable complexity | Section 3.6 and Appendix C. Material: `docs/CODE_STRUCTURE.md`, "Where the code comes from". |
| **IEEE references**, numbered `[1]`, cited in the text | Reference list below. |
| **Signed work log** of every member (theory, code, presentations, report writing) | Appendix A. ⚠️ `docs/worklog/` has no logs yet. |

---

## Front matter (not counted)
Title, group 11, members, course DAT160, date, **link to the repository and the videos**.

## 1. Introduction (≈ 450 words)
- **Robot teams for Search & Rescue, from a literature search** (≈ 150 words, 3–5
  references): why multi-robot teams (coverage, redundancy), centralised versus
  decentralised coordination, frontier/coverage-based exploration. Suggested sources
  to look up and cite: Burgard et al. on coordinated multi-robot exploration; a survey
  of multi-robot S&R or exploration (e.g. Murphy, *Disaster Robotics*); the art-gallery
  / coverage problem for viewpoint selection; Corke [textbook].
- **The task and the rules** (≈ 150 words): two TurtleBot3, five ArUco markers,
  blueprint known, positions unknown, 10 min, scoring service, meeting at the big fire,
  scoring formula, no pre-programmed paths, own navigation. Source: competition page,
  ARCHITECTURE.md §1–2.
- **Overview of our approach** (≈ 150 words): centralised leader, viewpoints from the
  blueprint, distance-transform navigation, marker pipeline, meeting. Source:
  SUMMARY.md "The idea in three lines".

## 2. Design process (≈ 700 words)
- **Design iterations** (≈ 300 words):
  1. Design first (ARCHITECTURE.md, issue plan) and spikes that measured the system
     before coding: odometry is ground truth, camera range 1.0–3.5 m, turn-and-look,
     UDP instead of shared memory (SPK-01, SPK-02).
  2. Building blocks with unit tests: grids, distance transform, controller, robot
     node, marker detector (NAV-, PER- issues).
  3. Mission: state machine, leader, reporting, meeting, exploration (MIS- issues).
  4. First full run: 3/5 markers, 600 points (INT-01).
  5. Tuning on all worlds: 4/5 on three worlds, about 840 average (INT-02).
  6. Coverage-based viewpoints after an offline visibility analysis: 5/5 on all
     worlds, 1071 average.
- **Who worked on what, and when** (≈ 150 words): roles A/B/C (ARCHITECTURE.md §14),
  timeline from the merged pull requests (24.10 set-up and spikes → 31.10–06.11
  navigation → 08.–09.11 mission → 09.–11.11 integration and tuning). Detail in the
  work logs (Appendix A). Include how AI tools were used (owners wrote the `impl:own`
  core; AI-generated code in `ai-allowed` issues and the coverage viewpoints, reviewed
  by the owner).
- **Problems and solutions** (≈ 250 words), pick 4–5:
  - camera images slow with Fast-DDS shared memory → UDP profile in the launch file;
  - course `marker_map_pose` up to 3 m wrong while turning → own `marker_detector`
    with TF at the image time (PER-03 table);
  - TF pose arriving after the image → retry queue (57–69 % of sightings saved);
  - markers seen once were never reported → `min_sightings` 1;
  - corridor deadlock between the robots → leader plans around the other robot after
    `peer`;
  - markers in narrow corridors never seen → coverage viewpoints.

## 3. Final implementation (≈ 1050 words)
Be brief; details in the appendices and the repository.
- 3.1 **System overview** (≈ 150 words) + **Figure 1** (nodes and topics/services/
  actions). Communication patterns and why (topic / service / action), centralised.
- 3.2 **Mission logic** (≈ 150 words) + **Figure 2** (state machine): events in,
  commands out, goal ids and deadlines.
- 3.3 **Navigation** (≈ 300 words) + **Figure 3** (distance-transform cost field and
  path on a map): grids, distance transform, path simplification, go-to-point
  controller, safety stop, stuck recovery, two-robot rules. Why a distance transform
  (not A*, D*, RRT, PRM) and why map-based (not wall following / Bug2).
- 3.4 **Exploration** (≈ 200 words) + **Figure 4** (lattice versus coverage
  viewpoints on w3 with the markers): coverage viewpoints, lattice pass, greedy
  allocation, turn-and-look.
- 3.5 **Markers and meeting** (≈ 150 words) + **Figure 5** (meeting sequence):
  marker detector, reporting, meeting points and the 1.8 m bound.
- 3.6 **Existing code and libraries** (≈ 100 words), required by the assignment:
  table of what we used as-is (ROS 2, Gazebo, `ros2_aruco`, `nav2_map_server`, course
  worlds and launch files, NumPy), what each does, how it is integrated, and what we
  wrote ourselves (everything in `core/`, the three nodes) and why that is of suitable
  complexity. Copied/adapted code listed in Appendix C.
Source for all of section 3: SUMMARY.md, SYSTEM_GUIDE.md, ARCHITECTURE.md §3–§12.

## 4. Experiments (≈ 550 words)
- **What we tested** (≈ 150 words): 289 unit tests; spikes; full 10-minute runs, two
  at a time with their own ROS domain and Gazebo port; start-order tests with the
  course world launch; real-time factor recorded.
- **Results table** (≈ 200 words around it): **Table 1**, score per world for the
  three stages (INT-01, INT-02 final, coverage), plus markers, time and meeting.
  Average per world where there are several runs (w1: 937, 946, 829, 700). Source:
  `docs/test_runs.md`.
  **Figure 6** (optional): score per world for the three stages, bar chart.
- **Final competition** (≈ 200 words): ⚠️ **write after the competition**: the drawn
  world, score, what went well and what did not, and why.

## 5. Conclusion (≈ 250 words)
- Briefly: what works (5/5 on every world in testing), what we learned.
- Improvements: turn only towards the walls a viewpoint covers (time), replan the
  route of both robots together, test on the competition PC, more runs per world.

## References (not counted, IEEE)
Numbered in order of first citation, e.g.:
- [1] P. Corke, *Robotics, Vision and Control*, 3rd ed. Springer, 2023. (check the edition we used)
- [2] HVL, "DAT160 Competition," 2026. [Online]. Available: https://frdedynamics.github.io/hvl_robotics_website/courses/dat160/competition
- [3] HVL Robotics, "ros2_students_25," GitHub repository. [Online]. Available: https://github.com/frdedynamics/ros2_students_25
- [4] Open Robotics, "ROS 2 Foxy documentation." [Online]. Available: https://docs.ros.org/en/foxy/
- [5] OpenCV, "Detection of ArUco markers." [Online].
- [6]–[8] the literature for section 1 (to be found).
- Our repository and the videos.

## Appendices (not counted)
- **A. Signed work log** of every member: theory, code, presentations, report writing
  (from `docs/worklog/<firstname>.md`).
- **B. Parameters** (`params.yaml`) and full test-run tables (`docs/test_runs.md`).
- **C. Code**: the most important own code (distance transform, viewpoints, mission
  state machine), and adapted course code clearly labelled with its source.
- **D. AI-use declaration** (what tool, for what, how it was checked).

---

## Figures to make

| # | Figure | From | Status |
|---|---|---|---|
| 1 | System overview | ARCHITECTURE.md §3 (mermaid) → mermaid.live → PNG | to export |
| 2 | Mission state machine | ARCHITECTURE.md §5 (mermaid) | to export |
| 3 | Distance-transform cost field + path on w1 | new script in `tools/` | to make |
| 4 | Lattice versus coverage viewpoints on w3 with markers | new script in `tools/` | to make |
| 5 | Meeting sequence | ARCHITECTURE.md §9 (mermaid) | to export |
| 6 | Score per world, three stages (optional) | `docs/test_runs.md` | to make |
| – | Screenshots from Gazebo (robots at a marker, meeting) | from the videos | to make |

## Videos to record (required)
- One full run per world, or at least two worlds, with the Gazebo window (`gui:=true`),
  scoring visible in a terminal. A 10× sped-up version is easier to watch.
- Short clips: turn-and-look at a viewpoint, the meeting at the big fire, two robots
  giving way in a corridor.

## Internal plan (suggestion)

| Date | What |
|---|---|
| 13.11 | Sections and figures split between members; repository visibility and video plan decided |
| 16.11 | Videos recorded and uploaded; figures 1–6 done |
| 20.11 | First draft of every section |
| 24.11 | Whole report read through; word count 2700–3300; references checked |
| 26.11 | Work logs signed; PDF made |
| 27.11 kl. 14:00 | Submitted on WiseFlow (and the competition section added if the competition is before) |

| Section | Suggested owner (ARCHITECTURE.md §14) |
|---|---|
| 3.3 Navigation, 3.4 viewpoints and allocation | A |
| 3.1 communication, robot node, 3.5 marker detector | B |
| 3.2 Mission, 3.5 meeting | C |
| 1, 2, 4, 5, 3.6, figures, videos | shared |
