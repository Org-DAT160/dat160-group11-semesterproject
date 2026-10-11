#!/usr/bin/env python3
"""Create GitHub labels, milestones and issues from issues.csv using the GitHub CLI.

Usage:
    python create_issues.py --repo Org-DAT160/dat160-group11-semesterproject --dry-run
    python create_issues.py --repo Org-DAT160/dat160-group11-semesterproject \
        --project "DAT160 Semesterproject - Group 11"

Requirements:
    - Python 3.8 or newer (standard library only)
    - GitHub CLI 2.31 or newer: `gh auth login`, and `gh auth refresh -s project` when using --project

Behaviour:
    - Issues are created in dependency order, so "Depends on" can link to real issue numbers.
    - Issue titles start with the key, for example "NAV-03: Implement the distance transform ...".
    - Re-running is safe: issues whose key already exists in the repository are skipped,
      labels are updated, and existing milestones are reused.
    - With --project, every new issue is added to the project board and its Status is set:
      issues labelled `ready` get --ready-status (default "Ready"), all others get
      --backlog-status (default "Backlog"). The project must be owned by the repository owner.
    - The key-to-number mapping is written to issues_created.json.
"""
import argparse
import csv
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time

LABEL_COLORS = {
    "area:setup": "bfd4f2", "area:navigation": "1d76db", "area:perception": "0e8a16",
    "area:mission": "d93f0b", "area:integration": "fbca04", "area:report": "5319e7",
    "owner:A": "c5def5", "owner:B": "c2e0c6", "owner:C": "f9d0c4",
    "size:S": "ededed", "size:M": "cccccc", "size:L": "999999",
    "ready": "2ea44f", "cross-area": "b60205",
    "impl:own": "b60205", "impl:ai-allowed": "c5def5",
}
LABEL_DESCRIPTIONS = {
    "ready": "No open dependencies: can be started now",
    "cross-area": "Depends on work by another owner",
    "size:S": "Under about 2 hours", "size:M": "About half a day", "size:L": "A day or more",
    "impl:own": "Core of the navigation algorithm (R7): implemented by the owner without AI code generation",
    "impl:ai-allowed": "AI code generation allowed; the owner reviews and can explain the code",
}
DEFAULT_COLOR = "d4c5f9"
KEY_RE = re.compile(r"^([A-Z]{3}-\d{2}):")


def run(args, check=True):
    result = subprocess.run(args, capture_output=True, text=True, encoding="utf-8")
    if check and result.returncode != 0:
        sys.exit("Command failed: {}\n{}".format(" ".join(args), result.stderr.strip()))
    return result


def read_issues(path, delimiter):
    with open(path, encoding="utf-8-sig", newline="") as f:
        header = f.readline()
        f.seek(0)
        if delimiter is None:
            delimiter = ";" if header.count(";") > header.count(",") else ","
        rows = list(csv.DictReader(f, delimiter=delimiter))
    required = {"key", "title", "milestone", "milestone_due", "depends_on", "labels", "body"}
    missing = required - set(rows[0].keys()) if rows else required
    if missing:
        sys.exit("CSV is missing columns: {}".format(", ".join(sorted(missing))))
    for row in rows:
        row["depends_on"] = row["depends_on"].split()
        row["labels"] = [x.strip() for x in row["labels"].split(",") if x.strip()]
    return rows


def topological_order(rows):
    by_key = {r["key"]: r for r in rows}
    for r in rows:
        for d in r["depends_on"]:
            if d not in by_key:
                sys.exit("{} depends on unknown key {}".format(r["key"], d))
    ordered, state = [], {}

    def visit(key, path):
        if state.get(key) == "done":
            return
        if state.get(key) == "visiting":
            sys.exit("Dependency cycle: {}".format(" -> ".join(path + [key])))
        state[key] = "visiting"
        for d in by_key[key]["depends_on"]:
            visit(d, path + [key])
        state[key] = "done"
        ordered.append(by_key[key])

    for r in rows:
        visit(r["key"], [])
    return ordered


def existing_issues(repo):
    out = run(["gh", "issue", "list", "--repo", repo, "--state", "all",
               "--limit", "1000", "--json", "number,title"]).stdout
    found = {}
    for issue in json.loads(out or "[]"):
        m = KEY_RE.match(issue["title"])
        if m:
            found[m.group(1)] = issue["number"]
    return found


def ensure_labels(repo, labels, dry_run):
    for name in sorted(labels):
        color = LABEL_COLORS.get(name, DEFAULT_COLOR)
        desc = LABEL_DESCRIPTIONS.get(name, "")
        if dry_run:
            print("  label  {}".format(name))
            continue
        run(["gh", "label", "create", name, "--repo", repo, "--color", color,
             "--description", desc, "--force"])


def ensure_milestones(repo, milestones, dry_run):
    existing = set()
    if not dry_run:
        out = run(["gh", "api", "repos/{}/milestones?state=all".format(repo),
                   "--paginate", "--jq", ".[].title"]).stdout
        existing = {line.strip() for line in out.splitlines() if line.strip()}
    for title, due in milestones.items():
        if title in existing:
            print("  milestone exists  {}".format(title))
            continue
        print("  milestone  {} (due {})".format(title, due))
        if not dry_run:
            run(["gh", "api", "-X", "POST", "repos/{}/milestones".format(repo),
                 "-f", "title={}".format(title), "-f", "due_on={}T23:59:59Z".format(due)])


def link_dependencies(body, numbers, titles):
    def replace(match):
        key = match.group(1)
        if key in numbers:
            return "- #{} {}".format(numbers[key], titles[key])
        return match.group(0)
    return re.sub(r"^- ([A-Z]{3}-\d{2})$", replace, body, flags=re.MULTILINE)


class Board:
    """A GitHub project (v2) owned by the repository owner, with its Status field."""

    def __init__(self, owner, title, ready_status, backlog_status):
        projects = json.loads(run(["gh", "project", "list", "--owner", owner,
                                   "--limit", "100", "--format", "json"]).stdout)
        match = [p for p in projects.get("projects", []) if p.get("title") == title]
        if not match:
            names = ", ".join(p.get("title", "?") for p in projects.get("projects", []))
            sys.exit("Project '{}' not found under {}. Found: {}".format(title, owner, names or "none"))
        self.owner = owner
        self.number = str(match[0]["number"])
        self.id = match[0]["id"]
        fields = json.loads(run(["gh", "project", "field-list", self.number, "--owner", owner,
                                 "--format", "json"]).stdout)
        status = [f for f in fields.get("fields", []) if f.get("name") == "Status"]
        if not status:
            sys.exit("Project '{}' has no Status field".format(title))
        self.field_id = status[0]["id"]
        options = {o["name"]: o["id"] for o in status[0].get("options", [])}
        for wanted in (ready_status, backlog_status):
            if wanted not in options:
                sys.exit("Status option '{}' not found. Options: {}".format(
                    wanted, ", ".join(options)))
        self.ready_option = options[ready_status]
        self.backlog_option = options[backlog_status]

    def add(self, issue_url, ready):
        item = json.loads(run(["gh", "project", "item-add", self.number, "--owner", self.owner,
                               "--url", issue_url, "--format", "json"]).stdout)
        run(["gh", "project", "item-edit", "--id", item["id"], "--project-id", self.id,
             "--field-id", self.field_id, "--single-select-option-id",
             self.ready_option if ready else self.backlog_option])


def create_issue(repo, row, body):
    with tempfile.NamedTemporaryFile("w", suffix=".md", delete=False, encoding="utf-8") as f:
        f.write(body)
        body_file = f.name
    try:
        cmd = ["gh", "issue", "create", "--repo", repo,
               "--title", "{}: {}".format(row["key"], row["title"]),
               "--body-file", body_file, "--milestone", row["milestone"]]
        for label in row["labels"]:
            cmd += ["--label", label]
        out = run(cmd).stdout.strip().splitlines()
    finally:
        os.unlink(body_file)
    url = out[-1] if out else ""
    m = re.search(r"/issues/(\d+)$", url)
    if not m:
        sys.exit("Could not read the issue number for {} from: {}".format(row["key"], url))
    return int(m.group(1)), url


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--repo", required=True, help="OWNER/REPO")
    parser.add_argument("--csv", default="issues.csv", help="Path to issues.csv")
    parser.add_argument("--project", help="Title of the GitHub project board to add issues to")
    parser.add_argument("--ready-status", default="Ready",
                        help="Status for issues labelled `ready` (default: Ready)")
    parser.add_argument("--backlog-status", default="Backlog",
                        help="Status for all other issues (default: Backlog)")
    parser.add_argument("--delimiter", choices=[",", ";"], help="CSV delimiter (default: detect)")
    parser.add_argument("--dry-run", action="store_true", help="Show what would be done")
    parser.add_argument("--delay", type=float, default=1.0,
                        help="Seconds between issues, to respect GitHub rate limits")
    args = parser.parse_args()

    if not args.dry_run:
        if shutil.which("gh") is None:
            sys.exit("GitHub CLI not found. Install it, then run `gh auth login`.")
        run(["gh", "auth", "status"])

    rows = topological_order(read_issues(args.csv, args.delimiter))
    titles = {r["key"]: r["title"] for r in rows}
    labels = {label for r in rows for label in r["labels"]}
    milestones = {}
    for r in rows:
        milestones.setdefault(r["milestone"], r["milestone_due"])

    print("{} issues, {} labels, {} milestones{}".format(
        len(rows), len(labels), len(milestones), " (dry run)" if args.dry_run else ""))
    print("Labels:")
    ensure_labels(args.repo, labels, args.dry_run)
    print("Milestones:")
    ensure_milestones(args.repo, milestones, args.dry_run)

    board = None
    if args.project and not args.dry_run:
        board = Board(args.repo.split("/")[0], args.project, args.ready_status, args.backlog_status)
        print("Project: {} (#{})".format(args.project, board.number))

    numbers = {} if args.dry_run else existing_issues(args.repo)
    print("Issues:")
    for row in rows:
        if row["key"] in numbers:
            print("  exists  #{}  {}".format(numbers[row["key"]], row["key"]))
            continue
        body = link_dependencies(row["body"], numbers, titles)
        ready = "ready" in row["labels"]
        status = args.ready_status if ready else args.backlog_status
        if args.dry_run:
            deps = ", ".join(row["depends_on"]) or "none"
            board_note = "  status: {}".format(status) if args.project else ""
            print("  create  {}: {}  [{}]  depends on: {}{}".format(
                row["key"], row["title"], ", ".join(row["labels"]), deps, board_note))
            continue
        numbers[row["key"]], url = create_issue(args.repo, row, body)
        if board:
            board.add(url, ready)
        print("  created #{}  {}{}".format(numbers[row["key"]], row["key"],
                                          "  -> " + status if board else ""))
        time.sleep(args.delay)

    if not args.dry_run:
        with open("issues_created.json", "w", encoding="utf-8") as f:
            json.dump(numbers, f, indent=2, sort_keys=True)
        print("Done. Mapping written to issues_created.json")


if __name__ == "__main__":
    main()
