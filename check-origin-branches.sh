#!/usr/bin/env bash
# Deletes branches on origin whose commits all exist on upstream
# (i.e. the branch is identical to, or strictly behind, some upstream history —
# it carries no unique commits), except the currently checked-out branch.
#
# Usage: ./check-origin-branches.sh [-v] [-n] [origin-remote] [upstream-remote]
#   -v  also list branches that DO have unique commits, with their commit count
#   -n  dry run: list what would be deleted without deleting

set -euo pipefail

verbose=0
dry_run=0
while [[ "${1:-}" == -* ]]; do
  case "$1" in
    -v) verbose=1 ;;
    -n) dry_run=1 ;;
    *) echo "Unknown option: $1" >&2; exit 1 ;;
  esac
  shift
done

origin="${1:-origin}"
upstream="${2:-upstream}"

echo "Fetching ${origin} and ${upstream}..." >&2
git fetch --quiet --prune "$origin"
git fetch --quiet --prune "$upstream"

current_branch="$(git symbolic-ref --quiet --short HEAD || true)"

clean=()
dirty=()
kept=()

while IFS= read -r ref; do
  branch="${ref#refs/remotes/"$origin"/}"
  [[ "$branch" == "HEAD" ]] && continue
  if [[ -n "$current_branch" && "$branch" == "$current_branch" ]]; then
    kept+=("$branch")
    continue
  fi
  unique=$(git rev-list --count "$ref" --not --remotes="$upstream")
  if [[ "$unique" -eq 0 ]]; then
    clean+=("$branch")
  else
    dirty+=("$branch ($unique unique commit(s))")
  fi
done < <(git for-each-ref --format='%(refname)' "refs/remotes/$origin/")

if [[ ${#kept[@]} -gt 0 ]]; then
  echo "Skipping current branch: ${kept[*]}"
fi

if [[ ${#clean[@]} -eq 0 ]]; then
  echo "No branches on ${origin} strictly from ${upstream} to delete."
elif [[ "$dry_run" -eq 1 ]]; then
  echo "Would delete from ${origin} (no unique commits vs ${upstream}):"
  printf '  %s\n' "${clean[@]}"
else
  echo "Deleting from ${origin} (no unique commits vs ${upstream}):"
  printf '  %s\n' "${clean[@]}"
  git push "$origin" --delete "${clean[@]}"
fi

if [[ "$verbose" -eq 1 ]]; then
  echo
  echo "Branches on ${origin} with commits not on ${upstream}:"
  if [[ ${#dirty[@]} -eq 0 ]]; then
    echo "  (none)"
  else
    printf '  %s\n' "${dirty[@]}"
  fi
fi
