#!/usr/bin/env bash
#
# Maintainer helper: pull, commit and push the current branch in one step.
#
#   bash gitpush.bash "what changed"
#
# What it does, in order:
#   1. reads the branch you are on and refuses to run on a detached HEAD (a
#      fresh submodule checkout is detached; `git checkout <branch>` first);
#   2. `git pull --ff-only` from that branch's upstream, so a diverged history
#      stops the script instead of producing a surprise merge commit;
#   3. stages everything, refuses to continue if the staged content contains a
#      credential or a machine-local absolute path, then commits with the given
#      message;
#   4. pushes the branch to `origin` under the same name. A push that touches
#      `book/**` on the default branch rebuilds the published book.
#
# The branch name is never hard-coded. The script never tags and never
# force-pushes.

set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")"

MSG=${1:?usage: bash gitpush.bash "commit message"}
REMOTE=origin

say()  { printf '\033[36m[gitpush %s]\033[0m %s\n' "$(basename "$PWD")" "$1"; }
fail() { printf '\033[31m[gitpush %s] %s\033[0m\n' "$(basename "$PWD")" "$1" >&2; exit 1; }

# 1. Which branch.
BRANCH=$(git symbolic-ref --short -q HEAD || true)
[ -n "$BRANCH" ] || fail "detached HEAD; run 'git checkout <branch>' first, then rerun"

# 2. Bring the branch up to date. Fast-forward only.
git fetch -q "$REMOTE"
if git show-ref -q --verify "refs/remotes/$REMOTE/$BRANCH"; then
    say "git pull --ff-only $REMOTE $BRANCH"
    git pull -q --ff-only "$REMOTE" "$BRANCH" \
        || fail "pull could not fast-forward; merge or rebase by hand first"
else
    say "branch '$BRANCH' does not exist on $REMOTE yet; it will be created"
fi

# 3. Stage and check.
git add -A
if git diff --cached --quiet; then
    say "nothing to commit"
else
    say "staged changes on $BRANCH:"
    git status --short

    # Never publish a credential or a path from someone's machine.
    leak_pattern='ghp_[A-Za-z0-9]{20,}|github_pat_[A-Za-z0-9_]{20,}|gho_[A-Za-z0-9]{20,}|https://[^/ @]+@github\.com|/mnt/[A-Za-z0-9]|/home/[a-z]|C:\\Users|[A-Z]:\\Github'
    if git diff --cached -U0 | grep -E '^\+' | grep -Ev '^\+\+\+' | grep -En "$leak_pattern"; then
        git reset -q
        fail "staged content contains a credential or a machine-local path (see above); nothing committed"
    fi

    git commit -q -m "$MSG"
    say "committed: $(git log --oneline -1)"
fi

# 4. Push the branch under its own name.
if git show-ref -q --verify "refs/remotes/$REMOTE/$BRANCH" \
   && [ -z "$(git log --oneline "$REMOTE/$BRANCH..$BRANCH")" ]; then
    say "nothing to push, $BRANCH is level with $REMOTE/$BRANCH"
    exit 0
fi
say "git push -u $REMOTE $BRANCH"
git push -u "$REMOTE" "$BRANCH"
say "done"
