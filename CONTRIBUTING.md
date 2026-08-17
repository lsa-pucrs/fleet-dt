# Contributing

## Before anything

    make clean && make

That builds the library, runs every test, builds the examples, runs the
benchmark campaign and regenerates the report. It takes about a minute.

## The rule that matters here

This repository implements a published paper. Every number in the code that
also appears in the manuscript is a claim someone can check.

**Any change that touches a published figure must cite the section and
equation of revision `-10` in its commit message.** If the manuscript revision
changes, `FDT_PAPER_REVISION` in `include/fleet_dt/version.h` changes with it,
`make test` fails, and `docs/paper-to-code.md` gets re-checked. That failure is
the feature.

Three published figures are static assertions rather than measurements:

- `sizeof(fdt_state_t) == 48` (Section IV)
- `fdt_queue_bytes(d) == 48 * d` (Section IV)
- 23040 bytes per minute at 8 Hz (Section IV)

Weakening one of these to make a change compile is not a fix.

## `make bench` dirties the tree

`results/` is committed, so the charts in `docs/RESULTS.md` resolve on a fresh
clone. Every benchmark run overwrites them with your machine's timings, which
means a clean checkout goes dirty the first time you build.

That is expected. Either commit the regenerated artefacts, or

    git checkout results/ docs/RESULTS.md

before pushing. You have not broken anything.

## What a test may assert

Tests assert what the paper states and what the arithmetic requires. They do
**not** assert a measurement taken on the machine that runs them: a benchmark
measures the host, and the host is not the host the paper measured.

That is why divergence from a published figure is printed as `[DIVERGE]` by
the benchmarks and never aborts a run, while a structural error — a vessel
lost from a frame, a counter that does not balance — is a plain assertion that
does abort.

## What must not be built

`docs/spec/paper-claims.md` section F lists seven items Section VI declares
future work. Implementing any of them contradicts the published text. If you
believe one should exist, that is a conversation about the paper, not a pull
request.

## Style

- C18, and the build runs with `-Wall -Wextra -Werror -pedantic-errors`.
- Public prefix `fdt_` / `FDT_`, no exceptions.
- Every public symbol carries a docstring in its header saying what it does,
  what it returns, and which line of the paper it discharges.
- No allocation and no file-scope state in the library: storage belongs to the
  caller, which is what lets a fleet of N vessels be N structs.
- Comments explain why, not what. A comment restating the line below it is
  noise; a comment explaining why a bound is `>=` rather than `>` is not.

## Commits

Conventional Commits. Author and committer are the human doing the work.
