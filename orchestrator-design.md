# Polyglot Monorepo Orchestrator — Requirements & Design

## Motivation

Developing a polyglot project (e.g. Python + C++ + C + Rust + LaTeX) as a single monorepo
is natural and productive. Distributing it as a set of interdependent packages across multiple
language-specific registries (PyPI, crates.io, vcpkg, etc.) is a hard requirement for broad
audience reach. The gap between these two modes — development and distribution — is the core
problem this tool addresses.

Existing tools (Bazel, uv workspaces, CMake, nox) each solve pieces of this problem but are
too opinionated, too language-specific, or too narrow in scope to serve as a general solution.

---

## Core Design: The Target Model

A **target** consists of three stages:

### 1. Workspace Population
Assemble the inputs for this target into a fresh or persistent workspace directory. Each
dependency is satisfied according to the target's configuration — from local monorepo source,
from a previously built product, or from an external registry (PyPI, crates.io, etc.). The
workspace is laid out however the command in stage 2 expects to find it.

### 2. Command Invocation
Run an arbitrary command in the workspace. The orchestrator is completely hands-off here —
the command can be `cmake`, `nox -s test`, `cargo build`, `latexmk`, a shell script, or
another invocation of the orchestrator itself. No opinions about what steps are allowed or
where outputs go within the workspace.

### 3. Product Extraction
Collect declared outputs from the workspace and assemble them into the target's **product**.
The product is the target's distributable result — what other targets can depend on, and what
gets published to a registry.

---

## Key Properties

### The orchestrator owns stages 1 and 3; stage 2 is a black box
This is the critical improvement over Bazel, which is opinionated about all three stages.
Native toolchains handle their own build logic; the orchestrator only manages environment
construction and output collection.

### Products are self-consistent inputs
A target's product (stage 3 output) is a valid input to another target's workspace population
(stage 1). The format is recursive — the orchestrator can consume its own output, enabling
composition across projects that use the same tool.

### No built-in registry
The orchestrator does not implement or replace package registries. PyPI, crates.io, vcpkg,
etc. are consumed as-is. The workspace population stage knows how to fetch from them; the
registries themselves are unchanged.

---

## Source Maps

The description of which source files belong to which target lives in the source tree, not
necessarily at the root. Source maps can be co-located with the code they describe (analogous
to Bazel's `BUILD` files) or aggregated from wherever the orchestrator can find them.

Source maps are not constrained to the root of the monorepo — the tool discovers and
aggregates them.

---

## Targets and Configurations

Dependencies within a target can be satisfied from different sources. Rather than a runtime
flag, different satisfaction strategies are expressed as named **targets**:

```
dev                    — all dependencies from local monorepo source
test-b-against-pypi-a  — package A from PyPI, package B from local source
release                — assemble everything for publishing
```

Each target is a complete, explicit specification of how every dependency is satisfied.
Targets are declared as part of the project description. This makes package boundary
verification a first-class target rather than a special CI step.

---

## Workspace Lifecycle

The workspace lifecycle is declared by the **build rule**, not imposed by the orchestrator.
Two valid strategies:

| Strategy | Behavior | When to use |
|---|---|---|
| **Clean** | Workspace is populated fresh on every invocation | Hermetic, reproducible builds; LaTeX, packaging steps |
| **Persistent** | Workspace is retained between invocations | Native toolchain incrementality (CMake, cargo, nox) |

The orchestrator provides the mechanism; the rule chooses the strategy. This avoids the
Bazel trap of mandating hermeticity globally, which breaks native toolchain caching.

There is no "no workspace" option — running directly in the source tree breaks isolation
and is exactly the problem this design avoids.

---

## Incrementality

The orchestrator tracks staleness **across target boundaries only** — specifically, on the
declared products of upstream dependencies:

- Before running target B, check whether A's product has changed since B last ran
  (via timestamps or content hashes)
- If unchanged, skip B entirely
- If changed, repopulate B's workspace with the fresh product and re-invoke

Incrementality *within* a target's workspace is delegated entirely to the native toolchain
(CMake's mtime tracking, cargo's fingerprinting, pytest's cache). The orchestrator does not
need to understand or replicate this — it only needs to know whether the boundary between
targets has been crossed.

This gives you native toolchain incrementality for free, while still propagating changes
correctly through the dependency graph.

---

## Relationship to Existing Tools

| Tool | Role | Relationship |
|---|---|---|
| `just` | Top-level task runner | Invokes orchestrator targets as recipes |
| `cmake` | C++ build system | Invoked as stage 2 command; manages its own workspace internally |
| `nox`/`pytest` | Python test runner | Invoked as stage 2 command |
| `cargo` | Rust build system | Invoked as stage 2 command |
| PyPI, crates.io, vcpkg | Package registries | Consumed during stage 1 workspace population |
| `uv`, `pip` | Python environment managers | Invoked during stage 1 for Python dependencies |

The orchestrator does not replace any of these — it orchestrates them.

---

## Open Questions

- What is the concrete format of a source map?
- What is the concrete format of a product — a directory, a manifest, a content-addressed store?
- How are workspace population recipes expressed — declarative config, scripted, or both?
- How does the orchestrator handle toolchains that are themselves environment managers
  (e.g. nox creating virtualenvs, cargo managing `target/`) with strong opinions about
  output locations?
- What is the distribution/installation story for the orchestrator itself?
