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

## Executable Targets

A target may declare that its product includes an **executable** — a binary or script that
other rules can reference as a tool. The orchestrator wires that executable into the consuming
target's execution environment (via PATH or direct reference) before stage 2 runs.

This makes tools first-class dependencies in the build graph. A target that builds a code
generator can be declared as a tool dependency of a target that uses it. The tool is built
first, then made available to the consuming target's command invocation — without the
consuming rule needing to know where the tool came from or how it was built.

---

## Guest Repos

An external source tree — typically a git clone — can be declared as a **guest repo**. Its
targets are addressable within the same orchestration system using the same target reference
syntax as local targets.

If the guest repo uses the same orchestrator, its targets integrate natively into the
dependency graph. If it doesn't, a rule/plugin bridges the gap — translating the guest's
native build system into the orchestrator's target model.

Guest repos extend the recursive property beyond the local monorepo boundary: the dependency
graph becomes a DAG that spans repositories. A guest repo may itself declare guest repos,
and a guest repo's executable target may be used as a tool in a local rule.

---

## Rule Responsibilities and Orchestrator Primitives

The orchestrator is strictly tool-agnostic — it has no built-in knowledge of any specific
toolchain. All tool-specific knowledge lives in **rules** (plugins). The orchestrator provides
primitives that rules build on:

- **Workspace layout** — create, populate, and clean workspace directories
- **Execution environment** — rules can set environment variables, PATH, working directory,
  and resource constraints for the stage 2 command invocation, independent of whether the
  toolchain itself provides escape hatches for output redirection
- **Product declaration** — rules declare what files/directories constitute the product
  and where to find them after stage 2 completes

Rules are responsible for:
- Knowing where their toolchain writes outputs within the workspace
- Deciding the workspace lifecycle strategy (clean vs. persistent)
- Handling product portability (see below)
- Redirecting toolchain outputs via execution environment manipulation where possible

---

## Product Portability

Not all products can be moved from the workspace to output storage after stage 2. A compiled
binary is portable; a Python virtualenv with baked-in absolute paths is not.

Rule authors must be aware of this. Where a product is not portable, the options are:

- **Stable output path** — configure the toolchain (via execution environment) to write
  directly to a known stable path that serves as both workspace and output storage
- **Descriptor as product** — the product is a description of how to reconstruct the
  environment (e.g. a lockfile, a manifest), not the environment itself; consumers
  reconstruct on demand
- **Workspace as output** — the workspace itself becomes the output storage for that target;
  downstream targets reference it in place rather than consuming a moved product

---

## Graph-Time Safety Annotations

Rules can declare statically — at graph construction time, before any execution — that
certain inputs are **consumed** or **unsafe** when the rule runs. This is analogous to
Rust's `cargo:rerun-if-changed` directives but expressed as properties of the rule rather
than runtime signals.

The orchestrator uses these annotations during graph analysis to make scheduling decisions:

- Do not run this target concurrently with other targets that share the same inputs
- Do not reuse a workspace that has been touched by a rule declaring those inputs consumed
- Flag to the user that certain inputs will be modified or invalidated by running this target

This keeps the safety contract at the graph level — visible, auditable, and reasoned about
before execution begins — rather than discovered at runtime.

---

## Workspace Structure

The workspace is a structured directory with well-known subdirectories, communicated to the
build command via environment variables:

```
workspace/
  src/   — inputs: populated by the orchestrator from dependencies and source map
  tmp/   — scratch space: toolchain writes here freely during stage 2
  out/   — outputs: rule places declared products here for stage 3 extraction
```

The build command receives env vars pointing to each subdirectory (e.g. `RG_SRC`, `RG_TMP`,
`RG_OUT`). Rule authors direct their toolchain to the appropriate locations, using shims
where necessary to move artifacts between subdirectories.

The `out/` directory is the interface between targets — what a target exposes to downstream
dependents. A dependency's `src/` and `tmp/` are never visible outside that target.

**Note:** The interaction between workspace structure, product portability, and opinionated
toolchains is not fully resolved — see Open Questions.

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
- **Toolchains with opinionated output locations**: the orchestrator provides execution
  environment manipulation as a primitive, and rules can use this to redirect toolchain
  outputs where the toolchain respects the relevant env vars or flags. But some toolchains
  hardcode output locations (e.g. nox writing to `.nox/`, some pip behaviors) and cannot
  be redirected. The product portability strategies (stable path, descriptor, workspace-as-
  output) handle the aftermath, but don't resolve the case where you cannot control where
  the toolchain writes in the first place. Is there a general solution, or is this an
  inherent limitation that rule authors must document and work around per-toolchain?
- What is the distribution/installation story for the orchestrator itself?
- Copy vs. link for workspace population: copying is hermetic but expensive and disconnects
  the workspace from live edits; linking is fast and dev-friendly but breaks hermeticity and
  risks commands modifying their inputs. The right strategy may vary by workspace lifecycle
  (clean vs. persistent) and target type (dev vs. release), but that entangles two decisions
  that might be better kept separate.
