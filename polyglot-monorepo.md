# The Polyglot Monorepo Problem

## Or: Why Nobody Has Solved This Yet

There is a class of software project that lives at the boundary between languages. A high-performance
algorithm library, say, that needs to be accessible from Python for data scientists, from C++ for
systems engineers, from C for embedded or FFI consumers, and perhaps from Rust or Julia down the
road. The core logic is computationally intensive enough that a pure-Python implementation is a
prototype, not a product. The author wants to develop everything together — the algorithm, the
bindings, the tests, the benchmarks — and then distribute it in whatever form each audience expects.

This is the polyglot monorepo problem, and nobody has fully solved it.

---

## The Monorepo Promise

The appeal of a monorepo is real and well-documented. You see the whole system at once. Refactors
cross package boundaries atomically. Tests can cover integration scenarios that multi-repo
workflows make awkward. There's no "which version of package A does package B need right now"
problem during active development — everything is just *there*.

Large companies have built serious tooling around this model. Google has Bazel. Meta has Buck.
Microsoft has Rush (for JavaScript). These tools work, at scale, for teams of hundreds. But they
share a common characteristic: they were built to solve a *scale* problem within a dominant
language or ecosystem, and the polyglot case is almost always an afterthought bolted on later.

Bazel in particular has a reputation for being deeply opinionated about what build steps are
allowed and where artifacts land. You can make it work for a Python + C++ + Rust project, but
you will spend a significant fraction of your engineering time fighting the tool rather than
building the thing. The return on that investment only makes sense at a certain scale, and most
projects never reach it.

---

## The Core Tension

To understand why this is hard, you have to distinguish two concerns that most developers conflate:

**The development unit** — the natural chunk of code you work on at once. During development,
you want the entire codebase visible and navigable. You want to change a data structure in the
core and immediately see the effect in the algorithm that uses it, without any intermediate
packaging or installation step.

**The distribution unit** — the chunk of code a consumer installs. A consumer might want just
the core geometry library, or just the matching algorithm, or both. They shouldn't have to
install the entire project to get the piece they need.

The fundamental problem is that **different languages handle this tension differently**, and
tooling designed for one language's model breaks down when applied to another's.

---

## How Each Language Handles It

### C++ — The Cleanest Story

C++ has no packaging concept during development. You have a directory of source files and headers.
`#include` is a file path. CMake is a build system that describes how to compile and link things,
and separately describes *install rules* — what headers and libraries get exported when you
package for distribution. The source tree has no packaging boundaries. The distribution boundaries
are declared in `CMakeLists.txt` and only matter at install time.

A C++ monorepo is just a directory tree with a `CMakeLists.txt` at the root. You add a new
module by creating a new subdirectory and a `add_subdirectory()` line. You carve it into
distributable chunks — a header-only library, a shared library, separate CMake packages — by
writing install rules. Development and distribution are completely decoupled.

This is the cleanest model. The development unit and the distribution unit are genuinely separate
concerns, handled by separate parts of the toolchain.

### Rust — Nearly As Clean

Rust has the concept of a *workspace* — a root `Cargo.toml` that declares a set of *crates*
(the distribution unit). From the perspective of `cargo build`, the workspace is the unit of
work. All crates are resolved together, built together, tested together. The crate boundaries
exist and matter for distribution, but they don't impede development. You run `cargo test` from
the root and everything is tested. You navigate the source tree freely. The `Cargo.toml` files
are mostly metadata.

Rust's workspace model is probably the best existing solution to the monorepo development
problem for a compiled language.

### Python — The Problematic Case

Python conflates the development unit and the distribution unit into a single concept: the
*package*. A Python package is simultaneously the thing you `import`, the thing you `pip install`,
and the directory structure you develop in. The `pyproject.toml` file, the `src/` layout, the
`test/` directory — these aren't just distribution metadata, they're the physical structure of
your development tree.

This means you cannot "forget" about package boundaries during development. They're encoded in
your directory structure. When you work on a project with multiple interdependent Python packages
in a monorepo, you are constantly aware of and navigating those boundaries — because they're
right there in the file tree.

Tools like `uv workspaces` and `hatch` try to smooth this over, but they're fundamentally
Python-centric. They understand Python packages and Python dependencies. The moment you add
a C++ extension module, a C shared library, or a Rust crate, you're outside their model.

---

## Why the Polyglot Case Is Uniquely Hard

A pure-Python monorepo is annoying but manageable. A pure-C++ monorepo is actually quite
comfortable. The polyglot case compounds the problems of each.

The C++ layer wants to be developed as one big CMake project, with the source tree reflecting
code organization rather than distribution boundaries. The Python layer wants `pyproject.toml`
files that describe each distributable package. These two models don't compose naturally.

Concretely: if you have a C++ algorithm library with Python bindings, the C++ source lives
somewhere in your tree, the pybind11 binding code lives somewhere else, and the Python package
that wraps it has its own directory structure with its own `pyproject.toml`. How do you run
`nox -s test` in the Python package against a freshly-compiled C++ extension? How do you
ensure the C++ coverage is measured by the Python test suite? How do you set up a new
contributor's development environment so that changing a `.hpp` file and running tests
Just Works?

None of these questions have clean answers that compose across languages.

---

## The Tooling Landscape

### Language-Agnostic Task Runners

The most useful tool in the polyglot monorepo is a language-agnostic task runner. The leading
option today is **`just`** — a command runner with no opinions about languages, build systems,
or artifact locations. A `justfile` is a set of named recipes that run shell commands. Each
recipe can depend on other recipes. The underlying tools — CMake, cargo, nox, latexmk,
whatever — are invoked as subprocesses. `just` doesn't try to understand them; it just
runs them in order.

`just` deliberately does not cache — it always re-runs every recipe it's asked to run. The
underlying tools are responsible for incrementality (CMake's file mtime tracking, cargo's
fingerprinting, pytest's `--lf`). This is the right division of responsibility: a thin
orchestration layer that knows ordering, and native tools that know what actually needs to
be rebuilt.

This is not a complete solution. It solves task orchestration but nothing else.

### Per-Language Dependency Management

There is no universal dependency management layer for a polyglot monorepo. Each language owns
its own:

- Python: `pip`/`uv`/`poetry`, editable installs, `pyproject.toml`
- C++: CMake `find_package` / `FetchContent`, pkg-config, Conan, vcpkg
- Rust: `Cargo.toml` path dependencies

These systems don't talk to each other and shouldn't be forced to. The practical approach is
to let each language's native toolchain manage its own dependencies, and use the task runner
to invoke them in the right order.

### Verification of Package Boundaries

The most insidious failure mode in a Python monorepo is code that works in your development
environment (where everything is installed editable) but fails on PyPI (where the consumer
only installed one package and its declared dependencies). The only reliable defense is a CI
step that installs each package in strict isolation and runs its tests — verifying that the
`pyproject.toml` dependency declarations accurately describe what the code actually needs.

This is essentially what per-session virtualenv isolation in `nox` buys you. It's annoying
during day-to-day development but genuinely valuable as a correctness check.

The pragmatic compromise: don't use per-session isolation as your primary dev workflow.
Have a `just dev` recipe that installs everything editable into a single environment for
fast iteration. Keep `nox -s test` as the verification step you run before committing, and
run it in CI. Get the correctness guarantee without paying the ergonomic cost constantly.

---

## The Honest State of the Art

There is no tool today that makes polyglot monorepo development genuinely comfortable. The
best available approach is a combination of pragmatic compromises:

1. **Accept the physical reality of Python package boundaries.** The `pyproject.toml` files
   are unavoidable if you want proper distribution. Maintain them honestly. They're the
   distribution manifests, not the development structure.

2. **Use a language-agnostic task runner (`just`) at the root** for cross-cutting concerns:
   running the full test suite, building everything, running benchmarks. Each recipe delegates
   to the appropriate native toolchain.

3. **Use native toolchains for what they're good at.** CMake for C++ build and install.
   `nox`/`pytest` for Python testing. `cargo` for Rust (if/when it arrives). Don't fight
   each tool's idioms.

4. **Separate the dev workflow from the verification workflow.** A flat `just dev` editable
   install for fast iteration. `nox -s test` for package-boundary verification before commits
   and in CI.

5. **Let the C++ layer be the real monorepo core.** The C++ source tree has no packaging
   boundaries during development. The Python packages are thin distribution wrappers around
   it. Heavy development happens in C++; the Python packages just declare what gets bound
   and shipped.

6. **Synchronized versioning across all packages.** One version number, one git tag, all
   packages published simultaneously. This eliminates the version matrix problem between
   interdependent packages.

7. **Accept that jumping Python package boundaries during development will always have
   some friction.** The language conflates development and distribution units. This is a
   fundamental limitation, not a tooling gap that a better `nox` plugin will fix.

---

## Who Feels This Pain

A small fraction of developers ever encounter this problem in its full form. Most software
projects live comfortably within a single language ecosystem. The ones that don't are usually:

- Algorithm or numerical libraries with performance requirements (needs C/C++) and broad
  accessibility requirements (needs Python, R, Julia bindings)
- Infrastructure or systems software that needs a C ABI for stability and higher-level
  language bindings for ergonomics
- Research software where the authors think in one language (often Python or MATLAB) but
  the users want production-grade performance

These projects share a common profile: the author cares deeply about correctness and
performance, wants to support a broad audience across multiple languages, and has neither
the appetite for Bazel's complexity nor the luxury of limiting themselves to one toolchain.

For this audience, the state of the art is genuinely disappointing. The tools that exist
were built by and for teams with different constraints. The polyglot library developer is
largely on their own, assembling a workable workflow from pieces that weren't designed to
fit together.

The good news is that `just`, CMake, pybind11, and the modern Python packaging ecosystem
(`pyproject.toml`, `scikit-build-core`) are all individually quite good. The assembly is
painful; the components are solid. That's the honest state of affairs.
