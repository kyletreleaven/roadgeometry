import nox
from pathlib import Path

nox.options.default_venv_backend = "uv"


def local_install(session, project_dir: str):
    project_file = str(Path(project_dir) / "pyproject.toml")
    toml = nox.project.load_toml(project_file)
    deps = toml["project"]["dependencies"]
    local_install_packages(session, *deps)
    session.install("-e", project_dir)


def local_install_packages(session, *packages):
    """

    TODO: Do we need to crawl for _all_ local packages,
    and use a dep manager (`uv`?) to resolve _external_ package versions?

    """
    local, remote = [], []
    for pkg in packages:
        local_path = Path("..") / pkg
        if local_path.exists():
            local.append(str(local_path))
        else:
            remote.append(pkg)

    # print(local, remote)

    for pkg_dir in local:
        local_install(session, pkg_dir)
    if remote:
        session.install(*remote)


def _install_test_deps(session):
    """Install test dependencies, building cpp packages with coverage instrumentation."""
    toml = nox.project.load_toml("pyproject.toml")
    deps = toml["project"]["dependencies"]
    test_deps = toml["project"]["optional-dependencies"]["test"]

    all_deps = [*deps, *test_deps]
    cpp_pkgs = {p for p in all_deps if p.endswith("-cpp")}
    other_deps = [p for p in all_deps if p not in cpp_pkgs]

    local_install_packages(session, *other_deps)

    session.install("pybind11", "scikit-build-core")
    for pkg in cpp_pkgs:
        local_path = Path("..") / pkg
        if local_path.exists():
            import shutil
            build_dir = local_path / "build"
            if build_dir.exists():
                shutil.rmtree(build_dir)
            args = ["-e", str(local_path)]
        else:
            args = [pkg]
        args += ["--no-build-isolation", "--config-settings", "cmake.args=-DROADGEOMETRY_COVERAGE=ON"]
        session.install(*args)


@nox.session
def test(session):
    _install_test_deps(session)
    # The matching package itself is not installed: pytest.ini sets `pythonpath = src`,
    # which puts the source tree on sys.path directly (subpackages without __init__.py
    # resolve as implicit namespace packages). Installing it editable creates a rival
    # editable finder that shadows those namespace subpackages (demo/, nxopt/, ...).
    session.install("gcovr")
    session.run("pytest", *(session.posargs or []))
    Path("coverage_report").mkdir(exist_ok=True)
    session.run(
        "gcovr",
        "--html-details", "coverage_report/index.html",
        "--filter", r"../setiptah-roadgeometry-matching-cpp/src/",
        "--filter", r"../cpp/include/",
        "--exclude", r".*pybind11.*",
        "--txt",
        "--print-summary",
        "--gcov-ignore-errors=source_not_found",
        "../setiptah-roadgeometry-matching-cpp/build/",
        external=True,
    )


@nox.session
def dev(session):
    local_install(session, ".")

    try:
        toml = nox.project.load_toml("pyproject.toml")
        dev_deps = toml["project"]["optional-dependencies"]["dev"]

    except:
        import json
        print(json.dumps(toml, indent=2))
        raise

    local_install_packages(session, *dev_deps)
    session.run("ipython")


@nox.session
def runpy(session):  # Wasn't working when the session was just "run"...
    local_install(session, ".")

    try:
        toml = nox.project.load_toml("pyproject.toml")
        dev_deps = toml["project"]["optional-dependencies"]["dev"]

    except:
        import json
        print(json.dumps(toml, indent=2))
        raise

    local_install_packages(session, *dev_deps)
    session.run("python", *(session.posargs or []))


@nox.session
def notebook(session):
    local_install(session, ".")

    try:
        toml = nox.project.load_toml("pyproject.toml")
        dev_deps = toml["project"]["optional-dependencies"]["dev"]

    except:
        import json
        print(json.dumps(toml, indent=2))
        raise

    session.install(*dev_deps)
    session.run("jupyter", "notebook")
