import nox
from pathlib import Path

nox.options.default_venv_backend = "uv"

PACKAGES = [
    "setiptah-roadgeometry",
    "setiptah-roadgeometry-matching",
]


def local_install(session, project_dir: str):
    project_file = str(Path(project_dir) / "pyproject.toml")
    toml = nox.project.load_toml(project_file)
    deps = toml["project"]["dependencies"]
    local_install_packages(session, *deps)
    session.install("-e", project_dir)


def local_install_packages(session, *packages):
    local, remote = [], []
    for pkg in packages:
        local_path = Path(pkg)
        if local_path.exists():
            local.append(str(local_path))
        else:
            remote.append(pkg)

    for pkg_dir in local:
        local_install(session, pkg_dir)
    if remote:
        session.install(*remote)


@nox.session
def build_roadgeometry_cpp(session):
    """Build and install the C++ extension for setiptah-roadgeometry."""
    session.install("-e", "setiptah-roadgeometry-cpp")
    session.run("python", "-c", "from setiptah.roadgeometry import _cpp; print('_cpp loaded:', _cpp)")


@nox.session
def build_matching_cpp(session):
    """Build and install the C++ extension package."""
    session.install("-e", "setiptah-roadgeometry-matching-cpp")
    session.run("python", "-c", "from setiptah.roadgeometry.matching import _cpp; print('_cpp loaded:', _cpp)")


@nox.session
def bench(session):
    for pkg in PACKAGES:
        local_install(session, pkg)

    session.install("numpy", "networkx", "line_profiler")
    session.run("python", *(session.posargs or ["bench/profile_matching.py"]))
