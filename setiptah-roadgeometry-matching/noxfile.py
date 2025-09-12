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


@nox.session
def test(session):
    try:
        toml = nox.project.load_toml("pyproject.toml")
        deps = toml["project"]["dependencies"]
        test_deps = toml["project"]["optional-dependencies"]["test"]

    except:
        import json
        print(json.dumps(toml, indent=2))
        raise

    local_install_packages(session, *deps, *test_deps)
    session.run("pytest", *(session.posargs or []))  # posargs for test filtering


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
