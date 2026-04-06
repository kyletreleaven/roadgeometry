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
    # TODO: C++ coverage — rebuild setiptah-roadgeometry-cpp with -fprofile-arcs
    # -ftest-coverage (via a CMake ROADGEOMETRY_COVERAGE option), run pytest, then
    # invoke lcov/genhtml to produce a combined Python+C++ coverage report.
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
def run(session):
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
    session.install("-e", ".")

    try:
        toml = nox.project.load_toml("pyproject.toml")
        dev_deps = toml["project"]["optional-dependencies"]["dev"]

    except:
        import json
        print(json.dumps(toml, indent=2))
        raise

    session.install(*dev_deps)
    session.run("jupyter", "notebook")


@nox.session
def format(session):
    try:
        toml = nox.project.load_toml("pyproject.toml")
        package_deps = toml["project"]["optional-dependencies"]["package"]

    except:
        import json
        print(json.dumps(toml, indent=2))
        raise

    session.install(*package_deps)
    session.run("black", "src")  # TODO: Migrate options to config?


@nox.session
def docs(session):
    """

    Configuring Sphinx for document generation seems like a travesty.

    https://eikonomega.medium.com/getting-started-with-sphinx-autodoc-part-1-2cebbbca5365
    https://www.youtube.com/watch?v=KKfQnxQBoWE
    https://stackoverflow.com/questions/2701998/automatically-document-all-modules-recursively-with-sphinx-autodoc/62613202#62613202

    """
    session.install("-e", ".")

    try:
        toml = nox.project.load_toml("pyproject.toml")
        pkg_deps = toml["project"]["optional-dependencies"]["package"]

    except:
        import json
        print(json.dumps(toml, indent=2))
        raise

    session.install(*pkg_deps)
    session.run("python", "-m", "sphinx_autobuild", "docs", "html")


@nox.session
def nbenv(session):
    """

    # https://nbconvert.readthedocs.io/en/latest/usage.html#notebook-and-preprocessors

    e.g.,
    # Execute the notebook in-place, i.e., as implicit test step for doc prep!
    jupyter nbconvert --execute --inplace docs/basic-demo.ipynb

    # Then, for version control..

    # Note quite enough, leaves the output metadata.
    jupyter nbconvert --clear-output --inplace docs/basic-demo.ipynb

    # Clear output _and_ metadata!
    jupyter nbconvert --inplace \
        --ClearOutputPreprocessor.enabled=True --ClearMetadataPreprocessor.enabled=True \
        docs/basic-demo.ipynb

    """
    session.install("-e", ".")
    toml = nox.project.load_toml("pyproject.toml")
    pkg_deps = toml["project"]["optional-dependencies"]["package"]
    session.install(*pkg_deps)
    session.run(*session.posargs)
    # session.run("make", "-C", "docs", "html")
