from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

PROJECT_ROOT = Path(__file__).resolve().parents[1]
WORKFLOW_PATH = PROJECT_ROOT / "scripts" / "faultnav_workflow.py"
WORKFLOW_MODULE_NAME = "faultnav_workflow_under_test"
WORKFLOW_SPEC = importlib.util.spec_from_file_location(WORKFLOW_MODULE_NAME, WORKFLOW_PATH)
assert WORKFLOW_SPEC is not None and WORKFLOW_SPEC.loader is not None
faultnav_workflow = importlib.util.module_from_spec(WORKFLOW_SPEC)
sys.modules[WORKFLOW_MODULE_NAME] = faultnav_workflow
WORKFLOW_SPEC.loader.exec_module(faultnav_workflow)

DEMO_ARTIFACT_NAMES = faultnav_workflow.DEMO_ARTIFACT_NAMES
DoctorCheck = faultnav_workflow.DoctorCheck
WorkflowError = faultnav_workflow.WorkflowError
demo_command = faultnav_workflow.demo_command
environment_is_ready = faultnav_workflow.environment_is_ready
expected_demo_artifacts = faultnav_workflow.expected_demo_artifacts
format_doctor_check = faultnav_workflow.format_doctor_check
sanitized_remote_url = faultnav_workflow.sanitized_remote_url
setup_fingerprint = faultnav_workflow.setup_fingerprint
venv_cli_path = faultnav_workflow.venv_cli_path
venv_python_path = faultnav_workflow.venv_python_path
verify_demo_artifacts = faultnav_workflow.verify_demo_artifacts


def _write_setup_inputs(project_root: Path, *, suffix: str = "") -> None:
    for name in faultnav_workflow.SETUP_INPUTS:
        (project_root / name).write_text(f"{name}{suffix}\n", encoding="utf-8")


def test_platform_specific_virtual_environment_paths() -> None:
    venv_dir = Path("environment")

    assert venv_python_path(venv_dir, "nt") == Path("environment/Scripts/python.exe")
    assert venv_cli_path(venv_dir, "nt") == Path(
        "environment/Scripts/faultnav-experiment.exe"
    )
    assert venv_python_path(venv_dir, "posix") == Path("environment/bin/python")
    assert venv_cli_path(venv_dir, "posix") == Path(
        "environment/bin/faultnav-experiment"
    )


def test_setup_fingerprint_is_repeatable_and_tracks_environment_inputs(tmp_path: Path) -> None:
    _write_setup_inputs(tmp_path)

    first = setup_fingerprint(tmp_path, (3, 12))
    second = setup_fingerprint(tmp_path, (3, 12))
    _write_setup_inputs(tmp_path, suffix="-changed")
    changed_input = setup_fingerprint(tmp_path, (3, 12))
    changed_python = setup_fingerprint(tmp_path, (3, 11))

    assert first == second
    assert changed_input != first
    assert changed_python != changed_input


def test_setup_fingerprint_reports_missing_required_file(tmp_path: Path) -> None:
    with pytest.raises(WorkflowError, match="Required setup file is missing"):
        setup_fingerprint(tmp_path, (3, 12))


def test_environment_ready_requires_matching_marker_cli_and_imports(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    project_root = tmp_path / "project"
    venv_dir = tmp_path / "venv"
    project_root.mkdir()
    _write_setup_inputs(project_root)
    python_path = venv_python_path(venv_dir)
    cli_path = venv_cli_path(venv_dir)
    python_path.parent.mkdir(parents=True)
    python_path.touch()
    cli_path.touch()
    faultnav_workflow._write_setup_marker(
        venv_dir / faultnav_workflow.MARKER_NAME,
        project_root,
        faultnav_workflow.sys.version_info[:2],
    )
    monkeypatch.setattr(faultnav_workflow, "_environment_imports_succeed", lambda *_: True)

    assert environment_is_ready(project_root, venv_dir)

    (project_root / "setup.py").write_text("changed\n", encoding="utf-8")
    assert not environment_is_ready(project_root, venv_dir)


def test_demo_command_uses_installed_cli_and_documented_deterministic_profile() -> None:
    command = demo_command(Path("venv/faultnav-experiment"), Path("artifacts/demo"))

    assert command[0] == str(Path("venv/faultnav-experiment"))
    assert command[command.index("--scenario") + 1] == "figure-eight"
    assert command[command.index("--step") + 1] == "0.1"
    assert command[command.index("--sensor-profile") + 1] == "combined-faults"
    assert command[command.index("--seed") + 1] == "7"
    assert command[command.index("--output-dir") + 1] == str(Path("artifacts/demo"))


def test_demo_artifact_validation_requires_all_non_empty_outputs(tmp_path: Path) -> None:
    expected = expected_demo_artifacts(tmp_path)
    assert tuple(path.name for path in expected) == DEMO_ARTIFACT_NAMES

    for path in expected[:-1]:
        path.write_text("generated\n", encoding="utf-8")
    expected[-1].touch()

    with pytest.raises(WorkflowError, match=expected[-1].name):
        verify_demo_artifacts(tmp_path)

    expected[-1].write_text("generated\n", encoding="utf-8")
    assert verify_demo_artifacts(tmp_path) == expected


def test_doctor_check_format_is_stable() -> None:
    row = format_doctor_check(DoctorCheck("Virtual environment", "OK", "ready"))

    assert row == "Virtual environment......... OK   ready"


@pytest.mark.parametrize(
    ("remote_url", "expected"),
    [
        (
            "https://user:secret@example.com/owner/repository.git?token=secret#fragment",
            "https://example.com/owner/repository.git",
        ),
        ("git@github.com:owner/repository.git", "git@github.com:owner/repository.git"),
    ],
)
def test_displayed_git_remote_does_not_expose_credentials(
    remote_url: str,
    expected: str,
) -> None:
    assert sanitized_remote_url(remote_url) == expected


@pytest.mark.parametrize(
    ("filename", "action"),
    [
        ("SETUP.bat", "setup"),
        ("RUN.bat", "run"),
        ("TEST.bat", "test"),
        ("DOCTOR.bat", "doctor"),
    ],
)
def test_windows_root_wrappers_delegate_to_shared_launcher(filename: str, action: str) -> None:
    contents = (PROJECT_ROOT / filename).read_text(encoding="utf-8")

    assert "scripts\\windows_launcher.bat" in contents
    assert f'windows_launcher.bat" {action}' in contents


def test_windows_launcher_keeps_results_visible_and_opens_demo_report() -> None:
    contents = (PROJECT_ROOT / "scripts" / "windows_launcher.bat").read_text(
        encoding="utf-8"
    )

    assert "FAULTNAV_NO_PAUSE" in contents
    assert "FAULTNAV_NO_OPEN" in contents
    assert "figure_eight_combined_faults_sensor_report.svg" in contents


@pytest.mark.parametrize(
    ("filename", "action"),
    [
        ("setup.sh", "setup"),
        ("run.sh", "run"),
        ("test.sh", "test"),
        ("doctor.sh", "doctor"),
    ],
)
def test_posix_root_wrappers_delegate_to_shared_launcher(filename: str, action: str) -> None:
    contents = (PROJECT_ROOT / filename).read_text(encoding="utf-8")

    assert "scripts/posix_launcher.sh" in contents
    assert contents.rstrip().endswith(action)
