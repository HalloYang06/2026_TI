from pathlib import Path


PROJECT = Path(__file__).resolve().parents[1]


def test_keil_build_extends_vendor_stack_to_two_kibibytes() -> None:
    reserve_path = (
        PROJECT / "Drivers" / "MSPM0" / "hball_stack_reserve.s"
    )
    build = (PROJECT / "tools" / "build-keil.ps1").read_text(
        encoding="utf-8"
    )
    generator = (PROJECT / "tools" / "generate-keil-project.ps1").read_text(
        encoding="utf-8"
    )

    assert reserve_path.exists()
    reserve = reserve_path.read_text(encoding="utf-8")
    assert "AREA    STACK, NOINIT, READWRITE, ALIGN=3" in reserve
    assert "EXPORT  hball_stack_extension" in reserve
    assert "SPACE   0x00000700" in reserve

    reserve_source = "'Drivers\\MSPM0\\hball_stack_reserve.s'"
    vendor_startup = (
        '"$SdkRoot\\source\\ti\\devices\\msp\\m0p\\'
        'startup_system_files\\keil\\startup_mspm0g350x_uvision.s"'
    )
    assert build.index(reserve_source) < build.index(vendor_startup)
    assert "$minimumStackBytes = 0x800" in build
    assert "__initial_sp" in build
    assert "--keep=hball_stack_extension" in build
    assert "[regex]::Matches" in build
    assert "Non-contiguous target STACK sections" in build

    reserve_project_entry = (
        "@('hball_stack_reserve.s', '2', "
        "'..\\Drivers\\MSPM0\\hball_stack_reserve.s')"
    )
    vendor_project_entry = "@('startup_mspm0g350x_uvision.s', '2'"
    assert generator.index(reserve_project_entry) < generator.index(
        vendor_project_entry
    )
    assert "--keep=hball_stack_extension" in generator
