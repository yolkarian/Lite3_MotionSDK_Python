import struct
import sys
from pathlib import Path

from setuptools import setup
from setuptools.command.bdist_wheel import bdist_wheel
from setuptools.command.build_py import build_py
from setuptools.dist import Distribution

PACKAGE_DIR = Path("lite3_motion_sdk")
EXTENSION_MODULE = PACKAGE_DIR / "_lib.abi3.so"

ARCH_RUNTIME_LIBS = {
    "x86_64": (
        PACKAGE_DIR / "libdeeprobotics_legged_sdk_x86_64.so",
        PACKAGE_DIR / "libdeeprobotics_legged_wrapped_sdk_x86_64.so",
    ),
    "aarch64": (
        PACKAGE_DIR / "libdeeprobotics_legged_sdk_aarch64.so",
        PACKAGE_DIR / "libdeeprobotics_legged_wrapped_sdk_aarch64.so",
    ),
}
WHEEL_PLATFORM_BY_ARCH = {
    "x86_64": "linux_x86_64",
    "aarch64": "linux_aarch64",
}
ELF_MACHINE_BY_ARCH = {
    "x86_64": 0x3E,
    "aarch64": 0xB7,
}
ARCH_BY_ELF_MACHINE = {value: key for key, value in ELF_MACHINE_BY_ARCH.items()}
ALL_RUNTIME_LIBS = tuple(path for libs in ARCH_RUNTIME_LIBS.values() for path in libs)


def extension_arch(path: Path) -> str:
    with path.open("rb") as file:
        ident = file.read(16)
        if ident[:4] != b"\x7fELF":
            msg = f"{path} is not an ELF shared library"
            raise RuntimeError(msg)
        if ident[4] != 2:
            msg = f"{path} is not an ELF64 shared library"
            raise RuntimeError(msg)
        if ident[5] not in (1, 2):
            msg = f"{path} has unknown ELF byte order {ident[5]}"
            raise RuntimeError(msg)
        endian = "<" if ident[5] == 1 else ">"
        file.seek(18)
        machine = struct.unpack(f"{endian}H", file.read(2))[0]

    try:
        return ARCH_BY_ELF_MACHINE[machine]
    except KeyError as exc:
        supported = ", ".join(sorted(ARCH_RUNTIME_LIBS))
        msg = f"Unsupported extension architecture e_machine=0x{machine:x}; expected one of: {supported}"
        raise RuntimeError(msg) from exc


def runtime_libs_for_extension() -> tuple[Path, ...]:
    arch = extension_arch(EXTENSION_MODULE)
    return ARCH_RUNTIME_LIBS[arch]


class BinaryDistribution(Distribution):
    def has_ext_modules(self) -> bool:
        return True


class Abi3Wheel(bdist_wheel):
    def finalize_options(self) -> None:
        super().finalize_options()
        self.py_limited_api = "cp311"

    def get_tag(self) -> tuple[str, str, str]:
        python_tag, abi_tag, platform_tag = super().get_tag()
        if EXTENSION_MODULE.is_file() and sys.platform.startswith("linux"):
            arch = extension_arch(EXTENSION_MODULE)
            platform_tag = WHEEL_PLATFORM_BY_ARCH[arch]
        return python_tag, abi_tag, platform_tag


class BuildPy(build_py):
    def run(self) -> None:
        if not EXTENSION_MODULE.is_file():
            msg = (
                f"Missing {EXTENSION_MODULE}. Run `uv run python scripts/build_extension.py` "
                "before building a wheel."
            )
            raise RuntimeError(msg)

        selected_runtime_libs = runtime_libs_for_extension()
        missing_runtime_libs = [str(path) for path in selected_runtime_libs if not path.is_file()]
        if missing_runtime_libs:
            msg = "Missing bundled runtime library files for compiled extension architecture: " + ", ".join(
                missing_runtime_libs
            )
            raise RuntimeError(msg)

        super().run()

        package_build_dir = Path(self.build_lib) / PACKAGE_DIR
        package_build_dir.mkdir(parents=True, exist_ok=True)
        for stale_runtime_lib in package_build_dir.glob("libdeeprobotics_*.so"):
            stale_runtime_lib.unlink()
        for runtime_lib in selected_runtime_libs:
            self.copy_file(str(runtime_lib), str(package_build_dir / runtime_lib.name))

        selected_names = {path.name for path in selected_runtime_libs}
        skipped_names = sorted(path.name for path in ALL_RUNTIME_LIBS if path.name not in selected_names)
        if skipped_names:
            self.announce("skipping runtime libraries for other architectures: " + ", ".join(skipped_names), level=2)


setup(
    distclass=BinaryDistribution,
    cmdclass={"bdist_wheel": Abi3Wheel, "build_py": BuildPy},
)
