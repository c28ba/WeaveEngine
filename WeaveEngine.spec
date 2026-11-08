# PyInstaller build of the WeaveEngine desktop app:
#
#     pyinstaller --noconfirm WeaveEngine.spec
#
# The result is dist/WeaveEngine.app on macOS and dist/WeaveEngine/ elsewhere.
import re
import sys

from PyInstaller.utils.hooks import collect_all

__version__ = re.search(r'__version__ = "([^"]+)"', open("weaveengine/__init__.py", encoding="utf-8").read())[1]

datas, binaries, hidden = [], [], []
# The geometry and compiler libraries carry compiled parts and data files that
# plain import analysis does not find.
for package in ("numba", "llvmlite", "shapely", "triangle"):
    d, b, h = collect_all(package)
    datas += d
    binaries += b
    hidden += h

a = Analysis(
    ["main.py"],
    datas=datas,
    binaries=binaries,
    hiddenimports=hidden + ["scipy.sparse.csgraph", "scipy.spatial", "scipy._lib.array_api_compat.numpy.fft",
                            "PySide6.QtOpenGL", "PySide6.QtOpenGLWidgets"],  # graphics-card drawing (imported only when switched on)
    excludes=["tkinter", "matplotlib", "IPython", "pytest", "PySide6.QtWebEngineCore", "PySide6.QtQml", "PySide6.QtQuick",
              "PySide6.Qt3DCore", "PySide6.QtMultimedia", "PySide6.QtPdf"],
)
pyz = PYZ(a.pure)
# The icon: assets/icon.png, as an .icns for macOS (made from it with iconutil) and an .ico for Windows; Linux takes the PNG.
icon = {"darwin": "assets/WeaveEngine.icns", "win32": "assets/WeaveEngine.ico"}.get(sys.platform, "assets/icon.png")
exe = EXE(pyz, a.scripts, [], exclude_binaries=True, name="WeaveEngine", console=False, upx=False, icon=icon)
coll = COLLECT(exe, a.binaries, a.datas, name="WeaveEngine", upx=False)
if sys.platform == "darwin":
    app = BUNDLE(coll, name="WeaveEngine.app", icon=icon, bundle_identifier="org.weaveengine.app",
                 info_plist={"NSHighResolutionCapable": True, "CFBundleShortVersionString": __version__,
                             "CFBundleVersion": __version__})
