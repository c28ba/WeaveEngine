# PyInstaller build of the WeaveEngine desktop app:
#
#     pyinstaller --noconfirm WeaveEngine.spec
#
# The result is dist/WeaveEngine.app on macOS and dist/WeaveEngine/ elsewhere.
import sys

from PyInstaller.utils.hooks import collect_all

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
exe = EXE(pyz, a.scripts, [], exclude_binaries=True, name="WeaveEngine", console=False, upx=False)
coll = COLLECT(exe, a.binaries, a.datas, name="WeaveEngine", upx=False)
if sys.platform == "darwin":
    app = BUNDLE(coll, name="WeaveEngine.app", bundle_identifier="org.weaveengine.app",
                 info_plist={"NSHighResolutionCapable": True, "CFBundleShortVersionString": "0.1.0"})
