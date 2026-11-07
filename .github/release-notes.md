A PCB autorouter for KiCad: export a Specctra DSN, route it in WeaveEngine,
import the session back. See the [README](https://github.com/c28ba/WeaveEngine#readme)
for how to use it and its known limitations.

**Download** the file for your system below and unpack it:

- **Windows:** `WeaveEngine-windows.zip`, then run `WeaveEngine\WeaveEngine.exe`.
  If SmartScreen warns, choose **More info**, then **Run anyway**.
- **macOS** (Apple Silicon): `WeaveEngine-macos.zip`, then open `WeaveEngine.app`.
  The first time, macOS will not open it: go to **System Settings → Privacy &
  Security** and choose **Open Anyway**.
- **Linux** (Ubuntu 22.04, Debian 12 or newer): `WeaveEngine-linux.tar.gz`, then
  run `WeaveEngine/WeaveEngine`. If it does not start, see the README for the
  libraries to install.

The builds are not signed. Always run KiCad's design rules check on the
imported result before you order a board.
