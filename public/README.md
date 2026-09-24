# QuestCalibrator

QuestCalibrator fuses two VR tracking systems, such as a Meta Quest headset and
lighthouse-tracked trackers or controllers, into a single SteamVR playspace. It
calibrates the two against each other, then keeps them aligned while you play.

> **Important:** uninstall or disable OpenVR-SpaceCalibrator (and any fork of it)
> before installing QuestCalibrator. Both register a SteamVR driver that rewrites
> device poses; two active at once apply two transforms and mangle tracking.

## Download and install

1. Download `QuestCalibrator-<version>.zip` from the
   [latest release](https://github.com/VividNightmareUnleashed/QuestCalibrator/releases/latest).
   Ignore GitHub's automatic **Source code** archives; they don't contain the program.
2. Close Steam completely, not just SteamVR.
3. Extract the zip and follow the included `README-INSTALL.txt`. The recommended
   path is to run `Install.ps1` with PowerShell.

Every release lists the SHA-256 of its zip and a VirusTotal report for each
file inside it. Only download QuestCalibrator from this repository's releases.

## Updates

Automatic updates are off by default. Turn them on in Settings to let
QuestCalibrator check this repository's releases and download a newer stable
package. A download must match the SHA-256 that GitHub publishes for it before
it is offered, and installation starts only when you choose it.

## Reporting problems

[Open an issue](https://github.com/VividNightmareUnleashed/QuestCalibrator/issues)
and attach both log files from `%LOCALAPPDATA%\QuestCalibrator\`:
`QuestCalibrator.log` and `QuestCalibrator.prev.log`. They record every
calibration and correction, with the numbers behind it.

For alignment problems during play, turn on **Detailed logging** in Settings,
then use **Save diagnostics file** once while alignment looks correct and again
after the problem appears, before recalibrating or restarting. Say whether the
problem shows in SteamVR's own view or only in the game, and which devices look
misplaced.

## License

QuestCalibrator is free to download and use under the terms in [LICENSE](LICENSE).
Redistribution needs permission first, and no one may charge for it.

It is based on [OpenVR-SpaceCalibrator](https://github.com/pushrax/OpenVR-SpaceCalibrator)
by Justin Li, and includes other open-source components. Their licenses are in
[THIRD-PARTY-NOTICES.txt](THIRD-PARTY-NOTICES.txt).

QuestCalibrator is not affiliated with Meta, Valve, HTC or VRChat.
