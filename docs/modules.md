# Modules

Most people install QuestCalibrator for the classic space calibrator, with
its improvements. Features the original calibrator never had, and that a
player may not want, ship as optional modules: the installer asks about each
one, and a plain install leaves them out.

A module is part of the same executable. It is not loaded or left out at
build time; the installer records which ones the player chose and the
overlay only turns on what was chosen.

## The modules

| Module | Status | What it controls |
| --- | --- | --- |
| Lighthouse | optional | Base station tools. So far one: the Lighthouse tab, each base station and which lighthouse devices see it (`docs/lighthouse-visibility.md`). More are planned. |
| Smoothing | not built | Its tab only, greyed out with "Work in progress". No installer offers it yet. |

What a module does not control stays core. Reading SteamVR's lighthouse
log, its session log lines, the `[base stations]` diagnostics section and
the rule that keeps a tracker's own base station changes out of the drift
evidence all run on every install, with or without the Lighthouse module:
they make calibration better whether or not the player looks at the tab.

## Module or core

Lighthouse is slim today and will grow, and so will any module after it.
Each new feature is sorted by one question: does it make the classic
calibrator better for someone who never asked for the module?

- **Core** if yes, and it changes nothing a classic user would notice
  beyond better calibration: shared data (the lighthouse log reader and
  `LighthouseVisibility`), rules that clean up calibration evidence, and
  diagnostics, which support needs from every install.
- **The module** if it is something the player sees or does that the
  original calibrator had not: new screens, controls, settings, overlays,
  notifications, or anything that acts on SteamVR's base stations. It
  checks `Modules::On(CalCtx.modules.<name>)` everywhere it shows up, not
  only on its tab.

Module features build on the core data; core never depends on a module.

## How a choice travels

1. `Install.ps1` asks about each module before it changes anything. On an
   upgrade the default is the previous install's choice, otherwise no.
   `-Lighthouse` installs the module without asking; an `-Unattended`
   install gets only the modules named on its command line.
2. It records each choice as a DWORD, 1 or 0, under
   `HKLM\Software\QuestCalibrator\Modules` (value `Lighthouse`). Running the
   installer again is how a player adds or removes a module.
3. At startup the overlay reads that key once (`ReadInstalledModules`,
   `Overlay/Modules.cpp`) into `CalCtx.modules`. A missing key or value
   means not installed. Everything a module gates asks `CalCtx.modules`.
4. `Uninstall.ps1` removes `HKLM\Software\QuestCalibrator` and the choices
   with it.

A manual install has no installer to ask; the install readme gives the
`reg add` line that turns a module on.

## What the player sees

- A module's tab is greyed out until it is installed. Hovering it says the
  module is not installed and to select it during installation; a module
  that is not built says "Work in progress".
- A diagnostics export has a `[modules]` section with each module's
  status (`installed`, `not installed`, `not built`).
- The preview never reads the registry: every module is off except in
  `-uipreview-lighthouse`, which turns the Lighthouse module on.

## Adding a module

1. A `ModuleStatus` field in `questcal::Modules` (`Overlay/Modules.h`),
   `NotBuilt` until it ships.
2. Read its registry value in `ReadInstalledModules`.
3. Gate what it owns on `Modules::On(CalCtx.modules.<name>)`; a tab goes in
   `moduleTabs` in `BuildHeader` (`Overlay/UserInterface.cpp`) with its
   not-installed tooltip, translated in every table.
4. A question and a switch in `Install.ps1`, a line in the install readme,
   a line in the `[modules]` diagnostics section, and a row above.
