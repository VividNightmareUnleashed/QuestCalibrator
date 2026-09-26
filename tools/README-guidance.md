# Calibration motion assets

The overlay embeds three PNG atlases and the model credits from
`Overlay/assets`. Building or running the application does not require Blender.

To render an authoring file with Blender 5.1:

```powershell
blender --background path-to-authoring-file.blend --python tools/render-guidance.py -- --preview
blender --background path-to-authoring-file.blend --python tools/render-guidance.py
python tools/pack-guidance.py
```

The renderer uses Cycles with an OptiX GPU. Packing requires Python and Pillow.
`--setup handheld`, `--setup headset`, or `--setup contact` limits rendering to
one setup. Preview frames go to `x64/guidance/checks`; complete renders go to
`x64/guidance/frames`. The packer requires the completion manifest for every
setup and checks frame dimensions and format.

The authoring file must contain these scenes:

| Scene | Setup | Movement pivot |
| --- | --- | --- |
| Guide wrist | Touch Pro against a VIVE tracker on the opposite wrist | Wrist calibration movement |
| Guide mounted | VIVE tracker on a camera mount above the front of the Quest Pro visor, tilted outward | Head movement |
| Guide contact | Upright Index controller with its trigger side against the front of the visor | Head movement |

Object names may have Blender's numeric suffix. Head scenes also contain an
armature with a `Head` bone; shoulders stay fixed while the head and device pair
rotate together. Keep device contact fixed within the movement pivot.

The primary wrist demo is a two-second placement followed by a six-second
figure-eight sweep, with gradual yaw, pitch and roll throughout. Both hands
move as one rigid pair once the controller reaches the tracker. A subtle
path shows the sweep; its 24 cm width is illustrative, not a required target.
The camera gives the pair room to travel while keeping the devices readable.

The head alternatives use a two-second placement followed by three four-second
rotation loops. The fixed headset tracker skips placement. A short dissolve
introduces the three examples. These illustrate comfortable head movement,
not a figure eight with the user's body.

All sequences play at 60 fps with eased starts and stops:

| Atlas | Frame size | Frames | Columns / rows | Decoded memory |
| --- | --- | --- | --- | --- |
| Wrist | 320 x 192 | 120 placement + 360 sweep | 16 / 30 | 112.5 MiB |
| Each head alternative | 192 x 168 | 120 placement + 3 x 240 motion | 24 / 35 | 103.36 MiB |

The renderer, packer, WIC bounds in `UiWidgets.cpp`, and UV layout in
`UiGuide.cpp` must agree. Only one atlas is resident at a time; it is released
after closing the guide or reaching the result. Replay restarts placement.
Pause freezes the whole sequence; the Windows animation preference starts
with an illustrative still. Visible animation uses display sync for pacing;
idle and minimized windows retain their event wait.

`-uipreview-guide` opens the primary wrist demonstration without SteamVR;
`-uipreview-result` opens its success screen. Both disable profile saves.

Inspect the extreme poses and the animation in the overlay after regenerating.
The wrist illustration uses a wrist strap; the Index illustration has no hand.
Credits and source model licenses are recorded in
`Overlay/assets/guide-credits.txt` and embedded in the application's settings.
