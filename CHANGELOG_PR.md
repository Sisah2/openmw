*** PLEASE PUT YOUR ISSUE DESCRIPTION FOR DUMMIES HERE FOR REVIEW ***

- I'm just a placeholder description (#1337)
- I'm also just a placeholder description, but I'm a more recent one (#42)

***

0.47.0
------

The OpenMW team is proud to announce the release of version 0.47.0! Grab it from our Downloads Page for all operating systems. ***short summary: XXX ***

Check out the release video (***add link***) and the OpenMW-CS release video (***add link***) by the ***add flattering adjective*** Atahualpa, and see below for the full list of changes.

Known Issues:
- To use generic Linux binaries, Qt4 and libpng12 must be installed on your system
- On macOS, launching OpenMW from OpenMW-CS requires OpenMW.app and OpenMW-CS.app to be siblings

New Features:
- Dialogue to split item stacks now displays the name of the trapped soul for stacks of soul gems (#5362)
- NIF files which contain an "AvoidNode" are ignored by the pathfinding algorithm (#1724)
- Navmeshes are used for AI pathfinding which should resolve most related issues (#2229)
- Movement input from gamepad joysticks is transformed into analogue values (#3025)
- Sane default values for openmw.cfg file to overcome the original morrowind.ini file (#3442)
- Option to invert x-axis for controllers (#3610)
- Local variables of objects selected in the console can now be directly read and set without explicitly stating the object (#3893)
- In-game option to enable or disable controllers (#3980)
- Sneak mode can be toggled using a controller (#4001)
- Controllers use original engine's default key bindings (#4360)
- Support for sheathing animations, including weapon holstering, scabbards (except for throwing weapons), and quivers for projectiles (#4673)
- Support for "NiRollController" in NIF files to ensure correct rotation of models in "Weapon Sheathing" mod (#4675)
- Support for native animated containers (#4730)
- Support for VAO ("Vertex Array Objects") from OSG 3.5.6 or later (#4756)
- Support for "NiSwitchNode" in NIF files to allow future implementation of native support for extended features like harvestable plants or glowing - windows (#4812)
- Native support for glowing windows (and other daytime-dependent meshes) by adding internal day-night-mode switch (#4836)
- Shadows (#4851)
- More configuration options for in-game water reflections (#4859)
- Command line option to specify a random seed to be used by the game's random-number generator ("RNG") for debugging purposes (#4887)
- Configuration options for distant terrain to adjust quality and performance impact (#4890)
- Head bobbing and matching weapon bobbing are now available in first-person view, disabled by default (#5043)
New Editor Features:
- ?

Bug Fixes:
- NiParticleColorModifier in NIF files is now properly handled which solves issues regarding particle effects, e.g., smoke and fire (#1952, #3676)
- Targetting non-unique actors in scripts is now supported (#2311)
- Guards no longer ignore attacks of invisible players but rather initiate dialogue and flee if the player resists being arrested (#4774)
- Changing the dialogue window without closing it no longer clears the dialogue history in order to allow, e.g., emulation of three-way dialogue via ForceGreeting (#5358)
- Scripts which try to start a non-existent global script now skip that step and continue execution instead of breaking (#5364)
- Selecting already equipped spells or magic items via hotkey no longer triggers the equip sound to play (#5367)
- 'Scale' argument in levelled creature lists is now taken into account when spawning creatures from such lists (#5369)
- Morrowind legacy madness: Using a key on a trapped door/container now only disarms the trap if the door/container is locked (#5370)

Editor Bug Fixes:
- Verifier no longer checks for alleged 'race' entries in clothing body parts (#5400)

Miscellaneous:
- Prevent save-game bloating by using an appropriate fog texture format (#5108)
- Ensure that 'Enchantment autocalc" flag is treated as flag in OpenMW-CS and in our esm tools (#5363)
