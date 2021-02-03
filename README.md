Video: [Using the grease pencil option](https://www.youtube.com/watch?v=axUO3-l1cuk)

# bullet-constraints-tool

This is the Bullet Constraints Tool addon for Blender, originally created by the poster bashi on blenderartists.org:

https://blenderartists.org/t/wip-bullet-constraints-tools-0-3-7/560060

I have ported it to Blender 2.8 and will hopefully finish the incomplete features and maybe make some improvements.

Note: you can now find the UI for this tool in the Physics tab of the Properties area, once it is installed.

Bashi's original description is as follows:

### Usage:

The Tool is mostly selection based.

#### X Constraints
Generate multiple Constraints between Objects. (Atm it does compare Object Pivot/Center)

#### Neighbour Limit
Max Number of Constraints for X-Constraints

#### Search Radius
Max Distance Objects can have to still make Constraint between.

#### Generate Constraints
The Main Function of the Addon is to generate Constraints between Objects, usually pre-fractured (see Cell Fracture Addon or Destructibility Editor). It uses currently a simple nearest search and doesn’t do multiple Constraints. The Way it works is, for each selected Object it looks up its nearest Mesh and makes a Constraint (if distance within Search Radius), then makes the same procedure for this nearest Mesh. This means there is some kind of logic to it, but not much.

#### GPencil Mode
Disabled = Edit constraints, Enabled = Edit and Generate Constraints
GPencil
Allows to Draw/Edit Constraints with Grease Pencil Strokes. Uses Search Radius and Neighbour Limit.
Can be used to draw Ropes (See Video)

#### Distance for GPencil
Set distance for GPencil to take effect

#### Constraint Selected 2 Active
Connects all the selected Object to the Active Object.

#### Collision Margin
Collsion Margin

#### Friction
Friction of Objects

#### Enable Deactivation/Start Deactivated
Set Rigid Body dynamics Deactivation

#### Breakable
Enables Breakable for Constraints

#### Break Threshold
Force needed to break Constraint. Like Strength/Integrity. The Value for each Object gets calculated by it’s Mass, so:
Object Break Threshold = Object Mass * Bullet Tool Break Threshold (The Reason for this is simple, heavy Objects need stronger Constraints)

#### Absolute
In some cases you might want to set Absolute Break Threshold Values. This will skip Mass from calculation so:
Object Break Threshold = Bullet Tool Break Threshold

#### Multiply
In some cases you might want to make a Setup of different Constraints (different Break Values!) or edit Objects with different Break Values. So:
Object Break Threshold = Object Break Threshold * Bullet Tool Break Threshold

#### Ground Connect (WIP)
Connects selected Objects to active Object Vertices (usually Ground). Currently it uses the active Objects vertices, so you need some Vertices there in order to it to work. This feature is, like anything else, not yet finished.

#### Remove Constraints
Removes all Constraints on selected Objects. (includes Empties made from Ground Connect)
