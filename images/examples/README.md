# Example maps

Launch `.venv/bin/python gui/server.py --open` and choose a map from **Map**.
Waffle remains the default. These ten PNGs are loaded automatically; they can
also be uploaded into an existing session. White (255) is free space, black (0)
is occupied. Coordinates and turning radius are in pixels/cells. No additional
inflation or smoothing is applied. Dimensions below are **width × height**;
the waffle reference is 551 × 771 (424,821 pixels).

| Map | Dimensions | Pixels | Features |
| --- | --- | ---: | --- |
| [U traps](01_u_traps.png) | 384 × 288 | 110,592 | Three open bug traps facing different directions. |
| [T junctions](02_t_junctions.png) | 512 × 384 | 196,608 | Connected corridor tree, T junctions and hooked dead ends. |
| [Switchbacks](03_switchbacks.png) | 600 × 360 | 216,000 | Alternating baffles, long detours and side traps. |
| [Pillar grid](04_pillar_grid.png) | 640 × 640 | 409,600 | Staggered round pillars of three sizes. |
| [Noisy pillars](05_noisy_pillars.png) | 768 × 576 | 442,368 | Jittered rectangular/elliptical pillars; 0.07% interior flip probability. |
| [Nonconvex islands](06_nonconvex_islands.png) | 800 × 540 | 432,000 | L, C, U, star and chevron-shaped obstacles. |
| [Rooms and doors](07_rooms_and_doors.png) | 720 × 600 | 432,000 | Nine rooms, varied doorway widths and interior blocks. |
| [Large maze](08_large_maze.png) | 1024 × 768 | 786,432 | Seeded maze with loops, U turns, junctions and dead ends. |
| [Large corrals](09_large_corrals.png) | 1280 × 768 | 983,040 | Large reentrant corrals, an L obstacle and a V trap. |
| [Salt pepper terrain](10_salt_pepper_terrain.png) | 1024 × 1024 | 1,048,576 | Mixed obstacles and bug traps; 0.2% interior flip probability. |

The eight clean maps have one connected free-space component under four-neighbor
pixel adjacency. Salt-and-pepper noise flips both free and occupied pixels;
the two noisy maps intentionally include isolated free pixels inside obstacles.
Pixel connectivity does not establish Reeds–Shepp reachability for a chosen
start orientation and turning radius. These are example inputs, without solver
coverage or optimality claims.
