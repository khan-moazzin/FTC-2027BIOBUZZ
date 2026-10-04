# BIOBUZZ field geometry

Sources: [FIRST field CAD v26-27.2, September 15](https://ftc-resources.firstinspires.org/ftc/archive/2027/field), [printable tag placement V1](https://ftc-resources.firstinspires.org/ftc/archive/2027/field/apriltag-us), and FTC SDK 12.0.0 AprilTagGameDatabase.getBioBuzzCluster.

The STEP file SHA-256 is 05b35961c7df847741031f00fda73ddd068537f809e92a11b1cd59a94bcc8331. Do not silently substitute another CAD revision.

CAD coordinates are meters with Y up and Z toward the audience. Convert to Pedro:
x = 72 + CAD.x / .0254; y = 72 - CAD.z / .0254; z = CAD.y / .0254.
Positive HIVE roll raises the scoring (+Y) cell. The CAD depicts blue at +30 degrees and red at -30 degrees.

The bearing inner-race cylindrical surfaces (representation #262123, radius .004 m) put the pivot at CAD Y=1.11632237999612 m, Z approximately zero. The blue bearing centers X=.342262318982907 and .305862318982908 average to .3240623189829075 m. Red is .6477 m to the left. This retains the CAD's 0.008359-inch X assembly offset rather than silently rounding it away.

The tag sticker representations are blue audience #262103, blue scoring #262130, red audience #262198, red scoring #262203. Sticker dimensions are .4318 by .127 m (17 by 5 inches). The tag centerline is 2.75 inches along the sticker Y axis, not the sticker's geometric center at 2.5 inches. Use the SDK's lateral tag offsets -6.5, -2.75, +2.75, +6.5 inches. Scoring-side ordering reverses in field X.

For example, blue scoring sticker placement origin is
(.108162318982907, 1.29337914002086, -.382270230572244) m;
its local Y direction is (0,-.5,.86602540378441).
Undo the CAD's +30-degree HIVE rotation after locating the tag centerline.
This gives neutral tag radius 14.2690594803 inches and neutral tag Z=-1.48816784224 inches relative to the pivot.

SDK cluster metadata places the tag row 7.1874 inches inward and 5.622 inches below the cell reference origin. Therefore the cell reference is at radius 21.4564594803 inches, Z=4.13383215776 inches. Shot-map heights refer to this target reference; actual successful trajectories are established by measured shot trials.

Tag clusters: red scoring 30â€“33, red audience 34â€“37, blue audience 38â€“41, blue scoring 42â€“45. Tags are 3.25 inches. SDK cluster poses at zero are local placeholders, not static world coordinates. Limelight must detect the actual tag size. Static botpose/MegaTag field maps are not used for moving tags.

Field.java is the single runtime geometry authority. GeometryTest includes a blue scoring CAD fixture as well as camera-transform and pose-recovery tests. Physical field tolerances and sticker placement still contribute measurement uncertainty.
