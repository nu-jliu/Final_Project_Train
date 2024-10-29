include </home/jingkun/.local/lib/python3.10/site-packages/solid2/extensions/bosl2/BOSL2/version.scad>;
include </home/jingkun/.local/lib/python3.10/site-packages/solid2/extensions/bosl2/BOSL2/constants.scad>;
include </home/jingkun/.local/lib/python3.10/site-packages/solid2/extensions/bosl2/BOSL2/transforms.scad>;
include </home/jingkun/.local/lib/python3.10/site-packages/solid2/extensions/bosl2/BOSL2/distributors.scad>;
include </home/jingkun/.local/lib/python3.10/site-packages/solid2/extensions/bosl2/BOSL2/mutators.scad>;
include </home/jingkun/.local/lib/python3.10/site-packages/solid2/extensions/bosl2/BOSL2/color.scad>;
include </home/jingkun/.local/lib/python3.10/site-packages/solid2/extensions/bosl2/BOSL2/attachments.scad>;
include </home/jingkun/.local/lib/python3.10/site-packages/solid2/extensions/bosl2/BOSL2/shapes3d.scad>;
include </home/jingkun/.local/lib/python3.10/site-packages/solid2/extensions/bosl2/BOSL2/shapes2d.scad>;
include </home/jingkun/.local/lib/python3.10/site-packages/solid2/extensions/bosl2/BOSL2/drawing.scad>;
include </home/jingkun/.local/lib/python3.10/site-packages/solid2/extensions/bosl2/BOSL2/masks3d.scad>;
include </home/jingkun/.local/lib/python3.10/site-packages/solid2/extensions/bosl2/BOSL2/masks2d.scad>;
include </home/jingkun/.local/lib/python3.10/site-packages/solid2/extensions/bosl2/BOSL2/math.scad>;
include </home/jingkun/.local/lib/python3.10/site-packages/solid2/extensions/bosl2/BOSL2/paths.scad>;
include </home/jingkun/.local/lib/python3.10/site-packages/solid2/extensions/bosl2/BOSL2/lists.scad>;
include </home/jingkun/.local/lib/python3.10/site-packages/solid2/extensions/bosl2/BOSL2/comparisons.scad>;
include </home/jingkun/.local/lib/python3.10/site-packages/solid2/extensions/bosl2/BOSL2/linalg.scad>;
include </home/jingkun/.local/lib/python3.10/site-packages/solid2/extensions/bosl2/BOSL2/trigonometry.scad>;
include </home/jingkun/.local/lib/python3.10/site-packages/solid2/extensions/bosl2/BOSL2/vectors.scad>;
include </home/jingkun/.local/lib/python3.10/site-packages/solid2/extensions/bosl2/BOSL2/affine.scad>;
include </home/jingkun/.local/lib/python3.10/site-packages/solid2/extensions/bosl2/BOSL2/coords.scad>;
include </home/jingkun/.local/lib/python3.10/site-packages/solid2/extensions/bosl2/BOSL2/geometry.scad>;
include </home/jingkun/.local/lib/python3.10/site-packages/solid2/extensions/bosl2/BOSL2/regions.scad>;
include </home/jingkun/.local/lib/python3.10/site-packages/solid2/extensions/bosl2/BOSL2/strings.scad>;
include </home/jingkun/.local/lib/python3.10/site-packages/solid2/extensions/bosl2/BOSL2/skin.scad>;
include </home/jingkun/.local/lib/python3.10/site-packages/solid2/extensions/bosl2/BOSL2/vnf.scad>;
include </home/jingkun/.local/lib/python3.10/site-packages/solid2/extensions/bosl2/BOSL2/utility.scad>;
include </home/jingkun/.local/lib/python3.10/site-packages/solid2/extensions/bosl2/BOSL2/partitions.scad>;
include </home/jingkun/.local/lib/python3.10/site-packages/solid2/extensions/bosl2/BOSL2/metric_screws.scad>;

union() {
	difference() {
		union() {
			path_sweep(path = arc(angle = 20, d = 900, n = 20), shape = [[0, 0], [20, 0], [20, 10], [15, 10], [15, 7], [10, 7], [10, 10], [0, 10], [-10, 10], [-10, 7], [-15, 7], [-15, 10], [-20, 10], [-20, 0]]);
			fwd(y = 15) {
				right(x = 450.0) {
					cylinder(h = 10, r = 5.5);
				}
			}
			fwd(y = 10) {
				right(x = 447.5) {
					cube(size = [5, 20, 10]);
				}
			}
		}
		down(z = 0.5) {
			back(y = 141.69306042633417) {
				right(x = 427.30794121689246) {
					cylinder(h = 11, r = 6.5);
				}
			}
		}
		translate(v = [0, -153.90906449655097, 0]) {
			translate(v = [422.86167935365876, 0, 0]) {
				rotate(a = 20) {
					cube(anchor = CENTER, size = [7, 15, 30]);
				}
			}
		}
	}
}
