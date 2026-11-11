# Nebula-YW local patches

Branch `nebula` = upstream `main` plus the changes recorded here. All changes are EPL-2.0.

## elk-native-splines.patch

Three changes that only affect `elk.edgeRouting = SPLINES`. They deviate from Java ELK on
purpose (the unpatched Rust code matches Java in all three places), so they are not proposed
upstream as parity fixes.

1. Hierarchy-crossing edges: keep each fragment's cubic segments and bridge fragments with an
   equivalent straight cubic, instead of joining control points with the orthogonal bend rule.
   Without it some cross-hierarchy edges get a control-point count that is not a valid cubic chain.
2. North/south ports: run `NorthSouthPortPostprocessor` after spline routing instead of before.
   Without it edges on north/south ports come out as straight lines with no control points.
3. Self loops: add near-corner support points so the curve does not cut into its own node.

The patch file is kept for provenance; it is already applied to the sources on this branch.
